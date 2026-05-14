# 敌方前哨站摧毁状态标志位接入方案

## 背景

当前导航侧 `decision_messages/msg/GameState.msg` 中已有 `outpoststate` 字段，但该字段目前没有接入电控层数据。串口节点发布 `/decision_messages/GameState` 时没有给该字段赋值，因此决策侧读取到的值通常保持默认 `0`。

由于裁判系统目前无法稳定读取敌方前哨站血量，不能直接通过敌方前哨站 HP 判断是否被摧毁。因此改为由自瞄侧基于视觉识别结果提供“发现不亮前哨站装甲板”的信息，再由导航侧结合目标相对位置、云台相对底盘 yaw、自身位姿和红蓝方信息判断该目标位于敌方半场还是己方半场，最终只把“敌方前哨站是否被摧毁”的结果传给决策侧。

仅识别到“不亮的前哨站装甲板”还不够，因为它可能是己方前哨站，也可能是敌方前哨站。必须增加空间归属判断，避免己方前哨站死亡时误触发“敌方前哨站已摧毁”的决策。

## 结论

该方案合理，但需要明确一点：决策侧最终使用的标志位不是裁判系统血量真值，也不应由自瞄侧单独直接判定为“敌方”。更稳妥的责任划分是：

- 自瞄侧：在过滤不亮装甲板之前，识别“不亮的前哨站装甲板”，并输出目标 ID 与相对位置
- 电控/下板：补齐并透传自瞄目标信息，同时提供云台相对底盘 yaw
- 导航侧：把目标相对位置转换到场地坐标，判断该目标是否在敌方半场或接近敌方前哨站区域
- 决策侧：只消费导航侧发布的最终 `0/1` 状态

## 统一字段语义

建议将决策侧语义改为只表示敌方前哨站状态：

| 值 | 含义 |
| --- | --- |
| `0` | 敌方前哨站未被确认摧毁，或当前没有可靠视觉确认 |
| `1` | 敌方前哨站已被视觉侧确认摧毁 |

注意：不要继续使用原注释里的 `1=红方前哨站被摧毁，2=蓝方前哨站被摧毁` 语义，否则会和本方案冲突。

建议字段命名：

- 决策消息层：优先新增或改用 `enemy_outpost_destroyed`
- 如果为了减少改动继续复用 `outpoststate`，则必须同步修改 `GameState.msg` 注释，明确 `0/1` 语义
- 串口通信帧：使用 `enemy_outpost_destroyed` 或等价的 `uint8_t`/bit 标志位

## 数据流

目标链路：

```text
自瞄侧在过滤前识别不亮前哨站装甲板
  -> 输出 _enemy_id / _enemy_x / _enemy_y / unlit_outpost_candidate
  -> 上板到下板通信协议
  -> 下板补充 _base_yaw 并打包到导航通信帧
  -> 导航侧 myserial 解析目标信息和 _base_yaw
  -> 导航侧结合自身位姿判断目标属于敌方半场还是己方半场
  -> /decision_messages/GameState
  -> 决策 Blackboard
  -> 决策行为切换
```

## 坐标、单位和发送时机约定

各侧必须按同一套定义实现，否则导航侧无法可靠判断敌我半场。

### 自瞄目标信息

`_enemy_id`、`_enemy_x`、`_enemy_y` 表示自瞄当前锁定目标的信息。只要自瞄锁到目标，无论目标是不是前哨站，都应该发送这三个字段。

字段约定：

```cpp
uint8_t _enemy_id;  // 当前锁定目标 ID；未锁定目标时为 0
float _enemy_x;     // 目标在云台坐标系下的相对 x 距离，单位：m
float _enemy_y;     // 目标在云台坐标系下的相对 y 距离，单位：m
```

坐标系约定：

- `_enemy_x`、`_enemy_y` 是目标相对于云台的距离，不是场地坐标，也不是底盘坐标
- 单位统一使用 `m`
- 云台坐标系的正方向必须由自瞄、电控、导航三方确认；建议约定 `x` 为云台前方，`y` 为云台左方
- 未锁定目标时，`_enemy_id = 0`，`_enemy_x = 0.0`，`_enemy_y = 0.0`
- 如果自瞄内部使用 `mm`、`cm` 或像素距离，发送前必须转换成 `m`

### 云台相对底盘 yaw

`_base_yaw` 表示云台相对于底盘的 yaw 角。该字段不依赖是否锁到目标，电控侧任何时候都应该发送。

字段约定：

```cpp
float _base_yaw;  // 云台相对底盘 yaw 角，单位：deg
```

角度约定：

- 单位统一使用 `deg`
- 零位：云台朝向和底盘正前方一致时，`_base_yaw = 0`
- 正方向必须由电控和导航确认；建议约定逆时针为正
- 导航侧如需三角函数计算，使用前转换为弧度
- 该字段应持续发送，不因未锁定目标而置零

### 前哨站不亮候选标志

`unlit_outpost_candidate` 只表示“自瞄在过滤不亮装甲板之前发现了不亮的前哨站装甲板候选”，不表示该前哨站一定属于敌方。

```cpp
uint8_t unlit_outpost_candidate;  // 0: 无可靠候选, 1: 有不亮前哨站装甲板候选
```

导航侧最终使用以下数据共同判断：

```text
_enemy_id
_enemy_x
_enemy_y
_base_yaw
unlit_outpost_candidate
机器人自身场地位姿
我方颜色/半场信息
```

## 各负责人任务

### 1. 决策负责人

目标：决策侧使用 `0/1` 的敌方前哨站摧毁状态进行策略切换。

需要完成：

- 修改 `src/RMUC/decision_messages/msg/GameState.msg` 中 `outpoststate` 的注释，或新增更清晰的字段 `enemy_outpost_destroyed`
- 决策侧 `Blackboard` 读取该字段，并转换为布尔语义，例如：
  - `enemy_outpost_destroyed = msg->outpoststate == 1`
- 决策逻辑中只使用这个布尔语义，不再按红蓝方 `1/2` 判断
- 明确默认值为 `0`，表示未确认摧毁
- 决策侧不负责判断前哨站属于敌方还是己方，该判断由导航侧完成

当前相关代码位置：

- `src/RMUC/decision_messages/msg/GameState.msg`
- `src/RMUC/robomaster_sentry_decision/src/Blackboard.cpp`
- `src/RMUC/robomaster_sentry_decision/include/sentry_decision/Blackboard.hpp`

验收标准：

- 手动发布 `/decision_messages/GameState`，`outpoststate=0` 时决策保持敌方前哨站未摧毁逻辑
- 手动发布 `/decision_messages/GameState`，`outpoststate=1` 时决策切换到敌方前哨站已摧毁逻辑
- 重新 `colcon build --packages-select decision_messages robomaster_sentry_decision` 能通过

### 2. 自瞄负责人

目标：自瞄侧在现有“不亮装甲板过滤”逻辑之前，识别被过滤目标是否为前哨站装甲板，并输出候选目标信息。自瞄侧不单独判断该前哨站属于敌方还是己方。

当前自瞄侧已有逻辑会过滤灯条不亮的装甲板。为了不破坏现有目标筛选流程，建议只在过滤之前插入一段检测逻辑：

```text
装甲板候选目标
  -> 判断是否为前哨站装甲板
  -> 判断灯条是否不亮
  -> 若是前哨站装甲板且灯条不亮，输出 unlit_outpost_candidate 和目标相对位置
  -> 继续进入原有“不亮装甲板过滤”流程
```

建议判定逻辑：

- 在不亮装甲板被过滤之前，先检查该装甲板类别是否为前哨站装甲板
- 如果该候选目标是前哨站装甲板，且灯条不亮，则计入“前哨站疑似摧毁候选”帧
- 连续多帧满足条件后输出 `unlit_outpost_candidate = 1`
- 同时输出当前锁定目标的 `_enemy_id`、`_enemy_x`、`_enemy_y`
- 如果候选目标不是前哨站装甲板，或者灯条仍亮，则输出 `unlit_outpost_candidate = 0`
- 原有的不亮装甲板过滤逻辑继续保留，不需要为了这个标志位改变后续自瞄选板逻辑

建议增加防抖：

- 连续 `N` 帧在过滤前识别到“前哨站装甲板且灯条不亮”后才置 `1`
- `N` 建议先取 `3` 到 `5`，后续实车调参
- 自瞄侧是否锁存可以由自瞄负责人决定；如果只输出瞬时候选，导航侧需要做连续帧确认和锁存
- 推荐最终锁存在导航侧完成，因为导航侧才能判断敌我归属

输出给上板的字段：

```cpp
uint8_t unlit_outpost_candidate;  // 0: 没有可靠候选, 1: 过滤前发现不亮前哨站装甲板
uint8_t _enemy_id;                // 当前锁定目标 ID；未锁定目标时为 0
float _enemy_x;                   // 目标在云台坐标系下的相对 x 距离，单位：m
float _enemy_y;                   // 目标在云台坐标系下的相对 y 距离，单位：m
```

验收标准：

- 非前哨站装甲板即使灯条不亮，也不会触发该标志位
- 前哨站装甲板灯条正常亮时，该标志位为 `0`
- 前哨站装甲板在过滤前被识别到且灯条连续多帧不亮时，`unlit_outpost_candidate` 为 `1`
- 只要自瞄锁到目标，`_enemy_id`、`_enemy_x`、`_enemy_y` 都会发送；未锁定时按默认值发送
- 原有“不亮装甲板过滤”行为不受影响
- 日志中能打印候选装甲板类型、灯条状态、是否会被过滤、连续帧计数、目标相对位置、最终候选标志位

### 3. 下板/电控负责人

目标：接收自瞄侧候选目标信息，补齐云台相对底盘 yaw，并在给导航侧的通信帧中透传。

需要完成：

- 上板到下板通信协议需要包含自瞄输出的：
  - `uint8_t _enemy_id`
  - `float _enemy_x`
  - `float _enemy_y`
  - `uint8_t unlit_outpost_candidate`
- 下板需要把云台相对底盘 yaw 写入发给导航侧的通信帧：
  - `float _base_yaw`
- 下板发给导航侧的通信帧需要同时包含 `_base_yaw`、`_enemy_id`、`_enemy_x`、`_enemy_y`、`unlit_outpost_candidate`
- 保证这些字段属于同一时刻或足够接近的同一帧数据，避免 yaw 和目标位置错帧导致归属判断错误
- `_base_yaw` 任何时候都要发送，不依赖自瞄是否锁到目标
- `_enemy_id`、`_enemy_x`、`_enemy_y` 在自瞄锁到目标时发送真实值，未锁定时发送默认值
- 比赛重置或通信异常时，`unlit_outpost_candidate` 默认置 `0`

建议协议字段：

```cpp
float _base_yaw;                  // 云台相对底盘 yaw 角，单位：deg
uint8_t _enemy_id;                // 当前锁定目标 ID；未锁定目标时为 0
float _enemy_x;                   // 目标在云台坐标系下的相对 x 距离，单位：m
float _enemy_y;                   // 目标在云台坐标系下的相对 y 距离，单位：m
uint8_t unlit_outpost_candidate;  // 0: 没有可靠候选, 1: 发现不亮前哨站装甲板
```

若通信帧空间紧张，可放入已有状态字的某个 bit，但必须在协议文档里明确 bit 位，例如：

```text
status_flags bit X: unlit_outpost_candidate
```

验收标准：

- 自瞄侧发 `_enemy_id / _enemy_x / _enemy_y / unlit_outpost_candidate`，下板能原样转发给导航侧
- 下板能持续提供实时 `_base_yaw`，单位为 `deg`，并明确正方向和零位
- 自瞄未锁目标时，下板仍持续发送 `_base_yaw`，并发送 `_enemy_id=0, _enemy_x=0.0, _enemy_y=0.0`
- 自瞄侧发 `unlit_outpost_candidate=0`，下板转发给导航侧为 `0`
- 自瞄侧发 `unlit_outpost_candidate=1`，下板转发给导航侧为 `1`
- 自瞄通信丢失时，下板不会错误保持一次性的误报状态；如采用锁存策略，必须有比赛重置清零机制

### 4. 导航/myserial 负责人

目标：导航侧从下板通信帧解析候选目标信息和 `_base_yaw`，判断不亮前哨站装甲板属于敌方还是己方，并发布最终决策标志位。

需要完成：

- 在 `src/myserial/include/myserial/myprotocol.hpp` 的接收帧结构中确认或加入以下字段：
  - `_base_yaw`
  - `_enemy_id`
  - `_enemy_x`
  - `_enemy_y`
  - `unlit_outpost_candidate` 或等价 bit
- 当前导航侧已有如下接收字段，但需要确认它们已经由自瞄和电控实际发送：

```cpp
float _base_yaw = 0;   // 云台相对底盘 yaw 角，单位：deg
uint8_t _enemy_id = 0;
float _enemy_x = 0;    // 目标在云台坐标系下的相对 x 距离，单位：m
float _enemy_y = 0;    // 目标在云台坐标系下的相对 y 距离，单位：m
```

- 使用 `_base_yaw`、`_enemy_x`、`_enemy_y`、机器人自身位姿，把候选目标估计到场地坐标
- `_base_yaw` 单位为 `deg`，导航侧计算前需要转换为 `rad`
- `_enemy_x`、`_enemy_y` 单位为 `m`，是云台坐标系下的相对距离
- 根据我方颜色、场地坐标、半场边界或前哨站已知区域，判断该不亮前哨站装甲板是否属于敌方
- 只有当 `unlit_outpost_candidate == 1` 且目标被判断为敌方前哨站时，才发布 `outpoststate = 1`
- 如果候选目标在己方半场、无法判断归属、坐标无效或 yaw 数据无效，则发布 `outpoststate = 0`
- 建议导航侧做最终锁存：一旦连续多帧确认敌方前哨站摧毁，本局内保持 `outpoststate = 1`
- 在 `src/myserial/src/serial_node.cpp` 发布 `decision_messages::msg::GameState` 时填充：

```cpp
gs.outpoststate = enemy_outpost_destroyed ? 1 : 0;
```

当前相关代码位置：

- `src/myserial/include/myserial/myprotocol.hpp`
- `src/myserial/src/serial_node.cpp`
- `src/RMUC/decision_messages/msg/GameState.msg`

验收标准：

- 串口帧中 `unlit_outpost_candidate=0` 时，`ros2 topic echo /decision_messages/GameState` 显示 `outpoststate: 0`
- 串口帧中 `unlit_outpost_candidate=1` 但目标被判断在己方半场时，`outpoststate: 0`
- 串口帧中 `unlit_outpost_candidate=1` 且目标被判断在敌方半场时，`outpoststate: 1`
- `_base_yaw`、`_enemy_id`、`_enemy_x`、`_enemy_y` 在导航日志中可见，方便联调坐标归属
- `colcon build --packages-select decision_messages myserial robomaster_sentry_decision` 能通过

## 推荐实现细节

### 是否复用 `outpoststate`

短期可以复用 `outpoststate`，改动最少。但必须修改注释，避免负责人误以为它仍表示红蓝方前哨站状态。

推荐短期注释：

```text
# 敌方前哨站视觉确认摧毁状态：0 未确认摧毁，1 已确认摧毁
int8 outpoststate
```

长期更推荐新增清晰字段：

```text
# 敌方前哨站视觉确认摧毁状态：0 未确认摧毁，1 已确认摧毁
uint8 enemy_outpost_destroyed
```

### 是否需要锁存

建议导航侧做最终锁存：一旦确认敌方前哨站摧毁，本局内保持 `1`，直到比赛重置。

原因：

- 前哨站摧毁是不可逆事件
- 视觉后续不一定持续看得到前哨站
- 只有导航侧同时拥有目标相对位置、云台 yaw、自身位姿和红蓝方信息，适合作为最终敌我归属判断位置
- 决策侧需要稳定状态，不适合跟随视觉瞬时识别抖动

建议清零条件：

- 比赛阶段从非比赛中进入新一局比赛
- 人工复位
- 系统启动默认值

### 异常处理

需要避免以下情况：

- 单帧误识别导致直接置 `1`
- 视觉识别到己方不亮前哨站时误认为敌方摧毁
- `_base_yaw`、`_enemy_x`、`_enemy_y` 错帧导致目标场地坐标估计错误
- 自瞄和电控对目标坐标系、yaw 单位、yaw 正方向理解不一致
- 通信中断后使用未初始化字段
- 上下板、导航、决策对 `0/1` 含义不一致

最低要求：

- 自瞄侧连续多帧确认
- 电控侧实际发送 `_base_yaw`、`_enemy_id`、`_enemy_x`、`_enemy_y` 和候选标志位
- 导航侧做敌我半场归属判断
- 通信字段默认值为 `0`
- 导航侧发布前强制归一化为 `0/1`
- 决策侧只把 `1` 当作摧毁，其他值都按 `0` 处理

## 联调步骤

1. 自瞄侧先用日志验证候选信息：候选装甲板类型、灯条状态、是否进入不亮过滤、连续帧计数、`_enemy_id`、`_enemy_x`、`_enemy_y`、`unlit_outpost_candidate`。
2. 下板打印接收到的自瞄候选信息、`_base_yaw`，以及发给导航侧的完整字段。
3. 导航侧打印 `_base_yaw`、`_enemy_id`、`_enemy_x`、`_enemy_y`、候选目标场地坐标、敌我半场判断结果。
4. 决策侧打印 `enemy_outpost_destroyed` 或 `outpost_state`，确认能触发策略切换。
5. 做误触发测试：己方前哨站不亮、锁定非前哨站、短暂丢灯、遮挡、通信断开、yaw 数据异常，确认不会误置 `1`。

## 最小改动版接口约定

如果本次希望最快接入，建议统一采用以下约定：

```text
字段名：outpoststate
类型：int8 / uint8_t
语义：
  0 = 敌方前哨站未确认摧毁
  1 = 敌方前哨站已由自瞄视觉确认摧毁
默认值：0
是否锁存：建议导航侧锁存到本局结束
数据来源：自瞄识别不亮前哨站装甲板 + 电控提供 `_base_yaw` + 导航侧判断敌我半场
```

各侧最小数据接口：

```text
自瞄 -> 电控：
  uint8_t _enemy_id
  float _enemy_x
  float _enemy_y
  uint8_t unlit_outpost_candidate

电控 -> 导航：
  float _base_yaw
  uint8_t _enemy_id
  float _enemy_x
  float _enemy_y
  uint8_t unlit_outpost_candidate

导航 -> 决策：
  int8 outpoststate
```

其中 `outpoststate` 只能由导航侧在完成敌我归属判断后置 `1`。
