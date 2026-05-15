# 敌方工程机器人自瞄屏蔽标志位接入方案

## 背景

自瞄侧在视觉锁敌时会输出当前锁定目标的 ID 和相对云台坐标。决策侧需要利用这些信息判断“敌方工程机器人是否处于指定屏蔽区域内”。如果满足条件，决策侧通过 `/sentry/control` 下发 `avoidengineer_flag`，由串口节点转发给电控/自瞄系统，使自瞄系统屏蔽该工程目标，避免攻击正在取矿或处于规则要求保护区域内的工程机器人。

当前 `sentry_decision/msg/SentryControl.msg` 中已有字段：

```text
uint8 avoidengineer_flag    # 0:正常，不用管，1：代表敌方工程在取矿，不可以打
```

因此本方案不建议新增消息字段，短期直接复用该字段作为“敌方工程机器人屏蔽标志位”。

## 结论

该方案合理，但责任边界必须明确：

- 自瞄侧：持续输出当前锁定目标的内部 ID 和相对云台坐标
- 电控/下板：补齐并透传云台相对底盘 yaw
- 导航/myserial：解析并发布视觉锁敌信息到 `decision_messages/msg/EnemyRobotState.msg`
- 决策侧：将目标转换到场地坐标，判断目标是否为敌方工程且是否落入指定屏蔽区域，并发布 `avoidengineer_flag`
- 自瞄/电控侧：只消费最终 `avoidengineer_flag`，不再重复做场地范围判断

最终语义：

| 值 | 含义 |
| --- | --- |
| `0` | 正常，不需要屏蔽工程目标 |
| `1` | 敌方工程机器人处于指定屏蔽区域内，自瞄应屏蔽该目标 |

## 数据流

目标链路：

```text
自瞄侧锁定目标
  -> 输出 _enemy_id / _enemy_x / _enemy_y
  -> 上板到下板通信协议
  -> 下板补充 _base_yaw 并打包到导航通信帧
  -> 导航侧 myserial 解析目标信息和 _base_yaw
  -> /decision_messages/EnemyRobotState
  -> 决策侧将目标转换到 map 坐标
  -> 判断 enemy_id 是否为工程机器人以及是否位于指定屏蔽区域
  -> /sentry/control 的 avoidengineer_flag
  -> myserial 转发给下板/自瞄系统
  -> 自瞄系统根据标志位屏蔽工程目标
```

## 统一字段语义

### 视觉锁敌信息

`base_yaw`、`enemy_id`、`enemy_x`、`enemy_y` 已在 `decision_messages/msg/EnemyRobotState.msg` 中定义：

```text
float32 base_yaw
uint8 enemy_id
float32 enemy_x
float32 enemy_y
```

字段约定：

```cpp
float _base_yaw;    // 云台相对底盘 yaw，导航发布到 EnemyRobotState 时建议使用 rad
uint8_t _enemy_id;  // 当前锁定目标内部 ID；未锁定目标时为 0
float _enemy_x;     // 目标在云台坐标系下的相对 x 距离，单位：m
float _enemy_y;     // 目标在云台坐标系下的相对 y 距离，单位：m
```

坐标和单位约定：

- `_enemy_x`、`_enemy_y` 是目标相对于当前云台坐标系的距离，不是场地坐标，也不是底盘坐标
- 单位统一使用 `m`
- 云台坐标系正方向必须由自瞄、电控、导航三方确认；建议约定 `x` 为云台前方，`y` 为云台左方
- 未锁定目标时，`_enemy_id = 0`，`_enemy_x = 0.0`，`_enemy_y = 0.0`
- `_base_yaw` 应持续发送，不因未锁定目标而置零
- 当前 myserial 已将 `_base_yaw` 转为 `rad` 后写入 `EnemyRobotState.base_yaw`，决策侧使用时必须按 `rad` 处理

### 工程屏蔽标志位

`avoidengineer_flag` 是决策发给自瞄/电控系统的最终结果，不是自瞄原始识别结果。

```cpp
uint8_t avoidengineer_flag;  // 0: 不屏蔽, 1: 屏蔽敌方工程目标
```

决策侧置 `1` 的必要条件：

```text
enemy_id == 2
目标坐标有效
目标被转换到 map 坐标后位于指定工程屏蔽区域内
```

如果未锁目标、锁到非工程机器人、坐标无效、TF/自身位姿无效，或目标不在指定区域内，则必须发布 `avoidengineer_flag = 0`。

## ID 映射

协议发送的是代码转换后的内部 ID，不是模型原始输出 ID。决策侧判断工程机器人时必须使用内部 ID `2`。

| 协议发送 ID | 含义 | 代码来源 |
| --- | --- | --- |
| `0` | 无目标 | 无目标时发送 |
| `1` | 1 号机器人装甲板 | 原始识别 `1` |
| `2` | 2 号机器人装甲板，工程机器人 | 原始识别 `2` |
| `3` | 3 号机器人装甲板 | 原始识别 `3` |
| `4` | 4 号机器人装甲板 | 原始识别 `4` |
| `5` | 5 号机器人装甲板 | 原始识别 `5` |
| `7` | 哨兵装甲板 | 原始识别 `0` 转换为 `7` |
| `8` | 基地装甲板 | 原始识别 `7` 或 `8` 转换为 `8` |
| `9` | 前哨站装甲板 | 原始识别 `6` 转换为 `9` |

## 指定屏蔽区域约定

屏蔽区域应由决策侧配置，不建议写死在自瞄或电控侧。推荐使用 map 坐标系下的矩形或多边形区域，例如敌方取矿区、敌方资源岛取矿点、工程保护区等。

建议配置项：

```yaml
engineer_shield_enabled: true
engineer_shield_confirm_frames: 3
engineer_shield_lost_frames: 3
red_engineer_shield_regions:
  - name: "red_mining_area"
    min_x: 0.0
    max_x: 0.0
    min_y: 0.0
    max_y: 0.0
blue_engineer_shield_regions:
  - name: "blue_mining_area"
    min_x: 0.0
    max_x: 0.0
    min_y: 0.0
    max_y: 0.0
```

坐标判断建议：

- 先用 `base_yaw`、`enemy_x`、`enemy_y` 将目标从云台坐标系转换到底盘坐标系
- 再结合机器人自身 map 位姿转换到 map 坐标系
- 根据我方颜色选择“敌方工程屏蔽区域”配置
- 目标落入任一有效区域时，计为命中屏蔽区域
- 建议连续多帧命中后才置 `avoidengineer_flag = 1`，连续多帧丢失后再恢复 `0`

## 各负责人任务

### 1. 自瞄负责人

目标：稳定输出当前锁定目标内部 ID 和目标相对云台坐标，并在收到 `avoidengineer_flag = 1` 时屏蔽工程机器人目标。

需要完成：

- 按统一 ID 映射发送 `_enemy_id`
- 识别到工程机器人时发送内部 ID `2`
- 持续发送当前锁定目标的 `_enemy_x`、`_enemy_y`，单位为 `m`
- 未锁定目标时发送 `_enemy_id=0, _enemy_x=0.0, _enemy_y=0.0`
- 接收到 `avoidengineer_flag = 1` 时，屏蔽工程机器人装甲板，不将其作为可攻击目标
- 接收到 `avoidengineer_flag = 0` 时，恢复正常目标选择

验收标准：

- 锁定工程机器人时，导航侧能看到 `enemy_id: 2`
- 锁定非工程机器人时，不会误发 `enemy_id: 2`
- `enemy_x/enemy_y` 单位为 `m`，方向与约定一致
- `avoidengineer_flag = 1` 时自瞄不会继续选择工程机器人作为攻击目标
- `avoidengineer_flag = 0` 时自瞄恢复正常逻辑

### 2. 下板/电控负责人

目标：透传自瞄目标信息，补齐云台相对底盘 yaw，并接收决策侧下发的工程屏蔽标志位。

需要完成：

- 上板到下板通信协议包含：
  - `uint8_t _enemy_id`
  - `float _enemy_x`
  - `float _enemy_y`
- 下板发给导航侧的通信帧包含：
  - `float _base_yaw`
  - `uint8_t _enemy_id`
  - `float _enemy_x`
  - `float _enemy_y`
- 下板接收导航侧发回的 `avoidengineer_flag`
- 将 `avoidengineer_flag` 转发给自瞄系统，或在电控侧完成对应屏蔽控制
- 保证 `_base_yaw` 与目标坐标尽量属于同一时刻，避免 yaw 和目标错帧导致区域判断错误

验收标准：

- 自瞄侧发送 `_enemy_id/_enemy_x/_enemy_y` 后，下板能原样转发给导航侧
- 下板持续提供 `_base_yaw`，并明确单位、零位和正方向
- 下板能收到并打印 `avoidengineer_flag`
- `avoidengineer_flag = 1` 时，自瞄/电控侧确实进入工程目标屏蔽状态
- 通信异常或比赛重置时，默认恢复 `avoidengineer_flag = 0`

### 3. 导航/myserial 负责人

目标：解析锁敌信息并发布给决策侧，同时把决策侧发布的 `avoidengineer_flag` 转发给下板。

当前相关代码位置：

- `src/myserial/include/myserial/myprotocol.hpp`
- `src/myserial/src/serial_node.cpp`
- `src/RMUC/decision_messages/msg/EnemyRobotState.msg`
- `src/RMUC/robomaster_sentry_decision/msg/SentryControl.msg`

需要完成：

- 确认接收帧结构中包含 `_base_yaw`、`_enemy_id`、`_enemy_x`、`_enemy_y`
- 确认 `/decision_messages/EnemyRobotState` 正确发布：
  - `base_yaw`
  - `enemy_id`
  - `enemy_x`
  - `enemy_y`
- 确认 `/sentry/control` 回调中读取 `avoidengineer_flag`
- 将 `avoidengineer_flag` 写入发给下板的通信帧或状态 bit
- 增加节流日志，打印收到的 `avoidengineer_flag`，方便联调

验收标准：

- 串口帧中 `_enemy_id=2` 时，`ros2 topic echo /decision_messages/EnemyRobotState` 能看到 `enemy_id: 2`
- `base_yaw/enemy_x/enemy_y` 数值与下板发送一致，单位符合约定
- 手动发布 `/sentry/control` 且 `avoidengineer_flag=1` 时，下板能收到对应标志
- 手动发布 `/sentry/control` 且 `avoidengineer_flag=0` 时，下板能恢复正常状态

### 4. 决策负责人

目标：根据视觉锁敌信息判断工程机器人是否在指定屏蔽区域内，并发布 `avoidengineer_flag`。

当前相关代码位置：

- `src/RMUC/robomaster_sentry_decision/src/main.cpp`
- `src/RMUC/robomaster_sentry_decision/src/DecisionManager.cpp`
- `src/RMUC/robomaster_sentry_decision/include/sentry_decision/Blackboard.hpp`
- `src/RMUC/robomaster_sentry_decision/config/sentry_decision_params.yaml`

需要完成：

- 在参数文件中增加敌方工程屏蔽区域配置
- 在 `EnemyRobotState` 回调或黑板更新逻辑中记录当前视觉锁敌目标的 map 坐标
- 只在 `enemy_id == 2` 时执行工程屏蔽区域判断
- 目标落入指定区域并连续确认后，设置 `SentryControl.avoidengineer_flag = 1`
- 目标离开区域、丢失、ID 不是 `2` 或坐标无效时，设置 `SentryControl.avoidengineer_flag = 0`
- 在发布所有 `SentryControl` 的路径上都填充该字段，避免部分行为分支漏发

建议伪代码：

```cpp
bool engineer_in_shield_region = false;

if (enemy_id == 2 && target_pose_valid) {
    auto [x_map, y_map] = gimbalToMap(robot_x, robot_y, robot_yaw, base_yaw, enemy_x, enemy_y);
    engineer_in_shield_region = pointInEnemyEngineerShieldRegion(x_map, y_map);
}

ctrl.avoidengineer_flag = engineer_in_shield_region ? 1 : 0;
```

验收标准：

- 锁定非工程机器人时，`avoidengineer_flag` 始终为 `0`
- 锁定工程机器人但不在指定区域内时，`avoidengineer_flag` 为 `0`
- 锁定工程机器人且位于指定区域内并连续确认后，`avoidengineer_flag` 为 `1`
- 工程机器人离开区域或视觉丢失后，`avoidengineer_flag` 恢复为 `0`
- 重新 `colcon build --packages-select robomaster_sentry_decision sentry_decision decision_messages myserial` 能通过

## 风险点

- `enemy_id` 使用了模型原始 ID 而不是协议内部 ID，导致工程机器人判断错位
- `_base_yaw` 单位或正方向不一致，导致目标 map 坐标偏移
- `_enemy_x/_enemy_y` 单位不是 `m`，导致区域判断范围错误
- 决策只在部分控制分支填充 `avoidengineer_flag`，导致状态间歇性丢失
- `avoidengineer_flag` 被锁存后没有清零条件，导致工程离开保护区域后仍被屏蔽
- 屏蔽区域坐标没有区分红蓝方，导致己方/敌方区域选错

## 联调建议

1. 自瞄侧先打印原始识别 ID、转换后的内部 ID、`enemy_x/enemy_y`、当前是否收到 `avoidengineer_flag`。
2. 下板打印 `_base_yaw`、`_enemy_id`、`_enemy_x`、`_enemy_y`，以及收到的 `avoidengineer_flag`。
3. 导航/myserial 使用 `ros2 topic echo /decision_messages/EnemyRobotState` 检查锁敌信息。
4. 决策侧打印工程目标 map 坐标、命中的屏蔽区域名称、连续帧计数、最终 `avoidengineer_flag`。
5. myserial 打印发给下板的最终通信帧，确认 `avoidengineer_flag` 没有在转发阶段丢失。

## 对外沟通摘要

字段名：`avoidengineer_flag`

消息位置：`src/RMUC/robomaster_sentry_decision/msg/SentryControl.msg`

发布话题：`/sentry/control`

字段语义：

| 值 | 含义 |
| --- | --- |
| `0` | 不屏蔽工程目标 |
| `1` | 屏蔽敌方工程目标，自瞄不应攻击 |

数据来源：自瞄锁敌 ID 与相对坐标 + 电控提供 `_base_yaw` + 决策侧指定区域判断

关键判断：只有内部协议 ID 为 `2` 的工程机器人，且转换到 map 坐标后位于指定屏蔽区域内，才能置 `avoidengineer_flag = 1`。
