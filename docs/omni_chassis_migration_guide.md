# 全向轮底盘迁移到 navigation2026 框架说明

本文记录将老全向轮车接入当前并联腿导航框架的迁移选择、实现边界和阶段落地状态。当前已完成阶段一的全向车独立启动入口与参数 profile 分离，并完成阶段二的全向速度适配闭环第一版。

## 1. 迁移目标

目标是复用 `navigation2026` 中已有的自研导航框架：

- `nav_server` 状态机、目标接收、规划控制流程。
- `LayeredMapManager` 静态层、动态层、膨胀、ESDF。
- A* 全局规划、B 样条平滑和 ROG-Map 动态障碍层。
- 当前恢复逻辑先保持现状。

全向轮车侧的明确选择：

- 控制器继续使用当前 NMPC，不重新导出 acados 模型。
- 使用 `base_link_fake` 作为导航控制坐标系。
- `nav_server` 得到的速度先认为是在 `base_link_fake` 下，再转换到 `base_link`。
- 电控负责将 `base_link` 下速度继续转换到底盘实际执行坐标。
- 特殊地形、腿长、台阶、飞坡逻辑默认关闭。
- 动态障碍统一选择 ROG-Map。
- 雷达、IMU、云台相关外参单独按老车实物修改。

## 2. NMPC 不重建模型时的能力边界

当前 NMPC 模型不是全向模型。源码中的模型状态为：

```text
x = [x, y, theta, v, omega, v_cmd, omega_cmd]
u = [a_cmd, alpha_cmd]
```

连续模型核心是：

```text
dx     = v * cos(theta)
dy     = v * sin(theta)
dtheta = omega
```

控制输出在源码中实际只填：

```text
cmd_vel.linear.x  = v_cmd
cmd_vel.angular.z = omega_cmd
```

因此，在不重建模型的前提下，NMPC 内部不会优化独立的 `vy`。它优化的是一个标量线速度 `v` 和角速度 `omega`。

所以可实现的是：

```text
NMPC 在 base_link_fake 中输出 (vx_fake = v, vy_fake = 0, wz)
        ↓
速度适配层将该速度旋转到 base_link
        ↓
得到 (vx_base, vy_base, wz)
        ↓
发送给电控
```

这是一种“虚拟坐标系带来的全向执行能力”，不是“真正的全向 NMPC”。它能让最终下发给电控的速度包含 `vx/vy`，但 NMPC 的预测轨迹仍假设机器人沿 `base_link_fake` 的 x 轴运动。

关键理解：

- `vy` 不是当前 NMPC 求解出来的。
- `vy` 是把 `base_link_fake` 下的前进速度旋转到 `base_link` 后得到的投影。
- 只要 `base_link_fake` 的 yaw 维护正确，且电控能正确执行 `base_link` 下的 `vx/vy/wz`，底盘就能运动起来并跟随路径。
- 这套方案的路径跟随能力来自“虚拟车头对准路径 + 全向底盘执行投影速度”，不是来自 NMPC 内部的横向速度优化。

## 3. 推荐控制适配方案

推荐采用最小且自洽的方案：保留当前 NMPC，只在输出层做坐标变换。

### 3.1 话题链路

建议不要让 `nav_server` 直接把速度发给 `myserial`，否则会绕过全向适配层。

推荐链路：

```text
nav_server
  publishes: /cmd_vel_fake        # Twist，语义为 base_link_fake 坐标

omni_cmd_adapter / fake_vel_transform
  subscribes: /cmd_vel_fake
  uses TF: base_link <- base_link_fake
  publishes: /cmd_vel             # Twist，语义为 base_link 坐标

myserial
  subscribes: /cmd_vel
  sends: speed_x, speed_y, speed_w
```

如果沿用旧全向轮代码的命名，也可以让适配层发布 `/cmd_vel_gimbal`，但要同步修改 `myserial` 订阅的话题或 launch remapping。关键原则是：`myserial` 只能吃适配后的速度。

### 3.2 速度旋转公式

设 `delta` 为 `base_link` 到 `base_link_fake` 的 yaw，也就是查询：

```text
lookupTransform("base_link", "base_link_fake")
```

若 NMPC 输出：

```text
vx_fake = cmd.linear.x
vy_fake = 0.0
wz      = cmd.angular.z
```

则转换到 `base_link`：

```text
vx_base = vx_fake * cos(delta) - vy_fake * sin(delta)
vy_base = vx_fake * sin(delta) + vy_fake * cos(delta)
wz_base = wz
```

因为 `vy_fake` 初始为 0，所以实际是：

```text
vx_base = v * cos(delta)
vy_base = v * sin(delta)
```

这样即使 NMPC 只输出一个前进速度，最终给电控的速度也可以同时包含 `speed_x` 和 `speed_y`。

直观例子：

```text
delta = 0 deg:
  v = 1.0 m/s -> vx_base = 1.0,   vy_base = 0.0

delta = 90 deg:
  v = 1.0 m/s -> vx_base = 0.0,   vy_base = 1.0

delta = 45 deg:
  v = 1.0 m/s -> vx_base = 0.707, vy_base = 0.707
```

因此，全向底盘最终收到 `vx/vy` 并不是因为 NMPC 原生支持全向，而是因为虚拟车头和 `base_link` 存在 yaw 差。

### 3.3 为什么能跟随路径

迁移后的控制闭环可以理解为：

```text
nav_server 当前位姿: map -> base_link_fake
NMPC 控制对象:      base_link_fake
NMPC 输出:          沿 base_link_fake 的 x 轴前进，并调节 base_link_fake 的 yaw
适配层输出:         将该前进速度旋转到 base_link，得到 vx_base/vy_base
电控执行:           根据 base_link 下速度驱动全向底盘
```

当 `base_link_fake` 的 x 轴被维护到接近路径切线方向时，NMPC 输出的 `linear.x` 就代表“沿路径方向走”。适配层把这个速度转成 `base_link` 下的 `vx/vy`，全向底盘执行后，机器人在地图中的实际运动方向仍然是 `base_link_fake` 的 x 轴方向，因此可以跟随路径。

这要求三件事同时成立：

- `nav_server.base_frame = base_link_fake`，让规划控制使用虚拟车头。
- `base_link_fake` 和 `base_link` 的 TF yaw 差正确。
- 电控收到的 `vx_base/vy_base/wz` 的坐标语义确实是 `base_link`。

### 3.4 是否要在 NMPC 后额外生成 vy_fake

不建议一开始就在 NMPC 输出后人为加很大的 `vy_fake`。

原因是当前 NMPC 的预测模型没有 `vy`。如果后处理生成较大的侧向速度，真实机器人会偏离 NMPC 预测轨迹，ESDF 代价、路径横向误差、控制进展判断都可能失真。

如果后期确实需要增强横向修正，可以加一个很小的横向补偿项：

```text
vy_fake = clamp(k_lat * e_lat, -vy_fake_max, vy_fake_max)
```

其中 `e_lat` 是目标点或局部路径在 `base_link_fake` 下的横向误差。建议初始限制：

```text
vy_fake_max <= 0.2 ~ 0.4 m/s
```

并且必须配套：

- 速度矢量总幅值限幅：`sqrt(vx^2 + vy^2) <= max_speed`
- 加速度限幅：限制 `vx/vy` 每周期变化
- 近障碍时关闭或降低横向补偿

该补偿只能作为工程修正，不应当被理解为全向 NMPC。

## 4. base_link_fake 的使用方式

`nav_server` 应以 `base_link_fake` 作为机器人控制基座：

```yaml
nav_server:
  ros__parameters:
    base_frame: "base_link_fake"
```

这样 `nav_server` 中查询当前位姿时，会使用：

```text
map -> base_link_fake
```

NMPC 看到的 `theta` 就是虚拟控制坐标系的 yaw。

建议 TF 结构保持为：

```text
map
  -> odom
    -> base_link              # 按当前迁移约定：云台/雷达侧速度参考系
      -> livox_frame
      -> base_link_fake       # 虚拟导航控制基座
```

`base_link_fake` 不是一个独立机器人，也不应有独立平移。推荐它与 `base_link` 同原点，只差一个 yaw：

```text
translation(base_link -> base_link_fake) = [0, 0, 0]
rotation(base_link -> base_link_fake)    = yaw(delta)
```

如果保留旧车 `base_link_static -> base_link_fake` 的做法，也必须保证整棵 TF 树连通，并且 `map -> base_link_fake` 能稳定查到。无论父节点叫 `base_link` 还是 `base_link_static`，工程语义都应该保持：`base_link_fake` 是挂在真实速度参考系上的虚拟车头。

`base_link_fake` 的 yaw 可以沿用旧 `fake_vel_transform` 的思想：根据 NMPC 输出的 `angular.z` 积分得到虚拟 heading。需要确认：

- 初始角度，例如旧代码中的 45 度，是否仍符合新 URDF/外参。
- `angular.z` 积分方向是否和 TF 正方向一致。
- 积分系数是否仍需要类似 `fake_angular_speed_coefficient`。
- 电控是否也消费同一个 `angular.z`，如果消费，虚拟 yaw 和实际底盘 yaw 的关系是否会长期漂移。

最小实现逻辑：

```text
delta += omega_cmd * dt
delta = normalize(delta)
publish TF: base_link -> base_link_fake, yaw = delta
```

其中 `omega_cmd` 来自 NMPC 输出的 `cmd_vel_fake.angular.z`。如果实车发现虚拟车头转得比路径跟踪期望慢或快，可以再评估是否引入比例系数；但第一版建议先保持物理一致，即系数为 1.0。

## 5. 里程计反馈要求

当前 NMPC 的速度反馈只使用：

```text
latest_odom.twist.twist.linear.x
latest_odom.twist.twist.angular.z
```

它不会使用 `linear.y`。

因此，若 `base_frame = base_link_fake`，建议给 NMPC 的 `nmpc.odom_topic` 也使用与 `base_link_fake` 语义一致的里程计，例如：

```yaml
nmpc:
  odom_topic: "/Odometry/fake"
```

`/Odometry/fake` 的推荐语义：

```text
header.frame_id = "odom"
child_frame_id  = "base_link_fake"
twist.linear.x  = 机器人速度在 base_link_fake x 轴上的投影
twist.angular.z = base_link_fake yaw rate
```

在适配层没有完全验证前，建议临时将：

```yaml
nmpc:
  odom_feedback_alpha: 0.0
```

这样 NMPC 主要使用上一周期命令作为速度估计，避免错误的里程计坐标把控制器拖偏。等 `/Odometry/fake` 验证正确后，再逐步提高 `odom_feedback_alpha`。

## 6. 动态障碍与 ROG-Map

老全向轮迁移后不再走 Nav2 STVL。推荐统一为：

```text
/cloud_registered
  -> ROG-Map integration
  -> /rog_map/map_2d
  -> nav_server dynamic layer
```

参数保持：

```yaml
enable_dynamic_layer: true
dynamic_layer_topic: "/rog_map/map_2d"
```

需要注意，老车雷达在云台上，因此 ROG-Map 输入点云和投影节点必须使用正确 TF。不能沿用腿车“雷达固定在底盘”的外参。

## 7. 特殊地形必须关闭

全向轮车默认关闭：

```yaml
special_terrain:
  enable_stair_layer: false
  enable_stair_fsm: false
  enable_stair_mode_detection: false
  enable_fly_slope_mode_detection: false
```

同时建议关闭或忽略：

- `LegLength`
- `stair_mode`
- 台阶固定速度策略
- 飞坡状态机
- 二级台阶上下行逻辑

否则可能出现无关的速度覆写、重规划、恢复触发或串口 `stair_mode` 输出。

## 8. 恢复逻辑阶段性保留

当前恢复逻辑先保持现状，包括直退、旋转、弧线退等。它们会输出：

```text
linear.x
angular.z
```

不会主动输出 `linear.y`。这意味着恢复阶段暂时不会利用全向横移能力。

这可以先接受，待主导航链路稳定后再考虑新增：

```text
OmniEscapeRecovery
```

根据 ESDF 梯度直接输出 `linear.x/linear.y` 进行横移脱困。

## 9. 推荐参数骨架

建议为老全向轮车单独创建配置文件，例如：

```text
src/my_navigation/nav_bringup/config/nav_params_omni.yaml
```

核心差异应包括：

```yaml
nav_server:
  ros__parameters:
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link_fake"

    enable_dynamic_layer: true
    dynamic_layer_topic: "/rog_map/map_2d"

    special_terrain:
      enable_stair_layer: false
      enable_stair_fsm: false
      enable_stair_mode_detection: false
      enable_fly_slope_mode_detection: false

    nmpc:
      odom_topic: "/Odometry/fake"
      chassis_odom_topic: "/Odometry/fake"
      odom_feedback_alpha: 0.0
      allow_reverse: true
      max_linear_vel: 2.0
      max_angular_vel: 3.0
```

`allow_reverse` 是否开启取决于虚拟坐标策略。如果 `base_link_fake` 始终会转到路径方向，可以保持 `false`；如果希望虚拟坐标允许反向走，才开启 `true`。

当前已补齐的全向车专用文件：

```text
src/my_navigation/nav_bringup/launch/run_omni.launch.py
src/my_navigation/nav_bringup/launch/navigation_omni.launch.py
src/my_navigation/nav_bringup/scripts/omni_cmd_adapter.py
src/my_navigation/nav_bringup/scripts/omni_stage3_check.py
src/my_navigation/nav_bringup/config/nav_params_omni.yaml

src/localization/small_point_lio/launch/small_point_lio_omni.launch.py
src/localization/small_point_lio/config/mid360_omni.yaml

src/localization/localization_initializer/config/initializer_omni.yaml

src/mapping/rog_map_ros2_node/config/rog_map_omni.yaml
src/mapping/rog_map_ros2_node/config/projector_omni.yaml
src/mapping/rog_map_ros2_node/config/stair_detector_omni.yaml
```

全向车顶层启动入口默认使用这些 `_omni` 文件，并默认启动 `omni_cmd_adapter.py`。LIO 入口使用 `mid360_omni.yaml`，并在 `small_point_lio_omni.launch.py` 内固定写入 `base_link -> livox_frame` 外参；同一份 xyz 会由 launch 传给 LIO 的 `base_link_to_lidar_xyz` 参数，避免 TF 外参和 LIO 点云过滤外参不一致。

## 10. 改造执行步骤

阶段不宜太碎。下面按耦合关系合并成 5 个工程阶段：先把双车 profile 分开，再一次性打通全向模式的控制坐标和速度适配，然后迁移定位/ROG-Map，最后做实车闭环和性能恢复。

### 阶段 1：双车 profile 与配置隔离

目标：保护现有腿车链路，同时给全向轮建立独立启动入口。

状态：已完成第一版文件分离。当前只新增/更新全向车入口和 `_omni` 参数文件，不改腿车默认入口。

动作：

- 保留当前腿车默认入口，例如 `run.launch.py`，或另建 `run_wheelleg.launch.py`。
- 新增全向轮入口，例如 `run_omni.launch.py`，不建议一开始就只靠运行时参数热切。
- 不直接覆盖腿车现有配置，新增全向轮专用配置：

```text
nav_params_omni.yaml
mid360_omni.yaml
initializer_omni.yaml
rog_map_omni.yaml
projector_omni.yaml
stair_detector_omni.yaml
```

- 全向轮 launch 中传入 `nav_params_omni.yaml`，并默认启动 `omni_cmd_adapter`。

验收：

- 腿车原启动命令仍能启动原链路。
- 全向轮有独立启动入口。
- 两车配置文件互不覆盖，避免改全向轮时破坏腿车。

### 阶段 2：全向控制适配闭环

目标：一次性打通 `base_link_fake`、`/cmd_vel_fake -> /cmd_vel`、开环 NMPC 三件强耦合的事情，让底盘能按虚拟车头方向运动。

状态：已完成第一版。当前实现不重建 NMPC 模型，而是在 `nav_bringup/scripts/omni_cmd_adapter.py` 中完成速度坐标转换、虚拟车头 TF 发布和 `/Odometry/fake` 生成。

动作：

- 在 `nav_params_omni.yaml` 中设置：

```yaml
base_frame: "base_link_fake"

nmpc:
  odom_topic: "/Odometry/fake"
  odom_feedback_alpha: 0.0
```

- 关闭特殊地形相关逻辑：

```yaml
special_terrain:
  enable_stair_layer: false
  enable_stair_fsm: false
  enable_stair_mode_detection: false
  enable_fly_slope_mode_detection: false
```

- 将 `nav_server` 的 `cmd_vel` remap 到 `/cmd_vel_fake`。
- 新增或迁移 `omni_cmd_adapter`：
  - 订阅 `/cmd_vel_fake`。
  - 维护并发布 `base_link -> base_link_fake` TF。
  - 使用自身维护并发布的 yaw 差 `delta`。
  - 发布 `/cmd_vel` 给 `myserial`。

速度转换：

```text
vx_base = vx_fake * cos(delta) - vy_fake * sin(delta)
vy_base = vx_fake * sin(delta) + vy_fake * cos(delta)
wz_base = wz_fake
```

第一版保持：

```text
vx_fake = NMPC linear.x
vy_fake = 0
wz_fake = NMPC angular.z
```

- 同时生成 `/Odometry/fake`。第一版中 `odom_feedback_alpha` 先保持 `0.0`，因此 `/Odometry/fake` 主要用于观测和后续闭环验证；确认坐标语义正确后再逐步提高反馈权重。

验收：

- `tf2_echo map base_link_fake` 稳定输出。
- `ros2 topic list` 中能看到 `/cmd_vel_fake`、`/cmd_vel`、`/Odometry/fake`。
- 手动发布 `/cmd_vel_fake.linear.x = 1.0`：
  - `delta = 0 deg` 时，`/cmd_vel` 主要为 `linear.x = 1.0`。
  - `delta = 90 deg` 时，`/cmd_vel` 主要为 `linear.y = 1.0`。
- `angular.z` 正方向和 `base_link_fake` yaw 增大方向一致。

### 阶段 3：定位外参与 ROG-Map 动态层

目标：把老车雷达/IMU/云台外参和动态障碍链路接到新框架里。

状态：已完成第一版工程接线。`small_point_lio_omni.launch.py` 固定写入全向车 `base_link -> livox_frame` 外参；`run_omni.launch.py` 现在集中暴露以下关键入口：

```text
cloud_registered_topic
source_odom_topic
dynamic_layer_topic
rog_map_frame
enable_stage3_check
```

其中：

- `small_point_lio_omni.launch.py` 发布 `base_link -> livox_frame`，并用同一份固定 xyz 给 LIO 设置 `base_link_to_lidar_xyz`。
- `localization_initializer` 默认使用 `/cloud_registered`，通过 launch remapping 接到 `cloud_registered_topic`。
- ROG-Map 默认使用 `/cloud_registered` 和 `/Odometry`，通过 launch remapping 接到 `cloud_registered_topic` 和 `source_odom_topic`。
- `projector.topic_name` 由 `dynamic_layer_topic` 覆盖，`nav_server.dynamic_layer_topic` 同步使用同一个 topic。
- `projector.frame_id` 由 `rog_map_frame` 覆盖，默认仍为 `odom`。
- `omni_stage3_check.py` 可选启动，用于检查 TF 和关键 topic 是否连通。

动作：

- 修改老车专用 LIO 配置，例如 `mid360_omni.yaml`。
- 修改雷达、IMU、云台相关静态/动态 TF。
- 确认 `/cloud_registered` 的 frame 与 TF 树一致。
- 确认 `map -> odom -> base_link -> base_link_fake` 连通。
- 使用 `rog_map_omni.yaml` 和 `projector_omni.yaml`，输出 `/rog_map/map_2d`。
- `nav_server` 保持：

```yaml
enable_dynamic_layer: true
dynamic_layer_topic: "/rog_map/map_2d"
```

验收：

- 静止时 `/Odometry` 不明显跳变。
- RViz 中点云、机器人模型、地图方向一致。
- `/rog_map/map_2d.header.frame_id` 与导航侧预期一致。
- 机器人前方真实障碍在融合地图中出现在正确方向。

可选检查命令：

```bash
ros2 launch nav_bringup run_omni.launch.py enable_stage3_check:=true
```

或者导航已经启动后单独运行：

```bash
ros2 run nav_bringup omni_stage3_check.py
```

检查脚本会确认：

- `map <- odom`
- `odom <- base_link`
- `base_link <- base_link_fake`
- `base_link <- livox_frame`
- `map <- base_link_fake`
- `/cloud_registered`
- `/Odometry`
- `/Odometry/fake`
- `/rog_map/map_2d`

### 阶段 4：低速实车路径跟随验证

目标：用保守速度确认底盘能动起来并沿路径走。

动作：

- 限低速度和角速度，例如：

```yaml
nmpc:
  max_linear_vel: 0.5
  max_angular_vel: 1.0
  odom_feedback_alpha: 0.0
```

- 在空旷区域给 1 到 2 米短目标。
- 观察 `/cmd_vel_fake`、`/cmd_vel`、`map -> base_link_fake`、实际底盘运动方向。
- 保持第一版不开大幅 `vy_fake` 横向补偿。

验收：

- `base_link_fake` 指向路径方向时，底盘实际运动方向与路径一致。
- `delta` 不同角度下，底盘仍能朝 `base_link_fake` x 轴方向移动。
- 机器人不会因为 TF 方向错误出现横着反走、绕圈或原地振荡。
- 开启 ROG-Map 动态层后，新增障碍能影响规划或代价地图。

### 阶段 5：反馈、速度和恢复逻辑收敛

目标：主链路稳定后，再逐步提高闭环程度、速度和恢复可靠性。

动作：

- 逐步提高 `odom_feedback_alpha`：

```text
0.0 -> 0.2 -> 0.5
```

- 根据实车表现调整 `vel_lag_tau`、`omega_lag_tau`。
- 再逐步提高 `max_linear_vel`、`max_angular_vel`。
- 保持现有直退、旋转、弧线退恢复逻辑，确认它们经过适配层后仍能被电控正确执行。
- 暂不引入全向横移恢复。主路径跟随稳定后，再考虑新增 `OmniEscapeRecovery`。

验收：

- 提高速度后路径跟随仍稳定。
- 里程计反馈不会让 NMPC 出现拖拽、反向回正或频繁停车。
- 恢复时 `/cmd_vel_fake` 到 `/cmd_vel` 的转换方向正确。
- 恢复失败不会卡死在错误 TF 或错误模式中。

## 11. 验证清单

TF 验证：

```bash
ros2 run tf2_ros tf2_echo map base_link_fake
ros2 run tf2_ros tf2_echo base_link base_link_fake
```

话题验证：

```bash
ros2 topic echo /cmd_vel_fake
ros2 topic echo /cmd_vel
ros2 topic echo /Odometry/fake
ros2 topic echo /rog_map/map_2d
```

方向验证：

- `base_link_fake` 与 `base_link` yaw 差为 0 时，`/cmd_vel_fake.linear.x > 0` 应主要变成 `/cmd_vel.linear.x > 0`。
- yaw 差为 90 度时，`/cmd_vel_fake.linear.x > 0` 应主要变成 `/cmd_vel.linear.y`。
- `angular.z` 正方向必须和 RViz 中 `base_link_fake` yaw 增大方向一致。
- 电控执行后，机器人在地图中的运动方向应与 `/cmd_vel_fake` 在 `base_link_fake` 下的 x 方向一致。

## 12. 主要风险

- 当前方案不是全向 NMPC，不能期望 NMPC 内部优化侧向避障。
- `base_link_fake` 若靠命令积分，长期可能漂移，需要实车验证重置或闭环策略。
- 如果 `/Odometry/fake` 的 twist 坐标不对，NMPC 会被错误速度反馈拖偏。
- 若 `nav_server` 速度没有 remap 到 `/cmd_vel_fake`，`myserial` 可能直接收到未适配速度。
- 大幅后处理 `vy_fake` 会造成模型预测和真实运动不一致，容易引入贴障或控制振荡。

## 13. 后续可升级方向

主链路稳定后，再考虑：

- 将 `nav_server` 控制器抽象为可配置控制器，而不是硬编码 `NMPC`。
- 新增真正的 `OmniNMPC` 模型，状态包含 `vx/vy/omega`，控制包含 `ax/ay/alpha`。
- 新增全向恢复行为，利用 `linear.y` 横移脱困。
- 将 `base_link_fake` 的 yaw 从纯命令积分升级为带反馈的虚拟航向估计。
