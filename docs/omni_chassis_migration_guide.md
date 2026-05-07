# 全向轮底盘迁移到 navigation2026 框架说明

本文记录将老全向轮车接入当前并联腿导航框架的迁移选择与实现边界。当前阶段只做设计说明，不修改源码。

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

### 3.3 是否要在 NMPC 后额外生成 vy_fake

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
    -> base_link              # 你的约定中为云台/雷达相关 base
      -> livox_frame
      -> base_link_fake       # 虚拟导航控制基座
```

如果保留旧车 `base_link_static -> base_link_fake` 的做法，也必须保证整棵 TF 树连通，并且 `map -> base_link_fake` 能稳定查到。

`base_link_fake` 的 yaw 可以沿用旧 `fake_vel_transform` 的思想：根据 NMPC 输出的 `angular.z` 积分得到虚拟 heading。需要确认：

- 初始角度，例如旧代码中的 45 度，是否仍符合新 URDF/外参。
- `angular.z` 积分方向是否和 TF 正方向一致。
- 积分系数是否仍需要类似 `fake_angular_speed_coefficient`。
- 电控是否也消费同一个 `angular.z`，如果消费，虚拟 yaw 和实际底盘 yaw 的关系是否会长期漂移。

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

## 10. 最小迁移步骤

1. 准备老车专用 launch，让 `nav_server` 的 `cmd_vel` remap 到 `/cmd_vel_fake`。
2. 启动全向速度适配层，订阅 `/cmd_vel_fake`，发布 `/cmd_vel`。
3. 设置 `base_frame = base_link_fake`。
4. 发布并验证 `map -> odom -> base_link -> base_link_fake` TF。
5. 修改老车雷达/IMU/云台外参，使 `/cloud_registered` 与 TF 一致。
6. 启动 ROG-Map，确认 `/rog_map/map_2d` frame、原点、障碍方向正确。
7. 关闭所有特殊地形和腿长逻辑。
8. 初期设置 `nmpc.odom_feedback_alpha = 0.0`，先验证开环命令方向。
9. 在 RViz 中给短目标，观察 `/cmd_vel_fake` 与 `/cmd_vel`：
   - `/cmd_vel_fake.linear.y` 可以为 0。
   - `/cmd_vel.linear.x/y` 应随 `base_link_fake` 和 `base_link` 的夹角变化。
10. 方向确认后，再逐步接入 `/Odometry/fake` 反馈。

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
