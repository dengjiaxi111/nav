# 云台雷达 TF 迁移方案

日期：2026-06-04

## 目标

把雷达从底盘刚性安装迁移到云台上，同时尽量保留原来的导航和控制接口：

- 导航仍然使用 `map -> odom -> base_link`。
- 控制器仍然输出底盘速度 `cmd_vel`。
- 局部障碍层仍然以 `base_link` 做高度裁剪和 rolling window。
- 云台 yaw 只在定位/点云外参补偿层消化，不向导航控制层扩散。

当前第一阶段只关注 `base_link` 位姿是否正确、连续、稳定。`/Odometry.twist` 中的线速度和角速度暂不作为本阶段目标，后续再单独优化。

本次实施范围只包含腿车启动链路和腿车 LIO。全向版本 `run_omni.launch.py` 和 `small_point_lio_omni.launch.py` 暂时保持原来的固定雷达 TF，不接入 `lidar_mount_mode`。

当前代码状态：

- 阶段 A 已完成：腿车可通过 `lidar_mount_mode` 在 `fixed` 和 `gimbal_yaw` 两种 TF 模式间切换。
- 阶段 B 已完成：腿车 LIO 在 `gimbal_yaw` 模式下按数据时间查询当前 `livox_frame -> base_link`，用于发布底盘 `odom -> base_link` 和 `/Odometry.pose`。
- 阶段 C 未做：`/Odometry.twist` 仍暂不作为本阶段验收目标。

当前已知限制：

- 串口里的 `_base_yaw` 当前没有硬件时间戳。
- `robots_msgs/msg/ChassisOdom.msg` 当前没有 `Header`，因此 `gimbal_yaw_tf_publisher.py` 只能用 ROS 收到消息时的 `now()` 给 `base_link -> gimbal_yaw_link` 打时间戳。
- 低速调试可以先这样跑通；云台高速旋转或比赛使用前，建议让电控上传云台 yaw 的采样时间戳。

当前代码里雷达被当作底盘刚性安装：

```text
map -> odom -> base_link -> livox_frame
```

迁移后目标 TF：

```text
map
└── odom
    └── base_link
        └── gimbal_yaw_link
            └── livox_frame
```

其中：

- `base_link -> gimbal_yaw_link`：动态 TF，只绕 yaw 旋转。
- `gimbal_yaw_link -> livox_frame`：静态 TF，表示雷达相对云台旋转轴的固定安装外参。

为了方便回退到原来的底盘固定雷达模式，增加一个 launch 参数即可：

```text
lidar_mount_mode: fixed | gimbal_yaw
```

该参数是唯一模式开关，避免再额外暴露多个容易配错的开关。

## 核心原则

不要让导航控制直接感知云台运动。LIO 对外发布的 `/Odometry` 必须仍然表示底盘 `base_link` 在 `odom` 下的位姿，`/cloud_registered` 必须仍然在 `odom` 下表达。

如果云台转动时直接用雷达位姿当底盘位姿，控制器会看到虚假的底盘旋转。正确做法是按当前云台 yaw 把雷达位姿换算到底盘位姿，再发布 `odom -> base_link`。

本阶段只要求 `/Odometry.pose` 和 TF 中的 `odom -> base_link` 正确。`/Odometry.twist` 可以先保持现状，只要控制器暂时不依赖它做关键闭环，或者在测试时明确忽略该项。

## 修改范围

### 1. 启动文件：拆分原静态外参

涉及文件：

- `src/localization/small_point_lio/launch/small_point_lio.launch.py`

当前腿车 LIO launch 直接发布：

```text
base_link -> livox_frame
```

需要改成：

```text
base_link -> gimbal_yaw_link      动态，由云台 yaw 发布
gimbal_yaw_link -> livox_frame    静态，由标定外参发布
```

原来的 `static_base_link_to_livox_frame` 必须禁用或删除，避免 TF 树里同时出现两个 `livox_frame` 父节点。

静态雷达外参应只描述雷达相对云台旋转轴的位置和姿态：

```text
gimbal_yaw_link -> livox_frame
```

动态 yaw TF 的平移部分是云台旋转轴相对 `base_link` 的位置：

```text
base_link -> gimbal_yaw_link
```

如果云台旋转轴和底盘中心不重合，这里的 xyz 必须填真实测量值。

模式切换规则：

```text
lidar_mount_mode=fixed:
  发布原来的静态 TF:
    base_link -> livox_frame
  不发布:
    base_link -> gimbal_yaw_link
    gimbal_yaw_link -> livox_frame
  LIO 使用固定外参逻辑。

lidar_mount_mode=gimbal_yaw:
  不发布原来的:
    base_link -> livox_frame
  发布:
    gimbal_yaw_link -> livox_frame
  另由云台 yaw 发布:
    base_link -> gimbal_yaw_link
  LIO 使用动态外参逻辑，每帧按数据时间查询 livox_frame -> base_link。
```

必须保证任意时刻 `livox_frame` 只有一个父节点。也就是说，`fixed` 和 `gimbal_yaw` 两种 TF 不能同时存在。

推荐默认值：

```text
lidar_mount_mode=fixed
```

这样改动上线后默认仍保持原行为，需要云台雷达时再显式切换：

```bash
ros2 launch nav_bringup run.launch.py lidar_mount_mode:=gimbal_yaw
```

### 2. 串口/云台 yaw：发布动态 TF

涉及文件：

- `src/myserial/src/serial_node.cpp`
- `src/myserial/include/myserial/serial_node.hpp`

当前串口已经读到 `_base_yaw`，并写入：

- `ChassisOdom.gimbal_angle`
- `decision_messages/EnemyRobotState.base_yaw`

但没有发布 `GimbalData`，也没有发布云台 TF。

建议最小实现：

- 在 `myserial` 内新增一个 `tf2_ros::TransformBroadcaster`，或者单独写一个轻量节点订阅云台 yaw 后发布 TF。
- 发布 `base_link -> gimbal_yaw_link`。
- TF 只绕 Z 轴 yaw，roll/pitch 保持 0。
- yaw 单位统一为 rad，若电控回传是 deg，需要转换。
- 处理 yaw 环绕，内部计算速度或预测时需要 unwrap。

更推荐单独节点：

```text
/gimbal/yaw 或 ChassisOdom.gimbal_angle -> gimbal_tf_publisher -> TF
```

这样串口协议和 TF 补偿解耦，后续也方便替换云台角来源。

### 3. 时间戳要求：不能只用 latest yaw

云台 yaw 是按频率发布的。如果 LIO 查询 `latest transform`，云台转动时会把外参变成阶梯函数，导致 `odom -> base_link` 小跳。

必须做到：

- 云台 yaw TF 带准确时间戳。
- 时间戳应尽量是电控测量 yaw 的时刻，而不是 ROS 收到串口包的时刻。
- LIO 查询雷达点云/里程计对应时间的 TF，不查 `TimePointZero/latest`。
- tf2 用缓存中的相邻 yaw 自动插值。

当前实现状态：

- LIO 已经按雷达/里程计数据时间查询 TF。
- 但 `base_link -> gimbal_yaw_link` 的 yaw TF 目前使用 ROS 接收 `ChassisOdom` 的时间戳，不是电控采样时间戳。
- 因此当前阶段满足“接口链路跑通”和“低速验证”，但还不是最终的高动态时间同步方案。

建议后续让电控上传云台 yaw 采样时间。优先级如下：

1. 最好：下位机和上位机时间同步，直接上传同一时间基准下的采样时间戳。
2. 次优：上传下位机单调 tick，例如 us/ms，上位机估计 offset 后换算成 ROS time。
3. 临时：继续使用 ROS 接收时间。该方式会把串口排队、解析、线程调度延迟变成 yaw 时间误差。

推荐新增一个专门的云台 yaw 消息，而不是强行改 `ChassisOdom` 破坏已有订阅者：

```text
std_msgs/Header header
float32 yaw
float32 yaw_velocity
```

可以命名为 `GimbalYawStamped` 或放入现有 `GimbalData` 的 stamped 版本。`header.stamp` 表示 yaw 的采样时间，`yaw` 单位建议统一为 rad。

云台 yaw 发布频率建议：

- 最低：不低于控制频率，当前控制约 50 Hz。
- 推荐：100 Hz。
- 云台快速旋转或雷达离转轴较远时：200 Hz 更稳。

误差估计：

```text
角度阶梯误差 ≈ 云台角速度 * yaw 发布周期
位置误差 ≈ 雷达到云台旋转轴距离 * 角度阶梯误差
```

例如云台 180 deg/s，yaw 50 Hz，则单帧约 3.6 deg；雷达到转轴 0.2 m 时，位置阶梯量约 1.2 cm。

### 4. LIO：不要启动时只缓存一次外参

涉及文件：

- `src/localization/small_point_lio/src/small_point_lio_node.cpp`
- `src/localization/small_point_lio/src/small_point_lio_node.hpp`

当前 `small_point_lio_node` 启动后通过 TF 查询一次 `base_link` 和 `livox_frame` 的关系，并缓存为静态外参。云台安装后这不成立。

需要改成：

- 保留“等待 TF 可用”的初始化检查。
- 不再把 `base_link <-> livox_frame` 当静态外参永久缓存。
- 在每次 odom callback 和 pointcloud callback 中，根据当前数据时间戳查询：

```text
lookupTransform("livox_frame", "base_link", sensor_stamp)
```

注意 tf2 语义：

```text
lookupTransform(target, source, time)
```

返回的是把 `source` 坐标下的量变换到 `target` 坐标下的变换。实现时建议统一命名为：

```text
T_target_source
```

避免 `T_base_to_lidar` 这种容易产生歧义的变量名。

如果查询对应时间 TF 失败：

- 短时间内可以跳过本帧发布，避免发布错误位姿。
- 不建议回退到 latest TF，因为这会重新引入阶梯误差。
- 日志需要节流，避免刷屏。

LIO 行为也由同一个 `lidar_mount_mode` 决定：

```text
fixed:
  兼容原逻辑，可启动时缓存 base_link <-> livox_frame。

gimbal_yaw:
  禁用永久缓存，每帧按 sensor_stamp 查询 livox_frame -> base_link。
```

不要再单独增加第二个用户可见参数，例如 `use_dynamic_lidar_extrinsic`。如果代码内部需要 bool，可以由 launch 根据 `lidar_mount_mode` 自动派生，避免用户把 TF 模式和 LIO 模式配反。

### 5. LIO 位姿换算

LIO 内部估计的是雷达/IMU body 的运动。对外仍要发布底盘：

```text
odom -> base_link
```

每帧应使用当前时刻的 `livox_frame -> base_link` 外参，把雷达位姿换算到底盘位姿。

建议用完整 SE(3) 链乘，不手动拆旋转和平移。

概念表达如下：

```text
T_odom_base = T_odom_lidar * T_lidar_base
```

如果代码里使用的是重力对齐坐标系，则把原有的 gravity alignment 继续放在最外层：

```text
T_odom_base = T_align * T_lio_odom_lidar * T_lidar_base
```

这里的 `T_lidar_base` 必须来自当前时刻 TF，而不是启动时缓存值。

### 6. 速度处理：第一阶段暂不修改

雷达装上云台后，雷达 IMU 角速度包含：

```text
雷达角速度 = 底盘角速度 + 云台相对底盘角速度
```

所以从最终正确性看，不能再直接把雷达角速度旋转到 `base_link` 后作为底盘角速度发布给 `/Odometry.twist`。

但当前阶段先不处理 `/Odometry.twist`，只保证 `base_link` 位姿正确。也就是说：

- `/Odometry.pose` 必须代表底盘 `base_link`。
- TF 中的 `odom -> base_link` 必须代表底盘 `base_link`。
- `/Odometry.twist` 可暂时保留原实现，不作为本阶段验收项。
- 如果控制器当前会强依赖 `/Odometry.twist`，测试时需要明确这一风险，或临时关闭相关反馈权重。

后续速度优化时再处理：

- 线速度继续用连续两帧 `odom -> base_link` 位置差分。
- 角速度也用连续两帧 `odom -> base_link` 姿态差分。
- 对 yaw 角速度做 unwrap 和低通滤波。
- 如果底盘电控能提供更可靠的底盘角速度，也可以直接使用电控底盘角速度。

控制器最关心的是 `base_link` 的底盘速度，不应混入云台相对转动。

### 7. 点云过滤参数

当前 `base_link_to_lidar_xyz` 会传给 LIO 的自车点过滤逻辑，用来按 `base_link` 半径过滤近距离点。

雷达上云台后，这个值不应再简单等于原来的固定 `base_link -> livox_frame`。建议：

- 如果只做最小可用，可先填云台 yaw 为 0 时的 `base_link -> livox_frame` 平移，用于近距离过滤近似。
- 更严格的做法是把点云过滤也改为按当前 `livox_frame -> base_link` 动态变换计算距离。

第一阶段可以先不动过滤，只要 `min_distance` 留有余量，通常不会影响主链路验证。

## 分阶段实施

### 阶段 A：TF 链路先跑通

目标：不改导航控制，只让 TF 树正确。

任务：

- 禁用原 `base_link -> livox_frame` 静态 TF。
- 发布静态 `gimbal_yaw_link -> livox_frame`。
- 发布动态 `base_link -> gimbal_yaw_link`。
- 用 `view_frames` 确认 `livox_frame` 只有一个父节点。
- 增加 `lidar_mount_mode` 参数，并验证 `fixed` 和 `gimbal_yaw` 两种模式不会同时发布 TF。

验证：

```text
ros2 run tf2_ros tf2_echo base_link gimbal_yaw_link
ros2 run tf2_ros tf2_echo gimbal_yaw_link livox_frame
ros2 run tf2_ros tf2_echo base_link livox_frame
```

只转云台、不动底盘时：

- `base_link -> gimbal_yaw_link` yaw 变化。
- `gimbal_yaw_link -> livox_frame` 不变。
- `base_link -> livox_frame` 连续变化。

### 阶段 B：LIO 使用动态外参

目标：`/Odometry` 仍然表示底盘 `base_link`。

任务：

- LIO 每帧按数据时间查询 `livox_frame -> base_link`。
- `odom -> base_link` 用当前 TF 计算。
- `/cloud_registered` 继续发布到 `odom`。
- `/Odometry.child_frame_id` 保持 `base_link`。
- `/Odometry.pose` 表示底盘位姿。
- `/Odometry.twist` 暂不作为本阶段修改目标。

验证：

只转云台、不动底盘时：

- `odom -> base_link` yaw 不应随云台明显变化。
- `/Odometry.pose.pose.orientation` 不应随云台明显变化。
- `/cloud_registered` 不应随着云台转动在 RViz 中明显扭动或跳变。

### 阶段 C：速度和控制稳定性（后续优化）

目标：在 `base_link` 位姿链路稳定后，再避免 NMPC 因 odom/twist 抖动出现求解异常。

任务：

- `/Odometry.twist` 的角速度改为底盘姿态差分或电控底盘角速度。
- 对差分速度做限幅和低通。
- 记录云台快速旋转时的 `/Odometry` 连续性。

验证：

固定底盘，云台快速 yaw：

- `base_link` 在 RViz 中基本稳定。
- `/Odometry.pose.pose.orientation` 基本稳定。
- `/Odometry.twist.twist.angular.z` 接近 0。

底盘和云台同时旋转：

- `odom -> base_link` 跟随底盘运动。
- 不出现明显阶跃。
- 控制器输入速度没有尖峰。

## 关键验收标准

1. TF 树无重复父节点：

```text
livox_frame 只能挂在 gimbal_yaw_link 下
```

在 `fixed` 模式下则应为：

```text
livox_frame 只能挂在 base_link 下
```

2. 只转云台时，底盘位姿稳定：

```text
odom -> base_link 不随云台 yaw 明显旋转
```

3. 同时转底盘和云台时，底盘位姿连续：

```text
odom -> base_link 不出现明显跳变
```

4. 点云输出稳定：

```text
/cloud_registered 在 odom 下不因云台 yaw 出现明显扭动或跳变
```

5. 局部障碍层接口不变：

```text
/cloud_registered frame_id = odom
/rog_map/map_2d frame_id = odom
base_frame 仍使用 base_link 或 base_link_fake
```

本阶段暂不验收：

```text
/Odometry.twist.twist.angular.z 不包含云台相对转速
```

该项放到后续速度优化阶段。

## 风险点

- 云台 yaw 时间戳不准会直接引入外参误差。
- yaw 发布频率太低会导致 `base_link` 位姿阶梯化。
- 查询 latest TF 会导致控制抖动，必须按 sensor stamp 查 TF。
- 雷达 IMU 角速度包含云台自转，后续不能直接作为底盘角速度；当前阶段先不验收 twist。
- 如果云台旋转轴 xyz 标定不准，会在云台转动时表现为底盘位置假位移。

## 推荐最小落地顺序

1. 增加 `lidar_mount_mode` 参数，默认 `fixed`，保证原逻辑不变。
2. 在 `gimbal_yaw` 模式实现 TF 拆分：`base_link -> gimbal_yaw_link -> livox_frame`。
3. 确认云台 yaw TF 时间戳和频率。
4. 改 LIO 在 `gimbal_yaw` 模式每帧查动态外参。
5. 验证只转云台时 `odom -> base_link` 稳定。
6. 最后再优化 `/Odometry.twist` 和点云过滤中的动态雷达位置。

这样改动集中在 TF 发布和 LIO 外参使用上，导航控制、规划器、局部障碍层参数可以保持原状。
