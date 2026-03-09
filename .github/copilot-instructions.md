# Navigation2026 - Copilot Instructions

适用范围：`/home/lehan/navigation2026` 整仓（ROS2 Humble）。

## 核心原则（必须遵守）

- 以瞎猜接口为耻，以认真查询为荣（Read before assuming）
- 以模糊执行为耻，以寻求确认为荣（Seek confirmation）
- 以臆想业务为耻，以复用现有为荣（Reuse existing code）
- 以创造接口为耻，以主动测试为荣（Test proactively）
- 以跳过验证为耻，以人类确认为荣（Verify with humans）
- 以破坏架构为耻，以遵循规范为荣（Follow architecture）
- 以假装理解为耻，以诚实无知为荣（Admit uncertainty）
- 以盲目修改为耻，以谨慎重构为荣（Refactor carefully）

## AI 执行规范（保留旧 instructions 精华）

- 修改前必须先读源码核实接口：先 `.hpp`/声明，再 `.cpp`/实现。
- 不猜测：优先确认函数签名、命名空间、参数名、默认值、话题名、frame。
- 先给根因，再做最小改动，不做与需求无关的大面积重构。
- 改动后必须给可执行验证路径（至少构建或话题/节点检查命令）。
- 无法确认时必须明确不确定点，并向人类提出精确问题。

## ROS2 工程规范（CMake/package）

- 新增依赖时必须同时更新：`package.xml` + `find_package()` + `ament_target_dependencies()`。
- 优先使用 `ament_target_dependencies()` 管理 ROS 依赖，避免新增硬编码库路径。
- 若遇到历史遗留硬编码链接（如本仓已有 `acados` 链接方式），默认“最小侵入”，非必要不扩散。
- 参数统一通过 launch/yaml 管理，YAML 键名必须与 `declare_parameter/get_parameter` 对齐。

## 代码风格规范（Google C++ + ROS2 约定）

- 文件命名：`snake_case.hpp/.cpp`。
- 类/结构体：`PascalCase`；私有成员：`snake_case_`。
- 函数：`camelCase`；局部变量和参数：`snake_case`。
- 缩进：4 空格，使用 Tab；建议行宽 ≤ 100（必要时 ≤ 120）。
- include 顺序遵循现有文件风格，优先与同目录已有代码保持一致。
- 注释只写“意图/约束/坑点”，避免解释显然代码；`TODO/FIXME` 要可执行。

## 当前真实工程结构（按代码）

```text
src/
├── localization/
│   ├── livox_ros_driver2/
│   └── small_point_lio/
├── mapping/
│   ├── rog_map/
│   └── rog_map_ros2_node/
├── my_navigation/
│   ├── nav_interfaces/
│   ├── nav_core/
│   ├── nav_components/
│   └── nav_bringup/
├── myserial/
├── robots_msgs/
└── tools/play_music/
```

## 运行链路（按 launch + 源码核对）

```text
small_point_lio
    → /cloud_registered
    → /Odometry

rog_map_ros2_node (integration_node)
    ← /cloud_registered + /Odometry
    → /rog_map/map_2d

nav_components/nav_server
    ← goal_pose (RViz) / navigate(action) / TF(map->base_link)
    ← /rog_map/map_2d (enable_dynamic_layer=true时)
    → cmd_vel / plan / static_map / fused_map / costmap / esdf_map(可选)

myserial/serial_node
    ← cmd_vel
    → RobotFeedBack / GameFeedBack / ChassisOdom / EnemyPose
```

## 启动与构建（高频命令）

```bash
# 1) 最小导航包
colcon build --packages-select nav_interfaces nav_core nav_components nav_bringup --symlink-install

# 2) 全链路（定位+建图+导航+串口）
colcon build --packages-select \
    small_point_lio rog_map rog_map_ros2_node \
    nav_interfaces nav_core nav_components nav_bringup \
    robots_msgs play_music myserial \
    --symlink-install

source install/setup.bash

# 3) 主导航链路（small_point_lio + rog_map integration + nav_server + rviz）
ros2 launch nav_bringup run.launch.py

# 4) 串口节点（单独起）
ros2 launch myserial myserial.launch.py
```

## 参数真源（改配置先看这里）

- `nav_bringup/config/nav_params.yaml` 必须与以下声明严格一致：
    - `nav_components/src/nav_server.cpp`
    - `nav_components/src/simple_planner.cpp`
    - `nav_components/src/nmpc.cpp`
    - `nav_components/src/pure_pursuit.cpp`
    - `nav_components/src/backup_recovery.cpp`
    - `nav_components/src/spin_recovery.cpp`
- `rog_map_ros2_node/config/*.yaml` 需匹配：
    - `mapping/rog_map/include/rog_map/rog_map_core/config.hpp`
    - `mapping/rog_map_ros2_node/src/map_2d_projector.cpp`
    - `mapping/rog_map_ros2_node/src/stair_detector.cpp`

## NMPC / acados 约束（关键）

- `nav_components` 依赖 `nmpc_solver/libacados_ocp_solver_wheelleg_nmpc.so`。
- 修改 `model_ocp/model.py`、`model_ocp/export_ocp.py` 或时域维度后，必须重新导出 solver。
- 生成入口：`nav_components/model_ocp/export_ocp.py`。
- 不要手改 `nmpc_solver/` 下生成代码，除非明确在做生成产物级调试。

## 代码事实（避免误判）

- `nav_server` 当前控制器实例是 `NMPC`，不是运行时插件切换。
- `run.launch.py` 中定义了 `lio_3se_launch` 变量，但未加入 `LaunchDescription`。
- `myserial` 订阅路径话题是 `mypath`；`nav_server` 发布的是 `plan`（两者默认不直连）。
- `navigation.launch.py` 含临时静态 TF `map -> odom`（测试用途）。

## 修改前检查清单（强制）

1. 先读 `.hpp` 接口，再改 `.cpp`。
2. 先读 `declare_parameter/get_parameter`，再改 YAML。
3. 话题名先在源码确认，不硬编码猜测。
4. 涉及地图/坐标，先确认 frame：`map`、`odom`、`base_link`。
5. 只改必要文件，不碰 `build/`、`install/`、`log/`。

## 包边界与依赖现实

- 当前 `rog_map` 本身已直接依赖 ROS2（`rclcpp/nav_msgs/...`），不要再按“纯C++无ROS依赖”假设处理。
- `rog_map_ros2_node` 的多个 target 依赖以下编译定义：
    - `USE_ROS2`
    - `ORIGIN_AT_CORNER`
    - `ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"`

## 建议验证命令

```bash
ros2 topic list | grep -E "Odometry|cloud_registered|rog_map/map_2d|cmd_vel|plan|costmap"
ros2 action list | grep navigate
ros2 node list | grep -E "small_point_lio|rog_map|nav_server|serial"
```

---
最后校对：2026-03-09（依据当前仓库源码）
