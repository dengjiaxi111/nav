# ROG-Map ROS2 使用说明

`rog_map_ros2_node` 是 `rog_map` 的 ROS2 包装层，提供：

- `rog_map_node`：ROG-Map 3D 概率地图
- `map_2d_projector`：3D→2D 投影（在 `rog_map_node` 内部创建）
- `stair_detector_node`：台阶检测（独立节点）

## 目录结构（当前有效）

```text
rog_map_ros2_node/
├── config/
│   ├── rog_map_config.yaml      # ROG-Map核心参数（库读取）
│   └── projector_params.yaml    # 2D投影参数（ROS2标准参数）
├── launch/
│   └── rog_map.launch.py        # ROG-Map + 2D投影 + RViz
├── src/
│   ├── rog_map_node.cpp         # 实例化 ROGMapROS + Map2DProjector
│   ├── map_2d_projector.cpp     # 3D→2D 投影
│   └── stair_detector.cpp       # 台阶检测
└── rviz/
    └── rog_map.rviz
```

## 参数来源（必须区分）

| 模块 | 参数文件 | 读取方式 | 入口 |
| --- | --- | --- | --- |
| ROG-Map核心 | `config/rog_map_config.yaml` | 库内部读取 | `-p config_file:=...` |
| 2D投影器 | `config/projector_params.yaml` | ROS2参数 | `parameters=[projector_params.yaml]` |
| 台阶检测 | 由外部launch传入 | ROS2参数 | `stair_detector_node` 参数文件 |

> `rog_map_config.yaml` 不再包含 `projector` 配置，避免“改了不生效”。

## 常用启动方式

```bash
colcon build --packages-select rog_map_ros2_node --symlink-install
```

```bash
ros2 launch rog_map_ros2_node rog_map.launch.py
```

如果使用全导航启动（`nav_bringup/launch/run.launch.py`），需要保证：

- `rog_map_config_file` 指向正确的 `rog_map_config.yaml`
- `projector_params.yaml` 已在 launch 中传入

## 关键话题（最小集合）

| 方向 | 话题 | 说明 |
| --- | --- | --- |
| 订阅 | `/cloud_registered` | ROG-Map点云输入（由配置控制） |
| 订阅 | `/Odometry` | 里程计输入（由配置控制） |
| 发布 | `/rog_map/occ` | 原始占据点云 |
| 发布 | `/rog_map/map_2d` | 2D投影地图（OccupancyGrid） |

## 已知问题（需注意）

1. **`rog_map.launch.py` 中台阶检测参数为空**
   - 当前从 `rog_map_config.yaml` 读取 `stair_detector`，但文件内已移除该段。
   - 结果：`stair_detector_node` 使用默认值。
   - 建议：为台阶检测增加标准参数文件并在 launch 中传入。

2. **README 曾包含不存在模块**
   - 旧文档中提到 `obstacle_perception`/`integration.launch.py` 等文件，当前仓库已不存在。

---

如需我继续：

- 补齐 `stair_detector` 参数文件与 launch 对接
- 统一各参数默认值与配置注释
- 添加一页“参数快速对照表”

请直接告诉我范围。
