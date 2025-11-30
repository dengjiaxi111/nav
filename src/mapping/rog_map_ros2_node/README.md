## ROG-Map ROS2 集成总结

### 架构设计

创建了新的 ROS2 包 `rog_map_ros2_node`，作为独立的包装层调用 `rog_map` 库：

```
src/mapping/
├── rog_map/               # 核心库（保持原样）
│   ├── include/
│   │   └── rog_map_ros/
│   │       └── rog_map_ros2.hpp  # ROGMapROS 类（已包含完整ROS2集成）
│   └── ...
└── rog_map_ros2_node/    # ROS2 包装节点
    ├── src/
    │   ├── rog_map_node.cpp         # 轻量级节点（实例化ROGMapROS）
    │   └── obstacle_perception.cpp  # 🆕 障碍物感知模块
    ├── include/rog_map_ros2_node/
    │   └── obstacle_perception.hpp  # 🆕 障碍物感知头文件
    ├── config/
    │   ├── rog_map_config.yaml      # ROG-Map配置
    │   └── perception_config.yaml   # 🆕 感知模块配置
    ├── launch/
    │   ├── rog_map.launch.py        # ROG-Map启动脚本
    │   ├── integration.launch.py    # 完整集成启动
    │   └── perception.launch.py     # 🆕 感知模块启动
    └── rviz/
        └── rog_map.rviz             # RVIZ可视化配置
```

---

## 🆕 障碍物感知模块

### 算法设计（灵感来源：中科大哨兵2025技术报告）

#### 核心思路

1. **高程分析**：基于ROG-Map输出的结构化点云，对每个(x,y)柱进行分析
   - 高度差 `H = max_z - min_z`
   - 占据率 `occupancy = (n * resolution) / H`
   
2. **可通行性分割**：
   - 墙壁特征：高占据率 + 足够高度差（垂直面扫描到一条线）
   - 低矮障碍：最低点高于地面但低于机器人高度
   - 稀疏结构：高度差大但占据率低（如栏杆）

3. **时空聚类**：利用Occupancy Map的尾迹特性
   - 空间聚类：DBSCAN变种
   - 时间关联：基于帧号追踪历史聚类

4. **卡尔曼滤波**：
   - 匀速模型预测
   - 卡方检验判断观测是否匹配
   - 估计障碍物运动速度

### 话题接口

| 订阅话题 | 类型 | 说明 |
|---------|------|-----|
| `rog_map/occ` | PointCloud2 | ROG-Map输出的占据点云 |
| `/Odometry` | Odometry | 里程计（用于局部范围过滤） |

| 发布话题 | 类型 | 说明 |
|---------|------|-----|
| `perception/obstacles` | PointCloud2 | 障碍物点（2D） |
| `perception/traversable` | PointCloud2 | 可通行点（2D） |
| `perception/markers` | MarkerArray | 障碍物可视化 |

### 参数配置

```yaml
obstacle_perception:
  ros__parameters:
    # 高程分析
    robot_height: 0.6           # 机器人高度
    height_diff_thresh: 0.15    # 高度差阈值
    occupancy_rate_thresh: 0.3  # 占据率阈值
    
    # 聚类
    cluster_distance: 0.3       # 聚类距离
    min_cluster_size: 3         # 最小聚类点数
    
    # 卡尔曼滤波
    velocity_thresh: 0.1        # 动态障碍物速度阈值
```

### 使用方式

```bash
# 单独运行感知节点
ros2 run rog_map_ros2_node obstacle_perception_node \
  --ros-args --params-file config/perception_config.yaml

# 完整感知管道
ros2 launch rog_map_ros2_node perception.launch.py
```

---

## 原有功能

### 关键点

1. **rog_map包保持不变**：核心库没有任何修改
2. **ROGMapROS类已实现ROS2集成**：
   - 自动订阅点云 (`/cloud_registered`) 和里程计 (`/Odometry`)
   - 自动发布多个map层的点云数据
   - 自动管理TF转换
   - 在构造函数中完成所有初始化

### 编译方式

```bash
colcon build --packages-select rog_map_ros2_node --symlink-install
```

### 启动方式

```bash
# ROG-Map节点
ros2 run rog_map_ros2_node rog_map_node --ros-args \
  -p config_file:=/path/to/rog_map_config.yaml

# 或使用launch文件
ros2 launch rog_map_ros2_node rog_map.launch.py

# 完整集成（small_point_lio + rog_map + rviz）
ros2 launch rog_map_ros2_node integration.launch.py
```

### 测试

运行 `test_publisher.py` 发送测试数据：
```bash
python3 src/mapping/rog_map_ros2_node/test_publisher.py
```
