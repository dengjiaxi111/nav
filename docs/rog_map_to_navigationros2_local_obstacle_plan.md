# ROG-Map 局部障碍替换为 navigationros2 风格方案

## 目标

把 `navigation2026` 当前基于 `rog_map + Map2DProjector` 的局部障碍处理，替换为 `/home/li/navigationros2` 中更简单的点云障碍加载方式。

目标不是把 `navigation2026` 整体改成 Nav2，而是保留现有自研导航框架：

`static map + dynamic OccupancyGrid -> LayeredMapManager -> ESDF/costmap -> planner/NMPC`

只替换上游动态障碍源。

## 当前 navigation2026 链路

相关代码：

- `/home/li/navigation2026/src/mapping/rog_map`
- `/home/li/navigation2026/src/mapping/rog_map_ros2_node`
- `/home/li/navigation2026/src/my_navigation/nav_bringup/launch/run.launch.py`
- `/home/li/navigation2026/src/my_navigation/nav_components/src/layered_map_manager.cpp`

现有链路：

```text
small_point_lio
  -> /cloud_registered
  -> rog_map_ros2_node::integration_node
       -> ROGMapROS: 概率占据 / raycasting / fading / inflation
       -> Map2DProjector: 3D occupied voxels -> 2D OccupancyGrid
  -> /rog_map/map_2d
  -> nav_server dynamic_layer_topic
  -> LayeredMapManager::updateDynamicLayer()
  -> fused_map / ESDF / costmap
```

`nav_server` 只要求动态层是 `nav_msgs/msg/OccupancyGrid`。默认订阅：

```yaml
dynamic_layer_topic: "/rog_map/map_2d"
```

因此替换点很清楚：新障碍节点只要继续发布 `OccupancyGrid`，导航核心不需要大改。

## navigationros2 的局部障碍方式

`/home/li/navigationros2` 的有效链路是：

```text
small_point_lio
  -> /cloud_registered
  -> pointcloud_segmentation
       -> 两帧点云合并
       -> 转 base_link
       -> z passthrough
       -> NormalEstimationOMP
       -> 保留 normal_z 不接近水平的点
       -> StatisticalOutlierRemoval
  -> /cloud_filtered
  -> spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer
  -> local/global costmap
```

关键配置在 `/home/li/navigationros2/src/mybringup/config/costmap2d.yaml`：

- 点云源：`/cloud_filtered`
- 类型：`PointCloud2`
- 局部层：`SpatioTemporalVoxelLayer`
- `voxel_size: 0.05`
- `voxel_decay: 0.5`
- `marking: true`
- `clearing: false`
- 局部高度窗口：`min_obstacle_height: -1.0`, `max_obstacle_height: 1.0`

这套方法的优点是参数少，逻辑直观：先从点云里提取非地面/障碍点，再按时间衰减投到局部障碍层。

## 推荐架构

不要直接把 Nav2 costmap 整套塞进 `navigation2026`。推荐做一个轻量的 `navigationros2` 风格适配包：

```text
/cloud_registered
  -> pointcloud_segmentation_node
  -> /cloud_filtered
  -> local_obstacle_grid_node
  -> /local_obstacle/map_2d 或 /rog_map/map_2d
  -> nav_server dynamic_layer_topic
```

推荐新建包：

```text
/home/li/navigation2026/src/mapping/pointcloud_obstacle_layer
```

包含两个节点：

1. `pointcloud_segmentation_node`
2. `local_obstacle_grid_node`

这样比继续维护 `rog_map + projector` 更简单，也不需要引入 Nav2 lifecycle/costmap 的完整运行时。

## 节点 1：pointcloud_segmentation_node

从 `/home/li/navigationros2/src/mapping_and_location/pointcloud_segmentation` 移植，但不要原样复制。

需要修正的点：

- 原代码硬编码 `cloud_registered`、`cloud_filtered`，建议改成参数。
- 原代码硬编码 `base_link`、`odom`，建议改成参数。
- 原代码构造函数里等待 `livox_left -> base_link`，但后续基本没用，建议删除。
- 原代码发布 `/cloud_filtered` 时没有设置 stamp，建议使用输入点云 stamp 或当前时间。
- 原代码参数名 `segementation_type` 拼写错误，建议新实现中改成 `segmentation_type`。

建议参数：

```yaml
pointcloud_segmentation:
  ros__parameters:
    input_cloud_topic: "/cloud_registered"
    output_cloud_topic: "/cloud_filtered"
    target_frame: "odom"
    base_frame: "base_link"
    merge_frame_count: 2

    z_min_base: -0.1
    z_max_base: 1.5
    normal_radius: 0.5
    normal_z_abs_max: 0.8

    sor_mean_k: 4
    sor_stddev: 1.1
```

保留的算法：

```text
PointCloud2 -> PCL XYZ
  -> transform to base_frame
  -> passthrough z_min_base/z_max_base
  -> NormalEstimationOMP
  -> keep abs(normal_z) < normal_z_abs_max
  -> StatisticalOutlierRemoval
  -> transform back to target_frame
  -> publish /cloud_filtered
```

## 节点 2：local_obstacle_grid_node

这个节点替代 Nav2 的 `SpatioTemporalVoxelLayer`，输出 `navigation2026` 可直接消费的 `OccupancyGrid`。

输入：

```text
/cloud_filtered    sensor_msgs/msg/PointCloud2
```

输出：

```text
/local_obstacle/map_2d   nav_msgs/msg/OccupancyGrid
```

为最小改动，也可以先发布到旧 topic：

```text
/rog_map/map_2d
```

推荐第一阶段先发布 `/rog_map/map_2d`，这样 `nav_params.yaml` 可以暂时不改。稳定后再改名为 `/local_obstacle/map_2d`。

核心行为：

```text
1. 订阅 /cloud_filtered
2. 将点云变换到 odom
3. 查询 odom -> base_link，得到机器人当前位置
4. 生成 odom 轴对齐 rolling OccupancyGrid
5. 把点云点投到 2D 栅格，占据值写 100
6. 使用 last_seen 时间戳做 0.3~0.7s 衰减
7. 按固定频率发布 OccupancyGrid
```

建议参数：

```yaml
local_obstacle_grid:
  ros__parameters:
    input_cloud_topic: "/cloud_filtered"
    output_map_topic: "/rog_map/map_2d"
    target_frame: "odom"
    base_frame: "base_link"

    resolution: 0.05
    range_x: 3.0        # 腿车可先对齐 navigationros2 local_costmap
    range_y: 3.0
    publish_rate: 50.0

    min_obstacle_height: -1.0
    max_obstacle_height: 1.0
    obstacle_range: 5.0
    voxel_decay: 0.5

    occupied_value: 100
    free_value: 0
```

`OccupancyGrid` 发布要求：

- `header.frame_id = "odom"`
- `info.resolution = 0.05`
- `info.origin.position.x = floor((base_x - range_x / 2) / resolution) * resolution`
- `info.origin.position.y = floor((base_y - range_y / 2) / resolution) * resolution`
- `info.origin.orientation.w = 1.0`
- `data` 默认填 `0`
- 命中的障碍栅格填 `100`

不要发布 base_link 坐标系下的局部地图。`LayeredMapManager` 会把 dynamic layer 从 `odom` 投影到 `map`。

## Launch 修改方案

### 腿车 run.launch.py

当前启动：

```python
integration_node,          # ROG-Map (3D地图 + 2D投影)
navigation_launch,
```

建议改为：

```python
local_obstacle_launch,     # pointcloud segmentation + local obstacle grid
navigation_launch,
```

第一阶段保留 topic `/rog_map/map_2d`，只禁用 `integration_node`，不改 `nav_params.yaml`。

后续稳定后：

```yaml
dynamic_layer_topic: "/local_obstacle/map_2d"
```

## CMake/package 依赖

如果新建 `pointcloud_obstacle_layer`，建议依赖：

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>pcl_conversions</depend>
<depend>pcl_ros</depend>
```

CMake 需要：

```cmake
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
```

不建议在第一版引入：

- `nav2_costmap_2d`
- `spatio_temporal_voxel_layer`
- lifecycle manager

原因是 `navigation2026` 的导航核心不是 Nav2，直接接入 Nav2 costmap 会增加生命周期、TF、topic、参数和调试复杂度。

## 参数简化收益

替换后应删除或不再依赖这些 ROG 参数组：

- `raycasting.p_hit/p_miss/p_min/p_max/p_occ/p_free`
- `batch_update_size`
- `unk_thresh`
- `fading_frame_thresh`
- `unk_inflation_en`
- `inflation_step`
- `virtual_ground_height`
- `virtual_ceil_height`
- `Map2DProjector` 的坡面、台阶、法向量、邻域连通、mask 清除参数

保留调参面压缩为：

- 点云分割高度窗口
- 法向量阈值
- SOR 离群点滤波
- local grid 范围/分辨率
- 衰减时间

## 上线步骤

1. 保留 ROG，新增新节点，输出到 `/local_obstacle/map_2d_test`。
2. 同时看 RViz：
   - `/cloud_registered`
   - `/cloud_filtered`
   - `/local_obstacle/map_2d_test`
   - 原 `/rog_map/map_2d`
   - `/costmap`
3. 用 rosbag 回放同一段数据，比较：
   - 低矮障碍是否检出
   - 地面/坡面误检是否可接受
   - 障碍消失延迟是否合理
   - CPU 占用和发布频率
4. 把新节点输出 remap 到 `/rog_map/map_2d`，禁用 `integration_node`。
5. 跑导航闭环，验证：
   - `nav_server` 是否正常收到动态层
   - `LayeredMapManager` 是否正常融合
   - 起点/路径碰撞检查是否正常
   - recovery 是否仍按 costmap 触发
6. 稳定后把 topic 改名为 `/local_obstacle/map_2d`，同步改 `dynamic_layer_topic` 和 RViz。

## 回滚方案

保留原 `rog_map_config.yaml`、`projector_params.yaml` 和 `integration_node` 启动块。

如果新局部障碍层出现漏检或误检：

```text
停止 local_obstacle_launch
恢复 integration_node
dynamic_layer_topic 改回 /rog_map/map_2d
```

导航核心不需要回滚。

## 风险点

1. `navigationros2` 的分割方法主要保留竖直/非地面结构，可能漏掉非常低、法向量接近水平的障碍。
2. 新方案没有完整 raycasting clearing，建议靠 `voxel_decay` 时间衰减清除残留。
3. `range_x/range_y` 过小会导致高速时避障反应不足；过大会增加点云投影开销。
4. 如果场地高度变化、坡道/台阶需要语义判断，应该继续交给 `navigation2026` 现有 stair/special terrain 层，而不是在局部障碍层里重新做复杂高程分类。

## 推荐结论

推荐实现“点云分割 + 轻量 rolling OccupancyGrid”方案，而不是直接引入 Nav2 STVL。

最小可行替换是：

```text
port pointcloud_segmentation
add local_obstacle_grid_node
publish /rog_map/map_2d
remove run.launch.py 中的 integration_node
```

这样可以保留 `navigation2026` 的规划、控制、ESDF、膨胀、特殊地形层，只把复杂的 ROG-Map 局部障碍算法替换掉。
