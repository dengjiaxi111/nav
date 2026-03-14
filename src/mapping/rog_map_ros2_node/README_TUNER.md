# 台阶检测参数动态调整工具

## 功能说明

这个工具提供了一个图形界面，可以在播放 rosbag 时实时调整台阶检测的所有参数，无需重启节点。

## 安装依赖

```bash
pip3 install PyQt5
```

## 使用方法

### 1. 启动导航系统

```bash
cd ~/navigation2026
source install/setup.bash
ros2 launch nav_bringup run.launch.py
```

### 2. 播放 rosbag（在另一个终端）

```bash
ros2 bag play your_bag_file.db3
```

### 3. 启动参数调整 GUI（在第三个终端）

```bash
cd ~/navigation2026
source install/setup.bash
python3 src/mapping/rog_map_ros2_node/scripts/stair_tuner_gui.py
```

或者直接运行：
```bash
ros2 run rog_map_ros2_node stair_tuner_gui.py
```

## GUI 界面说明

### 标签页 1: 预筛选参数 🔍
- **网格大小**: 控制 XY 平面网格划分精度（0.01-0.20m）
- **网格最小点数**: 每个网格至少需要多少点（1-10）
- **网格最小高度**: 过滤平坦地面的关键参数（0.00-0.10m）
- **网格最大高度**: 过滤墙壁的参数（0.10-1.00m）
- **网格最高点上限**: 限制识别物体的高度（0.10-1.00m）

**调试技巧**:
- 看不到黄色球 → 降低 `网格最小点数` 和 `网格最小高度`
- 地面也有黄色球 → 提高 `网格最小高度`

### 标签页 2: 几何约束参数 📏
- **台阶标准高度**: 你要识别的台阶高度（0.10-0.40m）
- **高度容差**: 允许的高度误差（0.02-0.20m）
- **最小宽度**: 过滤窄物体（0.10-0.50m）
- **最小深度**: 过滤薄片（0.01-0.20m）
- **最大厚度**: 防止识别墙壁（0.10-0.80m）

**调试技巧**:
- 红框显示 `height_mismatch` → 调整 `台阶标准高度` 或 `高度容差`
- 红框显示 `too_narrow` → 降低 `最小宽度`

### 标签页 3: 聚类参数 🔗
- **聚类搜索半径**: 点之间多近才算一个物体（0.02-0.20m）
- **最小聚类点数**: 物体至少要有多少点（20-500）
- **最大聚类点数**: 防止过大的聚类（1000-30000）

**调试技巧**:
- 台阶被分割成多块 → 增大 `聚类搜索半径`
- 计算慢 → 降低 `最大聚类点数`

### 标签页 4: 法向量验证参数 📐
- **启用法向量验证**: 勾选框，可以完全禁用法向量检查
- **平面性阈值**: 表面要多平整（0.30-1.00）
- **水平法向量 Z 分量下限**: 表面要多水平（0.50-1.00）
- **水平点占比下限**: 多少点要是水平的（0.30-1.00）

**调试技巧**:
- 红框显示 `surface_not_planar` → 降低 `平面性阈值`
- 红框显示 `normal_not_horizontal` → 降低 `水平法向量 Z 分量`

### 标签页 5: ROI 范围参数 📦
- **前方最小/最大距离**: X 方向搜索范围
- **左右范围**: Y 方向搜索范围
- **Z 最小/最大值**: 高度搜索范围

**调试技巧**:
- 青色框太小 → 扩大 ROI 范围
- 青色框包含太多地面 → 提高 `Z 最小值`

## 实时调参工作流程

1. **观察 RViz**:
   - 青色线框 = ROI 范围
   - 黄色小球 = 通过预筛选的点
   - 红色框 = 被拒绝的候选（看文字标签了解原因）
   - 黄色框 = 正在检测的台阶
   - 绿色框 = 已锁定的台阶

2. **逐步调整**:
   ```
   无黄色球 → 调预筛选参数（Tab 1）
   有黄色球但无候选 → 调几何约束（Tab 2）
   有候选但被拒绝 → 根据红框文字调整对应参数
   候选抖动 → 调跟踪参数（增加 min_detection_frames）
   ```

3. **保存参数**:
   调好参数后，手动复制到配置文件：
   ```bash
   nano src/mapping/rog_map_ros2_node/config/stair_detector_params.yaml
   ```

## 命令行方式（不用 GUI）

如果不想用 GUI，也可以直接用命令行：

```bash
# 查看当前参数
ros2 param get /stair_detector min_cell_height

# 设置参数
ros2 param set /stair_detector min_cell_height 0.01
ros2 param set /stair_detector height_tolerance 0.12

# 列出所有参数
ros2 param list /stair_detector
```

## 常用参数组合

### 宽松模式（最容易识别）
```yaml
min_cell_points: 1
min_cell_height: 0.01
height_tolerance: 0.15
min_stair_width: 0.15
enable_normal_estimation: false  # 禁用法向量验证
```

### 严格模式（低误报）
```yaml
min_cell_points: 5
min_cell_height: 0.05
height_tolerance: 0.05
min_stair_width: 0.30
enable_normal_estimation: true
min_planarity: 0.80
```

### 远距离检测
```yaml
roi_x_max: 8.0
min_cluster_size: 50
min_cell_points: 1
```

## 故障排除

### GUI 无法启动
```bash
# 检查 PyQt5 是否安装
python3 -c "import PyQt5; print('OK')"

# 重新安装
pip3 install --upgrade PyQt5
```

### 参数不生效
```bash
# 检查节点是否运行
ros2 node list | grep stair_detector

# 检查服务是否可用
ros2 service list | grep stair_detector
```

### RViz 中看不到效果
```bash
# 检查话题
ros2 topic hz /stair_detector/markers
ros2 topic hz /stair_detector/prefilter_cloud

# 查看日志
ros2 node info /stair_detector
```

## 性能提示

- 调参时如果发现处理变慢，查看终端日志中的 `[Perf]` 信息
- 降低 `update_rate` 可以减少 CPU 占用
- 增大 `min_cluster_size` 可以加快聚类速度

---

**祝调参顺利！🎯**
