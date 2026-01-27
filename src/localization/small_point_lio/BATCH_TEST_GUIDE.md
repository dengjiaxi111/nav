# Batch Update 快速测试指南

## 测试前准备

### 1. 确认编译成功
```bash
cd /home/lehan/navigation2026
colcon build --packages-select small_point_lio --symlink-install
source install/setup.bash
```

## 测试步骤

### 测试1：验证原始模式（确保代码不变性）

1. **修改配置文件**
```bash
vim src/localization/small_point_lio/config/mid360.yaml
```

设置为原始模式：
```yaml
use_batch_update: false  # 关闭Batch模式
```

2. **运行节点**
```bash
ros2 launch small_point_lio small_point_lio.launch.py
```

3. **预期行为**
- ✅ 逐点更新，行为与修改前完全一致
- ✅ 没有任何性能变化
- ✅ 轨迹精度保持不变

### 测试2：启用Batch模式

1. **修改配置文件**
```yaml
use_batch_update: true    # 启用Batch模式
batch_point_size: 50      # 每次更新50个点
batch_max_points: 500     # 最大缓存500个点
```

2. **运行节点**
```bash
ros2 launch small_point_lio small_point_lio.launch.py
```

3. **预期行为**
- ✅ 批量更新，更新频率降低
- ✅ CPU占用可能降低（减少卡尔曼更新次数）
- ✅ 点云去畸变，运动补偿更准确

### 测试3：对比不同Batch大小

尝试不同的`batch_point_size`值：
- 小Batch（20-30）：更新频率高，接近逐点
- 中Batch（40-60）：平衡效率和精度
- 大Batch（80-100）：效率高，但可能降低精度

```yaml
# 小Batch测试
batch_point_size: 30

# 中Batch测试
batch_point_size: 50

# 大Batch测试
batch_point_size: 100
```

## 监控指标

### 1. 运行时性能
```bash
# 监控CPU占用
top -p $(pgrep -f small_point_lio)

# 查看节点信息
ros2 node info /small_point_lio
```

### 2. 话题频率
```bash
# 检查里程计发布频率
ros2 topic hz /Odometry

# 检查点云发布频率
ros2 topic hz /cloud_registered
```

### 3. 可视化轨迹
```bash
# 在RViz中添加Odometry和PointCloud2
rviz2

# 添加以下显示：
# - Odometry: /Odometry (显示轨迹)
# - PointCloud2: /cloud_registered (显示地图点云)
# - TF: 显示坐标变换
```

## 调试技巧

### 1. 查看日志输出
```bash
# 查看Batch模式是否正确启用
ros2 topic echo /rosout | grep "Batch"
```

### 2. 检查参数加载
```bash
# 确认参数是否正确读取
ros2 param list /small_point_lio
ros2 param get /small_point_lio use_batch_update
ros2 param get /small_point_lio batch_point_size
```

### 3. 实时修改参数（需要重启节点生效）
```bash
ros2 param set /small_point_lio use_batch_update false
# 注意：参数在下次启动时会从YAML文件重新加载
```

## 性能对比

### 记录以下数据
1. **原始模式（use_batch_update=false）**
   - CPU占用率：_____%
   - 内存占用：_____MB
   - 轨迹精度（ATE/RPE）：_____m
   - 地图质量（主观评分1-5）：_____

2. **Batch模式（batch_point_size=50）**
   - CPU占用率：_____%
   - 内存占用：_____MB
   - 轨迹精度（ATE/RPE）：_____m
   - 地图质量（主观评分1-5）：_____

### 精度评估（如果有Ground Truth）
```bash
# 使用evo工具评估轨迹
evo_ape tum ground_truth.txt estimated_traj.txt -va --plot

# 绝对轨迹误差（ATE）
# 相对位姿误差（RPE）
```

## 故障排查

### 问题1：编译失败
```bash
# 清理构建缓存
rm -rf build/ install/ log/
colcon build --packages-select small_point_lio --symlink-install
```

### 问题2：节点启动失败
```bash
# 检查参数文件语法
yamllint src/localization/small_point_lio/config/mid360.yaml

# 检查launch文件
ros2 launch small_point_lio small_point_lio.launch.py --show-args
```

### 问题3：Batch更新不生效
- 检查`use_batch_update`参数是否为`true`
- 查看日志是否显示"Batch update mode: enabled"
- 确认点云数据正常发布

### 问题4：协方差矩阵异常
- 如果P矩阵不正定，可能是Batch过大
- 尝试减小`batch_point_size`
- 检查激光点协方差`laser_point_cov`参数

## 数据记录模板

### 测试配置
- 日期：2026-01-27
- 数据集：__________
- 传感器：Livox Mid360
- 场景：__________

### 测试结果

| 模式 | Batch大小 | CPU(%) | 内存(MB) | ATE(m) | 地图质量 | 备注 |
|------|-----------|--------|----------|--------|----------|------|
| 原始 | N/A       |        |          |        |          |      |
| Batch| 30        |        |          |        |          |      |
| Batch| 50        |        |          |        |          |      |
| Batch| 100       |        |          |        |          |      |

### 结论
- 最佳配置：__________
- 性能提升：__________
- 精度变化：__________
- 建议：__________

## 开关对照表

| 参数 | 值 | 行为 | 使用场景 |
|------|-----|------|----------|
| use_batch_update | false | 原始逐点更新 | 需要最高更新频率时 |
| use_batch_update | true | Batch批量更新 | 追求效率和去畸变 |
| batch_point_size | 20-30 | 小Batch | 高动态运动 |
| batch_point_size | 40-60 | 中Batch | 平衡模式（推荐） |
| batch_point_size | 80-100 | 大Batch | 静态或慢速运动 |

## 注意事项

⚠️ **重要提醒**
1. 修改YAML配置后需要重启节点
2. Batch模式会降低里程计发布频率
3. 高速运动场景建议使用较小的batch_point_size
4. 如果出现轨迹漂移，检查时间戳是否正确

✅ **最佳实践**
1. 首先在原始模式下验证系统正常
2. 逐步增大batch_point_size测试
3. 记录每次测试的参数和结果
4. 根据应用场景选择合适的配置
