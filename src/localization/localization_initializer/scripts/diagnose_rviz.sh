#!/bin/bash
# RViz 显示诊断脚本

echo "════════════════════════════════════════════════════════════════"
echo "🔍 RViz 显示问题诊断"
echo "════════════════════════════════════════════════════════════════"
echo ""

# 1. 检查节点
echo "1️⃣ 检查运行的节点："
ros2 node list | grep -E "localization_initializer|rviz2|small_point_lio"
echo ""

# 2. 检查地图话题
echo "2️⃣ 检查地图话题："
ros2 topic info /localization/map_cloud
echo ""

# 3. 检查地图点云数量
echo "3️⃣ 检查地图点云数量："
timeout 3 ros2 topic echo /localization/map_cloud --once 2>&1 | grep "width:"
echo ""

# 4. 检查 TF
echo "4️⃣ 检查 TF (map → odom)："
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -5
echo ""

# 5. 检查 RViz 配置文件
echo "5️⃣ 检查 RViz 配置文件："
RVIZ_CONFIG=$(ros2 pkg prefix localization_initializer)/share/localization_initializer/rviz/localization_init.rviz
if [ -f "$RVIZ_CONFIG" ]; then
    echo "✅ 配置文件存在: $RVIZ_CONFIG"
    echo "   Fixed Frame: $(grep "Fixed Frame:" $RVIZ_CONFIG | head -1)"
    echo "   Map Topic: $(grep -A 5 "Name: Map" $RVIZ_CONFIG | grep "Value:" | head -1)"
else
    echo "❌ 配置文件不存在"
fi
echo ""

# 6. 检查绿色点云（当前扫描）
echo "6️⃣ 检查当前扫描话题："
ros2 topic hz /cloud_registered 2>&1 | timeout 3 cat
echo ""

echo "════════════════════════════════════════════════════════════════"
echo "💡 建议操作："
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "如果看不到红色地图："
echo "  1. 在 RViz 中检查左侧 'Displays' → 'Map' 是否勾选 ✅"
echo "  2. 点击 'Map'，查看下方 'Status' 是否显示错误"
echo "  3. 尝试缩小视角（鼠标滚轮）或按 'r' 键重置视角"
echo "  4. 检查 'Global Options' → 'Fixed Frame' 是否为 'map'"
echo ""
echo "如果看不到绿色点云（当前扫描）："
echo "  1. 检查 small_point_lio 是否正常运行"
echo "  2. 在 RViz 中检查 'Current Scan' 是否勾选 ✅"
echo ""
echo "════════════════════════════════════════════════════════════════"
