#!/bin/bash
# 台阶检测参数调整工具 - 快速启动脚本

echo "================================"
echo "台阶检测参数调整工具"
echo "================================"
echo ""
echo "使用步骤："
echo "1. 确保导航系统已启动: ros2 launch nav_bringup run.launch.py"
echo "2. 运行本脚本启动 GUI"
echo ""
echo "按 Ctrl+C 退出"
echo "================================"
echo ""

cd $(dirname $0)/../../../..
source install/setup.bash

python3 src/mapping/rog_map_ros2_node/scripts/stair_tuner_gui.py
