#!/usr/bin/env python3
# src/main.py - 简化的启动脚本
import sys
import os
import time
import subprocess

# 添加工作空间的Python包路径
def setup_python_path():
    """设置Python路径以包含ROS2消息包"""
    # 获取当前工作目录
    workspace_path = os.path.dirname(os.path.abspath(__file__))
    
    # 可能的Python包路径
    possible_paths = [
        # install目录中的Python包
        os.path.join(workspace_path, "..", "install", "decision_messages", "local", "lib", "python3.10", "dist-packages"),
        os.path.join(workspace_path, "..", "install", "decision_messages", "lib", "python3.10", "site-packages"),
        # build目录中的Python包
        os.path.join(workspace_path, "..", "build", "decision_messages", "rosidl_generator_py"),
        # sentry_decision包的Python路径
        os.path.join(workspace_path, "..", "install", "sentry_decision", "local", "lib", "python3.10", "dist-packages"),
        os.path.join(workspace_path, "..", "install", "sentry_decision", "lib", "python3.10", "site-packages"),
    ]
    
    # 添加存在的路径到sys.path
    added = False
    for path in possible_paths:
        path = os.path.abspath(path)
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"✓ 添加Python路径: {path}")
            added = True
    
    if not added:
        print("⚠ 警告: 未找到ROS2消息包的Python路径")
        print("请确保已构建工作空间: colcon build --packages-select decision_messages sentry_decision")
    
    return added

def show_help():
    print("=" * 60)
    print("RoboMaster Referee System Simulator")
    print("=" * 60)
    print("使用方法:")
    print("  1. python3 src/main.py map        启动地图界面和ROS2节点")
    print("  2. python3 src/main.py control    启动控制面板")
    print("  3. python3 src/main.py both       同时启动两个界面")
    print("")
    print("注意：")
    print("  - 地图界面将作为ROS2节点发布消息并订阅决策系统消息")
    print("  - 请确保已构建ROS2工作空间: colcon build")
    print("  - 决策系统可以订阅话题: game_state, our_robot_state")
    print("  - 模拟器将订阅话题: /sentry/target_position, /sentry/control")
    print("")
    print("运行前请先source ROS2环境:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source install/setup.bash")
    print("=" * 60)

if __name__ == "__main__":
    # 首先设置Python路径
    setup_python_path()
    
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        
        if mode == "map":
            print("启动地图界面和ROS2节点...")
            
            # 尝试导入ROS2模块
            try:
                import rclpy
                print("✓ 已导入rclpy")
            except ImportError as e:
                print(f"错误: 无法导入rclpy: {e}")
                print("请确保已安装ROS2并source环境:")
                print("  source /opt/ros/humble/setup.bash")
                sys.exit(1)
            
            try:
                from decision_messages.msg import GameState, OurRobotState
                print("✓ 已导入ROS2消息类型")
            except ImportError as e:
                print(f"错误: 无法导入ROS2消息类型: {e}")
                print("请确保已构建decision_messages包:")
                print("  colcon build --packages-select decision_messages")
                print("  source install/setup.bash")
                sys.exit(1)
            
            # 修复：String从std_msgs导入，不是从geometry_msgs
            try:
                from geometry_msgs.msg import PointStamped
                from std_msgs.msg import String  # 修正：String从std_msgs导入
                from sentry_decision.msg import SentryControl
                print("✓ 已导入决策系统消息类型")
            except ImportError as e:
                print(f"错误: 无法导入决策系统消息类型: {e}")
                print("请确保已构建sentry_decision包:")
                print("  colcon build --packages-select sentry_decision")
                print("  source install/setup.bash")
                sys.exit(1)
            
            from map_ui import MapUI
            from game_state import GameState as GameStateClass
            from msg_interface import MessageInterface
            from sentry_controller import SentryController
            
            # 初始化ROS2
            rclpy.init()
            print("✓ ROS2已初始化")
            
            # 创建游戏状态
            game_state = GameStateClass()
            
            # 创建消息接口
            msg_interface = MessageInterface()
            
            # 创建哨兵控制器
            sentry_controller = SentryController(game_state)
            
            # 设置消息接口和哨兵控制器
            game_state.set_msg_interface(msg_interface)
            game_state.set_sentry_controller(sentry_controller)
            
            # 创建地图UI
            map_ui = MapUI(game_state, msg_interface)
            # 将哨兵控制器传递给地图UI
            map_ui.sentry_controller = sentry_controller
            
            try:
                map_ui.run()
            except KeyboardInterrupt:
                print("收到中断信号，正在关闭...")
            finally:
                sentry_controller.destroy_node()
                msg_interface.destroy_node()
                rclpy.shutdown()
            
        elif mode == "control":
            print("启动控制面板...")
            from control_panel import ControlPanel
            from game_state import GameState
            # 控制面板只负责文件同步，不发布ROS2消息
            msg_interface = None
            game_state = GameState()
            # 不设置msg_interface
            control_panel = ControlPanel(game_state, msg_interface)
            control_panel.run()
            
        elif mode == "both":
            print("同时启动两个界面...")
            
            # 首先启动地图界面
            map_process = subprocess.Popen([sys.executable, __file__, "map"])
            print("地图界面（带ROS2节点）已启动")
            
            # 等待3秒让地图界面初始化
            time.sleep(3)
            
            # 启动控制面板
            control_process = subprocess.Popen([sys.executable, __file__, "control"])
            print("控制面板已启动")
            
            print("\n两个界面已启动，请分别操作:")
            print("  - 地图界面: ROS2节点，发布消息到话题并订阅决策系统消息")
            print("  - 控制面板: 可修改参数，状态通过文件同步")
            print("\n决策系统需要订阅以下话题:")
            print("  - game_state")
            print("  - our_robot_state")
            print("\n决策系统需要发布以下话题:")
            print("  - /sentry/target_position (目标点)")
            print("  - /sentry/control (控制消息)")
            print("  - /sentry/debug (调试信息，可选)")
            print("\n按Ctrl+C关闭两个界面")
            
            try:
                map_process.wait()
                control_process.wait()
            except KeyboardInterrupt:
                print("\n正在关闭界面...")
                map_process.terminate()
                control_process.terminate()
                
        else:
            show_help()
    else:
        show_help()
