#!/usr/bin/env python3
# src/msg_interface.py - 省赛版本，始终使用红方哨兵数据，但robot_id可自定义

import rclpy
from rclpy.node import Node
import json
import os
from decision_messages.msg import GameState, OurRobotState

class MessageInterface(Node):
    def __init__(self):
        super().__init__('referee_simulator')
        self.game_state_pub = self.create_publisher(
            GameState, '/decision_messages/GameState', 10
        )
        self.our_robot_pub = self.create_publisher(
            OurRobotState, '/decision_messages/OurRobotState', 10
        )
        self.message_count = 0
        self.output_dir = "decision_messages"
        print("✓ ROS2消息接口（精简版）已初始化")
        print("✓ 发布到话题: /decision_messages/GameState, /decision_messages/OurRobotState")

    def create_game_state_msg(self, game_state):
        """创建GameState消息 - 只包含精简后的字段"""
        msg = GameState()
        msg.competition_type = game_state.competition_type
        msg.stage = game_state.stage
        msg.stage_remaining_time = float(game_state.stage_remaining_time)
        return msg

    def create_our_robot_state_msg(self, game_state):
        """创建OurRobotState消息 - 始终使用红方哨兵数据，但robot_id可自定义"""
        msg = OurRobotState()
        # 使用控制面板设置的机器人ID（0=红方，1=蓝方），但数据始终来自红方
        msg.robot_id = game_state.our_robot_id

        # 固定使用红方哨兵的数据
        our_robots = game_state.red_robots
        robot_id = 7  # 哨兵
        robot = our_robots.get(robot_id)

        if robot:
            msg.current_hp = robot.hp
            from config import DEFAULT_HP
            msg.max_hp = DEFAULT_HP.get(robot_id, 400)

            # 位置数据（单位：m）- 使用哨兵机器人的实时位置
            msg.x = float(robot.x) / 100.0
            msg.y = float(robot.y) / 100.0
            msg.yaw = 0.0  # 若无测速模块，保持默认0

            # 允许发弹量
            msg.allowance_17mm = robot.allowance
        else:
            # 如果没有机器人，设置默认值
            msg.current_hp = 0
            msg.max_hp = 400
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.allowance_17mm = 0

        return msg

    def update_messages(self, game_state):
        """发布所有消息到ROS2话题"""
        self.message_count += 1

        try:
            game_msg = self.create_game_state_msg(game_state)
            our_msg = self.create_our_robot_state_msg(game_state)

            self.game_state_pub.publish(game_msg)
            self.our_robot_pub.publish(our_msg)

            self.save_to_file(game_state)

        except Exception as e:
            print(f"发布消息时出错: {e}")
            import traceback
            traceback.print_exc()

    def save_to_file(self, game_state):
        """保存状态到文件（用于调试和UI同步）"""
        try:
            status = {
                'game_state': {
                    'stage': game_state.stage,
                    'stage_remaining_time': game_state.stage_remaining_time,
                    'is_running': game_state.is_running,
                    'competition_type': game_state.competition_type,
                },
                'red_robots': {},
                'blue_robots': {},
                'our_robot_id': game_state.our_robot_id
            }

            # 保存机器人位置
            for robot_id, robot in game_state.red_robots.items():
                status['red_robots'][str(robot_id)] = {
                    'x': robot.x,
                    'y': robot.y,
                    'hp': robot.hp,
                    'allowance': robot.allowance
                }

            for robot_id, robot in game_state.blue_robots.items():
                status['blue_robots'][str(robot_id)] = {
                    'x': robot.x,
                    'y': robot.y,
                    'hp': robot.hp,
                    'allowance': robot.allowance
                }

            with open(os.path.join(self.output_dir, "status.json"), "w", encoding='utf-8') as f:
                json.dump(status, f, indent=2, ensure_ascii=False)

        except Exception as e:
            print(f"保存状态到文件时出错: {e}")
