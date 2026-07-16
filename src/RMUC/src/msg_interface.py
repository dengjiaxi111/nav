#!/usr/bin/env python3
# src/msg_interface.py - 适配新版 decision_messages 消息定义（国赛版）

import rclpy
from rclpy.node import Node
import os
import json
from datetime import datetime

from decision_messages.msg import EnemyRobotState, GameState, OurRobotState
from config import (
    RED_SENTRY_ROBOT_ID,
    get_enemy_team,
    get_our_team,
    is_blue_robot_id,
)

class MessageInterface(Node):
    def __init__(self, node_name='referee_simulator'):
        super().__init__(node_name)

        # 创建发布者
        self.enemy_pub = self.create_publisher(
            EnemyRobotState, '/decision_messages/EnemyRobotState', 10
        )
        self.game_state_pub = self.create_publisher(
            GameState, '/decision_messages/GameState', 10
        )
        self.our_robot_pub = self.create_publisher(
            OurRobotState, '/decision_messages/OurRobotState', 10
        )

        self.message_count = 0
        self.output_dir = "decision_messages"

        print(f"✓ ROS2消息接口已初始化，节点名: {node_name}")
        print("✓ 发布到话题: /decision_messages/EnemyRobotState, /decision_messages/GameState, /decision_messages/OurRobotState")

    def create_enemy_robot_state_msg(self, game_state):
        """
        创建 EnemyRobotState 消息
        严格按照 EnemyRobotState.msg 定义填充字段
        """
        msg = EnemyRobotState()
        enemy_robots = game_state.blue_robots if get_enemy_team(game_state.our_robot_id) == 'blue' else game_state.red_robots

        # 位置坐标 (cm)
        for robot_id in [1, 2, 3, 4, 7]:
            robot = enemy_robots.get(robot_id)
            x = float(robot.x) if robot else 0.0
            y = float(robot.y) if robot else 0.0
            hp = robot.hp if robot else 0
            allowance = robot.allowance if robot and robot_id in [1, 3, 4, 7] else 0

            if robot_id == 1:
                msg.enemy_hero_x = x
                msg.enemy_hero_y = y
                msg.enemy_hero_hp = hp
                msg.enemy_hero_allowance = allowance
            elif robot_id == 2:
                msg.enemy_engineer_x = x
                msg.enemy_engineer_y = y
                msg.enemy_engineer_hp = hp
            elif robot_id == 3:
                msg.enemy_infantry3_x = x
                msg.enemy_infantry3_y = y
                msg.enemy_infantry3_hp = hp
                msg.enemy_infantry3_allowance = allowance
            elif robot_id == 4:
                msg.enemy_infantry4_x = x
                msg.enemy_infantry4_y = y
                msg.enemy_infantry4_hp = hp
                msg.enemy_infantry4_allowance = allowance
            elif robot_id == 7:
                msg.enemy_sentry_x = x
                msg.enemy_sentry_y = y
                msg.enemy_sentry_hp = hp
                msg.enemy_sentry_allowance = allowance

        # 空中机器人（无，全部填0）
        msg.enemy_aerial_allowance = 0

        # 视觉锁敌辅助字段（无数据，填0）
        msg.base_yaw = 0.0
        msg.enemy_id = 0
        msg.enemy_x = 0.0
        msg.enemy_y = 0.0

        return msg

    def create_game_state_msg(self, game_state):
        """
        创建 GameState 消息
        严格按照 GameState.msg 定义填充字段
        """
        msg = GameState()
        msg.competition_type = game_state.competition_type
        msg.stage = game_state.stage
        msg.stage_remaining_time = float(game_state.stage_remaining_time)

        # 堡垒增益点占领状态
        msg.fortress_gain_point_occupation = game_state.fortress_gain_point_occupation

        # 基地打开状态：0正常，1红方基地打开，2蓝方基地打开（模拟器暂不使用，固定0）
        msg.baseopen = 0

        # 前哨站存活标志位：1存活，0不存活。该字段不再从前哨站血量推导。
        msg.outpost_alive = getattr(game_state, 'outpost_alive', 1)

        return msg

    def create_our_robot_state_msg(self, game_state):
        """
        创建 OurRobotState 消息（以哨兵机器人为“本机器人”）
        严格按照 OurRobotState.msg 定义填充字段
        """
        msg = OurRobotState()

        # 根据本机器人ID选择己方队伍：7=红方，107=蓝方，其他默认红方
        if get_our_team(game_state.our_robot_id) == 'red':
            our_robots = game_state.red_robots
            msg.outpost_hp = game_state.red_outpost_hp
            msg.base_hp = game_state.red_base_hp
            msg.remaining_gold_coins = game_state.red_gold_coins
            msg.reserve_allowance_17mm = game_state.red_reserve_allowance_17mm
        else:
            our_robots = game_state.blue_robots
            msg.outpost_hp = game_state.blue_outpost_hp
            msg.base_hp = game_state.blue_base_hp
            msg.remaining_gold_coins = game_state.blue_gold_coins
            msg.reserve_allowance_17mm = game_state.blue_reserve_allowance_17mm

        # 本机器人（哨兵）数据
        sentry = our_robots.get(7)
        if sentry:
            msg.robot_id = int(game_state.our_robot_id)
            msg.current_hp = sentry.hp
            msg.max_hp = sentry.max_hp
            # 位置: cm -> m
            msg.x = float(sentry.x) / 100.0
            msg.y = float(sentry.y) / 100.0
            msg.yaw = sentry.yaw
            # 增益 buff
            msg.hp_recovery_buff = sentry.hp_recovery_buff
            msg.defense_buff = sentry.defense_buff
            msg.negative_defense_buff = sentry.negative_defense_buff
            msg.attack_buff = sentry.attack_buff
            msg.allowance_17mm = sentry.allowance
            msg.rfid_status = sentry.rfid_status
        else:
            # 哨兵不存在（不太可能），填默认值
            msg.robot_id = RED_SENTRY_ROBOT_ID
            msg.current_hp = 0
            msg.max_hp = 400
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.hp_recovery_buff = 0.0
            msg.defense_buff = 0.0
            msg.negative_defense_buff = 0.0
            msg.attack_buff = 0.0
            msg.allowance_17mm = 0
            msg.rfid_status = 0

        # 其他己方机器人位置 (m)
        for rid in [1, 2, 3, 4]:
            robot = our_robots.get(rid)
            x = float(robot.x) / 100.0 if robot else 0.0
            y = float(robot.y) / 100.0 if robot else 0.0
            if rid == 1:
                msg.hero_x = x
                msg.hero_y = y
            elif rid == 2:
                msg.engineer_x = x
                msg.engineer_y = y
            elif rid == 3:
                msg.infantry3_x = x
                msg.infantry3_y = y
            elif rid == 4:
                msg.infantry4_x = x
                msg.infantry4_y = y

        return msg

    def update_messages(self, game_state):
        """发布所有消息到 ROS2 话题"""
        self.message_count += 1

        try:
            enemy_msg = self.create_enemy_robot_state_msg(game_state)
            game_msg = self.create_game_state_msg(game_state)
            our_msg = self.create_our_robot_state_msg(game_state)

            self.enemy_pub.publish(enemy_msg)
            self.game_state_pub.publish(game_msg)
            self.our_robot_pub.publish(our_msg)

            # 可选调试输出（频率较高时可注释掉）
            # if self.message_count % 10 == 0:
            #     print(f"[{datetime.now().strftime('%H:%M:%S')}] ROS2消息已发布 #{self.message_count}")

        except Exception as e:
            print(f"发布消息时出错: {e}")
            import traceback
            traceback.print_exc()
