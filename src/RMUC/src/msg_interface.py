#!/usr/bin/env python3
# src/msg_interface.py - 精简版，只保留实际存在的消息字段

import rclpy
from rclpy.node import Node
import json
import os
from datetime import datetime

from decision_messages.msg import EnemyRobotState, GameState, OurRobotState

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
        """创建EnemyRobotState消息（蓝方作为敌方）"""
        msg = EnemyRobotState()
        
        # 蓝方机器人数据
        enemy_robots = game_state.blue_robots
        
        for robot_id in [1, 2, 3, 4, 7]:
            robot = enemy_robots.get(robot_id)
            x = float(robot.x) if robot else 0.0
            y = float(robot.y) if robot else 0.0
            hp = robot.hp if robot else 0
            allowance = robot.allowance if robot and robot_id in [1,3,4,7] else 0
            
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
        
        # 经济状态
        msg.enemy_remaining_gold_coins = game_state.blue_gold_coins
        msg.enemy_total_gold_coins = game_state.blue_total_gold_coins
        
        # 占领状态（蓝方视角）
        msg.enemy_supply_zone_occupation = game_state.enemy_supply_zone_occupation
        msg.enemy_central_highland_occupation = game_state.enemy_central_highland_occupation
        msg.enemy_trapezoid_highland_occupation = game_state.enemy_trapezoid_highland_occupation
        msg.enemy_fortress_gain_point_occupation = game_state.enemy_fortress_gain_point_occupation
        msg.enemy_outpost_gain_point_occupation = game_state.enemy_outpost_gain_point_occupation
        msg.enemy_base_gain_point_occupation = game_state.enemy_base_gain_point_occupation
        
        # 敌方哨兵姿态
        msg.enemy_sentry_posture = game_state.blue_sentry_posture
        
        return msg
    
    def create_game_state_msg(self, game_state):
        """创建GameState消息"""
        msg = GameState()
        msg.competition_type = game_state.competition_type
        msg.stage = game_state.stage
        msg.stage_remaining_time = float(game_state.stage_remaining_time)
        
        # 场地状态
        msg.supply_zone_no_overlap = game_state.supply_zone_no_overlap
        msg.supply_zone_overlap = game_state.supply_zone_overlap
        msg.supply_zone_occupation = game_state.supply_zone_occupation
        msg.energy_mechanism_status = game_state.energy_mechanism_status
        msg.small_energy_mechanism_activation = game_state.small_energy_mechanism_activation
        msg.large_energy_mechanism_activation = game_state.large_energy_mechanism_activation
        msg.central_highland_occupation = game_state.central_highland_occupation
        msg.trapezoid_highland_occupation = game_state.trapezoid_highland_occupation
        msg.dart_hit_time = game_state.dart_hit_time
        msg.dart_hit_target = game_state.dart_hit_target
        msg.center_gain_point_occupation = game_state.center_gain_point_occupation
        msg.fortress_gain_point_occupation = game_state.fortress_gain_point_occupation
        msg.outpost_gain_point_occupation = game_state.outpost_gain_point_occupation
        msg.base_gain_point_occupation = game_state.base_gain_point_occupation
        
        # 哨兵自主决策信息
        msg.exchanged_allowance = game_state.exchanged_allowance
        msg.free_resurrection_available = game_state.free_resurrection_available
        msg.sentry_posture = game_state.red_sentry_posture
        msg.energy_mechanism_activatable = game_state.energy_mechanism_activatable
        
        return msg
    
    def create_our_robot_state_msg(self, game_state):
        """创建OurRobotState消息"""
        msg = OurRobotState()
        msg.robot_id = game_state.our_robot_id
        
        # 根据ID选择己方队伍
        if msg.robot_id == 0:
            our_robots = game_state.red_robots
        else:
            our_robots = game_state.blue_robots
        
        robot = our_robots.get(7)  # 哨兵
        
        if robot:
            msg.current_hp = robot.hp
            from config import DEFAULT_HP
            msg.max_hp = DEFAULT_HP.get(7, 400)
            msg.x = float(robot.x) / 100.0
            msg.y = float(robot.y) / 100.0
            msg.yaw = 0.0
            from config import DEFAULT_ALLOWANCE
            msg.allowance_17mm = robot.allowance if 7 in DEFAULT_ALLOWANCE else 0
        else:
            msg.current_hp = 0
            msg.max_hp = 400
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.allowance_17mm = 0
        
        # 己方前哨站和基地血量
        if msg.robot_id == 0:
            msg.outpost_hp = game_state.red_outpost_hp
            msg.base_hp = game_state.red_base_hp
            msg.remaining_gold_coins = game_state.red_gold_coins
            msg.reserve_allowance_17mm = game_state.red_reserve_allowance_17mm
            # 己方其他机器人位置
            for rid in [1,2,3,4]:
                r = game_state.red_robots.get(rid)
                x = float(r.x)/100.0 if r else 0.0
                y = float(r.y)/100.0 if r else 0.0
                if rid == 1:
                    msg.hero_x, msg.hero_y = x, y
                elif rid == 2:
                    msg.engineer_x, msg.engineer_y = x, y
                elif rid == 3:
                    msg.infantry3_x, msg.infantry3_y = x, y
                elif rid == 4:
                    msg.infantry4_x, msg.infantry4_y = x, y
        else:
            msg.outpost_hp = game_state.blue_outpost_hp
            msg.base_hp = game_state.blue_base_hp
            msg.remaining_gold_coins = game_state.blue_gold_coins
            msg.reserve_allowance_17mm = game_state.blue_reserve_allowance_17mm
            for rid in [1,2,3,4]:
                r = game_state.blue_robots.get(rid)
                x = float(r.x)/100.0 if r else 0.0
                y = float(r.y)/100.0 if r else 0.0
                if rid == 1:
                    msg.hero_x, msg.hero_y = x, y
                elif rid == 2:
                    msg.engineer_x, msg.engineer_y = x, y
                elif rid == 3:
                    msg.infantry3_x, msg.infantry3_y = x, y
                elif rid == 4:
                    msg.infantry4_x, msg.infantry4_y = x, y
        
        # 本机器人增益buff（如果消息中没有这些字段，注释掉）
        # msg.hp_recovery_buff = 0.0
        # msg.defense_buff = 0.0
        # msg.negative_defense_buff = 0.0
        # msg.attack_buff = 0.0
        
        msg.rfid_status = 0
        
        return msg
    
    def update_messages(self, game_state):
        """发布所有消息到ROS2话题"""
        self.message_count += 1
        
        try:
            enemy_msg = self.create_enemy_robot_state_msg(game_state)
            game_msg = self.create_game_state_msg(game_state)
            our_msg = self.create_our_robot_state_msg(game_state)
            
            self.enemy_pub.publish(enemy_msg)
            self.game_state_pub.publish(game_msg)
            self.our_robot_pub.publish(our_msg)
            
            # 保存到文件（用于调试）
            self.save_to_file(game_state)
            
            # 可选打印频率
            # if self.message_count % 10 == 0:
            #     print(f"[{datetime.now().strftime('%H:%M:%S')}] ROS2消息已发布 #{self.message_count}")
            
        except Exception as e:
            print(f"发布消息时出错: {e}")
            import traceback
            traceback.print_exc()
    
    def save_to_file(self, game_state):
        """保存状态到文件"""
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
                'gain_zone_occupation': {
                    'supply_zone_occupation': game_state.supply_zone_occupation,
                    'central_highland_occupation': game_state.central_highland_occupation,
                    'trapezoid_highland_occupation': game_state.trapezoid_highland_occupation,
                    'fortress_gain_point_occupation': game_state.fortress_gain_point_occupation,
                    'outpost_gain_point_occupation': game_state.outpost_gain_point_occupation,
                    'base_gain_point_occupation': game_state.base_gain_point_occupation
                },
                'our_robot_id': game_state.our_robot_id
            }
            
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
            
            os.makedirs(self.output_dir, exist_ok=True)
            with open(os.path.join(self.output_dir, "status.json"), "w", encoding='utf-8') as f:
                json.dump(status, f, indent=2, ensure_ascii=False)
                
        except Exception as e:
            print(f"保存状态到文件时出错: {e}")
