#!/usr/bin/env python3
# src/msg_interface.py - 修改为ROS2节点，同时发布消息到话题和更新文件

import rclpy
from rclpy.node import Node
import json
import os
import time
from datetime import datetime

# 导入ROS2消息类型
from decision_messages.msg import EnemyRobotState, GameState, OurRobotState

class MessageInterface(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('referee_simulator')
        
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
        
        # 创建定时器，定期发布消息
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        print("✓ ROS2消息接口已初始化")
        print("✓ 发布到话题: /decision_messages/EnemyRobotState, /decision_messages/GameState, /decision_messages/OurRobotState")
    
    def timer_callback(self):
        """定时器回调，定期发布消息"""
        pass
    
    def create_enemy_robot_state_msg(self, game_state):
        """创建EnemyRobotState消息（蓝方作为敌方）"""
        msg = EnemyRobotState()
        
        # 蓝方作为敌方
        enemy_robots = game_state.blue_robots
        
        # 位置坐标（单位：cm）转换为float32
        for robot_id in [1, 2, 3, 4, 7]:
            robot = enemy_robots.get(robot_id)
            
            if robot:
                x, y = float(robot.x), float(robot.y)
            else:
                x, y = 0.0, 0.0
            
            if robot_id == 1:
                msg.enemy_hero_x = x
                msg.enemy_hero_y = y
            elif robot_id == 2:
                msg.enemy_engineer_x = x
                msg.enemy_engineer_y = y
            elif robot_id == 3:
                msg.enemy_infantry3_x = x
                msg.enemy_infantry3_y = y
            elif robot_id == 4:
                msg.enemy_infantry4_x = x
                msg.enemy_infantry4_y = y
            elif robot_id == 7:
                msg.enemy_sentry_x = x
                msg.enemy_sentry_y = y
        
        # 空中机器人位置设为0
        msg.enemy_aerial_x = 0.0
        msg.enemy_aerial_y = 0.0
        
        # 血量信息
        for robot_id in [1, 2, 3, 4, 7]:
            robot = enemy_robots.get(robot_id)
            hp = robot.hp if robot else 0
            
            if robot_id == 1:
                msg.enemy_hero_hp = hp
            elif robot_id == 2:
                msg.enemy_engineer_hp = hp
            elif robot_id == 3:
                msg.enemy_infantry3_hp = hp
            elif robot_id == 4:
                msg.enemy_infantry4_hp = hp
            elif robot_id == 7:
                msg.enemy_sentry_hp = hp
        
        # 发弹量信息
        for robot_id in [1, 3, 4, 7]:
            robot = enemy_robots.get(robot_id)
            allowance = robot.allowance if robot else 0
            
            if robot_id == 1:
                msg.enemy_hero_allowance = allowance
            elif robot_id == 3:
                msg.enemy_infantry3_allowance = allowance
            elif robot_id == 4:
                msg.enemy_infantry4_allowance = allowance
            elif robot_id == 7:
                msg.enemy_sentry_allowance = allowance
        
        # 空中机器人发弹量
        msg.enemy_aerial_allowance = 0
        
        # 经济状态
        msg.enemy_remaining_gold_coins = game_state.blue_gold_coins
        msg.enemy_total_gold_coins = game_state.blue_total_gold_coins
        
        # 占领状态
        msg.enemy_supply_zone_occupation = game_state.enemy_supply_zone_occupation
        msg.enemy_central_highland_occupation = game_state.enemy_central_highland_occupation
        msg.enemy_trapezoid_highland_occupation = game_state.enemy_trapezoid_highland_occupation
        msg.enemy_fortress_gain_point_occupation = game_state.enemy_fortress_gain_point_occupation
        msg.enemy_outpost_gain_point_occupation = game_state.enemy_outpost_gain_point_occupation
        msg.enemy_base_gain_point_occupation = game_state.enemy_base_gain_point_occupation
        
        # 模块卡状态（使用默认值）
        msg.enemy_near_tunnel_before_ramp_card = 0
        msg.enemy_near_tunnel_after_ramp_card = 0
        msg.own_near_tunnel_before_ramp_card = 0
        msg.own_near_tunnel_after_ramp_card = 0
        msg.enemy_highland_upper_card = 0
        msg.enemy_ramp_upper_card = 0
        msg.enemy_road_upper_card = 0
        
        # 增益效果 - 设置为0
        msg.enemy_hero_hp_recovery_buff = 0.0
        msg.enemy_hero_heat_cooling_buff = 0.0
        msg.enemy_hero_defense_buff = 0.0
        msg.enemy_hero_negative_defense_buff = 0.0
        msg.enemy_hero_attack_buff = 0.0
        
        msg.enemy_engineer_hp_recovery_buff = 0.0
        msg.enemy_engineer_heat_cooling_buff = 0.0
        msg.enemy_engineer_defense_buff = 0.0
        msg.enemy_engineer_negative_defense_buff = 0.0
        msg.enemy_engineer_attack_buff = 0.0
        
        msg.enemy_infantry3_hp_recovery_buff = 0.0
        msg.enemy_infantry3_heat_cooling_buff = 0.0
        msg.enemy_infantry3_defense_buff = 0.0
        msg.enemy_infantry3_negative_defense_buff = 0.0
        msg.enemy_infantry3_attack_buff = 0.0
        
        msg.enemy_infantry4_hp_recovery_buff = 0.0
        msg.enemy_infantry4_heat_cooling_buff = 0.0
        msg.enemy_infantry4_defense_buff = 0.0
        msg.enemy_infantry4_negative_defense_buff = 0.0
        msg.enemy_infantry4_attack_buff = 0.0
        
        msg.enemy_sentry_hp_recovery_buff = 0.0
        msg.enemy_sentry_heat_cooling_buff = 0.0
        msg.enemy_sentry_defense_buff = 0.0
        msg.enemy_sentry_negative_defense_buff = 0.0
        msg.enemy_sentry_attack_buff = 0.0
        
        # 对方哨兵机器人当前姿态
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
        
        # 哨兵自主决策信息同步
        msg.exchanged_allowance = game_state.exchanged_allowance
        msg.free_resurrection_available = game_state.free_resurrection_available
        msg.sentry_posture = game_state.red_sentry_posture
        msg.energy_mechanism_activatable = game_state.energy_mechanism_activatable
        
        return msg
    
    def create_our_robot_state_msg(self, game_state):
        """创建OurRobotState消息（红方作为己方）"""
        msg = OurRobotState()
        
        # 红方作为己方
        our_robots = game_state.red_robots
        
        # 血量信息
        for rid in [1, 2, 3, 4, 7]:
            robot = our_robots.get(rid)
            hp = robot.hp if robot else 0
            
            if rid == 1:
                msg.hero_hp = hp
            elif rid == 2:
                msg.engineer_hp = hp
            elif rid == 3:
                msg.infantry3_hp = hp
            elif rid == 4:
                msg.infantry4_hp = hp
            elif rid == 7:
                msg.sentry_hp = hp
        
        msg.outpost_hp = game_state.red_outpost_hp
        msg.base_hp = game_state.red_base_hp
        
        # 机器人特定数据（使用ID为7的哨兵机器人作为本机器人）
        robot_id = 7  # 使用哨兵机器人作为本机器人
        robot = our_robots.get(robot_id)
        
        if robot:
            msg.robot_id = robot.robot_id
            msg.current_hp = robot.hp
            # 使用哨兵血量上限
            from config import DEFAULT_HP
            max_hp = DEFAULT_HP.get(robot_id, 400)
            msg.max_hp = max_hp
            
            # 位置数据（单位：m）- 使用哨兵机器人的实时位置
            msg.x = float(robot.x) / 100.0
            msg.y = float(robot.y) / 100.0
            msg.yaw = 0.0
            
            # 增益数据 - 使用哨兵机器人的增益数据
            msg.hp_recovery_buff = float(robot.hp_recovery_buff)
            msg.defense_buff = float(robot.defense_buff)
            msg.negative_defense_buff = float(robot.negative_defense_buff)
            msg.attack_buff = float(robot.attack_buff)
            
            # 允许发弹量 - 使用哨兵机器人的实时发弹量
            msg.allowance_17mm = robot.allowance
        else:
            # 如果没有机器人，设置默认值
            msg.robot_id = 7
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
        
        msg.remaining_gold_coins = game_state.red_gold_coins
        msg.reserve_allowance_17mm = game_state.red_reserve_allowance_17mm
        
        # RFID模块状态 - 使用哨兵机器人的RFID状态
        if robot:
            msg.rfid_status = robot.rfid_status
        else:
            msg.rfid_status = 0
        
        # 所有机器人位置（单位：米）
        for rid in [1, 2, 3, 4]:
            robot = our_robots.get(rid)
            if robot:
                x = float(robot.x) / 100.0
                y = float(robot.y) / 100.0
            else:
                x, y = 0.0, 0.0
            
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
        """发布所有消息到ROS2话题"""
        self.message_count += 1
        
        try:
            # 创建消息
            enemy_msg = self.create_enemy_robot_state_msg(game_state)
            game_msg = self.create_game_state_msg(game_state)
            our_msg = self.create_our_robot_state_msg(game_state)
            
            # 发布消息
            self.enemy_pub.publish(enemy_msg)
            self.game_state_pub.publish(game_msg)
            self.our_robot_pub.publish(our_msg)
            
            # 同时保存到文件（用于调试和UI同步）
            self.save_to_file(game_state)
            
            # print(f"[{datetime.now().strftime('%H:%M:%S')}] ROS2消息已发布 #{self.message_count}")
            
        except Exception as e:
            print(f"发布消息时出错: {e}")
            import traceback
            traceback.print_exc()
    
    def save_to_file(self, game_state):
        """保存状态到文件（保持向后兼容）"""
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
                }
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
