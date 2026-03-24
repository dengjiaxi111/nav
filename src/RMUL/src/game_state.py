# src/game_state.py - 省赛版本（坐标已更新，无增益区）

import time
import json
import os
from datetime import datetime
from config import *
from robots import Robot

class GameState:
    def __init__(self):
        # 游戏基本状态
        self.stage = GameStage.NOT_STARTED
        self.stage_remaining_time = GAME_TIME
        self.start_time = None
        self.is_running = False
        self.competition_type = CompetitionType.SUPER_LEAGUE

        # 机器人初始化
        self.red_robots = {}
        self.blue_robots = {}
        self.initialize_robots()

        # 场地状态 - 血量
        self.red_outpost_hp = OUTPOST_HP
        self.blue_outpost_hp = OUTPOST_HP
        self.red_base_hp = BASE_HP
        self.blue_base_hp = BASE_HP

        # 经济系统
        self.red_gold_coins = GOLD_COINS
        self.blue_gold_coins = GOLD_COINS
        self.red_total_gold_coins = 0
        self.blue_total_gold_coins = 0

        # 哨兵状态
        self.exchanged_allowance = 0
        self.free_resurrection_available = 0
        self.red_sentry_posture = SENTRY_DEFENSE
        self.blue_sentry_posture = SENTRY_DEFENSE

        # 储备允许发弹量
        self.red_reserve_allowance_17mm = 0
        self.blue_reserve_allowance_17mm = 0

        # ROS2消息接口
        self.msg_interface = None

        # 哨兵控制器引用
        self.sentry_controller = None

        # 新增：自定义机器人ID（0=红方，1=蓝方）
        self.our_robot_id = 0

    def set_msg_interface(self, msg_interface):
        self.msg_interface = msg_interface

    def set_sentry_controller(self, sentry_controller):
        self.sentry_controller = sentry_controller

    def initialize_robots(self):
        """初始化机器人位置（使用新坐标）"""
        # 红色方
        red_positions = {
            1: (53, 639),
            3: (126, 678),
            7: (120, 634)
        }

        # 蓝色方
        blue_positions = {
            1: (1131, 176),
            3: (1075, 108),
            7: (1074, 172)
        }

        # 创建红色机器人
        for robot_id in ROBOT_IDS:
            x, y = red_positions[robot_id]
            robot = Robot(robot_id, 'red', x, y)
            robot.hp = DEFAULT_HP.get(robot_id, 100)
            if robot_id in DEFAULT_ALLOWANCE:
                robot.allowance = DEFAULT_ALLOWANCE[robot_id]
            self.red_robots[robot_id] = robot

        # 创建蓝色机器人
        for robot_id in ROBOT_IDS:
            x, y = blue_positions[robot_id]
            robot = Robot(robot_id, 'blue', x, y)
            robot.hp = DEFAULT_HP.get(robot_id, 100)
            if robot_id in DEFAULT_ALLOWANCE:
                robot.allowance = DEFAULT_ALLOWANCE[robot_id]
            self.blue_robots[robot_id] = robot

    def reset_robot_position(self, robot_id, team, x, y):
        robot = self.get_robot(team, robot_id)
        if robot:
            robot.x = x
            robot.y = y
            robot.hp = DEFAULT_HP.get(robot_id, 100)
            if robot_id in DEFAULT_ALLOWANCE:
                robot.allowance = DEFAULT_ALLOWANCE[robot_id]
            robot.has_target = False
            robot.is_moving = False
            robot.target_x = x
            robot.target_y = y

    def start_game(self):
        if self.stage in [GameStage.NOT_STARTED, GameStage.COUNTDOWN]:
            self.stage = GameStage.IN_PROGRESS
            self.start_time = time.time()
            self.is_running = True
            self.save_full_status_to_file()
            self.update_ros2_messages()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 比赛开始")

    def reset_game(self):
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 开始重置比赛...")

        self.stage = GameStage.NOT_STARTED
        self.stage_remaining_time = GAME_TIME
        self.start_time = None
        self.is_running = False

        # 重置位置（使用新坐标）
        red_positions = {
            1: (53, 639),
            3: (126, 678),
            7: (120, 634)
        }
        blue_positions = {
            1: (1131, 176),
            3: (1075, 108),
            7: (1074, 172)
        }

        for robot_id in ROBOT_IDS:
            self.reset_robot_position(robot_id, 'red', *red_positions[robot_id])
            self.reset_robot_position(robot_id, 'blue', *blue_positions[robot_id])

        self.red_outpost_hp = OUTPOST_HP
        self.blue_outpost_hp = OUTPOST_HP
        self.red_base_hp = BASE_HP
        self.blue_base_hp = BASE_HP
        self.red_gold_coins = GOLD_COINS
        self.blue_gold_coins = GOLD_COINS
        self.red_total_gold_coins = 0
        self.blue_total_gold_coins = 0

        self.exchanged_allowance = 0
        self.free_resurrection_available = 0
        self.red_sentry_posture = SENTRY_DEFENSE
        self.blue_sentry_posture = SENTRY_DEFENSE
        self.red_reserve_allowance_17mm = 0
        self.blue_reserve_allowance_17mm = 0

        # 重置自定义ID为默认值0
        self.our_robot_id = 0

        if self.sentry_controller:
            self.sentry_controller.sentry_robot = self.red_robots.get(7)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 已更新哨兵控制器引用")

        self.save_full_status_to_file()
        self.update_ros2_messages()
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 比赛已重置")

    def update_timer(self):
        if self.is_running and self.start_time:
            elapsed = time.time() - self.start_time
            self.stage_remaining_time = max(0, GAME_TIME - elapsed)
            if self.stage_remaining_time <= 0:
                self.is_running = False
                self.stage = GameStage.SETTLEMENT
                self.save_full_status_to_file()
                self.update_ros2_messages()
                print(f"[{datetime.now().strftime('%H:%M:%S')}] 比赛结束")

    def get_all_robots(self):
        all_robots = []
        for robot in self.red_robots.values():
            all_robots.append(robot)
        for robot in self.blue_robots.values():
            all_robots.append(robot)
        return all_robots

    def get_robot(self, team, robot_id):
        if team == 'red':
            return self.red_robots.get(robot_id)
        else:
            return self.blue_robots.get(robot_id)

    def update_robot_hp(self, team, robot_id, hp):
        robot = self.get_robot(team, robot_id)
        if robot:
            robot.hp = max(0, min(int(hp), 2000))
            self.save_full_status_to_file()
            self.update_ros2_messages()

    def update_robot_allowance(self, team, robot_id, allowance):
        robot = self.get_robot(team, robot_id)
        if robot and robot_id in DEFAULT_ALLOWANCE:
            robot.allowance = max(0, int(allowance))
            self.save_full_status_to_file()
            self.update_ros2_messages()

    def update_field(self, field, value):
        if hasattr(self, field):
            setattr(self, field, int(value))
            self.save_full_status_to_file()
            self.update_ros2_messages()

    def update_ros2_messages(self):
        if self.msg_interface:
            self.msg_interface.update_messages(self)

    def save_status_to_file(self):
        try:
            existing_status = {}
            status_file = os.path.join("decision_messages", "status.json")
            if os.path.exists(status_file):
                with open(status_file, "r", encoding='utf-8') as f:
                    existing_status = json.load(f)
            current_status = self.to_dict()
            merged_status = {**existing_status, **current_status}
            os.makedirs("decision_messages", exist_ok=True)
            with open(status_file, "w", encoding='utf-8') as f:
                json.dump(merged_status, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"保存状态文件时出错: {e}")

    def save_full_status_to_file(self):
        try:
            status_file = os.path.join("decision_messages", "status.json")
            full_status = self.to_dict()
            os.makedirs("decision_messages", exist_ok=True)
            with open(status_file, "w", encoding='utf-8') as f:
                json.dump(full_status, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"保存完整状态到文件时出错: {e}")

    def load_status_from_file(self):
        try:
            status_file = os.path.join("decision_messages", "status.json")
            if os.path.exists(status_file):
                with open(status_file, "r", encoding='utf-8') as f:
                    status = json.load(f)
                red_robots = status.get('red_robots', {})
                for robot_id_str, robot_data in red_robots.items():
                    robot_id = int(robot_id_str)
                    robot = self.get_robot('red', robot_id)
                    if robot:
                        robot.x = robot_data.get('x', robot.x)
                        robot.y = robot_data.get('y', robot.y)
                        robot.hp = robot_data.get('hp', robot.hp)
                        robot.allowance = robot_data.get('allowance', robot.allowance)
                blue_robots = status.get('blue_robots', {})
                for robot_id_str, robot_data in blue_robots.items():
                    robot_id = int(robot_id_str)
                    robot = self.get_robot('blue', robot_id)
                    if robot:
                        robot.x = robot_data.get('x', robot.x)
                        robot.y = robot_data.get('y', robot.y)
                        robot.hp = robot_data.get('hp', robot.hp)
                        robot.allowance = robot_data.get('allowance', robot.allowance)
                self.red_outpost_hp = status.get('red_outpost_hp', self.red_outpost_hp)
                self.blue_outpost_hp = status.get('blue_outpost_hp', self.blue_outpost_hp)
                self.red_base_hp = status.get('red_base_hp', self.red_base_hp)
                self.blue_base_hp = status.get('blue_base_hp', self.blue_base_hp)
                self.red_gold_coins = status.get('red_gold_coins', self.red_gold_coins)
                self.blue_gold_coins = status.get('blue_gold_coins', self.blue_gold_coins)
                game_state_data = status.get('game_state', {})
                self.stage = game_state_data.get('stage', self.stage)
                self.stage_remaining_time = game_state_data.get('stage_remaining_time', self.stage_remaining_time)
                self.is_running = game_state_data.get('is_running', self.is_running)
                # 新增：加载自定义机器人ID
                self.our_robot_id = status.get('our_robot_id', self.our_robot_id)
        except Exception as e:
            print(f"加载状态文件时出错: {e}")

    def to_dict(self):
        state_dict = {
            'game_state': {
                'stage': self.stage,
                'stage_remaining_time': self.stage_remaining_time,
                'is_running': self.is_running,
                'competition_type': self.competition_type
            },
            'red_robots': {},
            'blue_robots': {},
            'red_outpost_hp': self.red_outpost_hp,
            'blue_outpost_hp': self.blue_outpost_hp,
            'red_base_hp': self.red_base_hp,
            'blue_base_hp': self.blue_base_hp,
            'red_gold_coins': self.red_gold_coins,
            'blue_gold_coins': self.blue_gold_coins,
            # 新增：保存自定义机器人ID
            'our_robot_id': self.our_robot_id
        }
        for robot_id, robot in self.red_robots.items():
            state_dict['red_robots'][str(robot_id)] = {
                'x': robot.x,
                'y': robot.y,
                'hp': robot.hp,
                'allowance': robot.allowance
            }
        for robot_id, robot in self.blue_robots.items():
            state_dict['blue_robots'][str(robot_id)] = {
                'x': robot.x,
                'y': robot.y,
                'hp': robot.hp,
                'allowance': robot.allowance
            }
        return state_dict

    def update(self):
        if self.is_running:
            self.update_timer()
        # 增益区检测已完全移除
