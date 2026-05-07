# src/game_state.py - 修复reset后哨兵机器人移动问题，增加our_robot_id支持

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
        
        # 占领状态 - 红方视角
        self.supply_zone_no_overlap = 0
        self.supply_zone_overlap = 0
        self.supply_zone_occupation = OCCUPATION_UNOCCUPIED
        self.energy_mechanism_status = 0
        self.small_energy_mechanism_activation = 0
        self.large_energy_mechanism_activation = 0
        self.central_highland_occupation = OCCUPATION_UNOCCUPIED
        self.trapezoid_highland_occupation = OCCUPATION_UNOCCUPIED
        self.fortress_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.outpost_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.base_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.center_gain_point_occupation = OCCUPATION_UNOCCUPIED
        
        # 占领状态 - 蓝方视角
        self.enemy_supply_zone_occupation = 0
        self.enemy_central_highland_occupation = OCCUPATION_UNOCCUPIED
        self.enemy_trapezoid_highland_occupation = 0
        self.enemy_fortress_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.enemy_outpost_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.enemy_base_gain_point_occupation = 0
        
        # 模块卡状态
        self.enemy_near_tunnel_before_ramp_card = 0
        self.enemy_near_tunnel_after_ramp_card = 0
        self.own_near_tunnel_before_ramp_card = 0
        self.own_near_tunnel_after_ramp_card = 0
        self.enemy_highland_upper_card = 0
        self.enemy_ramp_upper_card = 0
        self.enemy_road_upper_card = 0
        
        # 能量机关状态
        self.energy_mechanism_status = ENERGY_MECHANISM_INACTIVE
        self.small_energy_mechanism_activation = ENERGY_MECHANISM_INACTIVE
        self.large_energy_mechanism_activation = ENERGY_MECHANISM_INACTIVE
        self.energy_mechanism_activatable = 0
        
        # 飞镖状态
        self.dart_hit_time = 0
        self.dart_hit_target = DART_TARGET_NONE
        
        # 哨兵状态
        self.exchanged_allowance = 0
        self.free_resurrection_available = 0
        self.red_sentry_posture = SENTRY_DEFENSE
        self.blue_sentry_posture = SENTRY_DEFENSE
        
        # 储备允许发弹量
        self.red_reserve_allowance_17mm = 0
        self.blue_reserve_allowance_17mm = 0
        
        # 增益区占领检测
        self.last_zone_check_time = 0
        self.zone_check_interval = 0.5
        
        # 增益区首次占领记录
        self.gain_zone_first_occupation = {
            'central_highland_occupation': OCCUPATION_UNOCCUPIED,
            'outpost_gain_point_occupation': OCCUPATION_UNOCCUPIED
        }
        
        # ROS2消息接口
        self.msg_interface = None
        
        # 哨兵控制器引用
        self.sentry_controller = None
        
        # 自定义本机器人ID (0=红方, 1=蓝方)
        self.our_robot_id = 0
        # 上次从文件成功写入的时间戳（用于避免旧文件覆盖新修改）
        self.status_file_write_ts = 0.0
    
    def set_msg_interface(self, msg_interface):
        """设置消息接口"""
        self.msg_interface = msg_interface
    
    def set_sentry_controller(self, sentry_controller):
        """设置哨兵控制器引用"""
        self.sentry_controller = sentry_controller
    
    def initialize_robots(self):
        """初始化机器人位置"""
        # 红色方（左半区）
        red_positions = {
            1: (174,664),
            2: (420,686),
            3: (244,624),
            4: (346,626),
            7: (408,808)
        }
        
        # 蓝色方（右半区）
        blue_positions = {
            1: (2464,638),
            2: (2398,688),
            3: (2558,624),
            4: (2634,654),
            7: (2398,798)
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
        """重置机器人位置（用于reset后保持哨兵移动能力）"""
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
        """开始比赛"""
        if self.stage in [GameStage.NOT_STARTED, GameStage.COUNTDOWN]:
            self.stage = GameStage.IN_PROGRESS
            self.start_time = time.time()
            self.is_running = True
            self.save_full_status_to_file()
            self.update_ros2_messages()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 比赛开始")
    
    def reset_game(self):
        """重置比赛 - 修复版本：保持机器人对象不变"""
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 开始重置比赛...")
        
        # 重置游戏状态
        self.stage = GameStage.NOT_STARTED
        self.stage_remaining_time = GAME_TIME
        self.start_time = None
        self.is_running = False
        
        # 红色方（左半区）初始位置
        red_positions = {
            1: (174,664),
            2: (420,686),
            3: (244,624),
            4: (346,626),
            7: (408,808)
        }
        
        # 蓝色方（右半区）初始位置
        blue_positions = {
            1: (2464,638),
            2: (2398,688),
            3: (2558,624),
            4: (2634,654),
            7: (2398,798)
        }
        
        # 重置机器人位置而不是重新创建对象
        for robot_id in ROBOT_IDS:
            if robot_id in red_positions:
                self.reset_robot_position(robot_id, 'red', *red_positions[robot_id])
            if robot_id in blue_positions:
                self.reset_robot_position(robot_id, 'blue', *blue_positions[robot_id])
        
        # 重置场地血量
        self.red_outpost_hp = OUTPOST_HP
        self.blue_outpost_hp = OUTPOST_HP
        self.red_base_hp = BASE_HP
        self.blue_base_hp = BASE_HP
        
        # 重置经济系统
        self.red_gold_coins = GOLD_COINS
        self.blue_gold_coins = GOLD_COINS
        self.red_total_gold_coins = 0
        self.blue_total_gold_coins = 0
        
        # 重置所有占领状态
        self.supply_zone_no_overlap = 0
        self.supply_zone_overlap = 0
        self.supply_zone_occupation = OCCUPATION_UNOCCUPIED
        self.energy_mechanism_status = 0
        self.small_energy_mechanism_activation = 0
        self.large_energy_mechanism_activation = 0
        self.central_highland_occupation = OCCUPATION_UNOCCUPIED
        self.trapezoid_highland_occupation = OCCUPATION_UNOCCUPIED
        self.fortress_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.outpost_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.base_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.center_gain_point_occupation = OCCUPATION_UNOCCUPIED
        
        self.enemy_supply_zone_occupation = 0
        self.enemy_central_highland_occupation = OCCUPATION_UNOCCUPIED
        self.enemy_trapezoid_highland_occupation = 0
        self.enemy_fortress_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.enemy_outpost_gain_point_occupation = OCCUPATION_UNOCCUPIED
        self.enemy_base_gain_point_occupation = 0
        
        # 重置模块卡状态
        self.enemy_near_tunnel_before_ramp_card = 0
        self.enemy_near_tunnel_after_ramp_card = 0
        self.own_near_tunnel_before_ramp_card = 0
        self.own_near_tunnel_after_ramp_card = 0
        self.enemy_highland_upper_card = 0
        self.enemy_ramp_upper_card = 0
        self.enemy_road_upper_card = 0
        
        # 重置能量机关状态
        self.energy_mechanism_status = ENERGY_MECHANISM_INACTIVE
        self.small_energy_mechanism_activation = ENERGY_MECHANISM_INACTIVE
        self.large_energy_mechanism_activation = ENERGY_MECHANISM_INACTIVE
        self.energy_mechanism_activatable = 0
        
        # 重置飞镖状态
        self.dart_hit_time = 0
        self.dart_hit_target = DART_TARGET_NONE
        
        # 重置哨兵状态
        self.exchanged_allowance = 0
        self.free_resurrection_available = 0
        self.red_sentry_posture = SENTRY_DEFENSE
        self.blue_sentry_posture = SENTRY_DEFENSE
        
        # 重置储备允许发弹量
        self.red_reserve_allowance_17mm = 0
        self.blue_reserve_allowance_17mm = 0
        
        # 重置增益区首次占领记录
        self.gain_zone_first_occupation = {
            'central_highland_occupation': OCCUPATION_UNOCCUPIED,
            'outpost_gain_point_occupation': OCCUPATION_UNOCCUPIED
        }
        
        # 重置自定义本机器人ID
        self.our_robot_id = 0
        
        # 更新哨兵控制器的哨兵机器人引用
        if self.sentry_controller:
            self.sentry_controller.sentry_robot = self.red_robots.get(7)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 已更新哨兵控制器引用到新的哨兵机器人")
        
        # 保存并同步状态
        self.save_full_status_to_file()
        self.update_ros2_messages()
        
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 比赛已重置，哨兵机器人引用已更新")
    
    def update_timer(self):
        """更新计时器"""
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
        """获取所有机器人"""
        all_robots = []
        for robot in self.red_robots.values():
            all_robots.append(robot)
        for robot in self.blue_robots.values():
            all_robots.append(robot)
        return all_robots
    
    def get_robot(self, team, robot_id):
        """获取指定机器人"""
        if team == 'red':
            return self.red_robots.get(robot_id)
        else:
            return self.blue_robots.get(robot_id)
    
    def update_robot_hp(self, team, robot_id, hp):
        """更新机器人血量"""
        robot = self.get_robot(team, robot_id)
        if robot:
            robot.hp = max(0, min(int(hp), 2000))
            self.save_full_status_to_file()
            self.update_ros2_messages()
    
    def update_robot_allowance(self, team, robot_id, allowance):
        """更新机器人发弹量"""
        robot = self.get_robot(team, robot_id)
        if robot and robot_id in DEFAULT_ALLOWANCE:
            robot.allowance = max(0, int(allowance))
            self.save_full_status_to_file()
            self.update_ros2_messages()
    
    def update_field(self, field, value):
        """更新游戏状态字段"""
        if hasattr(self, field):
            setattr(self, field, int(value))
            self.save_full_status_to_file()
            self.update_ros2_messages()
    
    def update_ros2_messages(self):
        """更新ROS2消息"""
        if self.msg_interface:
            self.msg_interface.update_messages(self)
    
    def point_in_polygon(self, point, polygon):
        """判断点是否在多边形内（射线法）"""
        x, y = point
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def check_gain_zone_occupation(self):
        """检查所有增益区的占领状态"""
        current_time = time.time()
        if current_time - self.last_zone_check_time < self.zone_check_interval:
            return
        
        self.last_zone_check_time = current_time
        
        # 保存旧状态用于比较
        old_occupation = {
            'supply_zone_occupation': self.supply_zone_occupation,
            'central_highland_occupation': self.central_highland_occupation,
            'trapezoid_highland_occupation': self.trapezoid_highland_occupation,
            'fortress_gain_point_occupation': self.fortress_gain_point_occupation,
            'outpost_gain_point_occupation': self.outpost_gain_point_occupation,
            'base_gain_point_occupation': self.base_gain_point_occupation
        }
        
        # 重置所有增益区的占领状态
        zone_occupation = {
            'supply_zone_occupation': {'red': False, 'blue': False},
            'central_highland_occupation': {'red': False, 'blue': False},
            'trapezoid_highland_occupation': {'red': False, 'blue': False},
            'fortress_gain_point_occupation': {'red': False, 'blue': False},
            'outpost_gain_point_occupation': {'red': False, 'blue': False},
            'base_gain_point_occupation': {'red': False, 'blue': False}
        }
        
        # 检查每个增益区
        for zone_name, zone_data in GAIN_ZONES.items():
            field = zone_data['field']
            points = zone_data['points']
            
            # 检查红方机器人
            for robot in self.red_robots.values():
                if robot.hp > 0:
                    if self.point_in_polygon((robot.x, robot.y), points):
                        zone_occupation[field]['red'] = True
            
            # 检查蓝方机器人
            for robot in self.blue_robots.values():
                if robot.hp > 0:
                    if self.point_in_polygon((robot.x, robot.y), points):
                        zone_occupation[field]['blue'] = True
        
        # 特殊处理中央高地（由两部分组成）
        central_highland_part1 = GAIN_ZONES['central_highland_part1']
        central_red1 = False
        central_blue1 = False
        
        for robot in self.red_robots.values():
            if robot.hp > 0 and self.point_in_polygon((robot.x, robot.y), central_highland_part1['points']):
                central_red1 = True
        
        for robot in self.blue_robots.values():
            if robot.hp > 0 and self.point_in_polygon((robot.x, robot.y), central_highland_part1['points']):
                central_blue1 = True
        
        # 检查中央高地第二部分
        central_highland_part2 = GAIN_ZONES['central_highland_part2']
        central_red2 = False
        central_blue2 = False
        
        for robot in self.red_robots.values():
            if robot.hp > 0 and self.point_in_polygon((robot.x, robot.y), central_highland_part2['points']):
                central_red2 = True
        
        for robot in self.blue_robots.values():
            if robot.hp > 0 and self.point_in_polygon((robot.x, robot.y), central_highland_part2['points']):
                central_blue2 = True
        
        zone_occupation['central_highland_occupation']['red'] = central_red1 or central_red2
        zone_occupation['central_highland_occupation']['blue'] = central_blue1 or central_blue2
        
        # 更新占领状态
        for field, occupation in zone_occupation.items():
            new_value = 0
            
            if field == 'supply_zone_occupation':
                new_value = 1 if occupation['red'] else 0
            elif field == 'trapezoid_highland_occupation':
                new_value = 1 if occupation['red'] else 0
            elif field == 'base_gain_point_occupation':
                new_value = 1 if occupation['red'] else 0
            elif field == 'central_highland_occupation':
                first_occupation = self.gain_zone_first_occupation[field]
                if first_occupation == OCCUPATION_UNOCCUPIED:
                    if occupation['red'] and not occupation['blue']:
                        new_value = 1
                        self.gain_zone_first_occupation[field] = OCCUPATION_OUR
                    elif occupation['blue'] and not occupation['red']:
                        new_value = 2
                        self.gain_zone_first_occupation[field] = OCCUPATION_ENEMY
                    elif occupation['red'] and occupation['blue']:
                        new_value = 1
                        self.gain_zone_first_occupation[field] = OCCUPATION_OUR
                    else:
                        new_value = 0
                else:
                    if first_occupation == OCCUPATION_OUR:
                        if occupation['red']:
                            new_value = 1
                        else:
                            if occupation['blue']:
                                new_value = 2
                                self.gain_zone_first_occupation[field] = OCCUPATION_ENEMY
                            else:
                                new_value = 0
                                self.gain_zone_first_occupation[field] = OCCUPATION_UNOCCUPIED
                    elif first_occupation == OCCUPATION_ENEMY:
                        if occupation['blue']:
                            new_value = 2
                        else:
                            if occupation['red']:
                                new_value = 1
                                self.gain_zone_first_occupation[field] = OCCUPATION_OUR
                            else:
                                new_value = 0
                                self.gain_zone_first_occupation[field] = OCCUPATION_UNOCCUPIED
                    else:
                        if occupation['red'] and not occupation['blue']:
                            new_value = 1
                        elif occupation['blue'] and not occupation['red']:
                            new_value = 2
                        else:
                            new_value = 0
            elif field == 'fortress_gain_point_occupation':
                if occupation['red'] and occupation['blue']:
                    new_value = 3
                elif occupation['red']:
                    new_value = 1
                elif occupation['blue']:
                    new_value = 2
                else:
                    new_value = 0
            elif field == 'outpost_gain_point_occupation':
                first_occupation = self.gain_zone_first_occupation[field]
                if first_occupation == OCCUPATION_UNOCCUPIED:
                    if occupation['red'] and not occupation['blue']:
                        new_value = 1
                        self.gain_zone_first_occupation[field] = OCCUPATION_OUR
                    elif occupation['blue'] and not occupation['red']:
                        new_value = 2
                        self.gain_zone_first_occupation[field] = OCCUPATION_ENEMY
                    elif occupation['red'] and occupation['blue']:
                        new_value = 1
                        self.gain_zone_first_occupation[field] = OCCUPATION_OUR
                    else:
                        new_value = 0
                else:
                    if first_occupation == OCCUPATION_OUR:
                        if occupation['red']:
                            new_value = 1
                        else:
                            if occupation['blue']:
                                new_value = 2
                                self.gain_zone_first_occupation[field] = OCCUPATION_ENEMY
                            else:
                                new_value = 0
                                self.gain_zone_first_occupation[field] = OCCUPATION_UNOCCUPIED
                    elif first_occupation == OCCUPATION_ENEMY:
                        if occupation['blue']:
                            new_value = 2
                        else:
                            if occupation['red']:
                                new_value = 1
                                self.gain_zone_first_occupation[field] = OCCUPATION_OUR
                            else:
                                new_value = 0
                                self.gain_zone_first_occupation[field] = OCCUPATION_UNOCCUPIED
                    else:
                        if occupation['red'] and not occupation['blue']:
                            new_value = 1
                        elif occupation['blue'] and not occupation['red']:
                            new_value = 2
                        else:
                            new_value = 0
            
            setattr(self, field, new_value)
            old_value = old_occupation[field]
            if old_value != new_value:
                zone_name = GAIN_ZONE_DISPLAY_NAMES.get(field, field)
                print(f"[{datetime.now().strftime('%H:%M:%S')}] 增益区状态更新: {zone_name} 从 {self.get_occupation_text(old_value)} 变为 {self.get_occupation_text(new_value)}")
        
        # 更新蓝方视角
        self.enemy_supply_zone_occupation = 1 if zone_occupation['supply_zone_occupation']['blue'] else 0
        self.enemy_central_highland_occupation = 1 if zone_occupation['central_highland_occupation']['blue'] else 2 if zone_occupation['central_highland_occupation']['red'] else 0
        self.enemy_trapezoid_highland_occupation = 1 if zone_occupation['trapezoid_highland_occupation']['blue'] else 0
        self.enemy_fortress_gain_point_occupation = self.fortress_gain_point_occupation
        self.enemy_outpost_gain_point_occupation = self.outpost_gain_point_occupation
        self.enemy_base_gain_point_occupation = 1 if zone_occupation['base_gain_point_occupation']['blue'] else 0
        
        self.save_full_status_to_file()
        self.update_ros2_messages()
    
    def get_occupation_text(self, status_value):
        if status_value == 0:
            return "无人占领"
        elif status_value == 1:
            return "红方占领"
        elif status_value == 2:
            return "蓝方占领"
        elif status_value == 3:
            return "双方占领"
        else:
            return f"未知({status_value})"
    
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
            # 原子写入：先写入临时文件再替换
            tmp_file = status_file + ".tmp"
            with open(tmp_file, "w", encoding='utf-8') as f:
                json.dump(merged_status, f, indent=2, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_file, status_file)
        except Exception as e:
            print(f"保存状态文件时出错: {e}")
    
    def save_full_status_to_file(self):
        try:
            status_file = os.path.join("decision_messages", "status.json")
            full_status = self.to_dict()
            # 添加元信息：写入时间戳
            import time as _time
            ts = _time.time()
            full_status['meta'] = {'write_ts': ts}
            os.makedirs("decision_messages", exist_ok=True)
            # 原子写入：先写入临时文件再替换，避免并发写入导致的 JSON 损坏
            tmp_file = status_file + ".tmp"
            with open(tmp_file, "w", encoding='utf-8') as f:
                json.dump(full_status, f, indent=2, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_file, status_file)
            # 记录写入时间戳，防止旧文件覆盖
            self.status_file_write_ts = ts
        except Exception as e:
            print(f"保存完整状态到文件时出错: {e}")
    
    def load_full_status_from_file(self):
        """从文件加载所有状态（包括血量、经济、增益区等），但保留坐标由调用方决定"""
        try:
            status_file = os.path.join("decision_messages", "status.json")
            if os.path.exists(status_file):
                with open(status_file, "r", encoding='utf-8') as f:
                    status = json.load(f)
                # 检查写入时间戳，避免旧文件覆盖新内存状态
                file_ts = 0.0
                try:
                    file_ts = float(status.get('meta', {}).get('write_ts', 0.0))
                except Exception:
                    file_ts = 0.0
                if file_ts <= self.status_file_write_ts:
                    return
                
                # 加载机器人血量、弹药（坐标不加载）
                red_robots = status.get('red_robots', {})
                for robot_id_str, robot_data in red_robots.items():
                    robot_id = int(robot_id_str)
                    robot = self.get_robot('red', robot_id)
                    if robot:
                        robot.hp = robot_data.get('hp', robot.hp)
                        robot.allowance = robot_data.get('allowance', robot.allowance)
                
                blue_robots = status.get('blue_robots', {})
                for robot_id_str, robot_data in blue_robots.items():
                    robot_id = int(robot_id_str)
                    robot = self.get_robot('blue', robot_id)
                    if robot:
                        robot.hp = robot_data.get('hp', robot.hp)
                        robot.allowance = robot_data.get('allowance', robot.allowance)
                
                # 加载场地血量
                self.red_outpost_hp = status.get('red_outpost_hp', self.red_outpost_hp)
                self.blue_outpost_hp = status.get('blue_outpost_hp', self.blue_outpost_hp)
                self.red_base_hp = status.get('red_base_hp', self.red_base_hp)
                self.blue_base_hp = status.get('blue_base_hp', self.blue_base_hp)
                
                # 加载经济
                self.red_gold_coins = status.get('red_gold_coins', self.red_gold_coins)
                self.blue_gold_coins = status.get('blue_gold_coins', self.blue_gold_coins)
                
                # 加载游戏阶段（可选）
                game_state_data = status.get('game_state', {})
                self.stage = game_state_data.get('stage', self.stage)
                self.stage_remaining_time = game_state_data.get('stage_remaining_time', self.stage_remaining_time)
                self.is_running = game_state_data.get('is_running', self.is_running)
                
                # 加载增益区占领状态
                gain_zone_data = status.get('gain_zone_occupation', {})
                for field in GAIN_ZONE_FIELDS:
                    if field in gain_zone_data:
                        setattr(self, field, gain_zone_data[field])
                
                # 加载本机器人ID
                self.our_robot_id = status.get('our_robot_id', self.our_robot_id)
        except Exception as e:
            print(f"加载完整状态文件时出错: {e}")
    
    def load_status_from_file(self):
        """从文件加载状态（兼容旧接口，实际调用load_full_status_from_file）"""
        self.load_full_status_from_file()
    
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
            'gain_zone_occupation': {
                'supply_zone_occupation': self.supply_zone_occupation,
                'central_highland_occupation': self.central_highland_occupation,
                'trapezoid_highland_occupation': self.trapezoid_highland_occupation,
                'fortress_gain_point_occupation': self.fortress_gain_point_occupation,
                'outpost_gain_point_occupation': self.outpost_gain_point_occupation,
                'base_gain_point_occupation': self.base_gain_point_occupation
            },
            'gain_zone_first_occupation': self.gain_zone_first_occupation,
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
        self.check_gain_zone_occupation()
