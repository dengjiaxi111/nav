# src/control_panel.py - 修复UI布局和数据修改问题

import pygame
import sys
import os
import time
import json
from config import *

class ControlPanel:
    def __init__(self, game_state, msg_interface):
        pygame.init()
        
        # 控制面板尺寸
        self.width = 1200
        self.height = 1100  # 增加高度以容纳增益区信息
        
        # 创建窗口
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("RoboMaster Control Panel")
        
        # 游戏状态和消息接口
        self.game_state = game_state
        self.msg_interface = msg_interface  # ROS2消息接口
        
        # 字体
        pygame.font.init()
        self.title_font = pygame.font.SysFont("arial", 32, bold=True)
        self.header_font = pygame.font.SysFont("arial", 24, bold=True)
        self.normal_font = pygame.font.SysFont("arial", 20)
        self.small_font = pygame.font.SysFont("arial", 16)
        
        # 颜色定义
        self.bg_color = (40, 40, 45)
        self.panel_color = (50, 55, 70)
        self.text_color = (230, 230, 230)
        self.highlight_color = (255, 180, 40)
        
        # 输入框管理 - 使用参考代码的改进方案
        self.input_boxes = {}
        self.active_input_box = None
        self.input_texts = {}  # 保存输入框的临时文本
        self.input_original = {}  # 保存原始值用于恢复
        
        # 显示框（用于坐标显示）
        self.display_boxes = {}
        
        # 增益区显示框
        self.gain_zone_boxes = {}
        
        # 初始化UI元素
        self.init_ui_elements()
        
        # 最后更新时间
        self.last_update_time = 0
        self.update_interval = 0.2
        
        # 最后同步时间
        self.last_sync_time = 0
        self.sync_interval = 0.5  # 0.5秒同步一次
        
        print("控制面板已初始化")
    
    def init_ui_elements(self):
        """初始化UI元素"""
        # 清除旧的输入框
        self.input_boxes.clear()
        self.display_boxes.clear()
        self.gain_zone_boxes.clear()
        self.input_texts.clear()
        self.input_original.clear()
        
        # 区域间距
        section_margin = 20
        column_width = (self.width - 3 * section_margin) // 2
        
        # ============ 红方区域 ============
        red_x = section_margin
        red_y = 140
        
        # 红方标题
        self.add_display_box('red_title', {
            'rect': pygame.Rect(red_x, red_y - 40, column_width, 35),
            'text': "RED TEAM",
            'color': RED,
            'font': self.header_font
        })
        
        # 红方机器人信息
        y_offset = red_y
        robot_labels = ["Hero", "Engineer", "Infantry3", "Infantry4", "Sentry"]
        
        for i, robot_id in enumerate(ROBOT_IDS):
            robot = self.game_state.get_robot('red', robot_id)
            if robot:
                robot_name = robot_labels[i]
                
                # 机器人标签
                self.add_display_box(f'red_{robot_id}_label', {
                    'rect': pygame.Rect(red_x, y_offset, 120, 30),
                    'text': f"{robot_name}:",
                    'color': WHITE,
                    'font': self.normal_font
                })
                
                # 坐标显示（只读）
                self.add_display_box(f'red_{robot_id}_pos', {
                    'rect': pygame.Rect(red_x + 130, y_offset, 180, 30),
                    'text': f"({int(robot.x)}, {int(robot.y)}) cm",
                    'color': GREEN,
                    'font': self.small_font
                })
                
                # 血量输入框
                hp_text = str(robot.hp)
                self.input_texts[f'red_{robot_id}_hp'] = hp_text
                self.input_original[f'red_{robot_id}_hp'] = hp_text
                self.add_input_box(f'red_{robot_id}_hp', {
                    'rect': pygame.Rect(red_x + 320, y_offset, 100, 35),
                    'label': "HP",
                    'active': False,
                    'type': 'int',
                    'min': 0,
                    'max': 2000,
                    'callback': lambda v, rid=robot_id: self.update_robot_hp('red', rid, v)
                })
                
                # 发弹量输入框（如果适用）
                if robot_id in [1, 3, 4, 7]:
                    allowance_text = str(robot.allowance)
                    self.input_texts[f'red_{robot_id}_allowance'] = allowance_text
                    self.input_original[f'red_{robot_id}_allowance'] = allowance_text
                    self.add_input_box(f'red_{robot_id}_allowance', {
                        'rect': pygame.Rect(red_x + 430, y_offset, 100, 35),
                        'label': "Allow",
                        'active': False,
                        'type': 'int',
                        'min': 0,
                        'max': 1000,
                        'callback': lambda v, rid=robot_id: self.update_robot_allowance('red', rid, v)
                    })
                
                y_offset += 60
        
        # 红方基地信息
        y_offset += 50
        self.add_display_box('red_base_label', {
            'rect': pygame.Rect(red_x, y_offset, column_width, 25),
            'text': "BASE STATUS",
            'color': RED,
            'font': self.normal_font
        })
        
        y_offset += 50
        outpost_hp_text = str(self.game_state.red_outpost_hp)
        self.input_texts['red_outpost_hp'] = outpost_hp_text
        self.input_original['red_outpost_hp'] = outpost_hp_text
        self.add_input_box('red_outpost_hp', {
            'rect': pygame.Rect(red_x, y_offset, 220, 35),
            'label': "Outpost HP",
            'active': False,
            'type': 'int',
            'min': 0,
            'max': 2000,
            'callback': lambda v: self.update_field('red_outpost_hp', v)
        })
        
        y_offset += 70
        base_hp_text = str(self.game_state.red_base_hp)
        self.input_texts['red_base_hp'] = base_hp_text
        self.input_original['red_base_hp'] = base_hp_text
        self.add_input_box('red_base_hp', {
            'rect': pygame.Rect(red_x, y_offset, 220, 35),
            'label': "Base HP",
            'active': False,
            'type': 'int',
            'min': 0,
            'max': 10000,
            'callback': lambda v: self.update_field('red_base_hp', v)
        })
        
        # 红方经济信息
        y_offset += 100
        self.add_display_box('red_econ_label', {
            'rect': pygame.Rect(red_x, y_offset, column_width, 25),
            'text': "ECONOMY",
            'color': RED,
            'font': self.normal_font
        })
        
        y_offset += 40
        gold_text = str(self.game_state.red_gold_coins)
        self.input_texts['red_gold_coins'] = gold_text
        self.input_original['red_gold_coins'] = gold_text
        self.add_input_box('red_gold_coins', {
            'rect': pygame.Rect(red_x, y_offset, 220, 35),
            'label': "Gold Coins",
            'active': False,
            'type': 'int',
            'min': 0,
            'max': 10000,
            'callback': lambda v: self.update_field('red_gold_coins', v)
        })
        
        # ============ 蓝方区域 ============
        blue_x = self.width - column_width - section_margin
        blue_y = 140
        
        # 蓝方标题
        self.add_display_box('blue_title', {
            'rect': pygame.Rect(blue_x, blue_y - 40, column_width, 35),
            'text': "BLUE TEAM",
            'color': BLUE,
            'font': self.header_font
        })
        
        # 蓝方机器人信息
        y_offset = blue_y
        
        for i, robot_id in enumerate(ROBOT_IDS):
            robot = self.game_state.get_robot('blue', robot_id)
            if robot:
                robot_name = robot_labels[i]
                
                # 机器人标签
                self.add_display_box(f'blue_{robot_id}_label', {
                    'rect': pygame.Rect(blue_x, y_offset, 120, 30),
                    'text': f"{robot_name}:",
                    'color': WHITE,
                    'font': self.normal_font
                })
                
                # 坐标显示（只读）
                self.add_display_box(f'blue_{robot_id}_pos', {
                    'rect': pygame.Rect(blue_x + 130, y_offset, 180, 30),
                    'text': f"({int(robot.x)}, {int(robot.y)}) cm",
                    'color': GREEN,
                    'font': self.small_font
                })
                
                # 血量输入框
                hp_text = str(robot.hp)
                self.input_texts[f'blue_{robot_id}_hp'] = hp_text
                self.input_original[f'blue_{robot_id}_hp'] = hp_text
                self.add_input_box(f'blue_{robot_id}_hp', {
                    'rect': pygame.Rect(blue_x + 320, y_offset, 100, 35),
                    'label': "HP",
                    'active': False,
                    'type': 'int',
                    'min': 0,
                    'max': 2000,
                    'callback': lambda v, rid=robot_id: self.update_robot_hp('blue', rid, v)
                })
                
                # 发弹量输入框（如果适用）
                if robot_id in [1, 3, 4, 7]:
                    allowance_text = str(robot.allowance)
                    self.input_texts[f'blue_{robot_id}_allowance'] = allowance_text
                    self.input_original[f'blue_{robot_id}_allowance'] = allowance_text
                    self.add_input_box(f'blue_{robot_id}_allowance', {
                        'rect': pygame.Rect(blue_x + 430, y_offset, 100, 35),
                        'label': "Allow",
                        'active': False,
                        'type': 'int',
                        'min': 0,
                        'max': 1000,
                        'callback': lambda v, rid=robot_id: self.update_robot_allowance('blue', rid, v)
                    })
                
                y_offset += 60
        
        # 蓝方基地信息
        y_offset += 50
        self.add_display_box('blue_base_label', {
            'rect': pygame.Rect(blue_x, y_offset, column_width, 25),
            'text': "BASE STATUS",
            'color': BLUE,
            'font': self.normal_font
        })
        
        y_offset += 50
        outpost_hp_text = str(self.game_state.blue_outpost_hp)
        self.input_texts['blue_outpost_hp'] = outpost_hp_text
        self.input_original['blue_outpost_hp'] = outpost_hp_text
        self.add_input_box('blue_outpost_hp', {
            'rect': pygame.Rect(blue_x, y_offset, 220, 35),
            'label': "Outpost HP",
            'active': False,
            'type': 'int',
            'min': 0,
            'max': 2000,
            'callback': lambda v: self.update_field('blue_outpost_hp', v)
        })
        
        y_offset += 70
        base_hp_text = str(self.game_state.blue_base_hp)
        self.input_texts['blue_base_hp'] = base_hp_text
        self.input_original['blue_base_hp'] = base_hp_text
        self.add_input_box('blue_base_hp', {
            'rect': pygame.Rect(blue_x, y_offset, 220, 35),
            'label': "Base HP",
            'active': False,
            'type': 'int',
            'min': 0,
            'max': 10000,
            'callback': lambda v: self.update_field('blue_base_hp', v)
        })
        
        # 蓝方经济信息
        y_offset += 100
        self.add_display_box('blue_econ_label', {
            'rect': pygame.Rect(blue_x, y_offset, column_width, 25),
            'text': "ECONOMY",
            'color': BLUE,
            'font': self.normal_font
        })
        
        y_offset += 40
        gold_text = str(self.game_state.blue_gold_coins)
        self.input_texts['blue_gold_coins'] = gold_text
        self.input_original['blue_gold_coins'] = gold_text
        self.add_input_box('blue_gold_coins', {
            'rect': pygame.Rect(blue_x, y_offset, 220, 35),
            'label': "Gold Coins",
            'active': False,
            'type': 'int',
            'min': 0,
            'max': 10000,
            'callback': lambda v: self.update_field('blue_gold_coins', v)
        })
        
        # ============ 增益区占领状态 ============
        # 增益区标题 - 向下平移
        gain_zone_y = 850  # 从820调整到850
        self.add_display_box('gain_zone_title', {
            'rect': pygame.Rect(section_margin, gain_zone_y - 40, self.width - 2 * section_margin, 35),
            'text': "GAIN ZONE OCCUPATION STATUS",
            'color': YELLOW,
            'font': self.header_font
        })
        
        # 增益区信息 - 分为两列
        gain_zone_col1_x = section_margin
        gain_zone_col2_x = self.width // 2 + 20
        gain_zone_y_start = gain_zone_y + 10
        
        # 英文增益区名称映射
        gain_zone_en_names = {
            'supply_zone_occupation': 'Supply Zone',
            'central_highland_occupation': 'Central Highland',
            'trapezoid_highland_occupation': 'Trapezoid Highland',
            'fortress_gain_point_occupation': 'Fortress Gain Point',
            'outpost_gain_point_occupation': 'Outpost Gain Point',
            'base_gain_point_occupation': 'Base Gain Point'
        }
        
        # 第一列增益区
        gain_zones_col1 = ['supply_zone_occupation', 'central_highland_occupation', 'trapezoid_highland_occupation']
        for i, field in enumerate(gain_zones_col1):
            y_pos = gain_zone_y_start + i * 60
            
            # 增益区名称 - 使用英文
            zone_name = gain_zone_en_names.get(field, field)
            self.add_display_box(f'gain_zone_{field}_label', {
                'rect': pygame.Rect(gain_zone_col1_x, y_pos, 150, 30),
                'text': f"{zone_name}:",
                'color': WHITE,
                'font': self.normal_font
            })
            
            # 增益区状态显示框
            self.add_gain_zone_box(field, {
                'rect': pygame.Rect(gain_zone_col1_x + 190, y_pos, 200, 35),
                'font': self.normal_font
            })
        
        # 第二列增益区
        gain_zones_col2 = ['fortress_gain_point_occupation', 'outpost_gain_point_occupation', 'base_gain_point_occupation']
        for i, field in enumerate(gain_zones_col2):
            y_pos = gain_zone_y_start + i * 60
            
            # 增益区名称 - 使用英文
            zone_name = gain_zone_en_names.get(field, field)
            self.add_display_box(f'gain_zone_{field}_label', {
                'rect': pygame.Rect(gain_zone_col2_x, y_pos, 150, 30),
                'text': f"{zone_name}:",
                'color': WHITE,
                'font': self.normal_font
            })
            
            # 增益区状态显示框
            self.add_gain_zone_box(field, {
                'rect': pygame.Rect(gain_zone_col2_x + 190, y_pos, 200, 35),
                'font': self.normal_font
            })
    
    def add_display_box(self, box_id, box_data):
        """添加只读显示框"""
        self.display_boxes[box_id] = box_data
    
    def add_input_box(self, box_id, box_data):
        """添加输入框"""
        self.input_boxes[box_id] = box_data
    
    def add_gain_zone_box(self, field, box_data):
        """添加增益区状态显示框"""
        self.gain_zone_boxes[field] = box_data
    
    def update_robot_hp(self, team, robot_id, value):
        """更新机器人HP"""
        self.game_state.update_robot_hp(team, robot_id, int(value))
        self.save_and_sync_state()
        print(f"Updated {team} robot {robot_id} HP to {value}")
    
    def update_robot_allowance(self, team, robot_id, value):
        """更新机器人发弹量"""
        self.game_state.update_robot_allowance(team, robot_id, int(value))
        self.save_and_sync_state()
        print(f"Updated {team} robot {robot_id} allowance to {value}")
    
    def update_field(self, field, value):
        """更新游戏状态字段"""
        if hasattr(self.game_state, field):
            setattr(self.game_state, field, int(value))
            self.save_and_sync_state()
            print(f"Updated {field} to {value}")
    
    def save_and_sync_state(self):
        """保存并同步状态"""
        # 保存到文件
        self.game_state.save_full_status_to_file()
        # 更新ROS2消息
        if self.msg_interface:
            self.msg_interface.update_messages(self.game_state)
    
    def draw_display_box(self, box_id, box_data):
        """绘制只读显示框"""
        rect = box_data['rect']
        text_surface = box_data['font'].render(box_data['text'], True, box_data['color'])
        text_rect = text_surface.get_rect(topleft=(rect.x, rect.y))
        self.screen.blit(text_surface, text_rect)
    
    def draw_input_box(self, box_id, box_data):
        """绘制输入框"""
        rect = box_data['rect']
        
        # 绘制标签
        if 'label' in box_data:
            label_surface = self.small_font.render(box_data['label'], True, self.text_color)
            label_rect = label_surface.get_rect(midbottom=(rect.centerx, rect.y - 5))
            self.screen.blit(label_surface, label_rect)
        
        # 获取当前文本
        current_text = self.input_texts.get(box_id, "")
        
        # 绘制输入框背景
        bg_color = self.highlight_color if box_data['active'] else WHITE
        pygame.draw.rect(self.screen, bg_color, rect, border_radius=3)
        pygame.draw.rect(self.screen, BLACK, rect, 1, border_radius=3)
        
        # 绘制文本
        text_surface = self.normal_font.render(current_text, True, BLACK)
        text_rect = text_surface.get_rect(center=rect.center)
        self.screen.blit(text_surface, text_rect)
    
    def draw_gain_zone_box(self, field, box_data):
        """绘制增益区状态显示框"""
        rect = box_data['rect']
        
        # 获取占领状态
        if hasattr(self.game_state, field):
            occupation_status = getattr(self.game_state, field)
            
            # 根据状态确定文本和颜色 - 使用英文
            if field in ['supply_zone_occupation', 'trapezoid_highland_occupation', 'base_gain_point_occupation']:
                # 这些字段使用1表示红方占领，0表示无人占领或蓝方占领
                if occupation_status == 1:
                    status_text = "Red Occupied"
                    status_color = RED
                else:
                    status_text = "Not Occupied"
                    status_color = OCCUPATION_NONE
            else:
                # 其他字段使用枚举值
                if occupation_status == 1:  # 红方占领
                    status_text = "Red Occupied"
                    status_color = RED
                elif occupation_status == 2:  # 蓝方占领
                    status_text = "Blue Occupied"
                    status_color = BLUE
                elif occupation_status == 3:  # 双方占领
                    status_text = "Both Occupied"
                    status_color = OCCUPATION_BOTH
                else:  # 0: 无人占领
                    status_text = "Not Occupied"
                    status_color = OCCUPATION_NONE
            
            # 绘制背景
            pygame.draw.rect(self.screen, status_color, rect, border_radius=3)
            pygame.draw.rect(self.screen, BLACK, rect, 1, border_radius=3)
            
            # 绘制文本
            text_surface = box_data['font'].render(status_text, True, WHITE)
            text_rect = text_surface.get_rect(center=rect.center)
            self.screen.blit(text_surface, text_rect)
    
    def draw_ui(self):
        """绘制UI"""
        # 绘制背景
        self.screen.fill(self.bg_color)
        
        # 绘制标题栏
        pygame.draw.rect(self.screen, self.panel_color, (0, 0, self.width, 80))
        
        # 绘制标题
        title = self.title_font.render("RoboMaster Control Panel", True, YELLOW)
        self.screen.blit(title, (self.width // 2 - title.get_width() // 2, 20))
        
        # 绘制游戏状态信息
        self.draw_game_state()
        
        # 绘制分隔线
        mid_x = self.width // 2
        pygame.draw.line(self.screen, WHITE, (mid_x, 90), (mid_x, self.height - 280), 1)
        
        # 绘制所有显示框
        for box_id, box_data in self.display_boxes.items():
            self.draw_display_box(box_id, box_data)
        
        # 绘制所有输入框
        for box_id, box_data in self.input_boxes.items():
            self.draw_input_box(box_id, box_data)
        
        # 绘制所有增益区状态显示框
        for field, box_data in self.gain_zone_boxes.items():
            self.draw_gain_zone_box(field, box_data)
        
        pygame.display.flip()
    
    def draw_game_state(self):
        """绘制游戏状态信息"""
        # 获取当前阶段
        stage = self.game_state.stage
        
        # 根据不同阶段显示不同的时间信息
        if stage == 4:  # 比赛中
            minutes = int(self.game_state.stage_remaining_time // 60)
            seconds = int(self.game_state.stage_remaining_time % 60)
            time_text = f"Match Time: {minutes:02d}:{seconds:02d}"
            
            if self.game_state.is_running:
                if self.game_state.stage_remaining_time < 60:
                    time_color = RED
                elif self.game_state.stage_remaining_time < 180:
                    time_color = YELLOW
                else:
                    time_color = GREEN
            else:
                time_color = WHITE
        elif stage == 1:  # 准备阶段
            time_text = "Stage Time: Ready"
            time_color = YELLOW
        elif stage == 2:  # 自检阶段
            time_text = "Stage Time: Self Check"
            time_color = ORANGE
        elif stage == 3:  # 倒计时阶段
            time_text = "Stage Time: Countdown"
            time_color = CYAN
        elif stage == 0:  # 未开始
            time_text = "Stage Time: Not Started"
            time_color = WHITE
        elif stage == 5:  # 比赛结算中
            time_text = "Stage Time: Settlement"
            time_color = PURPLE
        else:
            time_text = f"Match Time: {self.game_state.stage_remaining_time:.1f}"
            time_color = WHITE
            
        time_surface = self.header_font.render(time_text, True, time_color)
        self.screen.blit(time_surface, (20, 50))
        
        # 游戏阶段
        stage_text = f"Stage: {self.get_stage_text()}"
        stage_surface = self.normal_font.render(stage_text, True, LIGHT_GRAY)
        self.screen.blit(stage_surface, (self.width - 200, 50))
        
        # 状态
        status_text = f"Status: {'RUNNING' if self.game_state.is_running else 'PAUSED'}"
        status_color = GREEN if self.game_state.is_running else RED
        status_surface = self.normal_font.render(status_text, True, status_color)
        self.screen.blit(status_surface, (self.width - 400, 50))
        
        # 同步状态
        sync_text = "Sync: Active (ROS2 + File)"
        sync_surface = self.small_font.render(sync_text, True, GREEN)
        self.screen.blit(sync_surface, (self.width - 200, 75))
    
    def get_stage_text(self):
        """获取比赛阶段文本"""
        stages = {
            0: "NOT STARTED",
            1: "PREPARATION",
            2: "SELF CHECK",
            3: "COUNTDOWN",
            4: "IN PROGRESS",
            5: "SETTLEMENT"
        }
        return stages.get(self.game_state.stage, "UNKNOWN")
    
    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_mouse_down(event)
            
            elif event.type == pygame.KEYDOWN:
                self.handle_key_down(event)
        
        return True
    
    def handle_mouse_down(self, event):
        """处理鼠标按下事件"""
        mouse_pos = event.pos
        
        # 检查输入框点击
        clicked_box = None
        
        for box_id, box_data in self.input_boxes.items():
            if box_data['rect'].collidepoint(mouse_pos):
                clicked_box = box_id
                break
        
        # 取消所有激活的输入框
        for box_data in self.input_boxes.values():
            box_data['active'] = False
        
        # 激活点击的输入框
        if clicked_box:
            self.input_boxes[clicked_box]['active'] = True
            self.active_input_box = clicked_box
            # 将输入框文本复制到临时存储
            if clicked_box in self.input_original:
                self.input_texts[clicked_box] = self.input_original[clicked_box]
        else:
            self.active_input_box = None
    
    def handle_key_down(self, event):
        """处理键盘事件"""
        # 全局快捷键
        if event.key == pygame.K_r:
            self.game_state.reset_game()
            self.game_state.save_full_status_to_file()
            self.reload_input_texts()
            if self.msg_interface:
                self.msg_interface.update_messages(self.game_state)
            print("Match reset")
        elif event.key == pygame.K_s:
            self.game_state.start_game()
            self.game_state.save_full_status_to_file()
            if self.msg_interface:
                self.msg_interface.update_messages(self.game_state)
            print("Match started")
        elif event.key == pygame.K_SPACE:
            self.game_state.is_running = not self.game_state.is_running
            self.game_state.save_full_status_to_file()
            if self.msg_interface:
                self.msg_interface.update_messages(self.game_state)
            print("Game paused/resumed")
        
        # 处理输入框编辑
        if self.active_input_box and self.active_input_box in self.input_boxes:
            box_data = self.input_boxes[self.active_input_box]
            
            if event.key == pygame.K_RETURN:
                # 应用输入的值
                current_text = self.input_texts.get(self.active_input_box, "")
                try:
                    if box_data['type'] == 'int':
                        value = int(current_text)
                    
                    # 检查范围
                    if 'min' in box_data:
                        value = max(box_data['min'], value)
                    if 'max' in box_data:
                        value = min(box_data['max'], value)
                    
                    # 更新原始值
                    self.input_original[self.active_input_box] = str(value)
                    
                    # 调用回调函数
                    box_data['callback'](value)
                    
                except ValueError:
                    print(f"Invalid input: {current_text}")
                    # 恢复原始值
                    if self.active_input_box in self.input_original:
                        self.input_texts[self.active_input_box] = self.input_original[self.active_input_box]
                
                box_data['active'] = False
                self.active_input_box = None
            
            elif event.key == pygame.K_ESCAPE:
                # 恢复原始值
                if self.active_input_box in self.input_original:
                    self.input_texts[self.active_input_box] = self.input_original[self.active_input_box]
                box_data['active'] = False
                self.active_input_box = None
            
            elif event.key == pygame.K_BACKSPACE:
                current_text = self.input_texts.get(self.active_input_box, "")
                if current_text:
                    self.input_texts[self.active_input_box] = current_text[:-1]
            
            else:
                # 只允许数字输入
                if event.unicode.isdigit():
                    current_text = self.input_texts.get(self.active_input_box, "")
                    self.input_texts[self.active_input_box] = current_text + event.unicode
    
    def reload_input_texts(self):
        """重新加载输入框文本"""
        # 重新加载所有输入框文本
        for robot_id in ROBOT_IDS:
            # 红方机器人
            robot = self.game_state.get_robot('red', robot_id)
            if robot:
                self.input_texts[f'red_{robot_id}_hp'] = str(robot.hp)
                self.input_original[f'red_{robot_id}_hp'] = str(robot.hp)
                if robot_id in [1, 3, 4, 7]:
                    self.input_texts[f'red_{robot_id}_allowance'] = str(robot.allowance)
                    self.input_original[f'red_{robot_id}_allowance'] = str(robot.allowance)
            
            # 蓝方机器人
            robot = self.game_state.get_robot('blue', robot_id)
            if robot:
                self.input_texts[f'blue_{robot_id}_hp'] = str(robot.hp)
                self.input_original[f'blue_{robot_id}_hp'] = str(robot.hp)
                if robot_id in [1, 3, 4, 7]:
                    self.input_texts[f'blue_{robot_id}_allowance'] = str(robot.allowance)
                    self.input_original[f'blue_{robot_id}_allowance'] = str(robot.allowance)
        
        # 基地血量
        self.input_texts['red_outpost_hp'] = str(self.game_state.red_outpost_hp)
        self.input_original['red_outpost_hp'] = str(self.game_state.red_outpost_hp)
        
        self.input_texts['red_base_hp'] = str(self.game_state.red_base_hp)
        self.input_original['red_base_hp'] = str(self.game_state.red_base_hp)
        
        self.input_texts['blue_outpost_hp'] = str(self.game_state.blue_outpost_hp)
        self.input_original['blue_outpost_hp'] = str(self.game_state.blue_outpost_hp)
        
        self.input_texts['blue_base_hp'] = str(self.game_state.blue_base_hp)
        self.input_original['blue_base_hp'] = str(self.game_state.blue_base_hp)
        
        # 经济信息
        self.input_texts['red_gold_coins'] = str(self.game_state.red_gold_coins)
        self.input_original['red_gold_coins'] = str(self.game_state.red_gold_coins)
        
        self.input_texts['blue_gold_coins'] = str(self.game_state.blue_gold_coins)
        self.input_original['blue_gold_coins'] = str(self.game_state.blue_gold_coins)
    
    def run(self):
        """运行控制面板"""
        running = True
        clock = pygame.time.Clock()
        
        print("控制面板已启动")
        print("等待地图界面同步...")
        
        while running:
            current_time = time.time()
            
            # 处理事件
            running = self.handle_events()
            
            # 更新游戏状态
            self.game_state.update()
            
            # 定期从文件同步状态
            if current_time - self.last_update_time > self.update_interval:
                self.sync_from_file()
                self.last_update_time = current_time
            
            # 绘制UI
            self.draw_ui()
            
            clock.tick(60)
        
        pygame.quit()
    
    def sync_from_file(self):
        """从文件同步状态 - 核心修复方法"""
        status_file = os.path.join("decision_messages", "status.json")
        if not os.path.exists(status_file):
            return
        
        try:
            with open(status_file, "r", encoding='utf-8') as f:
                status = json.load(f)
            
            # 更新机器人位置（总是同步位置）
            for team in ['red', 'blue']:
                robots_key = f'{team}_robots'
                if robots_key in status:
                    for robot_id_str, robot_data in status[robots_key].items():
                        robot_id = int(robot_id_str)
                        robot = self.game_state.get_robot(team, robot_id)
                        if robot:
                            robot.x = robot_data.get('x', robot.x)
                            robot.y = robot_data.get('y', robot.y)
            
            # 更新游戏状态（总是同步时间和阶段）
            if 'game_state' in status:
                game_state_data = status['game_state']
                self.game_state.stage_remaining_time = game_state_data.get('stage_remaining_time', self.game_state.stage_remaining_time)
                self.game_state.stage = game_state_data.get('stage', self.game_state.stage)
                self.game_state.is_running = game_state_data.get('is_running', self.game_state.is_running)
            
            # 更新增益区占领状态
            if 'gain_zone_occupation' in status:
                gain_zone_data = status['gain_zone_occupation']
                for field in GAIN_ZONE_FIELDS:
                    if field in gain_zone_data:
                        setattr(self.game_state, field, gain_zone_data[field])
            
            # 更新显示 - 关键：只更新非激活的输入框
            self.update_display_from_state()
            
        except Exception as e:
            print(f"同步状态时出错: {e}")
    
    def update_display_from_state(self):
        """从游戏状态更新显示 - 核心修复"""
        # 更新红方机器人显示
        for robot_id in ROBOT_IDS:
            robot = self.game_state.get_robot('red', robot_id)
            if robot:
                # 更新坐标显示
                pos_box_id = f'red_{robot_id}_pos'
                if pos_box_id in self.display_boxes:
                    self.display_boxes[pos_box_id]['text'] = f"({int(robot.x)}, {int(robot.y)}) cm"
                
                # 更新血量输入框（仅当未激活时）
                hp_box_id = f'red_{robot_id}_hp'
                if hp_box_id in self.input_boxes and not self.input_boxes[hp_box_id]['active']:
                    self.input_texts[hp_box_id] = str(robot.hp)
                    self.input_original[hp_box_id] = str(robot.hp)
                
                # 更新发弹量输入框（仅当未激活时）
                allowance_box_id = f'red_{robot_id}_allowance'
                if allowance_box_id in self.input_boxes and not self.input_boxes[allowance_box_id]['active']:
                    self.input_texts[allowance_box_id] = str(robot.allowance)
                    self.input_original[allowance_box_id] = str(robot.allowance)
        
        # 更新蓝方机器人显示
        for robot_id in ROBOT_IDS:
            robot = self.game_state.get_robot('blue', robot_id)
            if robot:
                # 更新坐标显示
                pos_box_id = f'blue_{robot_id}_pos'
                if pos_box_id in self.display_boxes:
                    self.display_boxes[pos_box_id]['text'] = f"({int(robot.x)}, {int(robot.y)}) cm"
                
                # 更新血量输入框（仅当未激活时）
                hp_box_id = f'blue_{robot_id}_hp'
                if hp_box_id in self.input_boxes and not self.input_boxes[hp_box_id]['active']:
                    self.input_texts[hp_box_id] = str(robot.hp)
                    self.input_original[hp_box_id] = str(robot.hp)
                
                # 更新发弹量输入框（仅当未激活时）
                allowance_box_id = f'blue_{robot_id}_allowance'
                if allowance_box_id in self.input_boxes and not self.input_boxes[allowance_box_id]['active']:
                    self.input_texts[allowance_box_id] = str(robot.allowance)
                    self.input_original[allowance_box_id] = str(robot.allowance)
        
        # 更新基地血量（仅当未激活时）
        if 'red_outpost_hp' in self.input_boxes and not self.input_boxes['red_outpost_hp']['active']:
            self.input_texts['red_outpost_hp'] = str(self.game_state.red_outpost_hp)
            self.input_original['red_outpost_hp'] = str(self.game_state.red_outpost_hp)
        
        if 'red_base_hp' in self.input_boxes and not self.input_boxes['red_base_hp']['active']:
            self.input_texts['red_base_hp'] = str(self.game_state.red_base_hp)
            self.input_original['red_base_hp'] = str(self.game_state.red_base_hp)
        
        if 'blue_outpost_hp' in self.input_boxes and not self.input_boxes['blue_outpost_hp']['active']:
            self.input_texts['blue_outpost_hp'] = str(self.game_state.blue_outpost_hp)
            self.input_original['blue_outpost_hp'] = str(self.game_state.blue_outpost_hp)
        
        if 'blue_base_hp' in self.input_boxes and not self.input_boxes['blue_base_hp']['active']:
            self.input_texts['blue_base_hp'] = str(self.game_state.blue_base_hp)
            self.input_original['blue_base_hp'] = str(self.game_state.blue_base_hp)
        
        # 更新经济信息（仅当未激活时）
        if 'red_gold_coins' in self.input_boxes and not self.input_boxes['red_gold_coins']['active']:
            self.input_texts['red_gold_coins'] = str(self.game_state.red_gold_coins)
            self.input_original['red_gold_coins'] = str(self.game_state.red_gold_coins)
        
        if 'blue_gold_coins' in self.input_boxes and not self.input_boxes['blue_gold_coins']['active']:
            self.input_texts['blue_gold_coins'] = str(self.game_state.blue_gold_coins)
            self.input_original['blue_gold_coins'] = str(self.game_state.blue_gold_coins)
