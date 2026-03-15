# src/map_ui.py - 国赛版（修复：地图界面不再发布ROS2消息，仅同步位置文件）

import pygame
import sys
import os
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from config import *
from sentry_controller import SentryController
from sentry_subscriber import SentryStatusDisplay

class MapUI:
    def __init__(self, game_state, msg_interface):
        pygame.init()
        
        # 创建窗口
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("RoboMaster Referee System Simulator - Map View")
        
        # 游戏状态和消息接口（消息接口仅用于传递给哨兵控制器，但不再用于发布）
        self.game_state = game_state
        self.msg_interface = msg_interface
        
        # 添加哨兵控制器（作为ROS2节点，只订阅不发布）
        self.sentry_controller = SentryController(game_state)
        
        # 加载地图图片
        try:
            self.map_image = pygame.image.load("assets/2026map.png")
            self.map_image = pygame.transform.scale(
                self.map_image, (MAP_DISPLAY_WIDTH, MAP_DISPLAY_HEIGHT)
            )
            print(f"✓ Map image loaded: {MAP_DISPLAY_WIDTH}x{MAP_DISPLAY_HEIGHT}")
            print(f"✓ Map offset: ({MAP_OFFSET_X}, {MAP_OFFSET_Y})")
        except Exception as e:
            print(f"Warning: Cannot load map image: {e}")
            print("Using colored background instead")
            self.map_image = None
        
        # UI元素
        self.font = pygame.font.SysFont(None, 26)
        self.small_font = pygame.font.SysFont(None, 20)
        self.tiny_font = pygame.font.SysFont(None, 16)
        
        # 按钮定义
        button_width = 120
        button_height = 35
        button_y = 15
        button_spacing = 15
        total_buttons_width = 5 * button_width + 4 * button_spacing
        start_x = (SCREEN_WIDTH - total_buttons_width) // 2
        
        self.start_button = pygame.Rect(start_x, button_y, button_width, button_height)
        self.reset_button = pygame.Rect(start_x + button_width + button_spacing, button_y, button_width, button_height)
        self.prep_button = pygame.Rect(start_x + 2*(button_width + button_spacing), button_y, button_width, button_height)
        self.self_check_button = pygame.Rect(start_x + 3*(button_width + button_spacing), button_y, button_width, button_height)
        self.countdown_button = pygame.Rect(start_x + 4*(button_width + button_spacing), button_y, button_width, button_height)
        
        # 拖拽状态
        self.dragging_robot = None
        self.drag_offset = (0, 0)
        
        # 阶段倒计时相关
        self.stage_timers = {
            2: 15.0,
            3: 5.0,
            1: 0.0
        }
        self.stage_start_time = None
        self.current_stage_timer = None
        
        # 时钟
        self.clock = pygame.time.Clock()
        self.running = True
        
        # 上次文件更新时间（不再用于ROS2消息发布）
        self.last_msg_update = 0
        self.msg_update_interval = 0.1
        
        # 增益区占领检测
        self.last_zone_check = 0
        self.zone_check_interval = 0.5
        
        # 哨兵状态更新时间
        self.last_sentry_update = 0
        self.sentry_update_interval = 0.5
        
        # 时间相关
        self.last_time = time.time()
        
        print(f"Map UI initialized: {SCREEN_WIDTH}x{SCREEN_HEIGHT}")
        print(f"Map display area: {MAP_DISPLAY_WIDTH}x{MAP_DISPLAY_HEIGHT}")
        print(f"UI Scale: {UI_SCALE}")
        print("✓ 哨兵控制器已初始化")
        print("✓ 订阅决策系统话题: /sentry/target_position, /sentry/control")
        print("⚠ 地图界面不再发布ROS2消息，所有状态更新由控制面板负责")
    
    def save_state_to_file(self):
        """保存状态到文件 - 仅保存位置和游戏阶段，不发布ROS2消息"""
        try:
            # 先读取现有状态
            existing_status = {}
            status_file = os.path.join("decision_messages", "status.json")
            if os.path.exists(status_file):
                with open(status_file, "r", encoding='utf-8') as f:
                    existing_status = json.load(f)
            
            # 创建当前状态的基础结构
            current_status = {}
            
            # 只保存机器人位置和游戏状态
            current_status['red_robots'] = {}
            current_status['blue_robots'] = {}
            
            # 保存机器人位置
            for robot_id, robot in self.game_state.red_robots.items():
                current_status['red_robots'][str(robot_id)] = {
                    'x': robot.x,
                    'y': robot.y
                }
            
            for robot_id, robot in self.game_state.blue_robots.items():
                current_status['blue_robots'][str(robot_id)] = {
                    'x': robot.x,
                    'y': robot.y
                }
            
            # 保存游戏状态 - 确保阶段信息被保存
            current_status['game_state'] = {
                'stage': self.game_state.stage,
                'stage_remaining_time': self.game_state.stage_remaining_time,
                'is_running': self.game_state.is_running
            }
            
            # 保存增益区占领状态（可选，但控制面板也会同步）
            current_status['gain_zone_occupation'] = {
                'supply_zone_occupation': self.game_state.supply_zone_occupation,
                'central_highland_occupation': self.game_state.central_highland_occupation,
                'trapezoid_highland_occupation': self.game_state.trapezoid_highland_occupation,
                'fortress_gain_point_occupation': self.game_state.fortress_gain_point_occupation,
                'outpost_gain_point_occupation': self.game_state.outpost_gain_point_occupation,
                'base_gain_point_occupation': self.game_state.base_gain_point_occupation
            }
            
            # 合并状态：保留现有状态，只更新位置和游戏状态
            merged_status = existing_status.copy()
            
            # 更新机器人位置
            for team in ['red_robots', 'blue_robots']:
                if team in current_status:
                    if team not in merged_status:
                        merged_status[team] = {}
                    for robot_id, robot_data in current_status[team].items():
                        if robot_id not in merged_status[team]:
                            merged_status[team][robot_id] = {}
                        merged_status[team][robot_id]['x'] = robot_data['x']
                        merged_status[team][robot_id]['y'] = robot_data['y']
            
            # 更新游戏状态
            if 'game_state' in current_status:
                merged_status['game_state'] = current_status['game_state']
            
            # 更新增益区占领状态
            if 'gain_zone_occupation' in current_status:
                merged_status['gain_zone_occupation'] = current_status['gain_zone_occupation']
            
            # 确保目录存在
            os.makedirs("decision_messages", exist_ok=True)
            
            with open(status_file, "w", encoding='utf-8') as f:
                json.dump(merged_status, f, indent=2, ensure_ascii=False)
            
            # 注意：此处不再调用 msg_interface.update_messages()
            # ROS2消息发布已完全交给控制面板处理
            
        except Exception as e:
            print(f"地图UI保存状态时出错: {e}")
    
    def draw_ui(self):
        """Draw the entire UI"""
        # 绘制背景
        self.screen.fill(DARK_GRAY)
        
        # 绘制地图背景区域
        pygame.draw.rect(self.screen, BLACK, 
                        (MAP_OFFSET_X, MAP_OFFSET_Y, 
                         MAP_DISPLAY_WIDTH, MAP_DISPLAY_HEIGHT))
        
        # 绘制地图
        if self.map_image:
            self.screen.blit(self.map_image, (MAP_OFFSET_X, MAP_OFFSET_Y))
        else:
            # 绘制网格作为背景
            for i in range(0, MAP_REAL_WIDTH + 1, 500):  # 每5米一条线
                x = MAP_OFFSET_X + int(i * UI_SCALE)
                pygame.draw.line(self.screen, GRAY, 
                               (x, MAP_OFFSET_Y), 
                               (x, MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT), 1)
            
            for i in range(0, MAP_REAL_HEIGHT + 1, 300):  # 每3米一条线
                y = MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT - int(i * UI_SCALE)
                pygame.draw.line(self.screen, GRAY,
                               (MAP_OFFSET_X, y),
                               (MAP_OFFSET_X + MAP_DISPLAY_WIDTH, y), 1)
        
        # 绘制坐标轴
        self.draw_coordinate_axes()
        
        # 绘制五个按钮（全部在顶部一排）
        self.draw_button(self.start_button, "Start Match", GREEN)
        self.draw_button(self.reset_button, "Reset Match", RED)
        self.draw_button(self.prep_button, "Preparation", YELLOW)
        self.draw_button(self.self_check_button, "Self Check", ORANGE)
        self.draw_button(self.countdown_button, "Countdown", CYAN)
        
        # 绘制倒计时（统一在右上角）
        self.draw_timer()
        
        # 绘制机器人
        try:
            for robot in self.game_state.get_all_robots():
                robot.draw(self.screen)
                # 绘制机器人坐标
                self.draw_robot_coordinates(robot)
        except Exception as e:
            print(f"Error drawing robots: {e}")
        
        # 绘制坐标信息
        self.draw_coordinate_info()
        
        # 绘制队伍信息（调整到地图区域顶部）
        self.draw_team_info()
        
        # 绘制同步状态（提示ROS2发布仅由控制面板负责）
        self.draw_sync_status()
        
        # 绘制哨兵目标线（简化显示）
        self.draw_sentry_target_line()
        
        pygame.display.flip()
    
    def draw_sentry_target_line(self):
        """绘制哨兵目标线（简化版本）"""
        # 获取哨兵机器人
        sentry = self.game_state.red_robots.get(7)
        if not sentry or not sentry.has_target:
            return
        
        # 转换当前和目标位置到屏幕坐标
        current_screen = sentry.to_screen_pos()
        target_screen = sentry.to_screen_pos(sentry.target_x, sentry.target_y)
        
        # 绘制虚线
        line_color = ORANGE
        dash_length = 5
        gap_length = 3
        
        # 计算总距离和方向
        dx = target_screen[0] - current_screen[0]
        dy = target_screen[1] - current_screen[1]
        distance = (dx**2 + dy**2)**0.5
        
        if distance > 0:
            # 计算单位向量
            unit_dx = dx / distance
            unit_dy = dy / distance
            
            # 绘制虚线
            drawn_distance = 0
            while drawn_distance < distance:
                # 计算线段起点和终点
                start_distance = drawn_distance
                end_distance = min(drawn_distance + dash_length, distance)
                
                start_x = current_screen[0] + unit_dx * start_distance
                start_y = current_screen[1] + unit_dy * start_distance
                end_x = current_screen[0] + unit_dx * end_distance
                end_y = current_screen[1] + unit_dy * end_distance
                
                # 绘制线段
                pygame.draw.line(self.screen, line_color, 
                               (int(start_x), int(start_y)), 
                               (int(end_x), int(end_y)), 2)
                
                # 更新已绘制距离
                drawn_distance = end_distance + gap_length
    
    def draw_robot_coordinates(self, robot):
        """在机器人下方绘制坐标"""
        screen_x, screen_y = robot.to_screen_pos()
        
        # 坐标文本（转换为米）
        coord_text = f"({int(robot.x)}cm, {int(robot.y)}cm)"
        
        # 绘制坐标文本
        coord_surface = self.tiny_font.render(coord_text, True, WHITE)
        coord_rect = coord_surface.get_rect(center=(screen_x, screen_y + 20))
        
        # 绘制半透明背景
        bg_rect = coord_rect.copy()
        bg_rect.inflate_ip(6, 3)
        bg_surface = pygame.Surface(bg_rect.size, pygame.SRCALPHA)
        bg_surface.fill((0, 0, 0, 180))
        self.screen.blit(bg_surface, bg_rect)
        
        # 绘制坐标文本
        self.screen.blit(coord_surface, coord_rect)
    
    def draw_sync_status(self):
        """绘制同步状态 - 提示ROS2发布由控制面板负责"""
        sync_text = "Sync: File only (ROS2 publishing from Control Panel)"
        sync_surface = self.small_font.render(sync_text, True, GREEN)
        # 调整位置到右下角
        self.screen.blit(sync_surface, (SCREEN_WIDTH - 300, SCREEN_HEIGHT - 25))
    
    def draw_button(self, rect, text, color):
        """Draw a button"""
        # 绘制按钮背景
        pygame.draw.rect(self.screen, color, rect, border_radius=4)
        # 绘制按钮边框
        pygame.draw.rect(self.screen, BLACK, rect, 2, border_radius=4)
        
        # 绘制按钮文字
        text_surface = self.small_font.render(text, True, BLACK)
        text_rect = text_surface.get_rect(center=rect.center)
        self.screen.blit(text_surface, text_rect)
    
    def draw_timer(self):
        """Draw countdown timer - 统一在右上角显示"""
        stage = self.game_state.stage
        
        # 统一的位置：右上角
        time_pos_x = SCREEN_WIDTH - 230
        time_pos_y = 20
        
        if stage == 4:  # 比赛中
            minutes = int(self.game_state.stage_remaining_time // 60)
            seconds = int(self.game_state.stage_remaining_time % 60)
            time_text = f"{minutes:02d}:{seconds:02d}"
            
            # 根据剩余时间改变颜色
            if self.game_state.is_running:
                if self.game_state.stage_remaining_time < 60:  # Last 1 minute
                    color = RED
                elif self.game_state.stage_remaining_time < 180:  # Last 3 minutes
                    color = YELLOW
                else:
                    color = GREEN
            else:
                color = WHITE
            
            time_surface = self.font.render(time_text, True, color)
            # 确保文字不会超出屏幕
            text_width = time_surface.get_width()
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - text_width - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        elif stage == 1:  # 准备阶段
            time_text = "Ready"
            time_surface = self.font.render(time_text, True, YELLOW)
            text_width = time_surface.get_width()
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - text_width - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        elif stage == 2:  # 自检阶段
            if self.current_stage_timer is not None:
                time_text = f"Self Check: {int(self.current_stage_timer)}s"
            else:
                time_text = "Self Check: 15s"
            time_surface = self.font.render(time_text, True, ORANGE)
            text_width = time_surface.get_width()
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - text_width - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        elif stage == 3:  # 倒计时阶段
            if self.current_stage_timer is not None:
                time_text = f"Countdown: {int(self.current_stage_timer)}s"
            else:
                time_text = "Countdown: 5s"
            time_surface = self.font.render(time_text, True, CYAN)
            text_width = time_surface.get_width()
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - text_width - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        elif stage == 0:  # 未开始
            time_text = "Not Started"
            time_surface = self.font.render(time_text, True, WHITE)
            text_width = time_surface.get_width()
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - text_width - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        elif stage == 5:  # 结算中
            time_text = "Settlement"
            time_surface = self.font.render(time_text, True, PURPLE)
            text_width = time_surface.get_width()
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - text_width - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        # 绘制比赛阶段（在时间下方）
        stage_text = self.get_stage_text()
        stage_surface = self.small_font.render(stage_text, True, WHITE)
        stage_pos_x = time_pos_x
        stage_pos_y = time_pos_y + 30
        self.screen.blit(stage_surface, (stage_pos_x, stage_pos_y))
    
    def get_stage_text(self):
        """Get game stage text"""
        stages = {
            0: "Not Started",
            1: "Preparation",
            2: "Self Check",
            3: "Countdown",
            4: "In Progress",
            5: "Settlement"
        }
        return stages.get(self.game_state.stage, "Unknown")
    
    def draw_coordinate_axes(self):
        """Draw coordinate axes"""
        # X轴
        x_start = MAP_OFFSET_X
        x_end = MAP_OFFSET_X + MAP_DISPLAY_WIDTH
        y_bottom = MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT
        
        pygame.draw.line(self.screen, WHITE, 
                        (x_start, y_bottom),
                        (x_end, y_bottom), 2)
        
        # Y轴
        pygame.draw.line(self.screen, WHITE,
                        (x_start, MAP_OFFSET_Y),
                        (x_start, y_bottom), 2)
        
        # X轴刻度
        for i in range(0, MAP_REAL_WIDTH + 1, 500):  # Every 5 meters
            x = MAP_OFFSET_X + int(i * UI_SCALE)
            pygame.draw.line(self.screen, WHITE,
                           (x, y_bottom - 8),
                           (x, y_bottom + 8), 2)
            
            if i > 0:
                text = f"{i/100}m"
                text_surface = self.tiny_font.render(text, True, WHITE)
                self.screen.blit(text_surface, (x - 15, y_bottom + 12))
    
    def draw_coordinate_info(self):
        """Draw coordinate information"""
        # 显示坐标范围 - 调整到左下角
        info_text = f"Map: {MAP_REAL_WIDTH/100}m x {MAP_REAL_HEIGHT/100}m"
        info_surface = self.small_font.render(info_text, True, WHITE)
        self.screen.blit(info_surface, (10, SCREEN_HEIGHT - 25))
        
        # 显示鼠标位置
        mouse_pos = pygame.mouse.get_pos()
        if (MAP_OFFSET_X <= mouse_pos[0] <= MAP_OFFSET_X + MAP_DISPLAY_WIDTH and
            MAP_OFFSET_Y <= mouse_pos[1] <= MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT):
            
            # 转换为实际坐标
            rel_x = mouse_pos[0] - MAP_OFFSET_X
            rel_y = mouse_pos[1] - MAP_OFFSET_Y
            actual_x = int(rel_x / UI_SCALE)
            actual_y = MAP_REAL_HEIGHT - int(rel_y / UI_SCALE)
            
            mouse_text = f"Mouse: ({actual_x}, {actual_y}) cm"
            mouse_surface = self.small_font.render(mouse_text, True, YELLOW)
            self.screen.blit(mouse_surface, (10, SCREEN_HEIGHT - 50))
    
    def draw_team_info(self):
        """Draw team information - 调整到地图区域顶部"""
        # 红方信息 - 放在地图区域左上角
        red_text = self.small_font.render("Red Team", True, RED)
        self.screen.blit(red_text, (MAP_OFFSET_X + 10, MAP_OFFSET_Y + 10))
        
        # 蓝方信息 - 放在地图区域右上角
        blue_text = self.small_font.render("Blue Team", True, BLUE)
        self.screen.blit(blue_text, (MAP_OFFSET_X + MAP_DISPLAY_WIDTH - 90, MAP_OFFSET_Y + 10))
    
    def handle_events(self):
        """Handle events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # 获取当前时间
                current_time = time.time()
                self.handle_mouse_down(event, current_time)
            
            elif event.type == pygame.MOUSEBUTTONUP:
                current_time = time.time()
                self.handle_mouse_up(event, current_time)
            
            elif event.type == pygame.MOUSEMOTION:
                current_time = time.time()
                self.handle_mouse_motion(event, current_time)
            
            elif event.type == pygame.KEYDOWN:
                self.handle_key_down(event)
    
    def handle_key_down(self, event):
        """Handle keyboard events"""
        if event.key == pygame.K_SPACE:
            if self.game_state.is_running:
                print("Game paused")
            else:
                print("Game resumed")
            self.game_state.is_running = not self.game_state.is_running
            self.save_state_to_file()
        elif event.key == pygame.K_F1:
            print("Press Ctrl+C to exit, then run: python3 src/main.py control")
        elif event.key == pygame.K_r:
            # 重置比赛快捷键
            self.game_state.reset_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            # 重置哨兵控制器
            if self.sentry_controller:
                self.sentry_controller.reset()
            self.save_state_to_file()
            print("Match reset (R key)")
        elif event.key == pygame.K_s:
            # 开始比赛快捷键
            self.game_state.start_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            self.save_state_to_file()
            print("Match started (S key)")
        elif event.key == pygame.K_1:
            # 准备阶段快捷键
            self.game_state.stage = 1
            self.game_state.is_running = False
            self.stage_start_time = None
            self.current_stage_timer = None
            self.save_state_to_file()
            print("Preparation stage set (1 key)")
        elif event.key == pygame.K_2:
            # 自检阶段快捷键
            self.game_state.stage = 2
            self.game_state.is_running = False
            self.stage_start_time = time.time()
            self.current_stage_timer = 15.0
            self.save_state_to_file()
            print("Self check stage set (2 key)")
        elif event.key == pygame.K_3:
            # 倒计时阶段快捷键
            self.game_state.stage = 3
            self.game_state.is_running = False
            self.stage_start_time = time.time()
            self.current_stage_timer = 5.0
            self.save_state_to_file()
            print("Countdown stage set (3 key)")
    
    def handle_mouse_down(self, event, current_time):
        """Handle mouse down event"""
        mouse_pos = event.pos
        
        # 检查按钮点击
        if self.start_button.collidepoint(mouse_pos):
            # 开始比赛
            self.game_state.start_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            print("Match started from Map UI")
            # 更新状态（只存文件）
            self.save_state_to_file()
            return
        
        elif self.reset_button.collidepoint(mouse_pos):
            # 重置比赛
            self.game_state.reset_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            # 重置哨兵控制器
            if self.sentry_controller:
                self.sentry_controller.reset()
            print("Match reset from Map UI")
            # 更新状态
            self.save_state_to_file()
            return
        
        elif self.prep_button.collidepoint(mouse_pos):
            # 准备阶段
            self.game_state.stage = 1
            self.game_state.is_running = False
            self.stage_start_time = None
            self.current_stage_timer = None
            print("Preparation stage set from Map UI")
            self.save_state_to_file()
            return
        
        elif self.self_check_button.collidepoint(mouse_pos):
            # 自检阶段
            self.game_state.stage = 2
            self.game_state.is_running = False
            self.stage_start_time = current_time
            self.current_stage_timer = 15.0
            print("Self check stage set from Map UI (15s)")
            self.save_state_to_file()
            return
        
        elif self.countdown_button.collidepoint(mouse_pos):
            # 倒计时阶段
            self.game_state.stage = 3
            self.game_state.is_running = False
            self.stage_start_time = current_time
            self.current_stage_timer = 5.0
            print("Countdown stage set from Map UI (5s)")
            self.save_state_to_file()
            return
        
        # 检查是否在地图区域内
        if not (MAP_OFFSET_X <= mouse_pos[0] <= MAP_OFFSET_X + MAP_DISPLAY_WIDTH and
                MAP_OFFSET_Y <= mouse_pos[1] <= MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT):
            # 不在任何按钮或地图区域内
            return
        
        # 检查机器人拖拽（只允许拖拽非哨兵机器人）
        for robot in self.game_state.get_all_robots():
            try:
                if robot.is_draggable and robot.contains_point(mouse_pos):
                    self.dragging_robot = robot
                    self.drag_offset = (
                        mouse_pos[0] - robot.to_screen_pos()[0],
                        mouse_pos[1] - robot.to_screen_pos()[1]
                    )
                    robot.dragging = True
                    print(f"Start dragging {robot.team} team robot {robot.robot_id}")
                    print(f"Robot position: ({robot.x:.0f}, {robot.y:.0f})")
                    break
            except Exception as e:
                print(f"Error checking robot selection: {e}")
                continue
    
    def handle_mouse_up(self, event, current_time):
        """Handle mouse up event"""
        if self.dragging_robot:
            self.dragging_robot.dragging = False
            print(f"Stop dragging {self.dragging_robot.team} team robot {self.dragging_robot.robot_id}")
            print(f"New position: ({self.dragging_robot.x:.0f}, {self.dragging_robot.y:.0f})")
            self.dragging_robot = None
            # 拖拽结束后立即更新状态文件
            self.save_state_to_file()
    
    def handle_mouse_motion(self, event, current_time):
        """Handle mouse motion event"""
        if self.dragging_robot:
            try:
                # 计算新的屏幕位置
                screen_x = event.pos[0] - self.drag_offset[0]
                screen_y = event.pos[1] - self.drag_offset[1]
                
                # 更新机器人位置
                self.dragging_robot.update_position((screen_x, screen_y))
                
                # 定期更新文件（避免过于频繁）
                if current_time - self.last_msg_update > self.msg_update_interval:
                    self.save_state_to_file()
                    self.last_msg_update = current_time
            except Exception as e:
                print(f"Error updating robot position: {e}")
    
    def run(self):
        """Run map UI"""
        last_state_update = 0
        
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                
                # 处理事件
                self.handle_events()
                
                # 更新游戏状态（包括增益区占领检测）
                if self.game_state.is_running:
                    self.game_state.update_timer()
                
                # 检查增益区占领状态
                if current_time - self.last_zone_check > self.zone_check_interval:
                    self.game_state.check_gain_zone_occupation()
                    self.last_zone_check = current_time
                
                # 更新哨兵移动
                if self.sentry_controller:
                    self.sentry_controller.update_movement(dt)
                
                # 更新阶段倒计时
                if self.stage_start_time is not None and self.current_stage_timer is not None:
                    elapsed = current_time - self.stage_start_time
                    self.current_stage_timer = max(0, self.stage_timers.get(self.game_state.stage, 0) - elapsed)
                    
                    # 如果倒计时结束，自动进入下一个阶段
                    if self.current_stage_timer <= 0:
                        if self.game_state.stage == 2:  # 自检阶段结束后自动进入倒计时
                            self.game_state.stage = 3
                            self.stage_start_time = current_time
                            self.current_stage_timer = 5.0
                            self.save_state_to_file()
                            print("Self check finished, entering countdown stage")
                        elif self.game_state.stage == 3:  # 倒计时结束后自动进入比赛
                            self.game_state.start_game()
                            self.stage_start_time = None
                            self.current_stage_timer = None
                            self.save_state_to_file()
                            print("Countdown finished, starting match")
                
                # 不再发布ROS2消息，只处理哨兵控制器的订阅
                if self.sentry_controller:
                    rclpy.spin_once(self.sentry_controller, timeout_sec=0)
                
                # 每秒更新一次状态文件
                if current_time - last_state_update > 1.0:
                    self.save_state_to_file()
                    last_state_update = current_time
                
                # 绘制UI
                self.draw_ui()
                
                # 控制帧率
                self.clock.tick(FPS)
        
        finally:
            # 清理
            if self.sentry_controller:
                self.sentry_controller.destroy_node()
            pygame.quit()
            sys.exit()
