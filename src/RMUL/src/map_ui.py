# src/map_ui.py - 省赛版本（修复：坐标实时，血量弹药从文件加载）

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

class MapUI:
    def __init__(self, game_state, msg_interface):
        pygame.init()
        
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("RoboMaster Referee System Simulator - 省赛地图")
        
        self.game_state = game_state
        self.msg_interface = msg_interface          # 用于发布ROS2消息
        self.sentry_controller = SentryController(game_state)
        
        # 加载地图图片（省赛地图）
        try:
            self.map_image = pygame.image.load("assets/RMULmap.png")
            self.map_image = pygame.transform.scale(
                self.map_image, (MAP_DISPLAY_WIDTH, MAP_DISPLAY_HEIGHT)
            )
            print(f"✓ 省赛地图加载: {MAP_DISPLAY_WIDTH}x{MAP_DISPLAY_HEIGHT}")
            print(f"✓ 地图偏移: ({MAP_OFFSET_X}, {MAP_OFFSET_Y})")
        except Exception as e:
            print(f"警告: 无法加载地图图片: {e}")
            print("使用彩色背景代替")
            self.map_image = None
        
        self.font = pygame.font.SysFont(None, 26)
        self.small_font = pygame.font.SysFont(None, 20)
        self.tiny_font = pygame.font.SysFont(None, 16)
        
        # 按钮
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
        
        self.dragging_robot = None
        self.drag_offset = (0, 0)
        
        self.stage_timers = {
            2: 15.0,
            3: 5.0,
            1: 0.0
        }
        self.stage_start_time = None
        self.current_stage_timer = None
        
        self.clock = pygame.time.Clock()
        self.running = True
        self.last_msg_update = 0
        self.msg_update_interval = 0.1
        self.last_sentry_update = 0
        self.sentry_update_interval = 0.5
        self.last_time = time.time()
        
        # ROS2消息发布计时
        self.last_msg_publish = 0
        self.msg_publish_interval = 0.1   # 100ms发布一次
        
        print(f"地图UI初始化: {SCREEN_WIDTH}x{SCREEN_HEIGHT}")
        print(f"地图显示区域: {MAP_DISPLAY_WIDTH}x{MAP_DISPLAY_HEIGHT}")
        print(f"UI缩放: {UI_SCALE}")
        print("✓ 哨兵控制器已启动")
        print("✓ 订阅决策系统话题: /sentry/target_position, /sentry/control")
    
    def save_state_to_file(self):
        """保存机器人位置到文件（仅文件同步）"""
        try:
            existing_status = {}
            status_file = os.path.join("decision_messages", "status.json")
            if os.path.exists(status_file):
                with open(status_file, "r", encoding='utf-8') as f:
                    existing_status = json.load(f)
            current_status = {}
            current_status['red_robots'] = {}
            current_status['blue_robots'] = {}
            for robot_id, robot in self.game_state.red_robots.items():
                current_status['red_robots'][str(robot_id)] = {'x': robot.x, 'y': robot.y}
            for robot_id, robot in self.game_state.blue_robots.items():
                current_status['blue_robots'][str(robot_id)] = {'x': robot.x, 'y': robot.y}
            current_status['game_state'] = {
                'stage': self.game_state.stage,
                'stage_remaining_time': self.game_state.stage_remaining_time,
                'is_running': self.game_state.is_running
            }
            merged_status = existing_status.copy()
            for team in ['red_robots', 'blue_robots']:
                if team in current_status:
                    if team not in merged_status:
                        merged_status[team] = {}
                    for robot_id, robot_data in current_status[team].items():
                        if robot_id not in merged_status[team]:
                            merged_status[team][robot_id] = {}
                        merged_status[team][robot_id]['x'] = robot_data['x']
                        merged_status[team][robot_id]['y'] = robot_data['y']
            if 'game_state' in current_status:
                merged_status['game_state'] = current_status['game_state']
            os.makedirs("decision_messages", exist_ok=True)
            with open(status_file, "w", encoding='utf-8') as f:
                json.dump(merged_status, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"地图UI保存状态时出错: {e}")
    
    def draw_ui(self):
        self.screen.fill(DARK_GRAY)
        pygame.draw.rect(self.screen, BLACK, 
                        (MAP_OFFSET_X, MAP_OFFSET_Y, MAP_DISPLAY_WIDTH, MAP_DISPLAY_HEIGHT))
        
        if self.map_image:
            self.screen.blit(self.map_image, (MAP_OFFSET_X, MAP_OFFSET_Y))
        else:
            for i in range(0, MAP_REAL_WIDTH + 1, 500):
                x = MAP_OFFSET_X + int(i * UI_SCALE)
                pygame.draw.line(self.screen, GRAY, 
                               (x, MAP_OFFSET_Y), (x, MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT), 1)
            for i in range(0, MAP_REAL_HEIGHT + 1, 300):
                y = MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT - int(i * UI_SCALE)
                pygame.draw.line(self.screen, GRAY,
                               (MAP_OFFSET_X, y), (MAP_OFFSET_X + MAP_DISPLAY_WIDTH, y), 1)
        
        self.draw_coordinate_axes()
        self.draw_button(self.start_button, "Start Match", GREEN)
        self.draw_button(self.reset_button, "Reset Match", RED)
        self.draw_button(self.prep_button, "Preparation", YELLOW)
        self.draw_button(self.self_check_button, "Self Check", ORANGE)
        self.draw_button(self.countdown_button, "Countdown", CYAN)
        self.draw_timer()
        
        for robot in self.game_state.get_all_robots():
            robot.draw(self.screen)
            self.draw_robot_coordinates(robot)
        
        self.draw_coordinate_info()
        self.draw_team_info()
        self.draw_sync_status()
        self.draw_sentry_target_line()
        
        pygame.display.flip()
    
    def draw_sentry_target_line(self):
        sentry = self.game_state.red_robots.get(7)
        if not sentry or not sentry.has_target:
            return
        current_screen = sentry.to_screen_pos()
        target_screen = sentry.to_screen_pos(sentry.target_x, sentry.target_y)
        line_color = ORANGE
        dash_length = 5
        gap_length = 3
        dx = target_screen[0] - current_screen[0]
        dy = target_screen[1] - current_screen[1]
        distance = (dx**2 + dy**2)**0.5
        if distance > 0:
            unit_dx = dx / distance
            unit_dy = dy / distance
            drawn_distance = 0
            while drawn_distance < distance:
                start_distance = drawn_distance
                end_distance = min(drawn_distance + dash_length, distance)
                start_x = current_screen[0] + unit_dx * start_distance
                start_y = current_screen[1] + unit_dy * start_distance
                end_x = current_screen[0] + unit_dx * end_distance
                end_y = current_screen[1] + unit_dy * end_distance
                pygame.draw.line(self.screen, line_color, 
                               (int(start_x), int(start_y)), 
                               (int(end_x), int(end_y)), 2)
                drawn_distance = end_distance + gap_length
    
    def draw_robot_coordinates(self, robot):
        screen_x, screen_y = robot.to_screen_pos()
        coord_text = f"({int(robot.x)}cm, {int(robot.y)}cm)"
        coord_surface = self.tiny_font.render(coord_text, True, WHITE)
        coord_rect = coord_surface.get_rect(center=(screen_x, screen_y + 20))
        bg_rect = coord_rect.copy()
        bg_rect.inflate_ip(6, 3)
        bg_surface = pygame.Surface(bg_rect.size, pygame.SRCALPHA)
        bg_surface.fill((0, 0, 0, 180))
        self.screen.blit(bg_surface, bg_rect)
        self.screen.blit(coord_surface, coord_rect)
    
    def draw_sync_status(self):
        sync_text = "Sync: ROS2 + File"
        sync_surface = self.small_font.render(sync_text, True, GREEN)
        self.screen.blit(sync_surface, (SCREEN_WIDTH - 300, SCREEN_HEIGHT - 25))
    
    def draw_button(self, rect, text, color):
        pygame.draw.rect(self.screen, color, rect, border_radius=4)
        pygame.draw.rect(self.screen, BLACK, rect, 2, border_radius=4)
        text_surface = self.small_font.render(text, True, BLACK)
        text_rect = text_surface.get_rect(center=rect.center)
        self.screen.blit(text_surface, text_rect)
    
    def draw_timer(self):
        stage = self.game_state.stage
        time_pos_x = SCREEN_WIDTH - 230
        time_pos_y = 20
        if stage == 4:
            minutes = int(self.game_state.stage_remaining_time // 60)
            seconds = int(self.game_state.stage_remaining_time % 60)
            time_text = f"{minutes:02d}:{seconds:02d}"
            if self.game_state.is_running:
                if self.game_state.stage_remaining_time < 60:
                    color = RED
                elif self.game_state.stage_remaining_time < 180:
                    color = YELLOW
                else:
                    color = GREEN
            else:
                color = WHITE
            time_surface = self.font.render(time_text, True, color)
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - time_surface.get_width() - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        elif stage == 1:
            time_text = "Ready"
            time_surface = self.font.render(time_text, True, YELLOW)
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - time_surface.get_width() - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        elif stage == 2:
            if self.current_stage_timer is not None:
                time_text = f"Self Check: {int(self.current_stage_timer)}s"
            else:
                time_text = "Self Check: 15s"
            time_surface = self.font.render(time_text, True, ORANGE)
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - time_surface.get_width() - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        elif stage == 3:
            if self.current_stage_timer is not None:
                time_text = f"Countdown: {int(self.current_stage_timer)}s"
            else:
                time_text = "Countdown: 5s"
            time_surface = self.font.render(time_text, True, CYAN)
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - time_surface.get_width() - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        elif stage == 0:
            time_text = "Not Started"
            time_surface = self.font.render(time_text, True, WHITE)
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - time_surface.get_width() - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        elif stage == 5:
            time_text = "Settlement"
            time_surface = self.font.render(time_text, True, PURPLE)
            adjusted_x = min(time_pos_x, SCREEN_WIDTH - time_surface.get_width() - 10)
            self.screen.blit(time_surface, (adjusted_x, time_pos_y))
        
        stage_text = self.get_stage_text()
        stage_surface = self.small_font.render(stage_text, True, WHITE)
        stage_pos_x = time_pos_x
        stage_pos_y = time_pos_y + 30
        self.screen.blit(stage_surface, (stage_pos_x, stage_pos_y))
    
    def get_stage_text(self):
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
        x_start = MAP_OFFSET_X
        x_end = MAP_OFFSET_X + MAP_DISPLAY_WIDTH
        y_bottom = MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT
        pygame.draw.line(self.screen, WHITE, (x_start, y_bottom), (x_end, y_bottom), 2)
        pygame.draw.line(self.screen, WHITE, (x_start, MAP_OFFSET_Y), (x_start, y_bottom), 2)
        for i in range(0, MAP_REAL_WIDTH + 1, 500):
            x = MAP_OFFSET_X + int(i * UI_SCALE)
            pygame.draw.line(self.screen, WHITE, (x, y_bottom - 8), (x, y_bottom + 8), 2)
            if i > 0:
                text = f"{i/100}m"
                text_surface = self.tiny_font.render(text, True, WHITE)
                self.screen.blit(text_surface, (x - 15, y_bottom + 12))
    
    def draw_coordinate_info(self):
        info_text = f"Map: {MAP_REAL_WIDTH/100}m x {MAP_REAL_HEIGHT/100}m"
        info_surface = self.small_font.render(info_text, True, WHITE)
        self.screen.blit(info_surface, (10, SCREEN_HEIGHT - 25))
        mouse_pos = pygame.mouse.get_pos()
        if (MAP_OFFSET_X <= mouse_pos[0] <= MAP_OFFSET_X + MAP_DISPLAY_WIDTH and
            MAP_OFFSET_Y <= mouse_pos[1] <= MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT):
            rel_x = mouse_pos[0] - MAP_OFFSET_X
            rel_y = mouse_pos[1] - MAP_OFFSET_Y
            actual_x = int(rel_x / UI_SCALE)
            actual_y = MAP_REAL_HEIGHT - int(rel_y / UI_SCALE)
            mouse_text = f"Mouse: ({actual_x}, {actual_y}) cm"
            mouse_surface = self.small_font.render(mouse_text, True, YELLOW)
            self.screen.blit(mouse_surface, (10, SCREEN_HEIGHT - 50))
    
    def draw_team_info(self):
        red_text = self.small_font.render("Red Team", True, RED)
        self.screen.blit(red_text, (MAP_OFFSET_X + 10, MAP_OFFSET_Y + 10))
        blue_text = self.small_font.render("Blue Team", True, BLUE)
        self.screen.blit(blue_text, (MAP_OFFSET_X + MAP_DISPLAY_WIDTH - 90, MAP_OFFSET_Y + 10))
    
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
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
        if event.key == pygame.K_SPACE:
            self.game_state.is_running = not self.game_state.is_running
            self.save_state_to_file()
        elif event.key == pygame.K_r:
            self.game_state.reset_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            if self.sentry_controller:
                self.sentry_controller.reset()
            self.save_state_to_file()
            print("Match reset (R key)")
        elif event.key == pygame.K_s:
            self.game_state.start_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            self.save_state_to_file()
            print("Match started (S key)")
        elif event.key == pygame.K_1:
            self.game_state.stage = 1
            self.game_state.is_running = False
            self.stage_start_time = None
            self.current_stage_timer = None
            self.save_state_to_file()
            print("Preparation stage set (1 key)")
        elif event.key == pygame.K_2:
            self.game_state.stage = 2
            self.game_state.is_running = False
            self.stage_start_time = time.time()
            self.current_stage_timer = 15.0
            self.save_state_to_file()
            print("Self check stage set (2 key)")
        elif event.key == pygame.K_3:
            self.game_state.stage = 3
            self.game_state.is_running = False
            self.stage_start_time = time.time()
            self.current_stage_timer = 5.0
            self.save_state_to_file()
            print("Countdown stage set (3 key)")
    
    def handle_mouse_down(self, event, current_time):
        mouse_pos = event.pos
        if self.start_button.collidepoint(mouse_pos):
            self.game_state.start_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            print("Match started from Map UI")
            self.save_state_to_file()
            return
        elif self.reset_button.collidepoint(mouse_pos):
            self.game_state.reset_game()
            self.stage_start_time = None
            self.current_stage_timer = None
            if self.sentry_controller:
                self.sentry_controller.reset()
            print("Match reset from Map UI")
            self.save_state_to_file()
            return
        elif self.prep_button.collidepoint(mouse_pos):
            self.game_state.stage = 1
            self.game_state.is_running = False
            self.stage_start_time = None
            self.current_stage_timer = None
            print("Preparation stage set from Map UI")
            self.save_state_to_file()
            return
        elif self.self_check_button.collidepoint(mouse_pos):
            self.game_state.stage = 2
            self.game_state.is_running = False
            self.stage_start_time = current_time
            self.current_stage_timer = 15.0
            print("Self check stage set from Map UI (15s)")
            self.save_state_to_file()
            return
        elif self.countdown_button.collidepoint(mouse_pos):
            self.game_state.stage = 3
            self.game_state.is_running = False
            self.stage_start_time = current_time
            self.current_stage_timer = 5.0
            print("Countdown stage set from Map UI (5s)")
            self.save_state_to_file()
            return
        
        if not (MAP_OFFSET_X <= mouse_pos[0] <= MAP_OFFSET_X + MAP_DISPLAY_WIDTH and
                MAP_OFFSET_Y <= mouse_pos[1] <= MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT):
            return
        
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
        if self.dragging_robot:
            self.dragging_robot.dragging = False
            print(f"Stop dragging {self.dragging_robot.team} team robot {self.dragging_robot.robot_id}")
            print(f"New position: ({self.dragging_robot.x:.0f}, {self.dragging_robot.y:.0f})")
            self.dragging_robot = None
            self.save_state_to_file()
    
    def handle_mouse_motion(self, event, current_time):
        if self.dragging_robot:
            try:
                screen_x = event.pos[0] - self.drag_offset[0]
                screen_y = event.pos[1] - self.drag_offset[1]
                self.dragging_robot.update_position((screen_x, screen_y))
                if current_time - self.last_msg_update > self.msg_update_interval:
                    self.save_state_to_file()
                    self.last_msg_update = current_time
            except Exception as e:
                print(f"Error updating robot position: {e}")
    
    def run(self):
        last_state_update = 0
        last_msg_publish = 0
        msg_publish_interval = 0.1   # 100ms发布一次

        try:
            while self.running:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time

                self.handle_events()

                if self.game_state.is_running:
                    self.game_state.update_timer()

                if self.sentry_controller:
                    self.sentry_controller.update_movement(dt)

                if self.stage_start_time is not None and self.current_stage_timer is not None:
                    elapsed = current_time - self.stage_start_time
                    self.current_stage_timer = max(0, self.stage_timers.get(self.game_state.stage, 0) - elapsed)
                    if self.current_stage_timer <= 0:
                        if self.game_state.stage == 2:
                            self.game_state.stage = 3
                            self.stage_start_time = current_time
                            self.current_stage_timer = 5.0
                            self.save_state_to_file()
                            print("Self check finished, entering countdown stage")
                        elif self.game_state.stage == 3:
                            self.game_state.start_game()
                            self.stage_start_time = None
                            self.current_stage_timer = None
                            self.save_state_to_file()
                            print("Countdown finished, starting match")

                # 定期发布ROS2消息，确保坐标来自内存，血量等来自文件
                if current_time - last_msg_publish > msg_publish_interval:
                    if self.msg_interface:
                        # 1. 保存当前所有机器人坐标
                        saved_coords = {}
                        for robot in self.game_state.get_all_robots():
                            key = (robot.team, robot.robot_id)
                            saved_coords[key] = (robot.x, robot.y)
                        
                        # 2. 从文件加载完整状态（血量、弹药等）
                        self.game_state.load_status_from_file()
                        
                        # 3. 恢复坐标为保存的值
                        for (team, robot_id), (x, y) in saved_coords.items():
                            robot = self.game_state.get_robot(team, robot_id)
                            if robot:
                                robot.x = x
                                robot.y = y
                        
                        # 4. 发布消息（此时game_state中坐标是实时的，血量等是文件中的）
                        self.msg_interface.update_messages(self.game_state)
                    last_msg_publish = current_time

                if current_time - last_state_update > 1.0:
                    self.save_state_to_file()
                    last_state_update = current_time

                # 仅spin哨兵控制器（消息接口不需要spin）
                if self.sentry_controller:
                    rclpy.spin_once(self.sentry_controller, timeout_sec=0)

                self.draw_ui()
                self.clock.tick(FPS)

        finally:
            if self.sentry_controller:
                self.sentry_controller.destroy_node()
            pygame.quit()
            sys.exit()
