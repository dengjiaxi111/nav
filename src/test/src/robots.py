# src/robots.py - 完整版机器人类，修复哨兵移动控制
import pygame
import math
from config import *

class Robot:
    def __init__(self, robot_id, team, x, y):
        self.robot_id = robot_id
        self.team = team
        self.x = x  # cm (实际坐标)
        self.y = y  # cm (实际坐标)
        self.is_draggable = (robot_id != 7)  # Sentry不可拖动
        
        # 基本状态属性
        self.hp = DEFAULT_HP.get(robot_id, 100)
        self.allowance = DEFAULT_ALLOWANCE.get(robot_id, 100) if robot_id in DEFAULT_ALLOWANCE else 0
        
        # 新增：移动控制属性
        self.target_x = x  # 目标点x坐标
        self.target_y = y  # 目标点y坐标
        self.has_target = False  # 是否有目标点
        self.move_speed = 100.0  # 移动速度 (cm/s)
        self.is_moving = False  # 是否正在移动
        
        # 增益效果
        self.hp_recovery_buff = BUFF_DEFAULT
        self.heat_cooling_buff = BUFF_DEFAULT
        self.defense_buff = BUFF_DEFAULT
        self.negative_defense_buff = BUFF_DEFAULT
        self.attack_buff = BUFF_DEFAULT
        
        # 方向角
        self.yaw = 0.0  # 正北为0度
        
        # RFID状态
        self.rfid_status = 0
        
        # 显示属性 - 调整机器人显示大小
        self.radius = 16  # 从20减小到16以适应新分辨率
        self.dragging = False
        self.color = RED if team == 'red' else BLUE
        self.highlight_color = ORANGE
        
        # 机器人最大血量
        self.max_hp = DEFAULT_HP.get(robot_id, 100)
        
        # 是否在场上
        self.is_active = True
    
    def to_screen_pos(self, x=None, y=None):
        """将实际坐标转换为屏幕坐标"""
        if x is None:
            x = self.x
        if y is None:
            y = self.y
        
        screen_x = MAP_OFFSET_X + int(x * UI_SCALE)
        screen_y = MAP_OFFSET_Y + MAP_DISPLAY_HEIGHT - int(y * UI_SCALE)
        return screen_x, screen_y
    
    def from_screen_pos(self, screen_x, screen_y):
        """将屏幕坐标转换为实际坐标"""
        rel_x = screen_x - MAP_OFFSET_X
        rel_y = screen_y - MAP_OFFSET_Y
        
        x = int(rel_x / UI_SCALE)
        y = MAP_REAL_HEIGHT - int(rel_y / UI_SCALE)
        
        return x, y
    
    def draw(self, screen):
        """在屏幕上绘制机器人"""
        screen_x, screen_y = self.to_screen_pos()
        
        # 绘制圆形
        pygame.draw.circle(screen, self.color, (screen_x, screen_y), self.radius)
        pygame.draw.circle(screen, BLACK, (screen_x, screen_y), self.radius, 2)
        
        # 绘制机器人编号 - 调整字体大小
        font = pygame.font.SysFont(None, 22)  # 从24减小到22
        text = font.render(str(self.robot_id), True, WHITE)
        text_rect = text.get_rect(center=(screen_x, screen_y))
        screen.blit(text, text_rect)
        
        # 如果被选中，绘制高亮
        if self.dragging:
            pygame.draw.circle(screen, YELLOW, (screen_x, screen_y), self.radius + 2, 2)  # 从+3减小到+2
            
        # 如果是哨兵且有目标，绘制目标方向
        if self.robot_id == 7 and self.has_target:
            target_screen_x, target_screen_y = self.to_screen_pos(self.target_x, self.target_y)
            pygame.draw.line(screen, ORANGE, (screen_x, screen_y), (target_screen_x, target_screen_y), 1)
            
            # 绘制目标点
            pygame.draw.circle(screen, ORANGE, (target_screen_x, target_screen_y), 4)
    
    def contains_point(self, point):
        """检查点是否在机器人范围内"""
        screen_x, screen_y = self.to_screen_pos()
        distance = ((point[0] - screen_x) ** 2 + (point[1] - screen_y) ** 2) ** 0.5
        return distance <= self.radius
    
    def update_position(self, screen_pos):
        """根据屏幕坐标更新实际坐标（仅用于拖拽）"""
        if not self.is_draggable:
            return
            
        # 转换为实际坐标
        new_x, new_y = self.from_screen_pos(screen_pos[0], screen_pos[1])
        
        # 限制在边界内
        new_x = max(0, min(new_x, MAP_REAL_WIDTH))
        new_y = max(0, min(new_y, MAP_REAL_HEIGHT))
        
        # 更新实际坐标
        self.x = new_x
        self.y = new_y
    
    def set_target_point(self, target_x, target_y):
        """设置目标点并开始移动"""
        self.target_x = target_x
        self.target_y = target_y
        self.has_target = True
        self.is_moving = True
        
    def update_movement(self, dt):
        """更新机器人的移动（单位：cm/s）"""
        if not self.has_target or not self.is_moving:
            return
        
        # 计算当前位置到目标点的距离
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 5.0:  # 到达目标点（5cm范围内）
            self.x = self.target_x
            self.y = self.target_y
            self.has_target = False
            self.is_moving = False
            return
        
        # 计算移动方向
        if distance > 0:
            # 计算单位向量
            vx = dx / distance
            vy = dy / distance
            
            # 计算移动距离（速度 * 时间）
            move_distance = min(self.move_speed * dt, distance)
            
            # 更新位置
            self.x += vx * move_distance
            self.y += vy * move_distance
            
            # 检查是否接近目标点
            new_distance = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
            if new_distance < 5.0:
                self.x = self.target_x
                self.y = self.target_y
                self.has_target = False
                self.is_moving = False
    
    def get_status_dict(self):
        """获取机器人状态字典"""
        return {
            'x': self.x,
            'y': self.y,
            'hp': self.hp,
            'allowance': self.allowance,
            'hp_recovery_buff': self.hp_recovery_buff,
            'heat_cooling_buff': self.heat_cooling_buff,
            'defense_buff': self.defense_buff,
            'negative_defense_buff': self.negative_defense_buff,
            'attack_buff': self.attack_buff,
            'yaw': self.yaw,
            'rfid_status': self.rfid_status,
            'max_hp': self.max_hp,
            'is_active': self.is_active,
            'has_target': self.has_target,
            'target_x': self.target_x,
            'target_y': self.target_y,
            'is_moving': self.is_moving
        }
    
    def set_status_from_dict(self, status_dict):
        """从字典设置机器人状态"""
        self.x = status_dict.get('x', self.x)
        self.y = status_dict.get('y', self.y)
        self.hp = status_dict.get('hp', self.hp)
        self.allowance = status_dict.get('allowance', self.allowance)
        self.hp_recovery_buff = status_dict.get('hp_recovery_buff', self.hp_recovery_buff)
        self.heat_cooling_buff = status_dict.get('heat_cooling_buff', self.heat_cooling_buff)
        self.defense_buff = status_dict.get('defense_buff', self.defense_buff)
        self.negative_defense_buff = status_dict.get('negative_defense_buff', self.negative_defense_buff)
        self.attack_buff = status_dict.get('attack_buff', self.attack_buff)
        self.yaw = status_dict.get('yaw', self.yaw)
        self.rfid_status = status_dict.get('rfid_status', self.rfid_status)
        self.max_hp = status_dict.get('max_hp', self.max_hp)
        self.is_active = status_dict.get('is_active', self.is_active)
        self.has_target = status_dict.get('has_target', self.has_target)
        self.target_x = status_dict.get('target_x', self.target_x)
        self.target_y = status_dict.get('target_y', self.target_y)
        self.is_moving = status_dict.get('is_moving', self.is_moving)
