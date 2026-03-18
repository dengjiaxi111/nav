#!/usr/bin/env python3
"""
哨兵移动控制器 - 修复版本（增加坐标映射功能）
订阅决策系统的目标点，控制7号机器人移动
"""
import json
import os
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sentry_decision.msg import SentryControl
from datetime import datetime
import math
from config import LOGICAL_TO_REAL_MAP  # 导入坐标映射表

class SentryController(Node):
    def __init__(self, game_state):
        # 初始化ROS2节点
        super().__init__('sentry_controller')
        
        self.game_state = game_state
        self.sentry_robot = game_state.red_robots.get(7)
        
        # 订阅决策系统发布的目标点
        self.target_sub = self.create_subscription(
            PointStamped,
            '/sentry/target_position',
            self.target_callback,
            10
        )
        
        # 订阅决策系统发布的控制消息
        self.control_sub = self.create_subscription(
            SentryControl,
            '/sentry/control',
            self.control_callback,
            10
        )
        
        # 订阅调试消息
        self.debug_sub = self.create_subscription(
            String,
            '/sentry/debug',
            self.debug_callback,
            10
        )
        
        # 当前状态
        self.current_target = None
        self.current_control = None
        self.target_timestamp = 0
        self.debug_message = ""
        
        # 移动速度 (cm/s) - 与决策系统一致
        self.move_speed = 200.0  # 每秒200厘米
        
        # 姿态名称映射
        self.posture_names = {
            1: "进攻姿态",
            2: "防御姿态", 
            3: "移动姿态"
        }
        
        self.gimbal_names = {
            0: "打符模式",
            1: "打人模式",
            2: "打前哨站模式",
            3: "不动"
        }
        
        self.spin_names = {
            0: "不动",
            1: "低速转",
            2: "变速转",
            3: "高速转"
        }
        
        # 最后接收时间
        self.last_target_time = 0
        self.last_control_time = 0
        self.last_debug_time = 0
        
        print("✓ 哨兵移动控制器已启动")
        print("✓ 订阅话题: /sentry/target_position, /sentry/control, /sentry/debug")
        print("✓ 坐标映射已启用，逻辑点将转换为真实坐标")
        
    def target_callback(self, msg):
        """目标点回调函数（增加坐标映射）"""
        try:
            # 接收到的逻辑坐标（单位：米）
            logic_x = msg.point.x
            logic_y = msg.point.y
            
            # 默认处理：将米转换为厘米（如果未映射则使用此方式）
            real_x_cm = logic_x * 100.0
            real_y_cm = logic_y * 100.0
            
            # 检查是否在映射表中
            mapped_coords = LOGICAL_TO_REAL_MAP.get((logic_x, logic_y))
            if mapped_coords is not None:
                mapped_x, mapped_y = mapped_coords
                # 如果映射值不为None，则使用映射坐标
                if mapped_x is not None and mapped_y is not None:
                    real_x_cm = mapped_x
                    real_y_cm = mapped_y
                    print(f"坐标映射: 逻辑点 ({logic_x}, {logic_y}) -> 真实点 ({real_x_cm:.1f}, {real_y_cm:.1f}) cm")
                else:
                    # 映射存在但值为None，提示用户未配置
                    print(f"警告: 逻辑点 ({logic_x}, {logic_y}) 映射未配置，将按米坐标处理")
            else:
                # 不在映射表中，按米坐标处理
                print(f"逻辑点 ({logic_x}, {logic_y}) 未在映射表中，按米坐标处理")
            
            self.current_target = (real_x_cm, real_y_cm)
            self.target_timestamp = time.time()
            self.last_target_time = time.time()
            
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 收到目标点: ({real_x_cm:.1f}, {real_y_cm:.1f}) cm")
            print(f"    原始ROS消息: ({logic_x:.2f}, {logic_y:.2f}) m")
            
            # 立即设置目标点
            if self.sentry_robot:
                # 设置新目标点
                self.sentry_robot.target_x = real_x_cm
                self.sentry_robot.target_y = real_y_cm
                self.sentry_robot.has_target = True
                self.sentry_robot.is_moving = True
                
                print(f"开始移动到目标点: ({real_x_cm:.1f}, {real_y_cm:.1f}) cm")
                
        except Exception as e:
            print(f"目标点回调错误: {e}")
            import traceback
            traceback.print_exc()
            
    def control_callback(self, msg):
        """控制消息回调函数"""
        try:
            self.current_control = {
                'gimbal_mode': msg.gimbal_mode,
                'spin_mode': msg.spin_mode,
                'posture': msg.posture,
                'ramp_mode': msg.ramp_mode
            }
            
            self.last_control_time = time.time()
            
            # 输出控制消息
            posture = msg.posture
            posture_name = self.posture_names.get(posture, f"未知({posture})")
            
            gimbal = msg.gimbal_mode
            gimbal_name = self.gimbal_names.get(gimbal, f"未知({gimbal})")
            
            spin = msg.spin_mode
            spin_name = self.spin_names.get(spin, f"未知({spin})")
            
            ramp = "飞坡" if msg.ramp_mode == 1 else "不飞坡"
            
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 控制消息: {posture_name}, 云台:{gimbal_name}, 陀螺:{spin_name}, {ramp}")
            
        except Exception as e:
            print(f"控制消息回调错误: {e}")
            
    def debug_callback(self, msg):
        """调试消息回调函数"""
        try:
            self.debug_message = msg.data
            self.last_debug_time = time.time()
            
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 决策原因: {msg.data}")
            
        except Exception as e:
            print(f"调试消息回调错误: {e}")
            
    def update_movement(self, dt):
        """更新哨兵机器人位置"""
        if not self.sentry_robot:
            return
        
        # 如果没有目标点或不移动，停止移动
        if not self.sentry_robot.has_target or not self.sentry_robot.is_moving:
            return
        
        # 保存当前位置
        old_x, old_y = self.sentry_robot.x, self.sentry_robot.y
        
        # 计算当前位置到目标点的距离
        dx = self.sentry_robot.target_x - old_x
        dy = self.sentry_robot.target_y - old_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 如果距离小于5cm，认为到达目标点
        if distance < 5.0:
            self.sentry_robot.x = self.sentry_robot.target_x
            self.sentry_robot.y = self.sentry_robot.target_y
            self.sentry_robot.has_target = False
            self.sentry_robot.is_moving = False
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 已到达目标点: ({self.sentry_robot.x:.1f}, {self.sentry_robot.y:.1f}) cm")
            return
        
        # 计算移动方向
        if distance > 0:
            # 计算单位向量
            vx = dx / distance
            vy = dy / distance
            
            # 计算移动距离（速度 * 时间）
            move_distance = min(self.move_speed * dt, distance)
            
            # 更新位置
            self.sentry_robot.x += vx * move_distance
            self.sentry_robot.y += vy * move_distance
            
            # 检查是否接近目标点
            new_distance = math.sqrt((self.sentry_robot.target_x - self.sentry_robot.x)**2 + 
                                    (self.sentry_robot.target_y - self.sentry_robot.y)**2)
            
            if new_distance < 5.0:
                self.sentry_robot.x = self.sentry_robot.target_x
                self.sentry_robot.y = self.sentry_robot.target_y
                self.sentry_robot.has_target = False
                self.sentry_robot.is_moving = False
                print(f"[{datetime.now().strftime('%H:%M:%S')}] 已到达目标点: ({self.sentry_robot.x:.1f}, {self.sentry_robot.y:.1f}) cm")
        
        # 检查是否长时间没有收到目标点（超过2秒）
        current_time = time.time()
        if current_time - self.last_target_time > 2.0 and self.sentry_robot.has_target:
            print(f"警告: 超过2秒未收到新的目标点，当前位置: ({self.sentry_robot.x:.1f}, {self.sentry_robot.y:.1f}) cm")
            
    def get_status(self):
        """获取当前状态"""
        if self.sentry_robot:
            robot_status = {
                'x': self.sentry_robot.x,
                'y': self.sentry_robot.y,
                'has_target': self.sentry_robot.has_target,
                'target_x': self.sentry_robot.target_x,
                'target_y': self.sentry_robot.target_y,
                'is_moving': self.sentry_robot.is_moving
            }
        else:
            robot_status = None
            
        return {
            'has_target': self.current_target is not None,
            'target': self.current_target,
            'control': self.current_control,
            'debug': self.debug_message,
            'robot': robot_status,
            'last_target_time': self.last_target_time,
            'last_control_time': self.last_control_time,
            'last_debug_time': self.last_debug_time
        }
        
    def reset(self):
        """重置控制器"""
        # 重置哨兵机器人的移动状态
        if self.sentry_robot:
            # 重置移动状态
            self.sentry_robot.has_target = False
            self.sentry_robot.is_moving = False
            # 重置目标点到当前位置
            self.sentry_robot.target_x = self.sentry_robot.x
            self.sentry_robot.target_y = self.sentry_robot.y
        
        # 重置其他状态
        self.current_target = None
        self.current_control = None
        self.debug_message = ""
        
        print("哨兵控制器已重置")
