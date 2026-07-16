#!/usr/bin/env python3
"""
哨兵移动控制器 - 国赛版（增加TF广播），支持自定义节点名
订阅决策系统的目标点，控制7号机器人移动，并发布TF变换
"""
import json
import os
import time
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import String
from sentry_decision.msg import SentryControl
from config import is_blue_robot_id
import tf2_ros
from datetime import datetime

class SentryController(Node):
    def __init__(self, game_state, node_name='sentry_controller'):
        # 初始化ROS2节点，允许自定义名称
        super().__init__(node_name)
        
        self.game_state = game_state
        self.sentry_robot = None
        self.refresh_sentry_robot()
        
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
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 当前状态
        self.current_target = None
        self.current_control = None
        self.target_timestamp = 0
        self.debug_message = ""
        
        # 移动速度 (cm/s)
        self.move_speed = 100.0
        
        # 姿态名称映射
        self.posture_names = {
            1: "进攻姿态",
            2: "防御姿态", 
            3: "移动姿态"
        }
        
        self.gimbal_names = {
            0: "不动",
            1: "打人模式",
            2: "打前哨站模式",
        }
        
        self.spin_names = {
            0: "不转",
            1: "转动",
        }
        
        # 最后接收时间
        self.last_target_time = 0
        self.last_control_time = 0
        self.last_debug_time = 0
        
        print(f"✓ 哨兵移动控制器已启动（含TF广播），节点名: {node_name}")
        print("✓ 订阅话题: /sentry/target_position, /sentry/control, /sentry/debug")
        print("✓ 发布TF变换: map -> base_link")
        
    def target_callback(self, msg):
        """目标点回调函数"""
        try:
            self.refresh_sentry_robot()
            # 决策系统发布的是米，转换为厘米
            x = msg.point.x * 100.0
            y = msg.point.y * 100.0
            
            self.current_target = (x, y)
            self.target_timestamp = time.time()
            self.last_target_time = time.time()
            
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 收到目标点: ({x:.1f}, {y:.1f}) cm")
            print(f"    原始ROS消息: ({msg.point.x:.2f}, {msg.point.y:.2f}) m")
            
            if self.sentry_robot:
                self.sentry_robot.target_x = x
                self.sentry_robot.target_y = y
                self.sentry_robot.has_target = True
                self.sentry_robot.is_moving = True
                print(f"开始移动到目标点: ({x:.1f}, {y:.1f}) cm")
                
        except Exception as e:
            print(f"目标点回调错误: {e}")
            import traceback
            traceback.print_exc()

    def refresh_sentry_robot(self):
        """根据当前机器人ID刷新被控制的哨兵对象。107=蓝方，其余默认红方。"""
        team_robots = self.game_state.blue_robots if is_blue_robot_id(self.game_state.our_robot_id) else self.game_state.red_robots
        self.sentry_robot = team_robots.get(7)
        return self.sentry_robot
            
    def control_callback(self, msg):
        """控制消息回调函数"""
        try:
            self.current_control = {
                'gimbal_mode': msg.gimbal_mode,
                'spin_mode': msg.spin_mode,
                'posture': msg.posture
            }
            self.last_control_time = time.time()
            
            posture = msg.posture
            posture_name = self.posture_names.get(posture, f"未知({posture})")
            gimbal = msg.gimbal_mode
            gimbal_name = self.gimbal_names.get(gimbal, f"未知({gimbal})")
            spin = msg.spin_mode
            spin_name = self.spin_names.get(spin, f"未知({spin})")
            
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 控制消息: {posture_name}, 云台:{gimbal_name}, 陀螺:{spin_name}")
            
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
        """更新哨兵机器人位置，并发布TF变换"""
        self.refresh_sentry_robot()
        if not self.sentry_robot:
            return
        
        if not self.sentry_robot.has_target or not self.sentry_robot.is_moving:
            self.publish_tf_transform()
            return
        
        old_x, old_y = self.sentry_robot.x, self.sentry_robot.y
        dx = self.sentry_robot.target_x - old_x
        dy = self.sentry_robot.target_y - old_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 5.0:
            self.sentry_robot.x = self.sentry_robot.target_x
            self.sentry_robot.y = self.sentry_robot.target_y
            self.sentry_robot.has_target = False
            self.sentry_robot.is_moving = False
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 已到达目标点: ({self.sentry_robot.x:.1f}, {self.sentry_robot.y:.1f}) cm")
            self.publish_tf_transform()
            return
        
        if distance > 0:
            vx = dx / distance
            vy = dy / distance
            move_distance = min(self.move_speed * dt, distance)
            self.sentry_robot.x += vx * move_distance
            self.sentry_robot.y += vy * move_distance
            
            new_distance = math.sqrt((self.sentry_robot.target_x - self.sentry_robot.x)**2 + 
                                    (self.sentry_robot.target_y - self.sentry_robot.y)**2)
            if new_distance < 5.0:
                self.sentry_robot.x = self.sentry_robot.target_x
                self.sentry_robot.y = self.sentry_robot.target_y
                self.sentry_robot.has_target = False
                self.sentry_robot.is_moving = False
                print(f"[{datetime.now().strftime('%H:%M:%S')}] 已到达目标点: ({self.sentry_robot.x:.1f}, {self.sentry_robot.y:.1f}) cm")
        
        self.publish_tf_transform()
        
        current_time = time.time()
        if current_time - self.last_target_time > 2.0 and self.sentry_robot.has_target:
            print(f"警告: 超过2秒未收到新的目标点，当前位置: ({self.sentry_robot.x:.1f}, {self.sentry_robot.y:.1f}) cm")
            
    def publish_tf_transform(self):
        """发布从map到base_link的变换"""
        if not self.sentry_robot:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        # 位置：厘米 -> 米
        t.transform.translation.x = self.sentry_robot.x / 100.0
        t.transform.translation.y = self.sentry_robot.y / 100.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        
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
        if self.sentry_robot:
            self.sentry_robot.has_target = False
            self.sentry_robot.is_moving = False
            self.sentry_robot.target_x = self.sentry_robot.x
            self.sentry_robot.target_y = self.sentry_robot.y
        
        self.current_target = None
        self.current_control = None
        self.debug_message = ""
        self.publish_tf_transform()
        print("哨兵控制器已重置")
