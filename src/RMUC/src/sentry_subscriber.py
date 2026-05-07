#!/usr/bin/env python3
"""
哨兵决策系统消息显示器
显示决策系统发布的目标点和控制消息（现在通过ROS2订阅）
"""
import time
from datetime import datetime

class SentryStatusDisplay:
    def __init__(self):
        # 初始化状态
        self.current_target = {'x': 0.0, 'y': 0.0}
        self.current_control = {
            'gimbal_mode': 0,
            'spin_mode': 0,
            'posture': 3
        }
        
        # 名称映射
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
        
        # 状态历史
        self.status_history = []
        self.max_history = 10
        
        print("哨兵状态显示器已启动（ROS2订阅模式）")
        print("等待决策系统发布目标点和控制消息...")
        
    def update_from_controller(self, sentry_controller):
        """从控制器获取状态并显示"""
        try:
            status = sentry_controller.get_status()
            
            if status['has_target']:
                target_x, target_y = status['target']
                self.current_target = {'x': target_x, 'y': target_y}
                
            if status['control']:
                self.current_control = status['control']
                
            # 添加状态历史
            history_entry = {
                'time': datetime.now().strftime('%H:%M:%S'),
                'target': self.current_target.copy() if self.current_target else None,
                'control': self.current_control.copy() if self.current_control else None,
                'debug': status['debug'] if status['debug'] else ""
            }
            
            self.status_history.append(history_entry)
            if len(self.status_history) > self.max_history:
                self.status_history.pop(0)
                
        except Exception as e:
            print(f"更新状态显示失败: {e}")
    
    def get_current_status(self):
        """获取当前状态"""
        return {
            'target': self.current_target,
            'control': self.current_control,
            'history': self.status_history
        }
    
    def get_control_text(self):
        """获取控制消息的文本表示"""
        if not self.current_control:
            return "等待控制消息..."
        
        posture = self.current_control.get('posture', 3)
        posture_name = self.posture_names.get(posture, f"未知({posture})")
        
        gimbal = self.current_control.get('gimbal_mode', 3)
        gimbal_name = self.gimbal_names.get(gimbal, f"未知({gimbal})")
        
        spin = self.current_control.get('spin_mode', 0)
        spin_name = self.spin_names.get(spin, f"未知({spin})")
        
        return f"{posture_name} | 云台: {gimbal_name} | 陀螺: {spin_name}"
