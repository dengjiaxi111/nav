from PySide6.QtWidgets import QWidget, QLabel
from PySide6.QtGui import QPixmap, QTransform, QPainter, QColor
from PySide6.QtCore import Qt, QTimer
import os
from ament_index_python.packages import get_package_share_directory
from robots_msgs.msg import ModeCmd

class RobotStatusDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(200, 200)  # 设置合适的大小
        self.hi_speed = False
        self.current_background_color = QColor(255, 255, 255, 180)  # 默认白色半透明
        
        # 加载图片
        package_share_dir = get_package_share_directory('my_referee')
        chassis_path = os.path.join(package_share_dir, 'images', 'chassis.png')
        gimbal_path = os.path.join(package_share_dir, 'images', 'gimbal.png')
        
        # 创建标签
        self.chassis_label = QLabel(self)
        self.gimbal_label = QLabel(self)
        self.background_label = QLabel(self)
        self.background_label.setFixedSize(200, 200)

        # 创建半透明灰色背景
        self._update_background()
        
        # 加载并缩放图片
        self.chassis_pixmap = QPixmap(chassis_path).scaled(
            200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.gimbal_pixmap = QPixmap(gimbal_path).scaled(
            150, 150, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # 设置标签位置
        self.chassis_label.setFixedSize(200, 200)
        self.gimbal_label.setFixedSize(200, 200)

        # 设置标签对齐方式
        self.chassis_label.setAlignment(Qt.AlignCenter)
        self.gimbal_label.setAlignment(Qt.AlignCenter)
        
        # 初始化旋转角度
        self.chassis_angle = 0
        self.gimbal_angle = 0
        
        # 设置定时器用于动画
        self.chassis_timer = QTimer(self)
        self.gimbal_timer = QTimer(self)
        self.chassis_timer.timeout.connect(self._rotate_chassis)
        self.gimbal_timer.timeout.connect(self._rotate_gimbal)
        
        # 调整层级关系
        self.background_label.lower()  # 背景在最底层
        self.chassis_label.raise_()    # 底盘在背景之上
        self.gimbal_label.raise_()     # 云台在最上层

        # 初始显示
        self.update_display()
        
    def _rotate_chassis(self):
        if(self.hi_speed):
            self.chassis_angle = (self.chassis_angle + 24) % 360
        else:
            self.chassis_angle = (self.chassis_angle + 4) % 360
        self.update_display()
        
    def _rotate_gimbal(self):
        self.gimbal_angle = (self.gimbal_angle - 5) % 360
        self.update_display()
        
    def update_display(self):
        # 旋转底盘图片
        transform = QTransform().rotate(self.chassis_angle)
        rotated_chassis = self.chassis_pixmap.transformed(transform, Qt.SmoothTransformation)
        self.chassis_label.setPixmap(rotated_chassis)
        
        # 旋转云台图片
        transform = QTransform().rotate(self.gimbal_angle)
        rotated_gimbal = self.gimbal_pixmap.transformed(transform, Qt.SmoothTransformation)
        self.gimbal_label.setPixmap(rotated_gimbal)
        
    def set_status(self, mode_cmd: ModeCmd, robot_id):
        # 设置颜色
        if(robot_id == 107):
            self.current_background_color = QColor(8, 173, 239, 180) 
        else:
            self.current_background_color = QColor(255, 94, 0, 180)  
        self._update_background()
        # 控制底盘旋转
        if mode_cmd.should_chassis_spin >= 1:
            if( mode_cmd.should_chassis_spin == 2):
                self.hi_speed = False
            else:
                self.hi_speed = True
            if not self.chassis_timer.isActive():
                self.chassis_timer.start(50)  # 20fps
        else:
            self.chassis_timer.stop()
            
        # 控制云台旋转
        if mode_cmd.should_gimbal_patrol:
            if not self.gimbal_timer.isActive():
                self.gimbal_timer.start(50)
        else:
            self.gimbal_timer.stop()

    def _update_background(self):
        """更新背景颜色"""
        background_pixmap = QPixmap(200, 200)
        background_pixmap.fill(Qt.transparent)
        painter = QPainter(background_pixmap)
        painter.setBrush(self.current_background_color)
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, 200, 200, 10, 10)
        painter.end()
        self.background_label.setPixmap(background_pixmap)