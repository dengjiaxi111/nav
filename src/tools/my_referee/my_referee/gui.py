import os
import sys

from PySide6.QtWidgets import (
    QWidget, 
    QPushButton, 
    QVBoxLayout, 
    QLabel, 
    QHBoxLayout, 
    QLineEdit, 
    QGroupBox, 
    QFormLayout, 
    QCheckBox,
    QApplication,
)
from PySide6.QtCore import Qt,QPropertyAnimation, QEasingCurve
from PySide6.QtGui import QColor, QPalette, QImage, QBrush, QPixmap
from ament_index_python.packages import get_package_share_directory

from .common import GameStage
from .common import AudioManager
from .splash_screen import SplashScreen
from .music_player import MusicPlayer
from .robot_status_display import RobotStatusDisplay
from robots_msgs.msg import ModeCmd


class MyRefereeWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.node.set_gui(self)
        self.submit_sound = AudioManager()

        # 首先设置窗口大小
        self.setFixedSize(1200, 675)
        self.setWindowOpacity(0.0) 

        # 创建并设置启动画面
        self.splash = SplashScreen()
        self.splash.start_fade_in.connect(self._start_fade_in)
        self.splash.animation_finished.connect(self._on_splash_finished)
        
        # 计算居中位置
        screen = QApplication.primaryScreen().availableGeometry()
        x = int((screen.width() - 1200) / 2)
        y = int((screen.height() - 675) / 2)
        
        # 设置两个窗口的位置
        self.move(x, y)
        self.splash.move(x, y)
    
        # 初始化UI（但此时还不显示）
        self._setup_ui()
        
        # 显示启动画面并开始动画
        self.splash.show()
        self.splash.start_animations()

    def _on_splash_finished(self):
        # 显示主窗口
        print("DEBUG: Splash screen closed")
        self.splash.close()

    def _start_fade_in(self):
        print("Starting fade-in animation...")
        # 如果透明度已经是目标值，则不重复动画
        if self.windowOpacity() >= 1.0:
            return

        self.show()
        self.raise_()

        self.fade_anim = QPropertyAnimation(self, b"windowOpacity")
        self.fade_anim.setDuration(500)
        self.fade_anim.setStartValue(0.0)
        self.fade_anim.setEndValue(1.0)
        self.fade_anim.setEasingCurve(QEasingCurve.InOutQuad) 
        self.fade_anim.start()
        

    def _setup_ui(self):

        # 获取包资源目录
        package_share_dir = get_package_share_directory('my_referee')
        bg_image_path = os.path.join(package_share_dir,'images', 'IMG2.jpg')

        # 设置窗口基本属性
        self.setWindowTitle("3SE SENTRY FAKE REFEREE")
        self.setFixedSize(1200, 675) 

        # 创建主容器
        main_container = QWidget()
        main_container.setFixedWidth(800)

        # 设置背景
        image = QImage(bg_image_path)
        palette = self.palette()
        if not image.isNull():
            scaled_image = image.scaled(
                self.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            palette.setBrush(QPalette.Window, QBrush(QPixmap.fromImage(scaled_image)))
            self.setPalette(palette)
            self.setAutoFillBackground(True)
        else:
            print(f"Warning: Could not load image from {bg_image_path}")
            # 设置默认背景颜色
            palette.setColor(QPalette.Window, QColor('#2b2b2b'))
            self.setPalette(palette)

        self.setStyleSheet(f"""
            MyRefereeWindow {{
                background-image: url("{bg_image_path}");
                background-position: center;
                background-repeat: no-repeat;
                background-size: contain;
            }}
        """)

        # 设置按钮和标签样式
        button_style = """
            QPushButton {
                background-color: rgba(61, 61, 61, 200);
                color: white;
                border: none;
                padding: 10px;
                min-width: 100px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: rgba(77, 77, 77, 200);
            }
            QPushButton:pressed {
                background-color: rgba(45, 45, 45, 180);
            }
        """
        
        label_style = """
            QLabel {
                color: white;
                font-family: "Microsoft YaHei", "Arial";  /* 设置字体 */
                background: transparent;  /* 确保标签背景透明 */
                font-size: 36px;
                background-color: rgba(61, 61, 61, 180);  /* 半透明深灰色背景 */
                margin: 10px;
                padding: 10px 20px;  /* 内边距 */
                border-radius: 10px;  /* 圆角 */
                qproperty-alignment: AlignCenter;  /* 文本居中对齐 */
            }
        """
        # 参数设置框样式
        param_group_style = """
            QGroupBox {
                color: black;
                font-size: 18px;
                border: 2px solid #555;
                border-radius: 6px;
                margin-top: 12px;
                background-color: rgba(61, 61, 61, 180);
                outline: 5px solid rgba(255, 255, 255, 1.0);  /* 添加外部描边 */
                outline-offset: 2px;  /* 描边与边框的间距 */
            }
            QGroupBox::title {
                color: white;
                subcontrol-origin: margin;
                left: 7px;
                padding: 0px 5px 0px 5px;
            }
            QLineEdit {
                background-color: rgba(255, 255, 255, 220);
                border: 1px solid #555;
                border-radius: 4px;
                padding: 4px;
                color: black;
                font-size: 14px;
            }    
            QLabel {  /* 添加这部分来控制标签颜色 */
                color: white;  /* 或其他你想要的颜色 */
                font-size: 14px;
            }   
        """

        self.status_label = QLabel("当前状态：赛外")
        self.status_label.setStyleSheet(label_style)
        self.stage_remain_time_label = QLabel("剩余时间：0")
        self.stage_remain_time_label.setStyleSheet(label_style)

        # 比赛状态按钮
        self.btn_out   = QPushButton("赛外")
        self.btn_prepare = QPushButton("三分钟准备")
        self.btn_selfcheck = QPushButton("自检")
        self.btn_countdown = QPushButton("5s倒计时")
        self.btn_start = QPushButton("开始比赛")
        self.btn_end = QPushButton("终止比赛")

        self.btn_stop_bgm = QPushButton("关闭背景音乐")
        self.submit_button = QPushButton("应用参数")

        # 应用按钮样式
        for btn in [self.btn_out, self.btn_prepare, self.btn_selfcheck, 
                   self.btn_countdown, self.btn_start, self.btn_end, self.btn_stop_bgm, self.submit_button]:
            btn.setStyleSheet(button_style)

        self.btn_out.clicked.connect(lambda: self.change_status("赛外"))
        self.btn_prepare.clicked.connect(lambda: self.change_status("三分钟准备"))
        self.btn_selfcheck.clicked.connect(lambda: self.change_status("自检"))
        self.btn_countdown.clicked.connect(lambda: self.change_status("5s倒计时"))
        self.btn_start.clicked.connect(lambda: self.change_status("开始比赛"))
        self.btn_end.clicked.connect(lambda: self.change_status("终止比赛"))
        self.btn_stop_bgm.clicked.connect(self.node.state_machine.audio_manager.stop)
        self.submit_button.clicked.connect(self._submit_params)

        # 比赛参数输入
        def create_param_group(title, fields):
            group = QGroupBox(title)
            group.setStyleSheet(param_group_style)
            layout = QFormLayout()
            layout.setSpacing(10)
            inputs = {}
            for field, default in fields:
                if field == "当前方":
                    switch = ColorSwitch()
                    layout.addRow(field, switch)
                    inputs[field] = switch
                elif isinstance(default, bool):  # 处理复选框
                    checkbox = QCheckBox()
                    checkbox.setChecked(default)
                    layout.addRow(field, checkbox)
                    inputs[field] = checkbox
                else:
                    line_edit = QLineEdit(str(default))
                    line_edit.setFixedWidth(60)
                    layout.addRow(field, line_edit)
                    inputs[field] = line_edit
            group.setLayout(layout)
            return group, inputs
        # 创建红方参数组
        red_fields = [("基地血量", "5000"), ("前哨站血量", "1500")]
        self.red_group, self.red_inputs = create_param_group("红方", red_fields)

        # 创建中间参数组
        middle_fields = [
            ("当前方", None),  
            ("哨兵血量", "400"),
            ("允许发弹量", "300"),
            ("剩余能量", "23000"),
            ("位于补给区", False),  
            ("位于堡垒", False),  
            ("x: ", "0.0"),
            ("y: ", "0.0"),
        ]
        self.middle_group, self.middle_inputs = create_param_group("哨兵参数", middle_fields)

        # 创建敌方参数组
        enemy_fields = [
            ("英雄血量", "250"),
            ("工程血量", "250"),
            ("步兵3血量", "200"),
            ("步兵4血量", "150"),
            ("哨兵血量", "400"),
            ("敌方ID", "3"),
            ("敌方x: ", "1.5"),
            ("敌方y: ", "0.5")
        ]
        
        self.enemy_group, self.enemy_inputs = create_param_group("敌方机器人", enemy_fields)

        # 创建蓝方参数组
        blue_fields = [("基地血量", "5000"), ("前哨站血量", "1500")]
        self.blue_group, self.blue_inputs = create_param_group("蓝方", blue_fields)

        # 添加参数设置行
        param_layout = QHBoxLayout()
        param_layout.addWidget(self.red_group)
        param_layout.addStretch(1)
        param_layout.addWidget(self.enemy_group)
        param_layout.addStretch(1)
        param_layout.addWidget(self.middle_group)
        param_layout.addStretch(1)
        param_layout.addWidget(self.blue_group)
        # 布局
        main_layout = QVBoxLayout(main_container)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # 添加顶部弹性空间
        main_layout.addStretch(1)

        # 第一行：状态标签和停止音乐按钮
        row_1 = QHBoxLayout()
        row_1.addStretch(1)  # 左侧弹性空间
        row_1.addWidget(self.status_label, 0, Qt.AlignCenter)
        row_1.addStretch(1)  # 中间弹性空间
        row_1.addWidget(self.btn_stop_bgm, 0, Qt.AlignRight)
        main_layout.addLayout(row_1)

        # 中间弹性空间
        main_layout.addStretch(1)

        # 第二行：功能按钮
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.btn_out)
        btn_layout.addWidget(self.btn_prepare)
        btn_layout.addWidget(self.btn_selfcheck)
        btn_layout.addWidget(self.btn_countdown)
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_end)
        btn_layout.setSpacing(10)
        main_layout.addLayout(btn_layout)

        # 底部弹性空间
        main_layout.addStretch(1)

        # 第三行：时间标签
        temp_layout = QHBoxLayout()
        temp_layout.addWidget(self.stage_remain_time_label, 0, Qt.AlignCenter)
        self.submit_button.setFixedWidth(60)

        temp_layout.addWidget(self.submit_button)
        main_layout.addLayout(temp_layout)
        # 弹性空间
        main_layout.addStretch(1)

        main_layout.addLayout(param_layout)

        # 最外层布局
        outer_layout = QHBoxLayout()
        outer_layout.addWidget(main_container)  # 添加主容器
        outer_layout.addStretch(1) 

        # 添加音乐播放器和状态显示
        temp_column = QVBoxLayout()
        self.music_player = MusicPlayer()
        self.music_player.setFixedWidth(300)  # 设置音乐播放器宽度
        temp_column.addWidget(self.music_player)
        outer_layout.addStretch(1)
        self.robot_display = RobotStatusDisplay()
        temp_column.addWidget(self.robot_display)

        outer_layout.addLayout(temp_column)
        outer_layout.addStretch(1)

        self.setLayout(outer_layout)
    
        # 初始化缓存参数字典
        self._cached_params = self._read_current_params()  # 设置初始缓存
    

    def closeEvent(self, event):
        """重写closeEvent以在窗口关闭时结束程序"""
        # 停止ROS节点
        self.node.destroy_node()
        # 关闭窗口
        event.accept()
        # 退出程序
        sys.exit(0)
        
    def change_status(self, status):
        stage_remain_time = 0
        game_status = GameStage.OUT
        if status == "赛外":
            stage_remain_time = 0
            game_status = GameStage.OUT
        elif status == "三分钟准备":
            stage_remain_time = 180
            game_status = GameStage.PREPARE
        elif status == "自检":
            stage_remain_time = 15
            game_status = GameStage.SELFCHECK
        elif status == "5s倒计时":
            stage_remain_time = 5
            game_status = GameStage.COUNTDOWN
        elif status == "开始比赛":
            stage_remain_time = 420
            game_status = GameStage.RUNNING
        elif status == "终止比赛":
            stage_remain_time = 0
            game_status = GameStage.ENDED

        self.node.set_stage(game_status, stage_remain_time)

    def update_ui(self, time_remain, modecmd: ModeCmd, status=None):
        self.stage_remain_time_label.setText(f"剩余时间：{time_remain}")
        if status is not None:
            self.status_label.setText(f"当前状态：{status}")

        robot_id = 107 if self.middle_inputs["当前方"].is_blue else 7
        self.robot_display.set_status(modecmd,robot_id)
        

    def _read_current_params(self):
        """读取当前所有输入框的值"""
        return {
            'red': {
                'base_hp': int(self.red_inputs["基地血量"].text()),
                'outpost_hp': int(self.red_inputs["前哨站血量"].text())
            },
            'blue': {
                'base_hp': int(self.blue_inputs["基地血量"].text()),
                'outpost_hp': int(self.blue_inputs["前哨站血量"].text())
            },
            'enemy': {
                'hero_hp': int(self.enemy_inputs["英雄血量"].text()),
                'engineer_hp': int(self.enemy_inputs["工程血量"].text()),
                'infantry3_hp': int(self.enemy_inputs["步兵3血量"].text()),
                'infantry4_hp': int(self.enemy_inputs["步兵4血量"].text()),
                'sentry_hp': int(self.enemy_inputs["哨兵血量"].text()),
                'id':  int(self.enemy_inputs["敌方ID"].text()),
                'x': float(self.enemy_inputs["敌方x: "].text()),
                'y': float(self.enemy_inputs["敌方y: "].text()),
                
            },
            'sentry': {
                'id': 107 if self.middle_inputs["当前方"].is_blue else 7,
                'sentry_hp': int(self.middle_inputs["哨兵血量"].text()),
                'bullet_allowance': int(self.middle_inputs["允许发弹量"].text()),
                'remaining_energy': int(self.middle_inputs["剩余能量"].text()),
                'supply_blood_area': int(self.middle_inputs["位于补给区"].isChecked()),
                'fort_area': int(self.middle_inputs["位于堡垒"].isChecked()),
                'x': float(self.middle_inputs["x: "].text()),
                'y': float(self.middle_inputs["y: "].text()),
            }
        }

    def _submit_params(self):
        """提交新的参数并更新缓存"""
        self._cached_params = self._read_current_params()
        self.submit_sound.play('buff.mp3',False,False)

    def get_params(self):
        """获取参数值(使用缓存)"""
        return self._cached_params
    
class ColorSwitch(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(60, 30)
        self._is_checked = False
        self._bg_color = "#ff5e00"  # 默认红色

    def mousePressEvent(self, event):
        self._is_checked = not self._is_checked
        self._bg_color = "#08addf" if self._is_checked else "#ff5e00"  # 蓝色或红色
        self.update()

    def paintEvent(self, event):
        from PySide6.QtGui import QPainter, QColor, QPen
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 绘制背景
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(self._bg_color))
        painter.drawRoundedRect(0, 0, self.width(), self.height(), 15, 15)
        
        # 绘制滑块
        painter.setBrush(QColor("#ffffff"))
        if not self._is_checked:
            painter.drawEllipse(5, 5, 20, 20)
        else:
            painter.drawEllipse(self.width() - 25, 5, 20, 20)

        # 绘制文字
        painter.setPen(QPen(QColor("#ffffff")))
        if not self._is_checked:
            painter.drawText(28, 20, "红")
        else:
            painter.drawText(10, 20, "蓝")

    @property
    def is_blue(self):
        return self._is_checked