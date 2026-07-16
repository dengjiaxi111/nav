from time import sleep
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QApplication, QGraphicsOpacityEffect
from PySide6.QtCore import Qt, QPropertyAnimation, QSequentialAnimationGroup, Signal, QEasingCurve, QTimer
from PySide6.QtGui import QMovie, QPainter, QColor
from ament_index_python.packages import get_package_share_directory
import os

from .common import AudioManager

class SplashScreen(QWidget):
    animation_finished = Signal()
    start_fade_in = Signal()

    def __init__(self):
        super().__init__()

        self.sound_player = AudioManager()
        self.sound_player.play('init.mp3', False, False)
        
        sleep(2.9)
        # 设置窗口属性
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Tool)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setFixedSize(1500, 675)

        self.opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self.opacity_effect)
        self.opacity_effect.setOpacity(1.0)

        # 初始化布局
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)

        # 创建GIF标签
        self.gif_label = QLabel()
        self.gif_label.setFixedSize(600, 337)
        self.gif_label.setAlignment(Qt.AlignCenter)
        self.gif_label.setStyleSheet("""
            QLabel {
                background: transparent;
            }
        """)

        # 加载GIF动画
        package_share_dir = get_package_share_directory('my_referee')
        gif_path = os.path.join(package_share_dir, 'images', 'mygo!!!!!.gif')
        self.movie = QMovie(gif_path)
        self.movie.setScaledSize(self.gif_label.size())
        self.gif_label.setMovie(self.movie)
        self.movie.setSpeed(0)  # 初始速度设为0

        # 创建文本标签
        self.text_label = QLabel(" ")
        self.text_label.setAlignment(Qt.AlignCenter)
        self.text_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 36px;
                font-weight: bold;
                font-family: 'Microsoft YaHei';
                background: transparent;
            }
        """)

        # 添加到布局
        self.layout.addStretch(1)
        self.layout.addWidget(self.gif_label, 0, Qt.AlignCenter)
        self.layout.addWidget(self.text_label, 0, Qt.AlignCenter)
        self.layout.addStretch(1)

        # 初始化动画属性
        self._opacity = 1.0
        self.setup_animations()
        self.center_on_screen()

    def setup_animations(self):
        """设置动画"""
        # 只保留淡出动画
        self.fade_animation = QPropertyAnimation(self.opacity_effect, b'opacity')
        self.fade_animation.setDuration(1000)
        self.fade_animation.setStartValue(1.0)
        self.fade_animation.setEndValue(0.0)
        self.fade_animation.setEasingCurve(QEasingCurve.InOutQuad)
        self.fade_animation.finished.connect(self._on_animation_finished)

        # 简化序列动画组
        self.sequence_group = QSequentialAnimationGroup()
        self.sequence_group.addPause(1500)  # 播放时间
        self.sequence_group.addAnimation(self.fade_animation)
        self.sequence_group.currentAnimationChanged.connect(self._on_animation_change)


    def start_animations(self):
        """启动动画"""
        self.movie.start()
        self.movie.setSpeed(120)  # 直接设置正常速度
        self.setWindowOpacity(1.0)
        self.sequence_group.start()

    def _on_animation_change(self, animation):
        """动画切换时触发"""
        if animation == self.fade_animation:
            self.start_fade_in.emit()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setOpacity(self._opacity)
        painter.fillRect(self.rect(), QColor(255, 255, 255, 0))  # RGBA值,最后的100控制透明度(0-255)


    def _on_animation_finished(self):
        """动画完成后的处理"""
        self.movie.stop()
        self.animation_finished.emit()

    def center_on_screen(self):
        """将窗口居中显示"""
        screen = QApplication.primaryScreen().availableGeometry()
        size = self.geometry()
        x = (screen.width() - size.width()) // 2
        y = (screen.height() - size.height()) // 2
        self.setGeometry(x, y, size.width(), size.height())

    # 添加 gifSpeed 属性
    @property
    def gifSpeed(self):
        return self.movie.speed()

    @gifSpeed.setter
    def gifSpeed(self, value):
        self.movie.setSpeed(int(value))