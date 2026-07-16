from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, 
                              QListWidget, QLabel, QHBoxLayout)
from PySide6.QtCore import Qt
import os
from ament_index_python.packages import get_package_share_directory
from .common import AudioManager

class MusicPlayer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.audio_manager = AudioManager()
        self.current_song = None
        self.is_playing = False
        
        # Get audio files
        package_share_dir = get_package_share_directory('my_referee')
        self.audio_dir = os.path.join(package_share_dir, 'audio')
        self.songs = [f for f in os.listdir(self.audio_dir) 
                     if f.endswith(('.mp3', '.wav'))]
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Style
        self.setStyleSheet("""
            QWidget {
                background-color: rgba(45, 45, 45, 180);
                color: white;
            }
            QPushButton {
                background-color: #3d3d3d;
                border: none;
                padding: 8px;
                border-radius: 4px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #4d4d4d;
            }
            QListWidget {
                background-color: rgba(61, 61, 61, 180);
                border: 1px solid #555;
                border-radius: 4px;
            }
            QLabel {
                color: white;
                font-size: 14px;
            }
        """)

        # Title
        title = QLabel("音乐播放器")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Song list
        self.song_list = QListWidget()
        self.song_list.addItems(self.songs)
        self.song_list.itemClicked.connect(self.song_selected)
        layout.addWidget(self.song_list)

        # Current playing label
        self.current_label = QLabel("当前播放: 无")
        layout.addWidget(self.current_label)

        # Control buttons
        btn_layout = QHBoxLayout()
        
        self.play_btn = QPushButton("播放")
        self.play_btn.clicked.connect(self.toggle_play)
        
        self.stop_btn = QPushButton("停止")
        self.stop_btn.clicked.connect(self.stop_music)

        btn_layout.addWidget(self.play_btn)
        btn_layout.addWidget(self.stop_btn)
        
        layout.addLayout(btn_layout)

    def song_selected(self, item):
        self.current_song = item.text()
        self.play_music()

    def play_music(self):
        if self.current_song:
            self.audio_manager.play(self.current_song)
            self.current_label.setText(f"当前播放: {self.current_song}")
            self.is_playing = True
            self.play_btn.setText("暂停")

    def toggle_play(self):
        if not self.current_song:
            return
            
        if self.is_playing:
            self.audio_manager.stop()
            self.is_playing = False
            self.play_btn.setText("播放")
        else:
            self.play_music()

    def stop_music(self):
        self.audio_manager.stop()
        self.is_playing = False
        self.play_btn.setText("播放")
        self.current_label.setText("当前播放: 无")