import pygame
from ament_index_python.packages import get_package_share_directory
import os
from enum import Enum

class GameStage(Enum):
    OUT = "赛外"
    PREPARE = "三分钟准备"
    SELFCHECK = "15s自检"
    COUNTDOWN = "5s倒计时"
    RUNNING = "比赛中"
    ENDED = "比赛结束"


class AudioManager:
    def __init__(self):
        package_share_dir = get_package_share_directory('my_referee')
        self.audio_dir = os.path.join(package_share_dir, 'audio')
        self._is_playing = False
        self._stop_loop = False
        
        # 初始化pygame混音器，设置多个通道
        pygame.mixer.init()
        # 设置多个通道用于同时播放
        pygame.mixer.set_num_channels(8)
        
        # 缓存Sound对象
        self._sounds = {}
        self._volume = 1.0

    def play(self, filename: str, loop: bool = False, as_bgm: bool = True):
        """
        播放音频文件
        Args:
            filename: 音频文件名
            loop: 是否循环播放
            as_bgm: 是否作为背景音乐播放(使用music通道)
        """
        audio_path = os.path.join(self.audio_dir, filename)
        if not os.path.exists(audio_path):
            print(f"Warning: 音频文件不存在: {audio_path}")
            return

        if as_bgm:
            # BGM使用music通道
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play(-1 if loop else 0)
            self._is_playing = True
        else:
            # 效果音使用Sound对象和普通通道
            if filename not in self._sounds:
                self._sounds[filename] = pygame.mixer.Sound(audio_path)
            self._sounds[filename].play()

    def stop(self):
        """停止所有音频播放"""
        self._stop_loop = True
        self._is_playing = False
        pygame.mixer.music.stop()
        pygame.mixer.stop()

    @property
    def is_playing(self):
        """获取当前是否正在播放音频"""
        return self._is_playing and pygame.mixer.music.get_busy()

    def set_volume(self, volume):
        self._volume = max(0.0, min(1.0, volume))
        if self.player and self.player.is_playing():
            self.player.volume = self._volume
            
    def get_volume(self):
        return self._volume