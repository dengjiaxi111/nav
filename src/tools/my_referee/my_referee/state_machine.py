from enum import Enum, auto
from PySide6.QtCore import QTimer
from .common import GameStage
from .common import AudioManager

class RefereeStateMachine:
    def __init__(self, node):
        self.node = node
        self.current_stage = GameStage.OUT
        self.last_stage = GameStage.OUT
        self.stage_remain_time = 0

        self.state_timer = QTimer()  # 状态更新定时器
        self.countdown_timer = QTimer()  # 倒计时定时器
        # 设置定时器回调
        self.state_timer.timeout.connect(self._state_tick)
        self.countdown_timer.timeout.connect(self._countdown_tick)

        self.audio_manager = AudioManager()
        self._init_state_handlers()

    def _init_state_handlers(self):
        """初始化状态机"""
        self.state_handlers = {
            GameStage.OUT: self._handle_out_stage,
            GameStage.PREPARE: self._handle_prepare_stage,
            GameStage.SELFCHECK: self._handle_selfcheck_stage,
            GameStage.COUNTDOWN: self._handle_countdown_stage,
            GameStage.RUNNING: self._handle_running_stage,
            GameStage.ENDED: self._handle_ended_stage,
        }

    def change_stage(self, new_stage: GameStage, duration: int = 0):
        if new_stage == self.current_stage:
            return
        
        # 退出当前状态
        self._exit_current_stage()
        
        # 切换到新状态
        self.current_stage = new_stage
        self.stage_remain_time = duration
        self.node.get_logger().info(f"→ 切换到 [{new_stage.name}] 状态")
        self._enter_new_stage()
        
        # 分别启动两个定时器
        self.state_timer.start(100)  # 10Hz的状态更新
        if duration > 0:
            self.countdown_timer.start(1000)  # 1Hz的倒计时

    def _enter_new_stage(self):
        """进入新状态时的处理"""
        pass

    def _exit_current_stage(self):
        """退出当前状态时的清理工作"""
        self.audio_manager.stop()
        pass

    # 各状态的处理方法
    def _handle_out_stage(self):
        """赛外状态处理"""
        if(self.last_stage != self.current_stage):
            # pip install playsound
            self.audio_manager.play('out.mp3',True)
        pass

    def _handle_prepare_stage(self):
        """准备阶段处理"""
        if(self.last_stage != self.current_stage):
            # pip install playsound
            self.audio_manager.play('prepare.mp3',True)
        pass

    def _handle_selfcheck_stage(self):
        """自检阶段处理"""
        if(self.last_stage != self.current_stage):
            # pip install playsound
            self.audio_manager.play('prepare.mp3',True)
        pass

    def _handle_countdown_stage(self):
        """倒计时阶段处理"""
        if(self.last_stage != self.current_stage):
            self.audio_manager.play('countdown.mp3',False)
        pass

    def _handle_running_stage(self):
        """比赛运行阶段处理"""
        if(self.last_stage != self.current_stage):
            self.audio_manager.play('ingame.mp3',True)
        pass

    def _handle_ended_stage(self):
        """比赛结束阶段处理"""
        if(self.last_stage != self.current_stage):
            self.audio_manager.play('finish.mp3',False)
        pass
    

    def _state_tick(self):
        """状态更新定时器回调"""
        if self.current_stage in self.state_handlers:
            self.state_handlers[self.current_stage]()
        self.last_stage = self.current_stage

    def _countdown_tick(self):
        """倒计时定时器回调"""
        if self.stage_remain_time > 0:
            self.stage_remain_time -= 1
            self.node.update_remaining_time(self.stage_remain_time)
        else:
            self.countdown_timer.stop()
            self.state_timer.stop()
            self.last_stage = self.current_stage
            self.auto_transition()

    def auto_transition(self):
        """状态超时后自动切换逻辑"""
        if self.current_stage == GameStage.PREPARE:
            self.change_stage(GameStage.SELFCHECK, 15)
        elif self.current_stage == GameStage.SELFCHECK:
            self.change_stage(GameStage.COUNTDOWN, 5)
        elif self.current_stage == GameStage.COUNTDOWN:
            self.change_stage(GameStage.RUNNING, 420)
        elif self.current_stage == GameStage.RUNNING:
            self.change_stage(GameStage.ENDED, 0)
    def get_remaining_time(self):
        """获取当前阶段剩余时间"""
        return self.stage_remain_time