from dataclasses import dataclass
from robots_msgs.msg import RobotStatus, GameStatus, EnemyPose
from .common import GameStage

class MessageGenerator:
    def generate_status(self, params: dict, game_stage: GameStage, remaining_time: float):
        """根据当前状态生成所有状态消息"""
        robot_msg = RobotStatus()
        game_msg = GameStatus()
        enemypose_msg = EnemyPose()

        enemypose_msg._enemy_x = params.get('enemy', {}).get('x', 0)
        enemypose_msg._enemy_y = params.get('enemy', {}).get('y', 0)
        enemypose_msg.enemy_num = params.get('enemy', {}).get('id', 0)

        if game_stage == GameStage.OUT:
            self._set_out_stage_status(robot_msg, game_msg, params)
        elif game_stage == GameStage.PREPARE:
            self._set_prepare_stage_status(robot_msg, game_msg, params, remaining_time)
        elif game_stage == GameStage.SELFCHECK:
            self._set_selfcheck_stage_status(robot_msg, game_msg, params, remaining_time)
        elif game_stage == GameStage.COUNTDOWN:
            self._set_countdown_stage_status(robot_msg, game_msg, params, remaining_time)
        elif game_stage == GameStage.RUNNING:
            self._set_running_stage_status(robot_msg, game_msg, params, remaining_time)
        elif game_stage == GameStage.ENDED:
            self._set_ended_stage_status(robot_msg, game_msg, params)
        else:
            raise ValueError(f"unknown game stage: {game_stage}")
        
        return robot_msg, game_msg, enemypose_msg

    def _set_out_stage_status(self, robot_msg: RobotStatus, game_msg: GameStatus, params: dict):
        robot_msg._robot_id = params.get('sentry', {}).get('id', 0)
        robot_msg._ttk_status = 1
        robot_msg._remaining_energy = 0x32
        robot_msg._projectile_allowance_17mm_remain = 1500
        robot_msg._projectile_allowance_17mm = params.get('sentry', {}).get('bullet_allowance', 0)
        robot_msg._fort_area = params.get('sentry', {}).get('fort_area', 0)
        robot_msg._supply_blood_area = params.get('sentry', {}).get('supply_blood_area', 0)
        
        game_msg._game_type = 1
        game_msg._game_progress = 0
        game_msg._stage_remain_time = 0

        game_msg._enemy_1_robot_hp = 250
        game_msg._enemy_2_robot_hp = 250
        game_msg._enemy_3_robot_hp = 150
        game_msg._enemy_4_robot_hp = 150
        game_msg._enemy_5_robot_hp = 150
        game_msg._enemy_7_robot_hp = 400
        game_msg._enemy_base_hp = 5000
        game_msg._enemy_outpost_hp = 1500

        game_msg._my_base_hp = 5000
        game_msg._my_outpost_hp = 1500
        game_msg._my_hp = 400

    def _set_prepare_stage_status(self, robot_msg: RobotStatus, game_msg: GameStatus, 
                                params: dict, remaining_time: float):
        robot_msg._robot_id = params.get('sentry', {}).get('id', 0)
        robot_msg._ttk_status = 1
        robot_msg._remaining_energy = 0x32
        robot_msg._projectile_allowance_17mm_remain = 1500
        robot_msg._projectile_allowance_17mm = params.get('sentry', {}).get('bullet_allowance', 0)
        robot_msg._fort_area = params.get('sentry', {}).get('fort_area', 0)
        robot_msg._supply_blood_area = params.get('sentry', {}).get('supply_blood_area', 0)

        game_msg._game_type = 1
        game_msg._game_progress = 1
        game_msg._stage_remain_time = remaining_time

        game_msg._enemy_1_robot_hp = 250
        game_msg._enemy_2_robot_hp = 250
        game_msg._enemy_3_robot_hp = 150
        game_msg._enemy_4_robot_hp = 150
        game_msg._enemy_5_robot_hp = 150
        game_msg._enemy_7_robot_hp = 400
        game_msg._enemy_base_hp = 5000
        game_msg._enemy_outpost_hp = 1500

        game_msg._my_base_hp = 5000
        game_msg._my_outpost_hp = 1500
        game_msg._my_hp = params.get('sentry', {}).get('sentry_hp', 0)

    def _set_selfcheck_stage_status(self, robot_msg: RobotStatus, game_msg: GameStatus,
                                    params: dict, remaining_time: float):
        robot_msg._robot_id = params.get('sentry', {}).get('id', 0)
        robot_msg._ttk_status = 1
        robot_msg._remaining_energy = 0x32
        robot_msg._projectile_allowance_17mm_remain = 1500
        robot_msg._projectile_allowance_17mm = params.get('sentry', {}).get('bullet_allowance', 0)
        robot_msg._fort_area = params.get('sentry', {}).get('fort_area', 0)
        robot_msg._supply_blood_area = params.get('sentry', {}).get('supply_blood_area', 0)

        game_msg._game_type = 1
        game_msg._game_progress = 2
        game_msg._stage_remain_time = remaining_time

        game_msg._enemy_1_robot_hp = 250
        game_msg._enemy_2_robot_hp = 250
        game_msg._enemy_3_robot_hp = 150
        game_msg._enemy_4_robot_hp = 150
        game_msg._enemy_5_robot_hp = 150
        game_msg._enemy_7_robot_hp = 400
        game_msg._enemy_base_hp = 5000
        game_msg._enemy_outpost_hp = 1500

        game_msg._my_base_hp = 5000
        game_msg._my_outpost_hp = 1500
        game_msg._my_hp = params.get('sentry', {}).get('sentry_hp', 0)

    def _set_countdown_stage_status(self, robot_msg: RobotStatus, game_msg: GameStatus,
                                    params: dict, remaining_time: float):
        robot_msg._robot_id = params.get('sentry', {}).get('id', 0)
        robot_msg._ttk_status = 1
        robot_msg._remaining_energy = 0x32
        robot_msg._projectile_allowance_17mm_remain = 1500
        robot_msg._projectile_allowance_17mm = params.get('sentry', {}).get('bullet_allowance', 0)
        robot_msg._fort_area = params.get('sentry', {}).get('fort_area', 0)
        robot_msg._supply_blood_area = params.get('sentry', {}).get('supply_blood_area', 0)

        game_msg._game_type = 1
        game_msg._game_progress = 3
        game_msg._stage_remain_time = remaining_time

        game_msg._enemy_1_robot_hp = 250
        game_msg._enemy_2_robot_hp = 250
        game_msg._enemy_3_robot_hp = 150
        game_msg._enemy_4_robot_hp = 150
        game_msg._enemy_5_robot_hp = 150
        game_msg._enemy_7_robot_hp = 400
        game_msg._enemy_base_hp = 5000
        game_msg._enemy_outpost_hp = 1500

        game_msg._my_base_hp = 5000
        game_msg._my_outpost_hp = 1500
        game_msg._my_hp = params.get('sentry', {}).get('sentry_hp', 0)

    def _set_running_stage_status(self, robot_msg: RobotStatus, game_msg: GameStatus,
                                params: dict, remaining_time: float):
        robot_msg._robot_id = params.get('sentry', {}).get('id', 0)
        robot_msg._ttk_status = 1

        # 设置remaining energy
        remain_energy = params.get('sentry', {}).get('remaining_energy', 0)
        if(remain_energy >= 23000 * 0.5):
            robot_msg._remaining_energy = 0x32
        elif(remain_energy >= 23000 * 0.3):
            robot_msg._remaining_energy = 0x1E
        elif(remain_energy >= 23000 * 0.15):
            robot_msg._remaining_energy = 0x1C
        elif(remain_energy >= 23000 * 0.05):
            robot_msg._remaining_energy = 0x18
        elif(remain_energy >= 23000 * 0.01):
            robot_msg._remaining_energy = 0x10
        else:
            robot_msg._remaining_energy = 0x00

        robot_msg._projectile_allowance_17mm_remain = 1500
        robot_msg._projectile_allowance_17mm = params.get('sentry', {}).get('bullet_allowance', 0)
        robot_msg._fort_area = params.get('sentry', {}).get('fort_area', 0)
        robot_msg._supply_blood_area = params.get('sentry', {}).get('supply_blood_area', 0)

        game_msg._game_type = 1
        game_msg._game_progress = 4
        game_msg._stage_remain_time = remaining_time

        game_msg._enemy_1_robot_hp = params.get('enemy', {}).get('hero_hp', 250)
        game_msg._enemy_2_robot_hp = params.get('enemy', {}).get('engineer_hp', 250)
        game_msg._enemy_3_robot_hp = params.get('enemy', {}).get('infantry3_hp', 150)
        game_msg._enemy_4_robot_hp = params.get('enemy', {}).get('infantry4_hp', 150)
        game_msg._enemy_5_robot_hp = 200
        game_msg._enemy_7_robot_hp = params.get('enemy', {}).get('sentry_hp', 400)

        game_msg._my_hp = params.get('sentry', {}).get('sentry_hp', 0)
        if(robot_msg._robot_id == 7): #为红方
            game_msg._enemy_base_hp = params.get('blue', {}).get('base_hp', 5000)
            game_msg._enemy_outpost_hp = params.get('blue', {}).get('outpost_hp', 1500)
            game_msg._my_base_hp = params.get('red', {}).get('base_hp', 5000)
            game_msg._my_outpost_hp = params.get('red', {}).get('outpost_hp', 1500)
        else: #为蓝方
            game_msg._enemy_base_hp = params.get('red', {}).get('base_hp', 5000)
            game_msg._enemy_outpost_hp = params.get('red', {}).get('outpost_hp', 1500)
            game_msg._my_base_hp = params.get('blue', {}).get('base_hp', 5000)
            game_msg._my_outpost_hp = params.get('blue', {}).get('outpost_hp', 1500)


    def _set_ended_stage_status(self, robot_msg: RobotStatus, game_msg: GameStatus,
                                params: dict):
        robot_msg._robot_id = params.get('sentry', {}).get('id', 0)
        robot_msg._ttk_status = 1
        robot_msg._remaining_energy = 0x32
        robot_msg._projectile_allowance_17mm_remain = 1500
        robot_msg._projectile_allowance_17mm = params.get('sentry', {}).get('bullet_allowance', 0)
        robot_msg._fort_area = params.get('sentry', {}).get('fort_area', 0)
        robot_msg._supply_blood_area = params.get('sentry', {}).get('supply_blood_area', 0)
                
        game_msg._game_type = 1
        game_msg._game_progress = 5
        game_msg._stage_remain_time = 0

        game_msg._enemy_1_robot_hp = 250
        game_msg._enemy_2_robot_hp = 250
        game_msg._enemy_3_robot_hp = 150
        game_msg._enemy_4_robot_hp = 150
        game_msg._enemy_5_robot_hp = 150
        game_msg._enemy_7_robot_hp = 400
        game_msg._enemy_base_hp = 0
        game_msg._enemy_outpost_hp = 0

        game_msg._my_base_hp = 0
        game_msg._my_outpost_hp = 0
        game_msg._my_hp = 400
        pass