uint8 gimbal_mode      # 0:不动, 1:打人, 2:打前哨站
uint8 spin_mode        # 0:不转, 1:高速转, 2:低速转
uint8 posture          # 1:进攻姿态, 2:防御姿态, 3:移动姿态
float64 target_yaw_deg # 保留字段，当前决策不使用
bool target_yaw_valid  # 当前决策始终置 false
uint8 avoidengineer_flag    # 0:正常，不用管，1：代表敌方工程在取矿，不可以打
