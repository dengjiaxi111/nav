uint8 gimbal_mode      # 0:不动, 1:打人, 2:打前哨站
uint8 spin_mode        # 0:不转, 1:转动
uint8 posture          # 1:进攻姿态, 2:防御姿态, 3:移动姿态
float64 target_yaw_deg # gimbal_mode=2 且 spin_mode=0 时有效，单位 deg，相对 base_link
bool target_yaw_valid  # true: 电控使用 target_yaw_deg；false: 电控忽略 target_yaw_deg
