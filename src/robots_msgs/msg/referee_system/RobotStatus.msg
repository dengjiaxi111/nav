# 机器人状态信息
uint8  robot_id # 本机器人 ID

float32 x     # 本机器人位置 x 坐标，单位：m
float32 y     # 本机器人位置 y 坐标，单位：m
float32 angle # 本机器人测速模块的朝向，单位：度。正北为 0 度

uint8  recovery_buff      # 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
uint8  defence_buff       # 机器人防御增益（百分比，值为 50 表示 50% 防御增益）
uint8  vulnerability_buff # 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
uint16 attack_buff        # 机器人攻击增益（百分比，值为 50 表示 50% 攻击增益）

uint8 remaining_energy # 机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例
uint16 projectile_allowance_17mm # 17mm 弹丸允许发弹量

uint8  fort_area          # 己方堡垒状态
uint8  supply_blood_area  # 己方补血点状态

uint16 local_exchange_projectile_allowance_17mm # 本地兑换 17mm 弹丸数量
uint8 remote_exchange_projectile_times          # 远程兑换弹丸次数
uint8 remote_exchange_health_times              # 远程兑换血量次数

uint8  can_free_rebirth    # 是否可以免费复活
uint8  can_instant_rebirth # 是否可以立即复活
uint16 rebirth_need_coin   # 复活所需金币数量

uint8  ttk_status                      # TTK 状态
uint16 projectile_allowance_17mm_remain # 17mm 弹丸剩余允许发弹量

float32  target_position_x # 云台手发的控制目标位置 x 坐标
float32  target_position_y # 云台手发的控制目标位置 y 坐标
