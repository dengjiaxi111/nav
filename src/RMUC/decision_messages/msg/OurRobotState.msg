# 己方前哨站血量
uint16 outpost_hp 
# 己方基地血量
uint16 base_hp 

# 机器人性能体系数据：
# 本机器人 ID
uint8 robot_id 
# 机器人当前血量
uint16 current_hp 
# 机器人血量上限
uint16 max_hp 

# 机器人位置数据：
# 本机器人位置 x 坐标，单位：m
float32 x 
# 本机器人位置 y 坐标，单位：m
float32 y 
# 本机器人测速模块的朝向，单位：度。正北为 0 度
float32 yaw 

# 机器人增益和底盘能量数据：
# 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
float32 hp_recovery_buff 
# 机器人防御增益（百分比，值为 50 表示 50%防御增益）
float32 defense_buff 
# 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
float32 negative_defense_buff 
# 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
float32 attack_buff 

# 己方英雄机器人位置 x 轴坐标，单位：m
float32 hero_x 
# 己方英雄机器人位置 y 轴坐标，单位：m
float32 hero_y 
# 己方工程机器人位置 x 轴坐标，单位：m
float32 engineer_x 
# 己方工程机器人位置 y 轴坐标，单位：m
float32 engineer_y 
# 己方 3 号步兵机器人位置 x 轴坐标，单位：m
float32 infantry3_x 
# 己方 3 号步兵机器人位置 y 轴坐标，单位：m
float32 infantry3_y 
# 己方 4 号步兵机器人位置 x 轴坐标，单位：m
float32 infantry4_x 
# 己方 4 号步兵机器人位置 y 轴坐标，单位：m
float32 infantry4_y 

# 允许发弹量：
# 机器人自身拥有的 17mm 弹丸允许发弹量
uint16 allowance_17mm
# 剩余金币数量
uint16 remaining_gold_coins 
# 堡垒增益点提供的储备 17mm 弹丸允许发弹量；该值与机器人是否实际占领堡垒无关
uint16 reserve_allowance_17mm 
# 机器人 RFID 模块状态
uint32 rfid_status 

