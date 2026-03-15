# 对方机器人的位置坐标：
# 对方英雄机器人位置 x 轴坐标，单位：cm
float32 enemy_hero_x 
# 对方英雄机器人位置 y 轴坐标，单位：cm
float32 enemy_hero_y 
# 对方工程机器人位置 x 轴坐标，单位：cm
float32 enemy_engineer_x 
# 对方工程机器人位置 y 轴坐标，单位：cm
float32 enemy_engineer_y 
# 对方步兵3机器人位置 x 轴坐标，单位：cm
float32 enemy_infantry3_x 
# 对方步兵3机器人位置 y 轴坐标，单位：cm
float32 enemy_infantry3_y 
# 对方步兵4机器人位置 x 轴坐标，单位：cm
float32 enemy_infantry4_x 
# 对方步兵4机器人位置 y 轴坐标，单位：cm
float32 enemy_infantry4_y 
# 对方哨兵机器人位置 x 轴坐标，单位：cm
float32 enemy_sentry_x 
# 对方哨兵机器人位置 y 轴坐标，单位：cm
float32 enemy_sentry_y 
# 对方空中机器人位置 x 轴坐标，单位：cm
float32 enemy_aerial_x 
# 对方空中机器人位置 y 轴坐标，单位：cm
float32 enemy_aerial_y 

# 对方机器人的血量信息：
# 对方 1 号英雄机器人血量，若该机器人未上场或者被罚下，则血量为 0，下文同理
uint16 enemy_hero_hp 
# 对方 2 号工程机器人血量
uint16 enemy_engineer_hp 
# 对方 3 号步兵机器人血量
uint16 enemy_infantry3_hp 
# 对方 4 号步兵机器人血量
uint16 enemy_infantry4_hp 
# 对方 7 号哨兵机器人血量
uint16 enemy_sentry_hp 

# 对方机器人的剩余发弹量信息：
# 对方 1 号英雄机器人允许发弹量
uint16 enemy_hero_allowance 
# 对方 3 号步兵机器人允许发弹量（含堡垒提供的储备允许发弹量，下同）
uint16 enemy_infantry3_allowance 
# 对方 4 号步兵机器人允许发弹量
uint16 enemy_infantry4_allowance 
# 对方 7 号哨兵机器人允许发弹量
uint16 enemy_sentry_allowance 
# 对方 6 号空中机器人允许发弹量
uint16 enemy_aerial_allowance 

# 对方队伍的宏观状态信息：
# 对方剩余金币数
uint16 enemy_remaining_gold_coins 
# 对方累计总金币数
uint16 enemy_total_gold_coins 
# 对方补给区占领状态
uint8 enemy_supply_zone_occupation 
# 对方中央高地的占领状态，1 为被对方占领，2 为被己方占领
uint8 enemy_central_highland_occupation 
# 对方梯形高地的占领状态，1 为已占领
uint8 enemy_trapezoid_highland_occupation 
# 对方堡垒增益点的占领状态，0 为未被占领，1 为被对方占领，2 为被己方占领，3 为被双方占领
uint8 enemy_fortress_gain_point_occupation 
# 对方前哨站增益点的占领状态，0 为未被占领，1 为被对方占领，2 为被己方占领
uint8 enemy_outpost_gain_point_occupation 
# 对方基地增益点的占领状态，1 为已占领
uint8 enemy_base_gain_point_occupation 
# 靠近对方一侧飞坡前地形跨越增益点（隧道）中心处场地交互模块卡的状态，
uint8 enemy_near_tunnel_before_ramp_card 
# 靠近对方一侧飞坡后地形跨越增益点（隧道）中心处场地交互模块卡的状态，1 为被对方占领
uint8 enemy_near_tunnel_after_ramp_card 
# 靠近己方一侧飞坡前地形跨越增益点（隧道）中心处场地交互模块卡的状态，1 为被对方占领
uint8 own_near_tunnel_before_ramp_card 
# 靠近己方一侧飞坡后地形跨越增益点（隧道）中心处场地交互模块卡的状态，1 为被对方占领
uint8 own_near_tunnel_after_ramp_card 
# 对方地形跨越增益点（高地）上部场地交互模块卡的状态，1 为被对方占领
uint8 enemy_highland_upper_card 
# 对方地形跨越增益点（飞坡）上部场地交互模块卡的状态，1 为被对方占领
uint8 enemy_ramp_upper_card 
# 对方地形跨越增益点（公路）上部场地交互模块卡的状态，1 为被对方占领
uint8 enemy_road_upper_card 

# 对方各机器人当前增益效果：
# 对方英雄机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%，下同）
float32 enemy_hero_hp_recovery_buff 
# 对方英雄机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s，下同）
float32 enemy_hero_heat_cooling_buff 
# 对方英雄机器人防御增益（百分比，值为 50 表示 50%防御增益，下同）
float32 enemy_hero_defense_buff
# 对方英雄机器人负防御增益（百分比，值为 30 表示-30%防御增益，下同）
float32 enemy_hero_negative_defense_buff 
# 对方英雄机器人攻击增益（百分比，值为 50 表示 50%攻击增益，下同）
float32 enemy_hero_attack_buff 
# 对方工程机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%，下同）
float32 enemy_engineer_hp_recovery_buff 
# 对方工程机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s，下同）
float32 enemy_engineer_heat_cooling_buff 
# 对方工程机器人防御增益（百分比，值为 50 表示 50%防御增益，下同）
float32 enemy_engineer_defense_buff 
# 对方工程机器人负防御增益（百分比，值为 30 表示-30%防御增益，下同）
float32 enemy_engineer_negative_defense_buff 
# 对方工程机器人攻击增益（百分比，值为 50 表示 50%攻击增益，下同）
float32 enemy_engineer_attack_buff 
# 对方步兵3机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%，下同）
float32 enemy_infantry3_hp_recovery_buff 
# 对方步兵3机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s，下同）
float32 enemy_infantry3_heat_cooling_buff 
# 对方步兵3机器人防御增益（百分比，值为 50 表示 50%防御增益，下同）
float32 enemy_infantry3_defense_buff 
# 对方步兵3机器人负防御增益（百分比，值为 30 表示-30%防御增益，下同）
float32 enemy_infantry3_negative_defense_buff 
# 对方步兵3机器人攻击增益（百分比，值为 50 表示 50%攻击增益，下同）
float32 enemy_infantry3_attack_buff 
# 对方步兵4机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%，下同）
float32 enemy_infantry4_hp_recovery_buff 
# 对方步兵4机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s，下同）
float32 enemy_infantry4_heat_cooling_buff 
# 对方步兵4机器人防御增益（百分比，值为 50 表示 50%防御增益，下同）
float32 enemy_infantry4_defense_buff 
# 对方步兵4机器人负防御增益（百分比，值为 30 表示-30%防御增益，下同）
float32 enemy_infantry4_negative_defense_buff
# 对方步兵4机器人攻击增益（百分比，值为 50 表示 50%攻击增益，下同）
float32 enemy_infantry4_attack_buff
# 对方哨兵机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%，下同）
float32 enemy_sentry_hp_recovery_buff 
# 对方哨兵机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s，下同）
float32 enemy_sentry_heat_cooling_buff 
# 对方哨兵机器人防御增益（百分比，值为 50 表示 50%防御增益，下同）
float32 enemy_sentry_defense_buff 
# 对方哨兵机器人负防御增益（百分比，值为 30 表示-30%防御增益，下同）
float32 enemy_sentry_negative_defense_buff 
# 对方哨兵机器人攻击增益（百分比，值为 50 表示 50%攻击增益，下同）
float32 enemy_sentry_attack_buff 

# 对方哨兵机器人当前姿态
uint8 enemy_sentry_posture 
