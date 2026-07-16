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


# 视觉锁敌要用
# 当前云台相对底盘的角度
float32 base_yaw 
uint8 enemy_id
# baselink的x（米）
float32 enemy_x 
# baselink的y（米）
float32 enemy_y 
