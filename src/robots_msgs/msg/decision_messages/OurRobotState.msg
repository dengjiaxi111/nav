
# 机器人性能体系数据：
# 本机器人 ID，0红方 1蓝方
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


# 允许发弹量：
# 机器人自身拥有的 17mm 弹丸允许发弹量
uint16 allowance_17mm

