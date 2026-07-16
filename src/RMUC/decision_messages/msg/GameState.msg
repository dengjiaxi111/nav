# 比赛类型：
# 1：RoboMaster 机甲大师超级对抗赛
# 2：RoboMaster 机甲大师高校单项赛
# 3：ICRA RoboMaster 高校人工智能挑战赛
# 4：RoboMaster 机甲大师高校联盟赛 3V3 对抗
# 5：RoboMaster 机甲大师高校联盟赛步兵对抗
uint8 competition_type 

# 当前比赛阶段：
# 0：未开始比赛
# 1：准备阶段
# 2：十五秒裁判系统自检阶段
# 3：五秒倒计时
# 4：比赛中
# 5：比赛结算中
uint8 stage 

# 当前阶段剩余时间，单位：秒
float64 stage_remaining_time 


# 己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
uint8 fortress_gain_point_occupation 


# 基地打开状态：0正常，1红方基地打开，2蓝方基地打开
int8 baseopen
# 前哨站存活标志位：1存活，0不存活
int8 outpost_alive
