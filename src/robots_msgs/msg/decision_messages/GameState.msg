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


