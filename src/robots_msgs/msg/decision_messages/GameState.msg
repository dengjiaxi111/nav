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

# 场地事件数据：
# 己方与资源区区不重叠的补给区占领状态，1 为已占领
uint8 supply_zone_no_overlap 
# 己方与资源区重叠的补给区占领状态，1 为已占领
uint8 supply_zone_overlap 
# 己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
uint8 supply_zone_occupation 
# 己方能量机关状态
uint8 energy_mechanism_status 
# 己方小能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
uint8 small_energy_mechanism_activation 
# 己方大能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
uint8 large_energy_mechanism_activation 
# 己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
uint8 central_highland_occupation 
# 己方梯形高地的占领状态，1 为已占领
uint8 trapezoid_highland_occupation 
# 对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为 0）
uint16 dart_hit_time 
# 对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标，5 为击中基地末端移动目标
uint8 dart_hit_target 
# 中心增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）
uint8 center_gain_point_occupation 
# 己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
uint8 fortress_gain_point_occupation 
# 己方前哨站增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
uint8 outpost_gain_point_occupation 
# 己方基地增益点的占领状态，1 为已占领
uint8 base_gain_point_occupation 

# 哨兵自主决策信息同步：
# 除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值
uint16 exchanged_allowance 
# 哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为 1，否则为 0
uint8 free_resurrection_available 
# 哨兵当前姿态：1 为进攻姿态，2 为防御姿态，3 为移动姿态
uint8 sentry_posture 
# 己方能量机关是否能够进入正在激活状态：1 为当前可激活
uint8 energy_mechanism_activatable 
