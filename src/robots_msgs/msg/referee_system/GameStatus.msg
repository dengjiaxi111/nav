# 比赛状态信息
uint8  game_type          # 比赛类型
uint8  game_progress      # 当前比赛阶段
uint16 stage_remain_time  # 当前阶段剩余时间，单位：s

uint16 my_hp         # 本机器人当前血量
uint16 my_outpost_hp # 己方前哨站当前血量
uint16 my_base_hp    # 己方基地当前血量

float32 our_hero_x # 己方英雄机器人位置 x 坐标
float32 our_hero_y # 己方英雄机器人位置 y 坐标

uint16 enemy_1_robot_hp # 敌方 1 号机器人当前血量
uint16 enemy_2_robot_hp # 敌方 2 号机器人当前血量
uint16 enemy_3_robot_hp # 敌方 3 号机器人当前血量
uint16 enemy_4_robot_hp # 敌方 4 号机器人当前血量
uint16 enemy_5_robot_hp # 敌方 5 号机器人当前血量
uint16 enemy_7_robot_hp # 敌方 7 号机器人当前血量
uint16 enemy_outpost_hp # 敌方前哨站当前血量
uint16 enemy_base_hp    # 敌方基地当前血量

uint8  external_supply_area_occupied # 己方补给站前补血点的占领状态
uint8  inner_supply_area_occupied    # 己方补给站内部补血点的占领状态
uint8  supply_area_occupied          # 己方补给区的占领状态
uint8  our_small_buff_activated      # 己方小符激活状态
uint8  our_big_buff_activated        # 己方大符激活状态
uint8 our_fortress_status            # 己方堡垒占领状态

uint16 remaining_gold_coin # 己方剩余金币数量
