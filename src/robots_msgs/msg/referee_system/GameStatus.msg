#game status
uint8  game_type
uint8  game_progress
uint16 stage_remain_time             # 当前阶段剩余时间 

uint16 my_hp
uint16 my_outpost_hp
uint16 my_base_hp

float32 our_hero_x
float32 our_hero_y

uint16 enemy_1_robot_hp
uint16 enemy_2_robot_hp
uint16 enemy_3_robot_hp
uint16 enemy_4_robot_hp
uint16 enemy_5_robot_hp
uint16 enemy_7_robot_hp
uint16 enemy_outpost_hp
uint16 enemy_base_hp

uint8  external_supply_area_occupied # 己方补给站前补血点的占领状态
uint8  inner_supply_area_occupied    # 己方补给站内部补血点的占领状态
uint8  supply_area_occupied          # 己方补给区的占领状态
uint8  our_small_buff_activated      # 己方小符激活状态
uint8  our_big_buff_activated        # 己方大符激活状态
uint8 our_fortress_status            # 己方堡垒占领状态

uint16 remaining_gold_coin