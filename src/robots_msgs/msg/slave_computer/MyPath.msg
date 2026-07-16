# 路径规划/导航路径消息
uint8 intention  # 路径意图或任务类型
uint16 start_x   # 起点 x 坐标
uint16 start_y   # 起点 y 坐标
int8[] delta_x   # 路径相邻点 x 方向增量序列
int8[] delta_y   # 路径相邻点 y 方向增量序列
uint16 sender_id # 发送方 ID
