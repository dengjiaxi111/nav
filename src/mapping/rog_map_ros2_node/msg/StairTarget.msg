# 台阶检测目标信息
# 用于轮腿机器人助跑起跳上台阶

std_msgs/Header header

# 台阶中心位置 (在 odom 坐标系)
geometry_msgs/Point center

# 台阶前沿距离机器人的距离 (米)
# 用于判断起跳时机，当 edge_distance < 0.3m 时应进入盲区锁存模式
float32 edge_distance

# 台阶高度 (米)
# 可能的值: 0.20m (一级台阶) 或 0.35m (二级台阶: 0.20 + 0.15)
float32 height

# 台阶类型
# 1 = 一级台阶 (0.20m)
# 2 = 二级台阶 (0.35m = 0.20 + 0.15)
uint8 type

# 检测置信度 (0.0 ~ 1.0)
# 基于连续检测帧数: confidence = min(1.0, detected_frames / 3.0)
float32 confidence

# 台阶宽度 (米)
# 用于横向对准容差判断
float32 width

# 检测状态
# 0 = IDLE (未检测到)
# 1 = DETECTED (已检测到，但置信度未满)
# 2 = LOCKED (已锁定，可以执行上台阶动作)
# 3 = BLIND_ZONE (盲区模式，依赖里程计推算)
uint8 status

# 状态常量定义
uint8 STATUS_IDLE = 0
uint8 STATUS_DETECTED = 1
uint8 STATUS_LOCKED = 2
uint8 STATUS_BLIND_ZONE = 3
