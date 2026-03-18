# src/config.py - 完整版配置（新增坐标映射配置）

try:
    from std_msgs.msg import String  # String从std_msgs导入
    from geometry_msgs.msg import PointStamped
    from sentry_decision.msg import SentryControl
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("警告: ROS2消息类型不可用，哨兵控制功能可能受限")

# 地图实际尺寸 (单位：cm)
MAP_REAL_WIDTH = 2800  # 28m = 2800cm
MAP_REAL_HEIGHT = 1500  # 15m = 1500cm

# 显示配置 - 修改为1400:750保持28:15比例
SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 750

# 计算缩放比例
UI_SCALE = min(SCREEN_WIDTH / MAP_REAL_WIDTH, SCREEN_HEIGHT / MAP_REAL_HEIGHT)
MAP_DISPLAY_WIDTH = int(MAP_REAL_WIDTH * UI_SCALE)
MAP_DISPLAY_HEIGHT = int(MAP_REAL_HEIGHT * UI_SCALE)
MAP_OFFSET_X = (SCREEN_WIDTH - MAP_DISPLAY_WIDTH) // 2
MAP_OFFSET_Y = (SCREEN_HEIGHT - MAP_DISPLAY_HEIGHT) // 2

# 其他配置
FPS = 60

# 颜色定义
RED = (255, 50, 50)
BLUE = (50, 50, 255)
GREEN = (50, 255, 50)
YELLOW = (255, 255, 50)
ORANGE = (255, 165, 0)
PURPLE = (180, 50, 200)
CYAN = (0, 200, 200)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
LIGHT_GRAY = (200, 200, 200)
DARK_GRAY = (50, 50, 50)
PANEL_GRAY = (70, 70, 80)
BUTTON_GRAY = (100, 100, 110)
HIGHLIGHT_BLUE = (100, 150, 255)

# 占领状态颜色
OCCUPATION_RED = (255, 100, 100)
OCCUPATION_BLUE = (100, 100, 255)
OCCUPATION_BOTH = (180, 100, 180)
OCCUPATION_NONE = (150, 150, 150)

# 机器人编号
ROBOT_IDS = [1, 2, 3, 4, 7]
ROBOT_NAMES = {
    1: "Hero",
    2: "Engineer", 
    3: "Infantry3",
    4: "Infantry4",
    7: "Sentry"
}

# 机器人中文名称
ROBOT_CN_NAMES = {
    1: "英雄",
    2: "工程", 
    3: "步兵3",
    4: "步兵4",
    7: "哨兵"
}

# 游戏阶段枚举
class GameStage:
    NOT_STARTED = 0
    PREPARATION = 1
    SELF_CHECK = 2
    COUNTDOWN = 3
    IN_PROGRESS = 4
    SETTLEMENT = 5

# 比赛类型枚举
class CompetitionType:
    SUPER_LEAGUE = 1
    SINGLE_EVENT = 2
    ICRA = 3
    UNIVERSITY_3V3 = 4
    UNIVERSITY_INFANTRY = 5

# 占领状态枚举
OCCUPATION_UNOCCUPIED = 0
OCCUPATION_ENEMY = 1
OCCUPATION_OUR = 2
OCCUPATION_BOTH = 3

# 哨兵姿态枚举
SENTRY_ATTACK = 1
SENTRY_DEFENSE = 2
SENTRY_MOVE = 3

# 能量机关状态枚举
ENERGY_MECHANISM_INACTIVE = 0
ENERGY_MECHANISM_ACTIVE = 1
ENERGY_MECHANISM_ACTIVATING = 2

# 飞镖目标枚举
DART_TARGET_NONE = 0
DART_TARGET_OUTPOST = 1
DART_TARGET_BASE_FIXED = 2
DART_TARGET_BASE_RANDOM_FIXED = 3
DART_TARGET_BASE_RANDOM_MOVING = 4
DART_TARGET_BASE_END_MOVING = 5

# 默认值配置
DEFAULT_HP = {
    1: 200,    # Hero
    2: 250,    # Engineer
    3: 150,    # Infantry3
    4: 150,    # Infantry4
    7: 400     # Sentry
}

DEFAULT_ALLOWANCE = {
    1: 0,    # Hero
    3: 0,    # Infantry3
    4: 0,    # Infantry4
    7: 300      # Sentry
}

OUTPOST_HP = 1500
BASE_HP = 5000
GAME_TIME = 7 * 60  # 7 minutes
GOLD_COINS = 400

# 增益默认值
BUFF_DEFAULT = 0.0

# UI常量
UI_PADDING = 20
UI_MARGIN = 10
BUTTON_HEIGHT = 40
INPUT_HEIGHT = 35
LABEL_HEIGHT = 25
SECTION_SPACING = 30
COLUMN_SPACING = 20

# 控制面板尺寸
CONTROL_WIDTH = 1200
CONTROL_HEIGHT = 1100

# 增益区定义（红方视角）- 坐标单位为cm
GAIN_ZONES = {
    # 梯形高地
    'trapezoid_highland': {
        'points': [(358, 1070), (518, 1066), (552, 1144), (366, 1146)],
        'field': 'trapezoid_highland_occupation'
    },
    # 基地增益区 - 修正坐标错误
    'base_gain_point': {
        'points': [(274, 926), (140, 860), (144, 638), (274, 582), (458, 678), (454, 834)],
        'field': 'base_gain_point_occupation'
    },
    # 补给区增益区 - 修正坐标错误
    'supply_zone': {
        'points': [(170, 304), (380, 302), (392, 62), (202, 50)],
        'field': 'supply_zone_occupation'
    },
    # 堡垒区增益区
    'fortress_gain_point': {
        'points': [(638, 852), (746, 854), (798, 754), (752, 662), (628, 650), (572, 732)],
        'field': 'fortress_gain_point_occupation'
    },
    # 中央高地增益区 - 由两部分组成
    'central_highland_part1': {
        'points': [(1030, 626), (1126, 628), (1140, 898), (1054, 896)],
        'field': 'central_highland_occupation',
        'is_part': True
    },
    'central_highland_part2': {
        'points': [(1140, 898), (1054, 896), (1254, 1206), (1270, 1098)],
        'field': 'central_highland_occupation',
        'is_part': True
    },
    # 前哨站增益区
    'outpost_gain_point': {
        'points': [(1062, 480), (1146, 476), (1204, 416), (1194, 336), (1144, 304), (1054, 300)],
        'field': 'outpost_gain_point_occupation'
    }
}

# 增益区名称映射
GAIN_ZONE_NAMES = {
    'trapezoid_highland': '梯形高地',
    'base_gain_point': '基地增益点',
    'supply_zone': '补给区',
    'fortress_gain_point': '堡垒增益点',
    'central_highland': '中央高地',
    'outpost_gain_point': '前哨站增益点'
}

# 增益区字段映射（用于控制面板显示）
GAIN_ZONE_FIELDS = [
    'supply_zone_occupation',
    'central_highland_occupation',
    'trapezoid_highland_occupation',
    'fortress_gain_point_occupation',
    'outpost_gain_point_occupation',
    'base_gain_point_occupation'
]

# 增益区显示名称映射
GAIN_ZONE_DISPLAY_NAMES = {
    'supply_zone_occupation': '补给区',
    'central_highland_occupation': '中央高地',
    'trapezoid_highland_occupation': '梯形高地',
    'fortress_gain_point_occupation': '堡垒增益点',
    'outpost_gain_point_occupation': '前哨站增益点',
    'base_gain_point_occupation': '基地增益点'
}

# ==================== 新增坐标映射配置 ====================
# 逻辑坐标到真实坐标的映射（用于实车调试）
# 决策系统通过 /sentry/target_position 发送的逻辑坐标 (x, y)（单位：逻辑单位，通常为米）
# 该映射将逻辑坐标转换为地图上的真实坐标（单位：厘米）
# 请根据实际场地填写真实坐标 (x_cm, y_cm)
LOGICAL_TO_REAL_MAP = {
    # 逻辑坐标 (x, y)  -> 真实坐标 (x_cm, y_cm)
    # 以下预留三种情况，真实坐标请自行填写（当前为None表示未配置）
    (-5.8, -1.3): (1152.0, 992.0),  
    (-1.9, -3.8): (1642.0, 1146.0),  
    (-0.9, -0.3): (698.0, 756.0), 
    (-4.8, 1.2): (176.0, 262.0),  
}
# ========================================================
