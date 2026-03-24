# src/config.py - 省赛版本

try:
    from std_msgs.msg import String
    from geometry_msgs.msg import PointStamped
    from sentry_decision.msg import SentryControl
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("警告: ROS2消息类型不可用，哨兵控制功能可能受限")

# 地图实际尺寸 (单位：cm)  12m x 8m
MAP_REAL_WIDTH = 1200   # 12m = 1200cm
MAP_REAL_HEIGHT = 800   # 8m = 800cm

# 显示配置 - 保持 12:8 比例
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800

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

# 机器人编号（省赛只保留1,3,7）
ROBOT_IDS = [1, 3, 7]
ROBOT_NAMES = {
    1: "Hero",
    3: "Infantry3",
    7: "Sentry"
}
ROBOT_CN_NAMES = {
    1: "英雄",
    3: "步兵3",
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

# 哨兵姿态枚举
SENTRY_ATTACK = 1
SENTRY_DEFENSE = 2
SENTRY_MOVE = 3

# 默认值配置
DEFAULT_HP = {
    1: 200,    # Hero
    3: 150,    # Infantry3
    7: 400     # Sentry
}

DEFAULT_ALLOWANCE = {
    1: 0,    # Hero
    3: 0,    # Infantry3
    7: 300   # Sentry
}

OUTPOST_HP = 1500   # 前哨站血量（可根据省赛规则修改）
BASE_HP = 5000      # 基地血量
GAME_TIME = 7 * 60  # 7分钟
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
CONTROL_WIDTH = 1000
CONTROL_HEIGHT = 600
