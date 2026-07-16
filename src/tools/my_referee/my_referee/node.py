import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from robots_msgs.msg import ModeCmd
from .state_machine import RefereeStateMachine
from .referee_publisher import RefereePublisher

class MyRefereeNode(Node):
    def __init__(self):
        super().__init__('my_referee_node')
        self.state_machine = RefereeStateMachine(self)
        self.referee_publisher = RefereePublisher(self)
        self.remaining_time_callback = None  # GUI 可以绑定这个回调
        self.gui = None  # 存储GUI引用
        self.modecmd = ModeCmd()

        # 初始化TF发布
        self.tf_broadcaster = TransformBroadcaster(self)

        # 创建一个2Hz的定时器用于更新UI
        self.update_timer = self.create_timer(0.5, self.update_callback)

        #创建一个20Hz的定时器用于发布比赛状态
        self.message_timer = None
        self.get_logger().info("MyRefereeNode LAUNCHED")

        #订阅modemcmd消息
        self.modecmd_subscription = self.create_subscription(ModeCmd, "ModeCmd", self.modecmd_callback, 10)

    def set_gui(self, gui):
        """设置GUI引用"""
        self.gui = gui
        self.message_timer = self.create_timer(0.05, self.publish_status)

    def update_callback(self):
        """定时器回调函数"""
        if (self.gui is not None and hasattr(self.state_machine, 'get_remaining_time') and hasattr(self.gui, 'stage_remain_time_label')):  
            current_time = self.state_machine.get_remaining_time()
            current_stage = self.state_machine.current_stage.value if self.state_machine.current_stage else "未知"
            self.gui.update_ui(current_time, self.modecmd, current_stage)

    def publish_game_status(self, status: str):
        msg = String()
        msg.data = status
        self.get_logger().info(f"切换比赛状态: {status}")

    def set_stage(self, stage, duration=0):
        self.state_machine.change_stage(stage, duration)

    def update_remaining_time(self, seconds: int):
        if self.remaining_time_callback:
            self.remaining_time_callback(seconds)

    def publish_status(self):
        """发布当前状态"""
        if not self.gui or not hasattr(self.gui, '_cached_params'):
            return
        params = self.gui.get_params()
        game_stage = self.state_machine.current_stage
        remain_time = self.state_machine.get_remaining_time()
        self.referee_publisher.publish_status(params, game_stage, remain_time)

        t1 = TransformStamped()
        # 设置基本信息
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'  
        t1.child_frame_id = 'odom'  
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'  
        t2.child_frame_id = 'base_link'  
        t2.transform.translation.x = params.get('sentry', {}).get('x', 0.0)
        t2.transform.translation.y = params.get('sentry', {}).get('y', 0.0)
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        #self.tf_broadcaster.sendTransform(t1)
        #self.tf_broadcaster.sendTransform(t2)

    def modecmd_callback(self, msg: ModeCmd):
        self.modecmd = msg

