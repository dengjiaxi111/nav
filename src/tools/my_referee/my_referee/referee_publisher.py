from rclpy.node import Node
from robots_msgs.msg import RobotStatus, GameStatus, EnemyPose
from .common import GameStage
from .message_generator import MessageGenerator
class RefereePublisher:
    def __init__(self, node: Node):
        self.node = node
        self.message_generator = MessageGenerator()
        self.robot_status_publisher = self.node.create_publisher(
            RobotStatus,
            '/RobotFeedBack',
            10
        )
        self.game_status_publisher = self.node.create_publisher(
            GameStatus,
            '/GameFeedBack',
            10
        )
        self.enemy_pose_publisher = self.node.create_publisher(
            EnemyPose,
            '/EnemyPose',
            10
        )

    def publish_status(self, params: dict, game_stage: GameStage, remaining_time: float):
        """发布机器人状态信息"""
        robot_msg,game_msg,enemypose_msg = self.message_generator.generate_status(params, game_stage, remaining_time)
        
        self.robot_status_publisher.publish(robot_msg)
        self.game_status_publisher.publish(game_msg)
        self.enemy_pose_publisher.publish(enemypose_msg)