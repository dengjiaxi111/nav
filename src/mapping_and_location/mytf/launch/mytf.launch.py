from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='baselink2odom',
        #     arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'odom', 'base_link']
        #     #相当先旋转，然后相对原坐标系平移。
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_lidar2map',
            arguments=['0.012', '-0.18', '0.31', '0.0', '0.0', '-0.7853', 'map', 'base_lidar']
            #相当先旋转，然后相对原坐标系平移。 xyz ypr
        ),

        Node(
            package='mytf',
            executable='enemyTF',
            name='enemyTF'
        )
    ])