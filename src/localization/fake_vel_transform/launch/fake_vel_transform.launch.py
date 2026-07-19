import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_fake_vel = LaunchConfiguration('use_fake_vel', default='true')
    fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'use_fake_vel': use_fake_vel, 'alpha': 0.9, "fake_angular_speed_coefficient": 1.0}
        ]
    )

    return LaunchDescription([fake_vel_transform_node])
