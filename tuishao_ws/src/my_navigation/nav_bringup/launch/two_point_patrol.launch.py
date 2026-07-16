#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav_bringup')
    params_default = os.path.join(pkg_dir, 'config', 'nav_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_default,
            description='参数文件（可与 nav_server 共用）'
        ),
        Node(
            package='nav_components',
            executable='two_point_patrol',
            name='two_point_patrol',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
