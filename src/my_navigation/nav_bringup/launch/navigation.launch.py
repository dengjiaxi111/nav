#!/usr/bin/env python3
# 导航服务器启动文件

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('nav_bringup')
    
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    
    # 默认地图路径
    default_map = os.path.join(pkg_dir, 'maps', 'test_map.yaml')
    
    # 临时静态TF: map -> odom -> base_link (测试用)
    # RMUC地图原点 [-13.4, -23.8], 地图约 46x39m, 设置机器人在地图中部
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )
    
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['--x', '5', '--y', '3', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_link']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav_params.yaml'),
            description='导航参数文件'
        ),
        
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='地图yaml文件路径'
        ),
        
        # 静态TF (测试用)
        static_tf_map_odom,
        static_tf_odom_base,
        
        Node(
            package='nav_components',
            executable='nav_server',
            name='nav_server',
            parameters=[
                params_file,
                {'map_file': map_file}
            ],
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ]
        ),
    ])
