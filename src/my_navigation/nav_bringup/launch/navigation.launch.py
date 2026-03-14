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
    stair_mask_yaml = LaunchConfiguration('stair_mask_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_map = os.path.join(pkg_dir, 'maps', 'RMUC_final.yaml')
    default_stair_mask = os.path.join(pkg_dir, 'maps', 'RMUC_final_stair.yaml')
    
    # 注意: map → odom TF 现在由 localization_initializer (NDT重定位) 发布
    # 不再需要临时的 static_tf_map_odom
    # 如果需要在没有 localization_initializer 的情况下测试，可以取消下方注释:
    # static_tf_map_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_map_odom',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=['--x', '0', '--y', '0', '--z', '0.1',
    #                '--roll', '0', '--pitch', '0', '--yaw', '0',
    #                '--frame-id', 'map', '--child-frame-id', 'odom']
    # )
    
    # static_tf_odom_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_odom_base',
    #     arguments=['--x', '0', '--y', '0', '--z', '0', 
    #                '--roll', '0', '--pitch', '0', '--yaw', '0',
    #                '--frame-id', 'odom', '--child-frame-id', 'base_link']
    # )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav_params.yaml'),
            description='导航参数文件'
        ),

        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='全局静态地图yaml文件路径'
        ),

        DeclareLaunchArgument(
            'stair_mask_yaml',
            default_value=default_stair_mask,
            description='台阶mask yaml文件路径'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        
        # 静态TF (已由 localization_initializer 接管)
        # static_tf_map_odom,
        
        Node(
            package='nav_components',
            executable='nav_server',
            name='nav_server',
            parameters=[
                params_file,
                {'map_file': map_file},
                {'special_terrain.stair_mask_yaml': stair_mask_yaml},
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ]
        ),
    ])
