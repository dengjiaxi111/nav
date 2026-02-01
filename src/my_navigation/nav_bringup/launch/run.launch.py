#!/usr/bin/env python3
"""
完整导航系统启动文件
启动组件:
  1. small_point_lio - LiDAR-惯性SLAM定位
  2. rog_map - 概率占用栅格地图
  3. nav_server - 导航服务器 (规划器 + NMPC控制器)
  4. RViz2 - 可视化
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ==================== 包路径 ====================
    nav_bringup_dir = get_package_share_directory('nav_bringup')
    rog_map_dir = get_package_share_directory('rog_map_ros2_node')
    
    # ==================== 配置文件 ====================
    nav_params_file = os.path.join(nav_bringup_dir, 'config', 'nav_params.yaml')
    rog_map_config = os.path.join(rog_map_dir, 'config', 'rog_map_config.yaml')
    projector_params = os.path.join(rog_map_dir, 'config', 'projector_params.yaml')
    stair_detector_params = os.path.join(nav_bringup_dir, 'config', 'stair_detector_params.yaml')
    
    # RViz 配置 - 使用导航专用配置
    rviz_config_file = os.path.join(nav_bringup_dir, 'rviz', 'navigation_full.rviz')
    
    # ==================== Launch 参数 ====================
    declare_nav_params = DeclareLaunchArgument(
        'nav_params_file',
        default_value=nav_params_file,
        description='导航参数文件路径'
    )
    
    declare_rog_map_config = DeclareLaunchArgument(
        'rog_map_config_file',
        default_value=rog_map_config,
        description='ROG-Map 配置文件路径'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='RViz2 配置文件路径'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    # ==================== 1. small_point_lio (SLAM定位) ====================
    small_point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('small_point_lio'),
                'launch',
                'small_point_lio.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # ==================== 2. ROG-Map (概率占用地图) ====================
    rog_map_node = Node(
        package='rog_map_ros2_node',
        executable='rog_map_node',
        name='rog_map',
        parameters=[
            {'config_file': LaunchConfiguration('rog_map_config_file')},
            projector_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        remappings=[
            # small_point_lio 输出
            ('cloud_in', '/cloud_registered'),
            ('odom_in', '/Odometry')
        ]
    )
    
    # ==================== 2.5. 台阶检测器 (基于ROG-Map) ====================
    stair_detector_node = Node(
        package='rog_map_ros2_node',
        executable='stair_detector_node',
        name='stair_detector',
        parameters=[
            stair_detector_params,  # 标准 ROS2 参数文件路径
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # ==================== 3. 导航服务器 (规划 + NMPC控制) ====================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav_bringup'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('nav_params_file')
        }.items()
    )
    
    # ==================== 4. RViz2 ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # ==================== Launch Description ====================
    return LaunchDescription([
        # 声明参数
        declare_nav_params,
        declare_rog_map_config,
        declare_rviz_config,
        declare_use_sim_time,
        
        # 启动节点
        small_point_lio_launch,
        rog_map_node,
        stair_detector_node,  # 台阶检测（依赖 ROG-Map）
        navigation_launch,
        rviz_node,
    ])
