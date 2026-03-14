#!/usr/bin/env python3
"""
完整导航系统启动文件
启动组件:
  1. small_point_lio - LiDAR-惯性SLAM定位
  2. localization_initializer - NDT 三阶段重定位 (map → odom TF)
  3. rog_map - 概率占用栅格地图
  4. nav_server - 导航服务器 (规划器 + NMPC控制器)
  5. RViz2 - 可视化 (含 2D Pose Estimate 支持)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ==================== 包路径 ====================
    nav_bringup_dir = get_package_share_directory('nav_bringup')
    rog_map_dir = get_package_share_directory('rog_map_ros2_node')
    loc_init_dir = get_package_share_directory('localization_initializer')
    
    # ==================== 配置文件 ====================
    nav_params_file = os.path.join(nav_bringup_dir, 'config', 'nav_params.yaml')
    rog_map_config = os.path.join(rog_map_dir, 'config', 'rog_map_config.yaml')
    projector_params = os.path.join(rog_map_dir, 'config', 'projector_params.yaml')
    stair_detector_params = os.path.join(rog_map_dir, 'config', 'stair_detector_params.yaml')
    loc_init_config = os.path.join(loc_init_dir, 'config', 'initializer_params.yaml')
    
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

    # ==================== 2. NDT 重定位 (map → odom TF) ====================
    # 启动后请在 RViz 中使用 '2D Pose Estimate' 工具给出初始位姿
    # 配准完成后会发布 map → odom 的静态 TF，替代原来 navigation.launch.py 中的临时 TF
    localization_init_node = Node(
        package='localization_initializer',
        executable='localization_initializer_node',
        name='localization_initializer',
        output='screen',
        parameters=[
            loc_init_config,
            {'auto_initialize': 'false'},
        ],
        remappings=[
            ('/cloud_registered', '/cloud_registered'),
            ('/initialpose', '/initialpose'),
            ('/localization/aligned_scan', '/localization/aligned_scan'),
            ('/localization/map_cloud', '/localization/map_cloud'),
            ('/localization/status', '/localization/status'),
            ('/localization/fitness_marker', '/localization/fitness_marker'),
        ]
    )

    
    # ==================== 2. ROG-Map 集成节点 (3D地图 + 2D投影 + 台阶检测) ====================
    # 使用组件化的 integration_node，直接调用内部 API，避免 topic 开销
    integration_node = Node(
        package='rog_map_ros2_node',
        executable='integration_node',
        name='rog_map_integration',
        parameters=[
            {'config_file': LaunchConfiguration('rog_map_config_file')},
            projector_params,
            stair_detector_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        remappings=[
            # small_point_lio 输出
            ('/cloud_registered', '/cloud_registered'),
            ('/Odometry', '/Odometry')
        ]
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
            'params_file': LaunchConfiguration('nav_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
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
        # 增加栈空间到 16MB，避免大点云处理时栈溢出
        SetEnvironmentVariable('ROS_STACK_SIZE', '16777216'),

        # 声明参数
        declare_nav_params,
        declare_rog_map_config,
        declare_rviz_config,
        declare_use_sim_time,
        
        # 启动节点 (按依赖顺序)
        small_point_lio_launch,    # 1. LIO 里程计 (odom → base_link)
        localization_init_node,    # 2. NDT 重定位 (map → odom)，等待 RViz 选点
        integration_node,          # 3. ROG-Map (3D地图 + 2D投影 + 台阶检测)
        navigation_launch,         # 4. 导航服务器 (规划 + NMPC)
        rviz_node,                 # 5. RViz (含 2D Pose Estimate)
    ])
