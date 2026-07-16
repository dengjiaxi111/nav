#!/usr/bin/env python3
"""
定位初始化启动文件
启动定位初始化节点 + RViz，自动配置显示项
- 地图点云（红色）
- 当前扫描（绿色）
- 配准结果（蓝色）
- TF 树
- 2D Pose Estimate 工具
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('localization_initializer')
    config_file = os.path.join(pkg_dir, 'config', 'initializer_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'localization_init.rviz')
    
    # ==================== Launch 参数 ====================
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='定位初始化参数文件路径'
    )
    
    # 注意：map_file 默认从 YAML 配置文件读取
    # 只有在 launch 时显式指定 map_file:=/path/to/map.pcd 才会覆盖
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',  # 空值表示使用 YAML 配置
        description='官方地图 PCD 文件路径（可选，会覆盖 config 文件中的设置）'
    )
    
    declare_auto_init = DeclareLaunchArgument(
        'auto_initialize',
        default_value='false',
        description='是否自动初始化（跳过 RViz 手动设置）'
    )

    declare_use_initialpose_as_static_tf = DeclareLaunchArgument(
        'use_initialpose_as_static_tf',
        default_value='false',
        description='是否直接使用 RViz /initialpose 发布静态 map->odom TF'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动 RViz 可视化'
    )
    
    # ==================== 定位初始化节点 ====================
    # 参数优先级：launch 命令行 > YAML 配置文件
    # 如果 launch 时不指定 map_file，则使用 YAML 中的配置
    node_params = [LaunchConfiguration('config_file')]
    
    # 只在非空时覆盖 map_file（避免空字符串覆盖 YAML）
    map_file_arg = LaunchConfiguration('map_file')
    
    localization_init_node = Node(
        package='localization_initializer',
        executable='localization_initializer_node',
        name='localization_initializer',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # 只有显式指定时才覆盖 YAML
                'auto_initialize': LaunchConfiguration('auto_initialize'),
                'use_initialpose_as_static_tf': LaunchConfiguration('use_initialpose_as_static_tf'),
            }
        ],
        remappings=[
            # 输入
            ('/cloud_registered', '/cloud_registered'),  # 来自 small_point_lio
            ('/initialpose', '/initialpose'),            # 来自 RViz
            
            # 输出（可视化）
            ('/localization/aligned_scan', '/localization/aligned_scan'),
            ('/localization/map_cloud', '/localization/map_cloud'),
            ('/localization/status', '/localization/status'),
            ('/localization/fitness_marker', '/localization/fitness_marker'),
        ]
    )
    
    # ==================== RViz 可视化 ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        # 🔧 增加栈空间到 16MB (默认 8MB)，避免大点云处理时栈溢出
        SetEnvironmentVariable('ROS_STACK_SIZE', '16777216'),
        
        declare_config_file,
        declare_map_file,
        declare_auto_init,
        declare_use_initialpose_as_static_tf,
        declare_use_rviz,
        localization_init_node,
        rviz_node,
    ])
