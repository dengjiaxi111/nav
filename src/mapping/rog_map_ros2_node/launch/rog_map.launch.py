from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    package_dir = get_package_share_directory('rog_map_ros2_node')
    config_dir = os.path.join(package_dir, 'config')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_dir, 'rog_map_config.yaml'),
        description='Path to ROG-Map config file'
    )
    
    projector_params_file = os.path.join(config_dir, 'projector_params.yaml')
    stair_detector_params_file = os.path.join(config_dir, 'stair_detector_params.yaml')
    
    rviz_config = os.path.join(package_dir, 'rviz', 'rog_map.rviz')
    
    # ROG-Map 节点
    rog_map_node = Node(
        package='rog_map_ros2_node',
        executable='rog_map_node',
        name='rog_map',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            projector_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # 台阶检测节点
    stair_detector_node = Node(
        package='rog_map_ros2_node',
        executable='stair_detector_node',
        name='stair_detector',
        parameters=[
            stair_detector_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        config_file_arg,
        rog_map_node,
        stair_detector_node,
        rviz_node
    ])
