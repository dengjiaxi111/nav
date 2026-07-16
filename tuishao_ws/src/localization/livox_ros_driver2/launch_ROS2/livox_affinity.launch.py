#!/usr/bin/env python3
"""
Livox ROS2 Driver with CPU Affinity Support
Supports thread binding to specific CPU cores and optional real-time priority.

Usage examples:
  # Basic: no CPU affinity
  ros2 launch livox_ros_driver2 livox_affinity.launch.py

  # With CPU core binding (recommended)
  ros2 launch livox_ros_driver2 livox_affinity.launch.py imu_cpu_core:=4 pcd_cpu_core:=5

  # With real-time priority (requires sudo)
  sudo ros2 launch livox_ros_driver2 livox_affinity.launch.py \\
    imu_cpu_core:=4 pcd_cpu_core:=5 realtime_priority:=80
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory for config files
    pkg_dir = get_package_share_directory('livox_ros_driver2')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Default config file (MID360)
    default_config = os.path.join(config_dir, 'MID360_config.json')
    
    # Declare launch arguments - CPU affinity
    imu_cpu_core_arg = DeclareLaunchArgument(
        'imu_cpu_core',
        default_value='-1',
        description='CPU core for IMU polling thread (-1 = disabled)'
    )
    pcd_cpu_core_arg = DeclareLaunchArgument(
        'pcd_cpu_core',
        default_value='-1',
        description='CPU core for PointCloud polling thread (-1 = disabled)'
    )
    realtime_priority_arg = DeclareLaunchArgument(
        'realtime_priority',
        default_value='-1',
        description='Real-time scheduling priority (1-99, -1 = disabled, requires sudo)'
    )
    
    # Declare launch arguments - standard parameters
    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='1',
        description='Transfer format (0=PointCloud2, 1=CustomMsg)'
    )
    multi_topic_arg = DeclareLaunchArgument(
        'multi_topic',
        default_value='1',
        description='Multi-topic mode (0=shared, 1=per-lidar)'
    )
    publish_freq_arg = DeclareLaunchArgument(
        'publish_freq',
        default_value='20.0',
        description='Publish frequency (5.0, 10.0, 20.0, 50.0, etc.)'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='Frame ID for published messages'
    )
    user_config_path_arg = DeclareLaunchArgument(
        'user_config_path',
        default_value=default_config,
        description='Path to user config file'
    )
    
    # Create node with parameters
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {
                'xfer_format': LaunchConfiguration('xfer_format'),
                'multi_topic': LaunchConfiguration('multi_topic'),
                'data_src': 0,
                'publish_freq': LaunchConfiguration('publish_freq'),
                'output_data_type': 0,
                'frame_id': LaunchConfiguration('frame_id'),
                'user_config_path': LaunchConfiguration('user_config_path'),
                'cmdline_input_bd_code': '000000000000001',
                'lvx_file_path': '/tmp/livox_test.lvx',
                'enable_timestamp_logging': False,
                'timestamp_log_file': '/tmp/livox_timestamp.csv',
                
                # CPU affinity parameters (new)
                'imu_cpu_core': LaunchConfiguration('imu_cpu_core'),
                'pcd_cpu_core': LaunchConfiguration('pcd_cpu_core'),
                'realtime_priority': LaunchConfiguration('realtime_priority'),
                'enable_timestamp_logging': False
            },
        ],
    )
    
    return LaunchDescription([
        # CPU affinity arguments
        imu_cpu_core_arg,
        pcd_cpu_core_arg,
        realtime_priority_arg,
        
        # Standard arguments
        xfer_format_arg,
        multi_topic_arg,
        publish_freq_arg,
        frame_id_arg,
        user_config_path_arg,
        
        # Node
        livox_driver,
    ])

