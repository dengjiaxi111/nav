from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get paths for rog_map_ros2_node configuration
    rog_map_package_dir = get_package_share_directory('rog_map_ros2_node')
    rog_map_config_dir = os.path.join(rog_map_package_dir, 'config')
    rog_map_rviz_config = os.path.join(rog_map_package_dir, 'rviz', 'rog_map.rviz')
    
    # Get paths for small_point_lio configuration (if available)
    try:
        small_point_lio_package_dir = get_package_share_directory('small_point_lio')
        small_point_lio_config_dir = os.path.join(small_point_lio_package_dir, 'config')
    except:
        small_point_lio_package_dir = None
        small_point_lio_config_dir = None
    
    # Launch argument for rog_map config file
    config_file_arg = DeclareLaunchArgument(
        'rog_map_config_file',
        default_value=os.path.join(rog_map_config_dir, 'rog_map_config.yaml'),
        description='Path to ROG-Map config file'
    )
    
    # small_point_lio ROS2 node
    small_point_lio_node = Node(
        package='small_point_lio',
        executable='small_point_lio_node',
        name='small_point_lio',
        output='screen'
    )
    
    # ROG-Map ROS2 node (subscribes to /cloud_registered from small_point_lio)
    rog_map_node = Node(
        package='rog_map_ros2_node',
        executable='rog_map_node',
        name='rog_map',
        parameters=[
            {'config_file': LaunchConfiguration('rog_map_config_file')}
        ],
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rog_map_rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        small_point_lio_node,
        rog_map_node,
        rviz_node
    ])
