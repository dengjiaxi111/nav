from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('rog_map_ros2_node')
    config_dir = os.path.join(package_dir, 'config')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_dir, 'rog_map_config.yaml'),
        description='Path to ROG-Map config file'
    )
    
    rviz_config = os.path.join(package_dir, 'rviz', 'rog_map.rviz')
    
    rog_map_node = Node(
        package='rog_map_ros2_node',
        executable='rog_map_node',
        name='rog_map',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')}
        ],
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        rog_map_node,
        rviz_node
    ])
