"""
感知模块 Launch 文件 (独立进程版本)
启动 ROG-Map + Obstacle Perception（各自独立进程）

如需零拷贝优化，请使用 perception_intraprocess.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('rog_map_ros2_node')
    
    # 配置文件路径
    rog_map_config = os.path.join(pkg_dir, 'config', 'rog_map_config.yaml')
    perception_config = os.path.join(pkg_dir, 'config', 'perception_config.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'perception.rviz')
    
    # Launch 参数
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    use_component = LaunchConfiguration('use_component')
    use_component_arg = DeclareLaunchArgument(
        'use_component',
        default_value='false',
        description='Use component node instead of standalone (for rog_map)'
    )
    
    # ROG-Map 节点 (独立进程)
    rog_map_node = Node(
        package='rog_map_ros2_node',
        executable='rog_map_node',
        name='rog_map',
        parameters=[{'config_file': rog_map_config}],
        output='screen'
    )
    
    # 障碍物感知节点 (独立进程)
    perception_node = Node(
        package='rog_map_ros2_node',
        executable='obstacle_perception_node',
        name='obstacle_perception',
        parameters=[perception_config],
        output='screen',
        remappings=[
            ('rog_map/occ', '/rog_map/occ'),
            ('/Odometry', '/Odometry'),
        ]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        use_component_arg,
        rog_map_node,
        perception_node,
    ])
