"""
零拷贝感知模块 Launch 文件
使用 ComposableNodeContainer 实现进程内通信，真正的零拷贝

零拷贝工作原理：
1. ROGMapComponent 和 ObstaclePerceptionNode 在同一进程内运行
2. 发布端使用 std::unique_ptr<Message> + std::move() 发布
3. 订阅端直接获得消息的 shared_ptr，无需序列化/反序列化
4. 内存占用更低，延迟更小（约减少 50-70% 传输开销）

使用方式：
  ros2 launch rog_map_ros2_node perception_intraprocess.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rog_map_ros2_node')
    
    rog_map_config = os.path.join(pkg_dir, 'config', 'rog_map_config.yaml')
    perception_config = os.path.join(pkg_dir, 'config', 'perception_config.yaml')
    
    # 所有节点在同一个容器中运行，实现真正的进程内零拷贝
    # 默认使用单线程容器以减少全局线程数。若需要更高并发可改为 'component_container_mt'
    container = ComposableNodeContainer(
        name='rog_map_perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',  # 单线程容器，避免 spawn 过多 executor 线程
        composable_node_descriptions=[
            # ROG-Map 组件
            ComposableNode(
                package='rog_map_ros2_node',
                plugin='rog_map_ros2_node::ROGMapComponent',
                name='rog_map',
                parameters=[{'config_file': rog_map_config}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # 障碍物感知组件
            ComposableNode(
                package='rog_map_ros2_node',
                plugin='obstacle_perception::ObstaclePerceptionNode',
                name='obstacle_perception',
                parameters=[perception_config],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('rog_map/occ', '/rog_map/occ'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        container,
    ])

