import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_directory('lio_3se')
    default_config_path = os.path.join(package_path,'config')
    default_config_file = 'mid360.yaml'
    default_rviz_path   = os.path.join(package_path,'rviz','relocalization.rviz')
    rviz_use = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument('rviz', default_value='true', description='Use RViz to monitor results')

    lio_3se_node = Node(
    package='lio_3se',
    executable='lio_3se_relocalization',
    parameters=[PathJoinSubstitution([default_config_path, default_config_file])],output='screen')
    
    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', default_rviz_path],
    condition=IfCondition(rviz_use))

    ld = LaunchDescription()
    ld.add_action(declare_rviz_cmd)

    ld.add_action(lio_3se_node)
    ld.add_action(rviz_node)

    return ld





# def generate_launch_description():
#     return LaunchDescription([
#         # Declare launch arguments
#         DeclareLaunchArgument(
#             'rviz',
#             default_value='true',
#             description='Launch RViz if set to true'),

#         # Node for relocalization with parameters set directly
#         Node(
#             package='lio_3se',
#             executable='lio_3se_relocalization',
#             name='relocalization',
#             output='screen',
#             parameters=[{
#                 'feature_extract_enable': 0,
#                 'point_filter_num': 2,
#                 'max_iteration': 4,
#                 'filter_size_surf': 0.5,
#                 'filter_size_map': 0.5,
#                 'cube_side_length': 1000,
#                 'preprocess.scan_rate': 10,
#                 'ICP/icp_max_correspondence_distance': 5.0,
#                 'ICP/icp_transformation_epsilon': 1e-8,
#                 'ICP/icp_euclidean_fitness_epsilon': 1e-8,
#                 'ICP/icp_maximum_iterations': 50,
#                 'ICP/icp_need_iterations': 5,
#             }]
#         ),

#         # Group action to conditionally launch RViz
#         GroupAction([
#             Node(
#                 package='rviz2',
#                 executable='rviz2',
#                 name='rviz',
#                 output='screen',
#                 arguments=['-d', '$(find lio_3se)/config/relocalization.rviz']
#             )
#         ], condition=IfCondition(LaunchConfiguration('rviz'))),
#     ])
