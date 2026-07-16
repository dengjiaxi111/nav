from launch import LaunchDescription
from launch_ros.actions import Node
import os.path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_path            = get_package_share_directory('mybringup')
    default_rviz_config_path = os.path.join(bringup_path, 'rviz', 'test.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/super259/navigationros2/src/mapping_and_location/RMUC_pcd_pgm/map.yaml'
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path]
        )
        ])
