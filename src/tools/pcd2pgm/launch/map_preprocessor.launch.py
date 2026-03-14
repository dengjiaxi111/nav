from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd2pgm',  
            executable='map_preprocessor',
            name='map_preprocessor',
            parameters=[
                {'map_path': '/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/scans.pcd'},        # 替换为实际路径
                {'save_path': '/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/scans_prior.pcd'},  # 替换为实际路径
                {'voxel_leaf': 0.1},
                {'z_min': -0.3},
                {'z_max': 25.0},
                {'lidar_pos': [0.045, 0.123, 0.31]},
                {'lidar_rot': [-0.383, 0.0, 0.0, 0.924]}
            ],
            output='screen'
        )
    ])
