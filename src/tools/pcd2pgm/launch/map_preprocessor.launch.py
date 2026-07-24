from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd2pgm',  
            executable='map_preprocessor',
            name='map_preprocessor',
            parameters=[
                {'map_path': '/home/super259/nav/src/tools/pcd2pgm/save_pcd/scans.pcd'},        # 替换为实际路径
                {'save_path': '/home/super259/nav/src/tools/pcd2pgm/save_pcd/scans_prior.pcd'},  # 替换为实际路径
                {'voxel_leaf': 0.1},
                {'z_min': -0.3},
                {'z_max': 25.0},
                {'lidar_pos': [0.2, 0.0, 0.1]},
                {'lidar_rot': [-0.1749, -0.1749, -0.6851, -0.6851]}  # 对应 RPY(0.5, 0.0, 1.5708)，顺序为 [x,y,z,w]
            ],
            output='screen'
        )
    ])
