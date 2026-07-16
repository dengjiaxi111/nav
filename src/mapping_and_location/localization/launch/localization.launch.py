import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数，这些参数会从命令行传递给节点
        DeclareLaunchArgument('pcl_path', default_value='/home/super259/navigationros2/src/mapping_and_location/localization/pcd/'),
        DeclareLaunchArgument('file_name', default_value='RMUC.pcd'),
        DeclareLaunchArgument('lidar_pos', default_value='[0.045, 0.123, 0.31]'),
        DeclareLaunchArgument('lidar_rot', default_value='[-0.383, 0.0, 0.0, 0.924]'), 
        DeclareLaunchArgument('init_pos', default_value='[20.6, -1.18, 0.3]'),  #[0.0, 0.0, 0.3]  #[20.6, -1.18, 0.3]
        DeclareLaunchArgument('init_rot', default_value='[0.0, 0.0, 1.0, 0.0]'), # 旋转180度： 0 0 1 0 
        DeclareLaunchArgument('converge_once', default_value='false'), 
        DeclareLaunchArgument('use_relocalization', default_value='true'), 
        DeclareLaunchArgument('test_mode', default_value='false'), 
        DeclareLaunchArgument('con_frame', default_value='50'), 

        # 启动 relocalization 节点
        Node(
            package='localization', 
            executable='relocalization',
            name='relocalization_node',
            output='screen',  # 输出到屏幕
            parameters=[{
                'pcl_path': LaunchConfiguration('pcl_path'),
                'file_name':LaunchConfiguration('file_name'),
                'lidar_pos': LaunchConfiguration('lidar_pos'),
                'lidar_rot': LaunchConfiguration('lidar_rot'),
                'init_pos': LaunchConfiguration('init_pos'),
                'init_rot': LaunchConfiguration('init_rot'),
                'converge_once': LaunchConfiguration('converge_once'),
                'use_relocalization': LaunchConfiguration('use_relocalization'),
                'test_mode': LaunchConfiguration('test_mode'),
                'con_frame': LaunchConfiguration('con_frame'),
                'use_sim_time':False
            }]
        )

    ])
