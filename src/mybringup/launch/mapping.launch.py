
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import TimerAction

from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    livox_ros_driver_path   = get_package_share_directory('livox_ros_driver2')
    lio_3se_path            = get_package_share_directory('lio_3se')
    mytf_path               = get_package_share_directory('mytf')
    segmentation_path       = get_package_share_directory('pointcloud_segmentation')
    bringup_path            = get_package_share_directory('mybringup')
    fake_cmd_vel_path	     = get_package_share_directory('fake_vel_transform')
    
    fake_cmd_vel_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(fake_cmd_vel_path, 'launch', 'fake_vel_transform.launch.py'))) 
    
    livox_ros_driver_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(livox_ros_driver_path, 'launch_ROS2', 'msg_MID360_launch.py')))  
    lio_3se_launch          = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(lio_3se_path, 'launch', 'relocalization.launch.py')))
    mytf_launch             = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mytf_path, 'launch', 'mytf.launch.py')))
    segmentation_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(segmentation_path, 'launch', 'segmentation.launch.py')))

    return LaunchDescription([

        # 一些参数
        DeclareLaunchArgument(
            'map', default_value=bringup_path + '/map/RMUC.yaml',
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),


        # 机器人自身状态发布
        mytf_launch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(bringup_path+'/urdf/sentry.urdf', 'r').read()}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # 上下位机通讯
        # LifecycleNode(
        #     name='serial_node',
        #     package='myserial',
        #     executable='myserial',
        #     output='screen',
        #     parameters = [{"on_court": False}]
        # ),

        # 雷达驱动+点云预处理
        livox_ros_driver_launch,
        #point_preprocess_launch,

        # SLAM
        lio_3se_launch,


        # 点云障碍提取
        segmentation_launch,

        
        # 导航相关
        fake_cmd_vel_launch,
        
        #临时措施（延时启动）
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         Node(
        #             package='robot_localization',
        #             executable='ekf_node',
        #             name='ekf_filter_node',
        #             output='screen',
        #             parameters=[os.path.join(bringup_path, 'config/tf_ekf.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        #         )
        #     ])
    ])
