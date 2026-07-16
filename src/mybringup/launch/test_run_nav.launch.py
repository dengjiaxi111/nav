
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import TimerAction,LogInfo

from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    livox_ros_driver_path   = get_package_share_directory('livox_ros_driver2')
    lio_3se_path            = get_package_share_directory('lio_3se')
    point_lio_path          = get_package_share_directory('point_lio_cxr')
    mytf_path               = get_package_share_directory('mytf')
    segmentation_path       = get_package_share_directory('pointcloud_segmentation')
    bringup_path            = get_package_share_directory('mybringup')
    fake_cmd_vel_path	    = get_package_share_directory('fake_vel_transform')
    localization_path       = get_package_share_directory('localization')
    mytree_path             = get_package_share_directory("mytree")
    myserial_path           = get_package_share_directory('myserial')

    fake_cmd_vel_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(fake_cmd_vel_path, 'launch', 'fake_vel_transform.launch.py'))) 
    
    point_lio_launch        = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(point_lio_path, 'launch', 'run_lio.launch.py')))  
    livox_ros_driver_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(livox_ros_driver_path, 'launch_ROS2', 'msg_MID360_launch.py')))  
    lio_3se_launch          = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(lio_3se_path, 'launch', 'relocalization.launch.py')))
    mytf_launch             = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mytf_path, 'launch', 'mytf.launch.py')))
    segmentation_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(segmentation_path, 'launch', 'segmentation.launch.py')))
    localization_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localization_path, 'launch', 'localization.launch.py')))
    mytree_launch           = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mytree_path, 'launch', 'mytree.launch.py')))
    myserial_launch         = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(myserial_path, 'launch','myserial.launch.py')))
    default_rviz_config_path = os.path.join(bringup_path, 'rviz', 'nav.rviz')
    
    return LaunchDescription([

        # 一些参数
        DeclareLaunchArgument(
            'map', default_value=bringup_path + '/map/RMUC.yaml',
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # 打开串口
        #myserial_launch,
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

        localization_launch,

        # # 雷达驱动
        livox_ros_driver_launch,

        # # SLAM
        # #lio_3se_launch,
        TimerAction(
            period=1.5,
            actions=[
        point_lio_launch]),

        # # 点云障碍提取
        segmentation_launch,

        # 导航相关
        # fake_cmd_vel_launch,

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path],
        ),    

        # TimerAction(
        #     period=3.0,
        #     actions=[
        #     LifecycleNode(
        #         package='nav2_map_server',
        #         executable='map_server',
        #         name='map_server',
        #         namespace='',
        #         output='screen',
        #         parameters=[{'yaml_filename': LaunchConfiguration('map'),'use_sim_time': LaunchConfiguration('use_sim_time')}]),

        #     LifecycleNode(
        #         package='nav2_planner',
        #         executable='planner_server',
        #         name='planner_server',
        #         namespace='',
        #         output='screen',
        #         parameters=[os.path.join(bringup_path, 'config', 'costmap2d.yaml'),os.path.join(bringup_path, 'config', 'planner.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #         remappings=[('local_costmap/scan', '/scan')]),

        #     LifecycleNode(
        #         package='nav2_controller',
        #         executable='controller_server',
        #         name='controller_server',
        #         namespace='',
        #         output='screen',
        #         parameters=[os.path.join(bringup_path, 'config', 'costmap2d.yaml'),os.path.join(bringup_path, 'config', 'controller.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
            
        #     LifecycleNode(
        #         package='nav2_bt_navigator',
        #         executable='bt_navigator',
        #         name='bt_navigator',
        #         namespace='',
        #         output='screen',
        #         parameters=[os.path.join(bringup_path, 'config', 'nav_bt.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]),  
        #     LifecycleNode(
        #         package='nav2_behaviors',
        #         executable='behavior_server',
        #         name='behavior_server',
        #         namespace='',
        #         output='screen',
        #         parameters=[os.path.join(bringup_path, 'config', 'nav_bt.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]),                 

        #     # nav2自带的生命周期管理工具，也可以用来配置非nav2的生命周期节点
        #     TimerAction(
        #         period=1.0,
        #         actions=[
        #             Node(
        #             package='nav2_lifecycle_manager',
        #             executable='lifecycle_manager',
        #             name='lifecycle_manager',
        #             output='screen',
        #             parameters=[{'autostart': True},
        #                         {'node_names': ['map_server','controller_server','planner_server','bt_navigator','behavior_server']},{'use_sim_time': LaunchConfiguration('use_sim_time')}])
        #         ])
        #     ])
        # 临时措施（延时启动）
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

        # TimerAction(
        #     period=5.0,
        #     actions=[mytree_launch])
    ])
