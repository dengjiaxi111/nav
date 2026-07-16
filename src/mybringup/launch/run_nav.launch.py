
import os.path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch.actions import TimerAction,LogInfo
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def load_navigation_mode(config_path):
    try:
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        raise RuntimeError(f'Failed to load navigation mode config: {config_path}: {exc}')

    mode = data.get('navigation_mode', {})
    return {
        'use_initialpose_as_static_tf': bool(mode.get('use_initialpose_as_static_tf', False)),
        'map_frame': mode.get('map_frame', 'map'),
        'odom_frame': mode.get('odom_frame', 'odom'),
        'base_frame': mode.get('base_frame', 'base_link'),
        'initialpose_topic': mode.get('initialpose_topic', '/initialpose'),
    }

def generate_launch_description():
    livox_ros_driver_path   = get_package_share_directory('livox_ros_driver2')
    point_lio_path          = get_package_share_directory('point_lio')
    mytf_path               = get_package_share_directory('mytf')
    segmentation_path       = get_package_share_directory('pointcloud_segmentation')
    bringup_path            = get_package_share_directory('mybringup')
    fake_cmd_vel_path	    = get_package_share_directory('fake_vel_transform')
    loc_init_path           = get_package_share_directory('localization_initializer')
    myserial_path           = get_package_share_directory('myserial')
    small_point_lio_path    = get_package_share_directory('small_point_lio')

    fake_cmd_vel_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(fake_cmd_vel_path, 'launch', 'fake_vel_transform.launch.py'))) 
    
    point_lio_launch        = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(point_lio_path, 'launch', 'point_lio.launch.py')))  
    livox_ros_driver_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(livox_ros_driver_path, 'launch_ROS2', 'msg_MID360_launch.py')))  
    mytf_launch             = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mytf_path, 'launch', 'mytf.launch.py')))
    segmentation_launch     = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(segmentation_path, 'launch', 'segmentation.launch.py')))
    myserial_launch         = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(myserial_path, 'launch','myserial.launch.py')))
    small_point_lio_launch  = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(small_point_lio_path,'launch','small_point_lio.launch.py')))
    default_rviz_config_path = os.path.join(bringup_path, 'rviz', 'nav.rviz')
    navigation_mode_path = os.path.join(bringup_path, 'config', 'navigation_mode.yaml')
    navigation_mode = load_navigation_mode(navigation_mode_path)
    use_initialpose_static_tf = TextSubstitution(
        text='true' if navigation_mode['use_initialpose_as_static_tf'] else 'false')
    localization_init_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(loc_init_path, 'launch', 'localization_init.launch.py')),
        launch_arguments={
            'use_rviz': 'false',
            'auto_initialize': 'false',
            'use_initialpose_as_static_tf': 'false',
        }.items(),
        condition=UnlessCondition(use_initialpose_static_tf)
    )
    
    return LaunchDescription([

        # 一些参数
        DeclareLaunchArgument(
            'map', default_value='/home/super259/navigationros2/src/mybringup/map/map_5.yaml',
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # 打开串口
        #myserial_launch,
        # 机器人自身状态发布
        #mytf_launch,
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

        localization_init_launch,

        Node(
            package='mybringup',
            executable='initialpose_static_tf_node.py',
            name='initialpose_static_tf',
            output='screen',
            parameters=[{
                'map_frame': navigation_mode['map_frame'],
                'odom_frame': navigation_mode['odom_frame'],
                'base_frame': navigation_mode['base_frame'],
                'initialpose_topic': navigation_mode['initialpose_topic'],
            }],
            condition=IfCondition(use_initialpose_static_tf),
        ),

        # # 雷达驱动
        livox_ros_driver_launch,

        # # SLAM
        # #lio_3se_launch,
        TimerAction(
            period=1.5,
            actions=[
        small_point_lio_launch]),

        # # 点云障碍提取
        segmentation_launch,

        # 导航相关
        fake_cmd_vel_launch,

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path],
        ),    

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='sentry_decision',
                    executable='sentry_decision_node',
                    name='sentry_decision',
                    output='screen',
                )
            ]),

        TimerAction(
            period=3.0,
            actions=[
            LifecycleNode(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace='',
                output='screen',
                parameters=[{'yaml_filename': LaunchConfiguration('map'),'use_sim_time': LaunchConfiguration('use_sim_time')}]),

            LifecycleNode(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace='',
                output='screen',
                parameters=[os.path.join(bringup_path, 'config', 'costmap2d.yaml'),os.path.join(bringup_path, 'config', 'planner.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('local_costmap/scan', '/scan')]),

            LifecycleNode(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace='',
                output='screen',
                parameters=[os.path.join(bringup_path, 'config', 'costmap2d.yaml'),os.path.join(bringup_path, 'config', 'controller.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
            
            LifecycleNode(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace='',
                output='screen',
                parameters=[os.path.join(bringup_path, 'config', 'nav_bt.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]),  
            LifecycleNode(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace='',
                output='screen',
                parameters=[os.path.join(bringup_path, 'config', 'nav_bt.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}]),                 

            # nav2自带的生命周期管理工具，也可以用来配置非nav2的生命周期节点
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[{'autostart': True},
                                {'node_names': ['map_server','controller_server','planner_server','bt_navigator','behavior_server']},{'use_sim_time': LaunchConfiguration('use_sim_time')}])
                ])
            ])
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

    ])
