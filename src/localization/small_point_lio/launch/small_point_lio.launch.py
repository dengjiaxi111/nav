from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "config",                                                                                                                                                                                                                                                                              
                    "mid360.yaml",
                ]
            ),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            "--x",
            "0.2",
            "--y",
            "0.0",
            "--z",
            "0.05",
            "--roll",
            "0.785",# "-0.5236", 
            "--pitch",
            "0.0",
            "--yaw",
            "1.5708",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
        # arguments=[
        #     "--x",
        #     "0.0",
        #     "--y",
        #     "0.0",
        #     "--z",
        #     "0.0",
        #     "--roll",
        #     "3.14159265", 
        #     "--pitch",
        #     "0.0",
        #     "--yaw",
        #     "0.0",
        #     "--frame-id",
        #     "base_link",
        #     "--child-frame-id",
        #     "livox_frame",
        # ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        small_point_lio_node,
        static_base_link_to_livox_frame
    ])
