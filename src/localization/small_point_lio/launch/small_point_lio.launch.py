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

    # 腿车固定外参：base_link -> livox_frame。
    # 修改外参时，只改这里的 xyz/rpy；xyz 会同步传给 LIO 点云过滤。
    lidar_x = "0.2"
    lidar_y = "0.0"
    lidar_z = "0.05"
    lidar_roll = "0.5236"  # 30度朝下
    lidar_pitch = "0.0"
    lidar_yaw = "1.5708"

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
            {
                "base_link_to_lidar_xyz": [
                    float(lidar_x),
                    float(lidar_y),
                    float(lidar_z),
                ],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            "--x",
            lidar_x,
            "--y",
            lidar_y,
            "--z",
            lidar_z,
            "--roll",
            lidar_roll,  # "-0.5236",
            "--pitch",
            lidar_pitch,
            "--yaw",
            lidar_yaw,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
        # arguments=[
        #     "--x",
        #     "0.06",
        #     "--y",
        #     "0.0",
        #     "--z",
        #     "0.2",
        #     "--roll",
        #     "-0.7854",# "-0.5236",川大为3.14159 
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
