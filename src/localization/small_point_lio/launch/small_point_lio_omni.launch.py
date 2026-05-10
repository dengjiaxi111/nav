from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时间"
    )
    declare_cloud_registered_topic = DeclareLaunchArgument(
        "cloud_registered_topic",
        default_value="/cloud_registered",
        description="配准点云输出话题"
    )
    declare_source_odom_topic = DeclareLaunchArgument(
        "source_odom_topic",
        default_value="/Odometry",
        description="LIO 里程计输出话题"
    )

    # 全向轮车固定外参：base_link -> livox_frame。
    # 修改外参时，只改这里的 xyz/rpy；xyz 会同步传给 LIO 点云过滤。
    lidar_x = "0.2"
    lidar_y = "0.0"
    lidar_z = "0.05"
    lidar_roll = "0.52"
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
                    "mid360_omni.yaml",
                ]
            ),
            {
                "base_link_to_lidar_xyz": [
                    float(lidar_x),
                    float(lidar_y),
                    float(lidar_z),
                ],
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("/cloud_registered", LaunchConfiguration("cloud_registered_topic")),
            ("/Odometry", LaunchConfiguration("source_odom_topic")),
        ],
    )

    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "--x",
            lidar_x,
            "--y",
            lidar_y,
            "--z",
            lidar_z,
            "--roll",
            lidar_roll,
            "--pitch",
            lidar_pitch,
            "--yaw",
            lidar_yaw,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cloud_registered_topic,
        declare_source_odom_topic,
        small_point_lio_node,
        static_base_link_to_livox_frame
    ])
