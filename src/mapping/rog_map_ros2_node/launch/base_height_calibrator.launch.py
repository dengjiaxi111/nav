from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "pointcloud_type",
            default_value="livox_custom",
            description="'livox_custom' for livox_ros_driver2/CustomMsg, or 'pointcloud2'",
        ),
        DeclareLaunchArgument(
            "livox_topic",
            default_value="/livox/lidar_192_168_1_199",
            description="Raw Livox CustomMsg topic",
        ),
        DeclareLaunchArgument(
            "pointcloud2_topic",
            default_value="/livox/lidar",
            description="Raw PointCloud2 topic when pointcloud_type:=pointcloud2",
        ),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("current_roll", default_value="0.5236"),
        DeclareLaunchArgument("current_pitch", default_value="0.0"),
        DeclareLaunchArgument("current_yaw", default_value="1.5708"),
        DeclareLaunchArgument("lidar_x", default_value="0.2"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.05"),
        DeclareLaunchArgument("collect_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("candidate_base_z_min", default_value="-1.5"),
        DeclareLaunchArgument("candidate_base_z_max", default_value="0.4"),
        DeclareLaunchArgument("min_candidates", default_value="3000"),
        Node(
            package="rog_map_ros2_node",
            executable="base_height_calibrator",
            name="base_height_calibrator",
            output="screen",
            parameters=[{
                "pointcloud_type": LaunchConfiguration("pointcloud_type"),
                "livox_topic": LaunchConfiguration("livox_topic"),
                "pointcloud2_topic": LaunchConfiguration("pointcloud2_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "current_roll": ParameterValue(LaunchConfiguration("current_roll"), value_type=float),
                "current_pitch": ParameterValue(LaunchConfiguration("current_pitch"), value_type=float),
                "current_yaw": ParameterValue(LaunchConfiguration("current_yaw"), value_type=float),
                "lidar_x": ParameterValue(LaunchConfiguration("lidar_x"), value_type=float),
                "lidar_y": ParameterValue(LaunchConfiguration("lidar_y"), value_type=float),
                "lidar_z": ParameterValue(LaunchConfiguration("lidar_z"), value_type=float),
                "collect_duration_sec": ParameterValue(
                    LaunchConfiguration("collect_duration_sec"), value_type=float),
                "candidate_base_z_min": ParameterValue(
                    LaunchConfiguration("candidate_base_z_min"), value_type=float),
                "candidate_base_z_max": ParameterValue(
                    LaunchConfiguration("candidate_base_z_max"), value_type=float),
                "min_candidates": ParameterValue(LaunchConfiguration("min_candidates"), value_type=int),
            }],
        ),
    ])
