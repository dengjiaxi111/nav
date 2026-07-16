from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_params = PathJoinSubstitution([
        FindPackageShare("pointcloud_obstacle_layer"),
        "config",
        "legged_local_obstacle.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Point cloud obstacle layer parameter file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        Node(
            package="pointcloud_obstacle_layer",
            executable="pointcloud_segmentation_node",
            name="pointcloud_segmentation",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="pointcloud_obstacle_layer",
            executable="local_obstacle_grid_node",
            name="local_obstacle_grid",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
