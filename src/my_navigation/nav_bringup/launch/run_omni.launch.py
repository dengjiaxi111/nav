#!/usr/bin/env python3
"""
Omni chassis navigation bringup.

This launch keeps the wheel-leg bringup untouched and starts an omni-specific
profile. The omni velocity adapter is started by default:

  nav_server/cmd_vel -> /cmd_vel_fake -> omni_cmd_adapter -> /cmd_vel
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool_default_from_nav_params(nav_params_file, key, default_value=False):
    try:
        with open(nav_params_file, "r") as f:
            params_dict = yaml.safe_load(f)
        ros_params = params_dict.get("nav_server", {}).get("ros__parameters", {})
        return "true" if bool(ros_params.get(key, default_value)) else "false"
    except Exception as exc:
        print(f"[Warning] Failed to parse {nav_params_file} for {key}: {exc}")
        return "true" if default_value else "false"


def _pcd_map_from_localization_config(config_file):
    try:
        with open(config_file, "r") as f:
            params_dict = yaml.safe_load(f)
        ros_params = params_dict.get("/**", {}).get("ros__parameters", {})
        return ros_params.get("map_file", "")
    except Exception as exc:
        print(f"[Warning] Failed to parse {config_file} for map_file: {exc}")
        return ""


def generate_launch_description():
    nav_bringup_dir = get_package_share_directory("nav_bringup")
    rog_map_dir = get_package_share_directory("rog_map_ros2_node")
    loc_init_dir = get_package_share_directory("localization_initializer")

    default_nav_params = os.path.join(nav_bringup_dir, "config", "nav_params_omni.yaml")
    default_loc_config = os.path.join(loc_init_dir, "config", "initializer_omni.yaml")
    default_rog_map_config = os.path.join(rog_map_dir, "config", "rog_map_omni.yaml")
    default_projector_params = os.path.join(rog_map_dir, "config", "projector_omni.yaml")
    default_rviz_config = os.path.join(nav_bringup_dir, "rviz", "navigation_full.rviz")

    default_use_static_map_odom = _bool_default_from_nav_params(
        default_nav_params, "use_static_map_odom", False
    )
    default_debug_reset_odom_to_base = _bool_default_from_nav_params(
        default_nav_params, "debug_reset_odom_to_base_link", False
    )
    default_pcd_map_file = _pcd_map_from_localization_config(default_loc_config)

    declare_nav_params = DeclareLaunchArgument(
        "nav_params_file",
        default_value=default_nav_params,
        description="Omni nav_server parameter file",
    )
    declare_localization_config = DeclareLaunchArgument(
        "localization_config_file",
        default_value=default_loc_config,
        description="Localization initializer parameter file",
    )
    declare_rog_map_config = DeclareLaunchArgument(
        "rog_map_config_file",
        default_value=default_rog_map_config,
        description="ROG-Map config file",
    )
    declare_projector_params = DeclareLaunchArgument(
        "projector_params_file",
        default_value=default_projector_params,
        description="ROG-Map 2D projector parameter file",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz2 config file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )
    declare_use_static_map_odom = DeclareLaunchArgument(
        "use_static_map_odom",
        default_value=default_use_static_map_odom,
        description="true: use static map->odom helper; false: use localization_initializer",
    )
    declare_debug_reset_odom_to_base = DeclareLaunchArgument(
        "debug_reset_odom_to_base_link",
        default_value=default_debug_reset_odom_to_base,
        description="Debug mode: force odom==base_link after initialpose",
    )
    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel_fake",
        description="nav_server output before omni velocity adaptation",
    )
    declare_enable_omni_cmd_adapter = DeclareLaunchArgument(
        "enable_omni_cmd_adapter",
        default_value="true",
        description="Start the omni command adapter for /cmd_vel_fake -> /cmd_vel",
    )
    declare_output_cmd_vel_topic = DeclareLaunchArgument(
        "output_cmd_vel_topic",
        default_value="/cmd_vel",
        description="Omni adapter output command topic consumed by myserial",
    )
    declare_source_odom_topic = DeclareLaunchArgument(
        "source_odom_topic",
        default_value="/Odometry",
        description="Base odometry topic used to generate /Odometry/fake",
    )
    declare_cloud_registered_topic = DeclareLaunchArgument(
        "cloud_registered_topic",
        default_value="/cloud_registered",
        description="Registered cloud topic from LIO to localization and ROG-Map",
    )
    declare_dynamic_layer_topic = DeclareLaunchArgument(
        "dynamic_layer_topic",
        default_value="/rog_map/map_2d",
        description="ROG-Map 2D dynamic layer topic consumed by nav_server",
    )
    declare_rog_map_frame = DeclareLaunchArgument(
        "rog_map_frame",
        default_value="odom",
        description="Frame id for ROG-Map 2D projection output",
    )
    declare_enable_stage3_check = DeclareLaunchArgument(
        "enable_stage3_check",
        default_value="false",
        description="Run a short TF/topic check for localization and ROG-Map wiring",
    )
    declare_fake_odom_topic = DeclareLaunchArgument(
        "fake_odom_topic",
        default_value="/Odometry/fake",
        description="Odometry topic expressed in base_link_fake for NMPC",
    )
    declare_initial_fake_yaw = DeclareLaunchArgument(
        "initial_fake_yaw",
        default_value="0.0",
        description="Initial yaw from base_link to base_link_fake",
    )
    declare_fake_angular_speed_coefficient = DeclareLaunchArgument(
        "fake_angular_speed_coefficient",
        default_value="1.0",
        description="Scale applied when integrating cmd_vel_fake.angular.z into base_link_fake yaw",
    )
    small_point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("small_point_lio"),
                "launch",
                "small_point_lio_omni.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "cloud_registered_topic": LaunchConfiguration("cloud_registered_topic"),
            "source_odom_topic": LaunchConfiguration("source_odom_topic"),
        }.items(),
    )

    localization_init_node = Node(
        package="localization_initializer",
        executable="localization_initializer_node",
        name="localization_initializer",
        condition=UnlessCondition(LaunchConfiguration("use_static_map_odom")),
        output="screen",
        parameters=[
            LaunchConfiguration("localization_config_file"),
            {"auto_initialize": "false"},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/cloud_registered", LaunchConfiguration("cloud_registered_topic")),
            ("/initialpose", "/initialpose"),
            ("/localization/aligned_scan", "/localization/aligned_scan"),
            ("/localization/map_cloud", "/localization/map_cloud"),
            ("/localization/status", "/localization/status"),
            ("/localization/fitness_marker", "/localization/fitness_marker"),
        ],
    )

    static_tf_map_odom = Node(
        package="nav_bringup",
        executable="static_relocator.py",
        name="static_relocator",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_static_map_odom"), "' == 'true' and '",
            LaunchConfiguration("debug_reset_odom_to_base_link"), "' != 'true'",
        ])),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    odom_base_debug_rebaser = Node(
        package="nav_bringup",
        executable="odom_base_debug_rebaser.py",
        name="odom_base_debug_rebaser",
        condition=IfCondition(LaunchConfiguration("debug_reset_odom_to_base_link")),
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_link_fake",
            "odom_topic": "/Odometry/fake",
            "initialpose_topic": "/initialpose",
            "localization_status_topic": "/localization/status",
            "tf_rate_hz": 50.0,
        }],
        output="screen",
    )

    pcd_map_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="pcd_publisher",
        condition=IfCondition(LaunchConfiguration("use_static_map_odom")),
        parameters=[{
            "file_name": default_pcd_map_file,
            "tf_frame": "map",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=[
            ("cloud_pcd", "/localization/map_cloud"),
        ],
        output="screen",
    )

    integration_node = Node(
        package="rog_map_ros2_node",
        executable="integration_node",
        name="rog_map_integration",
        parameters=[
            {"config_file": LaunchConfiguration("rog_map_config_file")},
            LaunchConfiguration("projector_params_file"),
            {
                "projector.frame_id": LaunchConfiguration("rog_map_frame"),
                "projector.topic_name": LaunchConfiguration("dynamic_layer_topic"),
            },
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
        remappings=[
            ("/cloud_registered", LaunchConfiguration("cloud_registered_topic")),
            ("/Odometry", LaunchConfiguration("source_odom_topic")),
        ],
    )

    omni_cmd_adapter_node = Node(
        package="nav_bringup",
        executable="omni_cmd_adapter.py",
        name="omni_cmd_adapter",
        condition=IfCondition(LaunchConfiguration("enable_omni_cmd_adapter")),
        parameters=[
            LaunchConfiguration("nav_params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "output_cmd_vel_topic": LaunchConfiguration("output_cmd_vel_topic"),
                "source_odom_topic": LaunchConfiguration("source_odom_topic"),
                "fake_odom_topic": LaunchConfiguration("fake_odom_topic"),
                "initial_fake_yaw": LaunchConfiguration("initial_fake_yaw"),
                "fake_angular_speed_coefficient": LaunchConfiguration(
                    "fake_angular_speed_coefficient"
                ),
            },
        ],
        output="screen",
    )

    stage3_check_node = Node(
        package="nav_bringup",
        executable="omni_stage3_check.py",
        name="omni_stage3_check",
        condition=IfCondition(LaunchConfiguration("enable_stage3_check")),
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map_frame": "map",
            "odom_frame": LaunchConfiguration("rog_map_frame"),
            "base_frame": "base_link",
            "fake_frame": "base_link_fake",
            "lidar_frame": "livox_frame",
            "cloud_registered_topic": LaunchConfiguration("cloud_registered_topic"),
            "odom_topic": LaunchConfiguration("source_odom_topic"),
            "fake_odom_topic": LaunchConfiguration("fake_odom_topic"),
            "dynamic_layer_topic": LaunchConfiguration("dynamic_layer_topic"),
            "check_duration_sec": 5.0,
        }],
        output="screen",
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav_bringup"),
                "launch",
                "navigation_omni.launch.py",
            ])
        ]),
        launch_arguments={
            "params_file": LaunchConfiguration("nav_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "dynamic_layer_topic": LaunchConfiguration("dynamic_layer_topic"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_STACK_SIZE", "16777216"),
        declare_nav_params,
        declare_localization_config,
        declare_rog_map_config,
        declare_projector_params,
        declare_rviz_config,
        declare_use_sim_time,
        declare_use_static_map_odom,
        declare_debug_reset_odom_to_base,
        declare_cmd_vel_topic,
        declare_enable_omni_cmd_adapter,
        declare_output_cmd_vel_topic,
        declare_source_odom_topic,
        declare_cloud_registered_topic,
        declare_dynamic_layer_topic,
        declare_rog_map_frame,
        declare_enable_stage3_check,
        declare_fake_odom_topic,
        declare_initial_fake_yaw,
        declare_fake_angular_speed_coefficient,
        small_point_lio_launch,
        localization_init_node,
        static_tf_map_odom,
        odom_base_debug_rebaser,
        pcd_map_publisher_node,
        integration_node,
        omni_cmd_adapter_node,
        stage3_check_node,
        navigation_launch,
        rviz_node,
    ])
