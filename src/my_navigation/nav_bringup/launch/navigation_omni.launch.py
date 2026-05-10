#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav_bringup")

    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map_file")
    stair_mask_yaml = LaunchConfiguration("stair_mask_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    dynamic_layer_topic = LaunchConfiguration("dynamic_layer_topic")

    default_map = os.path.join(pkg_dir, "maps", "map.yaml")
    default_stair_mask = os.path.join(pkg_dir, "maps", "stair.yaml")
    default_params = os.path.join(pkg_dir, "config", "nav_params_omni.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Omni chassis nav_server parameter file",
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value=default_map,
            description="Static map yaml path",
        ),
        DeclareLaunchArgument(
            "stair_mask_yaml",
            default_value=default_stair_mask,
            description="Stair mask yaml path; disabled by omni params by default",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel_fake",
            description="nav_server output before omni velocity adaptation",
        ),
        DeclareLaunchArgument(
            "dynamic_layer_topic",
            default_value="/rog_map/map_2d",
            description="ROG-Map 2D dynamic layer topic",
        ),
        Node(
            package="nav_components",
            executable="nav_server",
            name="nav_server",
            parameters=[
                params_file,
                {"map_file": map_file},
                {"special_terrain.stair_mask_yaml": stair_mask_yaml},
                {"dynamic_layer_topic": dynamic_layer_topic},
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
            remappings=[
                ("cmd_vel", cmd_vel_topic),
            ],
        ),
    ])
