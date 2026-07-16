import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'sor1_k',
            default_value='4.0',
        ),
        DeclareLaunchArgument(
            'sor1_s',
            default_value='1.1',           
        ),
        DeclareLaunchArgument(
            'sor2_k',
            default_value='10.0',            
        ),
        DeclareLaunchArgument(
            'sor2_s',
            default_value='2.0',
        ),

        Node(
            package='pointcloud_segmentation',
            executable='segmentation',
            name='Segmentation',
            namespace='',
            output='screen',
            parameters=[{
                'sor1_k': LaunchConfiguration('sor1_k'),
                'sor1_s': LaunchConfiguration('sor1_s'),
                'sor2_k': LaunchConfiguration('sor2_k'),
                'sor2_s': LaunchConfiguration('sor2_s'),
            }],
            emulate_tty=True,
        )
    ])
