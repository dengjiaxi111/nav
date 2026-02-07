from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myserial',
            executable='myserial',
            output='screen',
            parameters = [{"on_court": False,"debug_flag": False,"log_path":"/home/nuc/logs"\
                           ,"info_pub": True}]
        )
    ])
