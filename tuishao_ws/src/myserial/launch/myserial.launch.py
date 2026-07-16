from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myserial',
            executable='myserial',
            output='screen',
            parameters = [{"on_court": False,"debug_flag": False,"log_path":"/tmp/tuishao_serial_logs"\
                           ,"info_pub": True,"enable_rtt_measure": False,"enable_batch_read": True,"enable_improved_framing": True}]
        )
    ])
