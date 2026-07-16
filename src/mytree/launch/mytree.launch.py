import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    yaml_path = os.path.join(get_package_share_directory('mytree'), 'config', 'footprints.yaml')
    tick_rate = 20  # 20Hz
    return LaunchDescription([
        # 获取 YAML 配置文件路径

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='mytree',
                    executable='mytree',
                    name='mytree',
                    output='screen',
                    parameters=[
                        ParameterFile(yaml_path, allow_substs=True),
                        {'tick_rate': tick_rate}]
                )
            ])

    ])