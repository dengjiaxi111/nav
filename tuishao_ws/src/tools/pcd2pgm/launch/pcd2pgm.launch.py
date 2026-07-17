from launch import LaunchDescription
from launch.actions import RegisterEventHandler,LogInfo,TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit,OnProcessIO
from launch.events.process import ProcessIO
def generate_launch_description():
    remove_ground = Node(
            package='pcd2pgm',
            executable='removing_ground',
            name='removing_ground',
            output='screen',
            parameters=[{
                #存放pcd文件的路径
                'pcd_path': '/home/lehan/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/RMUC_2025_final.pcd',
                #pcd文件输出路径
                'output_path': '/home/lehan/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/GlobalMap_processed.pcd'
            }])

    pcd2pgm = Node(
            package='pcd2pgm',
            executable='pcd2pgm',
            name='pcd2pgm',
            output='screen',
            parameters=[{
                #存放pcd文件的路径
                'file_directory': '/home/lehan/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/',
                #pcd文件名称
                'file_name': 'test',
                #选取的范围　最小的高度
                'thre_z_min': -0.1,
                #选取的范围　最大的高度
                'thre_z_max': 2.0,
                #0 选取高度范围内的，１选取高度范围外的
                'flag_pass_through': 0,
                #半径滤波的半径
                'thre_radius': 0.1,
                #半径滤波的要求点数个数
                'thres_point_count': 10,
                #存储的栅格map的分辨率
                'map_resolution': 0.05,
                #转换后发布的二维地图的topic，默认使用map即可，可使用map_server保存
                'map_topic_name': 'map'
            }])
    
    pcd2pgm_condition = RegisterEventHandler(
            OnProcessExit(
                target_action=remove_ground,
                on_exit=[LogInfo(msg='remove_ground finished, starting pcd2pgm...'),
                               TimerAction(period=2.0,actions=[pcd2pgm])]
            )
    )
    map_saver_condition = RegisterEventHandler(
            OnProcessIO(
                target_action = pcd2pgm,
                on_stdout = handle_pcd2pgm_output
            )
    )

    #launch_nodes = [remove_ground,map_saver_condition, pcd2pgm_condition ]
    launch_nodes = [pcd2pgm, map_saver_condition ]
    return LaunchDescription(launch_nodes)
    
def handle_pcd2pgm_output(event: ProcessIO):
    map_saver = Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name = 'map_saver_cli',
            output = 'screen',
            parameters=[{}],
            arguments = ['-f', '/home/lehan/navigationros2/ros2-humble/src/map']
    )
    output = event.text.decode().strip()
    print(output)
    if 'publishing map' in output:
        return [map_saver]
    return []
