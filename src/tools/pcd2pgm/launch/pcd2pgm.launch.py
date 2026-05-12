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
                'pcd_path': '/home/li/navigation2026/src/tools/pcd_transform/RMUC_final.pcd',
                #pcd文件输出路径
                'output_path': '/home/li/navigation2026/src/tools/pcd2pgm/save_pcd/GlobalMap_processed.pcd',
                # none: 不旋转; auto: 平面拟合自动旋平; manual_extrinsic: 使用旧的固定外参旋转
                'level_method': 'manual_extrinsic',
                # 自动拟合主地面平面并旋平：保证后续PGM沿真实竖直方向投影
                'auto_level': False,
                # true时旋平失败直接中止，避免继续生成歪图
                'require_auto_level': False,
                # 旋平后把拟合地面平均高度平移到z=0
                'translate_ground_to_zero': False,
                # 旋转用平面找水平，高度原点用点云低分位数，避免选中天花板时把天花板当z=0
                'level_height_origin_mode': 'low_percentile',
                'level_ground_percentile': 0.02,
                # RANSAC地面拟合阈值，地面点较厚/噪声大时适当调大
                'level_distance_threshold': 0.08,
                'level_max_iterations': 500,
                # 连续提取多个水平平面，选择旋平后高度最低的平面作为地面，避免误选天花板
                'level_candidate_planes': 4,
                'level_min_plane_inliers': 1000,
                # 若后续候选比第一个大平面低超过该阈值，认为已找到地面并提前停止
                'level_early_stop_below_first_m': 0.10,
                # manual_extrinsic 模式参数，默认恢复旧实现的三段变换
                'manual_level_roll1': 0.5,
                'manual_level_pitch1': 0.0,
                'manual_level_yaw1': 0.0,
                'manual_level_roll2': 0.0,
                'manual_level_pitch2': 0.0,
                'manual_level_yaw2': -1.5708,
                'manual_level_x': 0.2,
                'manual_level_y': 0.0,
                'manual_level_z': 0.05,
                # 保存旋平后的完整点云，方便RViz检查；PGM仍使用去地面后的output_path
                'leveled_full_output_path': '/home/li/navigation2026/src/tools/pcd2pgm/save_pcd/GlobalMap_level_full.pcd',
            }])

    pcd2pgm = Node(
            package='pcd2pgm',
            executable='pcd2pgm',
            name='pcd2pgm',
            output='screen',
            parameters=[{
                #存放pcd文件的路径
                'file_directory': '/home/li/navigation2026/src/tools/pcd2pgm/save_pcd/',
                #pcd文件名称
                'file_name': 'GlobalMap_processed',
                #选取的范围　最小的高度
                'thre_z_min': 0.05,
                #选取的范围　最大的高度
                'thre_z_max': 1.6,
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

    launch_nodes = [remove_ground,map_saver_condition, pcd2pgm_condition ]
    # launch_nodes = [pcd2pgm, map_saver_condition ]
    return LaunchDescription(launch_nodes)
    
def handle_pcd2pgm_output(event: ProcessIO):
    map_saver = Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name = 'map_saver_cli',
            output = 'screen',
            parameters=[{}],
            arguments = ['-f', '/home/li/navigation2026/map/map']
    )
    output = event.text.decode().strip()
    print(output)
    if 'publishing map' in output:
        return [map_saver]
    return []
