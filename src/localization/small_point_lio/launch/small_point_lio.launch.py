from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    declare_lidar_mount_mode = DeclareLaunchArgument(
        "lidar_mount_mode",
        default_value="fixed",
        description="fixed: base_link->livox_frame; gimbal_yaw: base_link->gimbal_yaw_link->livox_frame"
    )

    # fixed 模式：腿车固定外参 base_link -> livox_frame。
    # 修改外参时，只改这里的 xyz/rpy；xyz 会同步传给 LIO 点云过滤。
    lidar_x = "0.2"
    lidar_y = "0.0"
    lidar_z = "0.05"
    lidar_roll = "0.52"  # 30度朝下
    lidar_pitch = "0.0"
    lidar_yaw = "1.5708"

    # gimbal_yaw 模式：沿用 navigationros2 中已验证的
    # gimbal_link -> livox_left 安装外参，只把 frame 名统一为 livox_frame。
    # 原始云台角约以 -90 度为机械零位，该零位补偿必须放在动态云台关节上，
    # 不能塞进这里的静态 yaw，否则雷达平移不会被零位补偿正确旋转。
    gimbal_lidar_x = "0.045"
    gimbal_lidar_y = "0.123"
    gimbal_lidar_z = "0.0"
    gimbal_lidar_roll = "-0.785"
    gimbal_lidar_pitch = "0.0"
    gimbal_lidar_yaw = "0.0"

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "config",                                                                                                                                                                                                                                                                              
                    "mid360.yaml",
                ]
            ),
            {
                "base_link_to_lidar_xyz": [
                    float(lidar_x),
                    float(lidar_y),
                    float(lidar_z),
                ],
                "lidar_mount_mode": LaunchConfiguration("lidar_mount_mode"),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("lidar_mount_mode"), "' == 'fixed'"
        ])),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            "--x",
            lidar_x,
            "--y",
            lidar_y,
            "--z",
            lidar_z,
            "--roll",
            lidar_roll,  # "-0.5236",
            "--pitch",
            lidar_pitch,
            "--yaw",
            lidar_yaw,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
        # arguments=[
        #     "--x",
        #     "0.06",
        #     "--y",
        #     "0.0",
        #     "--z",
        #     "0.2",
        #     "--roll",
        #     "-0.7854",# "-0.5236",川大为3.14159 
        #     "--pitch",
        #     "0.0",
        #     "--yaw",
        #     "0.0",
        #     "--frame-id",
        #     "base_link",
        #     "--child-frame-id",
        #     "livox_frame",
        # ],
    )

    static_gimbal_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("lidar_mount_mode"), "' == 'gimbal_yaw'"
        ])),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            "--x",
            gimbal_lidar_x,
            "--y",
            gimbal_lidar_y,
            "--z",
            gimbal_lidar_z,
            "--roll",
            gimbal_lidar_roll,
            "--pitch",
            gimbal_lidar_pitch,
            "--yaw",
            gimbal_lidar_yaw,
            "--frame-id",
            "gimbal_yaw_link",
            "--child-frame-id",
            "livox_frame",
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_lidar_mount_mode,
        small_point_lio_node,
        static_base_link_to_livox_frame,
        static_gimbal_to_livox_frame,
    ])
