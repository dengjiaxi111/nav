#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    t = [pose.position.x, pose.position.y, pose.position.z]
    mat = np.eye(4)
    mat[:3, :3] = R.from_quat(q).as_matrix()
    mat[:3, 3] = t
    return mat

def transform_to_matrix(transform):
    q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    t = [transform.translation.x, transform.translation.y, transform.translation.z]
    mat = np.eye(4)
    mat[:3, :3] = R.from_quat(q).as_matrix()
    mat[:3, 3] = t
    return mat

def matrix_to_transform(mat):
    t = mat[:3, 3]
    q = R.from_matrix(mat[:3, :3]).as_quat()
    return t, q

class StaticRelocator(Node):
    def __init__(self):
        super().__init__('static_relocator')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"
        
        # 如果你已测量好初始位姿（用于比赛抢时间），可以直接在这里写死。
        # 启动时会先发布所选侧的 map->odom 变换；之后你在 RViz 里继续用
        # "2D Pose Estimate" 修正，会以 /initialpose 回调为准覆盖此初始值。
        self.publish_initial_pose_on_startup = bool(
            self.declare_parameter('publish_initial_pose_on_startup', True).value
        )

        # 通过参数选择初始位姿属于红蓝方。
        # 用法示例（launch 里传参）：initial_side:=red 或 initial_side:=blue
        self.initial_side = self.declare_parameter('initial_side', 'blue').value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        # 写成 map 坐标系下的 (x, y, yaw)；yaw 为 rad（绕 z 轴偏航角）。
        # 注意：你需要把测量值替换进这两套初始位姿。
        self.initial_pose_red = {
            'x': 0.40,
            'y': 7.6,
            'yaw': 0.0,
            'z': 0.002,
        }
        self.initial_pose_blue = {
            'x': 11.6,
            'y': 0.35,
            'yaw': 3.14159,
            'z': 0.002,
        }

        self.current_map_to_odom = None

        self.fallback_assume_odom_equals_base = bool(
            self.declare_parameter("fallback_assume_odom_equals_base", True).value
        )

        if self.publish_initial_pose_on_startup:
            self.publish_measured_initial_pose()
        self.publish_timer = self.create_timer(0.05, self.publish_current_transform)
        
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        self.get_logger().info("静态重定位节点启动！(use_static_map_odom=true 模式)")
        self.get_logger().info("请在 RViz 中通过 '2D Pose Estimate' 选择初始位姿（输入是 map->base_link，不是 map->odom）...")
        if self.fallback_assume_odom_equals_base:
            self.get_logger().warn(
                "若 odom->base_link 暂不可用，将回退假设 odom==base_link 来更新 map->odom"
            )

    def publish_current_transform(self):
        if self.current_map_to_odom is None:
            return
        self.current_map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.current_map_to_odom)

    def _get_selected_initial_pose(self):
        side = str(self.initial_side).strip().lower()
        if side == 'blue':
            return self.initial_pose_blue, 'blue'
        # 默认走 red，避免参数没配导致完全不发 TF
        return self.initial_pose_red, 'red'

    def publish_measured_initial_pose(self):
        pose, side = self._get_selected_initial_pose()

        # 这里的预设值语义是 map->base_link（机器人在地图中的初始位姿），
        # 不是 map->odom。需结合当前 odom->base_link 计算 map->odom。
        q = R.from_euler('xyz', [0.0, 0.0, pose['yaw']]).as_quat()  # [x,y,z,w]
        map_to_base_mat = np.eye(4)
        map_to_base_mat[:3, :3] = R.from_quat(q).as_matrix()
        map_to_base_mat[:3, 3] = [float(pose['x']), float(pose['y']), float(pose['z'])]

        try:
            odom_to_base_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            odom_to_base_mat = transform_to_matrix(odom_to_base_tf.transform)
        except Exception as ex:
            if not self.fallback_assume_odom_equals_base:
                self.get_logger().warn(
                    f"启动时无法获取 {self.odom_frame}->{self.base_frame}，本次不发布预设初值: {ex}"
                )
                return
            self.get_logger().warn(
                f"启动时无法获取 {self.odom_frame}->{self.base_frame}，回退为 odom==base_link 假设: {ex}"
            )
            odom_to_base_mat = np.eye(4)

        map_to_odom_mat = map_to_base_mat @ np.linalg.inv(odom_to_base_mat)
        t, q_map_odom = matrix_to_transform(map_to_odom_mat)

        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = self.map_frame
        trans.child_frame_id = self.odom_frame
        trans.transform.translation.x = float(t[0])
        trans.transform.translation.y = float(t[1])
        trans.transform.translation.z = float(t[2])
        trans.transform.rotation.x = float(q_map_odom[0])
        trans.transform.rotation.y = float(q_map_odom[1])
        trans.transform.rotation.z = float(q_map_odom[2])
        trans.transform.rotation.w = float(q_map_odom[3])

        self.current_map_to_odom = trans
        self.publish_current_transform()
        self.get_logger().info(
            f"已在启动时应用测量初始 map->base_link({side}): x={pose['x']:.3f}, "
            f"y={pose['y']:.3f}, yaw={pose['yaw'] * 180.0 / 3.14159:.2f}度；"
            f"并计算发布 map->odom。"
        )
        self.get_logger().info(
            f"当前 map->odom: x={t[0]:.3f}, y={t[1]:.3f}；"
            f"之后可在 RViz 用 2D Pose Estimate 覆盖。"
        )

    def initial_pose_callback(self, msg):
        self.get_logger().info("接收到 /initialpose，正在由 map->base_link 计算 map->odom...")
        
        # RViz /initialpose 表示 map->base_link，而非 map->odom。
        map_to_base_mat = pose_to_matrix(msg.pose.pose)

        # 当前 odom->base_link（来自里程计/LIO）
        try:
            odom_to_base_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            odom_to_base_mat = transform_to_matrix(odom_to_base_tf.transform)
        except Exception as ex:
            if not self.fallback_assume_odom_equals_base:
                self.get_logger().warn(
                    f"无法获取 {self.odom_frame}->{self.base_frame}，本次不更新 map->odom: {ex}"
                )
                return
            self.get_logger().warn(
                f"无法获取 {self.odom_frame}->{self.base_frame}，回退为 odom==base_link 假设更新 map->odom: {ex}"
            )
            odom_to_base_mat = np.eye(4)

        # 公式：T_map_odom = T_map_base * inv(T_odom_base)
        map_to_odom_mat = map_to_base_mat @ np.linalg.inv(odom_to_base_mat)
        
        t, q = matrix_to_transform(map_to_odom_mat)
        
        trans = TransformStamped()
        trans.header.frame_id = self.map_frame
        trans.child_frame_id = self.odom_frame
        trans.transform.translation.x = float(t[0])
        trans.transform.translation.y = float(t[1])
        trans.transform.translation.z = float(t[2])
        trans.transform.rotation.x = float(q[0])
        trans.transform.rotation.y = float(q[1])
        trans.transform.rotation.z = float(q[2])
        trans.transform.rotation.w = float(q[3])

        self.current_map_to_odom = trans
        self.publish_current_transform()
        
        euler = R.from_quat(q).as_euler('xyz')
        self.get_logger().info(
            f"已更新 map->odom: x={t[0]:.3f}, y={t[1]:.3f}, yaw={euler[2]*180.0/3.14159:.2f}度"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StaticRelocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
