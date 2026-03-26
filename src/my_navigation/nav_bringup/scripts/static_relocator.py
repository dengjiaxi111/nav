#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
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
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"
        
        # 如果你已测量好初始位姿（用于比赛抢时间），可以直接在这里写死。
        # 启动时会先发布所选侧的 map->odom 变换；之后你在 RViz 里继续用
        # "2D Pose Estimate" 修正，会以 /initialpose 回调为准覆盖此初始值。
        self.publish_initial_pose_on_startup = True

        # 通过参数选择初始位姿属于红蓝方。
        # 用法示例（launch 里传参）：initial_side:=red 或 initial_side:=blue
        self.initial_side = self.declare_parameter('initial_side', 'blue').value

        # 写成 map 坐标系下的 (x, y, yaw)；yaw 为 rad（绕 z 轴偏航角）。
        # 注意：你需要把测量值替换进这两套初始位姿。
        self.initial_pose_red = {
            'x': 0.40,
            'y': 7.7,
            'yaw': 0.0,
            'z': 0.002,
        }
        self.initial_pose_blue = {
            'x': 11.6,
            'y': 0.3,
            'yaw': 3.14159,
            'z': 0.002,
        }

        if self.publish_initial_pose_on_startup:
            self.publish_measured_initial_pose()
        
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        self.get_logger().info("静态重定位节点启动！(use_static_map_odom=true 模式)")
        self.get_logger().info("请在 RViz 中通过 '2D Pose Estimate' 选择初始位姿...")

    def publish_initial_identity_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info("已发布初始 map->odom 恒等变换，等待用户在 RViz 指定新位姿。")

    def _get_selected_initial_pose(self):
        side = str(self.initial_side).strip().lower()
        if side == 'blue':
            return self.initial_pose_blue, 'blue'
        # 默认走 red，避免参数没配导致完全不发 TF
        return self.initial_pose_red, 'red'

    def publish_measured_initial_pose(self):
        pose, side = self._get_selected_initial_pose()

        # 仅使用 yaw 构建四元数（roll/pitch=0），匹配 2D Pose Estimate 的常见用法
        q = R.from_euler('xyz', [0.0, 0.0, pose['yaw']]).as_quat()  # [x,y,z,w]

        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = self.map_frame
        trans.child_frame_id = self.odom_frame
        trans.transform.translation.x = float(pose['x'])
        trans.transform.translation.y = float(pose['y'])
        trans.transform.translation.z = float(pose['z'])
        trans.transform.rotation.x = float(q[0])
        trans.transform.rotation.y = float(q[1])
        trans.transform.rotation.z = float(q[2])
        trans.transform.rotation.w = float(q[3])

        self.tf_static_broadcaster.sendTransform(trans)
        self.get_logger().info(
            f"已在启动时发布测量初始 map->odom({side}): x={pose['x']:.3f}, "
            f"y={pose['y']:.3f}, yaw={pose['yaw'] * 180.0 / 3.14159:.2f}度；"
            f"之后可在 RViz 用 2D Pose Estimate 覆盖。"
        )

    def initial_pose_callback(self, msg):
        self.get_logger().info("接收到 /initialpose 输入，直接作为 map->odom 变换发布...")
        
        # 接收到的 Pose 直接作为 map -> odom（与重定位输出一致，不进行 NDT 配准）
        map_to_odom_mat = pose_to_matrix(msg.pose.pose)
        
        t, q = matrix_to_transform(map_to_odom_mat)
        
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = self.map_frame
        trans.child_frame_id = self.odom_frame
        trans.transform.translation.x = t[0]
        trans.transform.translation.y = t[1]
        trans.transform.translation.z = t[2]
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        
        self.tf_static_broadcaster.sendTransform(trans)
        euler = R.from_quat(q).as_euler('xyz')
        self.get_logger().info(f"已更新 map->odom: x={t[0]:.3f}, y={t[1]:.3f}, yaw={euler[2]*180.0/3.14159:.2f}度")

def main(args=None):
    rclpy.init(args=args)
    node = StaticRelocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
