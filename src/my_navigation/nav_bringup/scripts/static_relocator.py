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
        
        # 初始不再发 0 0 0，等待 rviz 指定
        # self.publish_initial_identity_transform()
        
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
