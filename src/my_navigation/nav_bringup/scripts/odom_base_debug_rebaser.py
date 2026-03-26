#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
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


class OdomBaseDebugRebaser(Node):
    """
    Debug purpose:
      - Always enforce TF(odom -> base_link) = Identity
      - After (re)localization trigger, re-anchor map motion from raw odom pose
    """

    def __init__(self):
        super().__init__("odom_base_debug_rebaser")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry").value
        self.initialpose_topic = self.declare_parameter("initialpose_topic", "/initialpose").value
        self.localization_status_topic = self.declare_parameter("localization_status_topic", "/localization/status").value
        self.tf_rate_hz = float(self.declare_parameter("tf_rate_hz", 50.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.latest_odom_pose = None
        self.has_anchor = False
        self.need_reanchor = True
        self.pending_initialpose_mat = None
        self._warned_waiting_odom = False

        self.anchor_map_to_base = np.eye(4)
        self.anchor_odom_to_base = np.eye(4)

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )
        self.create_subscription(
            String,
            self.localization_status_topic,
            self.localization_status_callback,
            10,
        )

        period = max(0.005, 1.0 / max(1e-3, self.tf_rate_hz))
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().warn("[DEBUG] odom==base_link 模式已启动")

    def odom_callback(self, msg: Odometry):
        self.latest_odom_pose = msg.pose.pose
        self._warned_waiting_odom = False

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        # 直接使用 RViz 2D Pose Estimate 作为 map->base 的目标锚点，
        # 避免与其他 map->odom 发布源竞争时通过 TF 回读出现旧值。
        self.pending_initialpose_mat = pose_to_matrix(msg.pose.pose)
        self.need_reanchor = True
        self.get_logger().warn("[DEBUG] 收到 /initialpose，准备重锚定 map 运动")

    def localization_status_callback(self, msg: String):
        text = msg.data
        if "定位成功" in text or "map → odom TF 已发布" in text:
            self.need_reanchor = True
            self.get_logger().warn("[DEBUG] 收到定位成功状态，准备重锚定 map 运动")

    def publish_identity_odom_to_base(self):
        now = self.get_clock().now().to_msg()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

    def try_reanchor(self):
        if self.latest_odom_pose is None and self.pending_initialpose_mat is None:
            if not self._warned_waiting_odom:
                self.get_logger().warn("[DEBUG] 尚未收到里程计，暂不能重锚定（等待 /Odometry）")
                self._warned_waiting_odom = True
            return

        # /initialpose 触发时：优先采用用户输入作为 map->base 锚点。
        if self.pending_initialpose_mat is not None:
            self.anchor_map_to_base = self.pending_initialpose_mat
            self.pending_initialpose_mat = None
        else:
            # 其他触发（例如 localization/status）回退到 TF 回读。
            try:
                map_to_base_tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                )
            except Exception:
                return
            self.anchor_map_to_base = transform_to_matrix(map_to_base_tf.transform)

        if self.latest_odom_pose is not None:
            self.anchor_odom_to_base = pose_to_matrix(self.latest_odom_pose)
        else:
            # /initialpose 先到、/Odometry 暂未到时，先用恒等作为临时锚点，
            # 保障 map->odom 能立即跳转到用户给定初值。
            self.anchor_odom_to_base = np.eye(4)

        self.has_anchor = True
        self.need_reanchor = False
        self.get_logger().warn("[DEBUG] 重锚定成功，开始持续发布 map->odom")

    def publish_rebased_map_to_odom(self):
        if not self.has_anchor:
            return

        t_odom_to_base_now = (
            pose_to_matrix(self.latest_odom_pose)
            if self.latest_odom_pose is not None
            else np.eye(4)
        )
        t_map_to_base_now = self.anchor_map_to_base @ np.linalg.inv(self.anchor_odom_to_base) @ t_odom_to_base_now

        # 因为强制了 odom==base_link，所以 map->odom = map->base_link
        trans, quat = matrix_to_transform(t_map_to_base_now)
        now = self.get_clock().now().to_msg()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = float(trans[0])
        tf_msg.transform.translation.y = float(trans[1])
        tf_msg.transform.translation.z = float(trans[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

    def timer_callback(self):
        # 先保证 odom/base_link 一直连通，避免 TF 断树
        self.publish_identity_odom_to_base()

        if self.need_reanchor:
            self.try_reanchor()

        self.publish_rebased_map_to_odom()


def main(args=None):
    rclpy.init(args=args)
    node = OdomBaseDebugRebaser()
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


if __name__ == "__main__":
    main()
