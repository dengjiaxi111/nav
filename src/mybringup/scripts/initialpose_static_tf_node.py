#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def quat_rotate(q, v):
    q = quat_normalize(q)
    rotated = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), quat_conjugate(q))
    return rotated[:3]


def transform_inverse(transform):
    t, q = transform
    qi = quat_conjugate(quat_normalize(q))
    ti = quat_rotate(qi, (-t[0], -t[1], -t[2]))
    return ti, qi


def transform_multiply(a, b):
    ta, qa = a
    tb, qb = b
    rb = quat_rotate(qa, tb)
    return (
        (ta[0] + rb[0], ta[1] + rb[1], ta[2] + rb[2]),
        quat_normalize(quat_multiply(qa, qb)),
    )


def transform_from_pose(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def transform_from_msg(msg):
    return (
        (msg.translation.x, msg.translation.y, msg.translation.z),
        (msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w),
    )


class InitialPoseStaticTf(Node):
    def __init__(self):
        super().__init__("initialpose_static_tf")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("initialpose_topic", "/initialpose")

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        initialpose_topic = self.get_parameter("initialpose_topic").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            initialpose_topic,
            self.initialpose_callback,
            10,
        )

        self.get_logger().info(
            f"Waiting for RViz 2D Pose Estimate on {initialpose_topic}; "
            f"will publish static {self.map_frame}->{self.odom_frame}"
        )

    def initialpose_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"/initialpose frame is '{msg.header.frame_id}', expected '{self.map_frame}'"
            )

        map_to_base = transform_from_pose(msg.pose.pose)

        try:
            odom_to_base_msg = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            odom_to_base = transform_from_msg(odom_to_base_msg.transform)
            map_to_odom = transform_multiply(map_to_base, transform_inverse(odom_to_base))
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot lookup {self.odom_frame}->{self.base_frame}; "
                f"using /initialpose directly as {self.map_frame}->{self.odom_frame}: {exc}"
            )
            map_to_odom = map_to_base

        self.publish_static_tf(map_to_odom)

    def publish_static_tf(self, transform):
        translation, rotation = transform
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = translation[0]
        msg.transform.translation.y = translation[1]
        msg.transform.translation.z = translation[2]
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]
        self.static_broadcaster.sendTransform(msg)
        self.get_logger().info(
            f"Published static {self.map_frame}->{self.odom_frame}: "
            f"x={translation[0]:.3f}, y={translation[1]:.3f}, z={translation[2]:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseStaticTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
