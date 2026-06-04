#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from robots_msgs.msg import ChassisOdom
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class GimbalYawTfPublisher(Node):
    def __init__(self):
        super().__init__("gimbal_yaw_tf_publisher")

        self.gimbal_angle_topic = self.declare_parameter(
            "gimbal_angle_topic", "/ChassisOdom"
        ).value
        self.parent_frame = self.declare_parameter("parent_frame", "base_link").value
        self.child_frame = self.declare_parameter(
            "child_frame", "gimbal_yaw_link"
        ).value

        self.gimbal_axis_x = float(self.declare_parameter("gimbal_axis_x", 0.0).value)
        self.gimbal_axis_y = float(self.declare_parameter("gimbal_axis_y", 0.0).value)
        self.gimbal_axis_z = float(self.declare_parameter("gimbal_axis_z", 0.0).value)

        self.yaw_unit = str(self.declare_parameter("yaw_unit", "deg").value).lower()
        self.yaw_sign = float(self.declare_parameter("yaw_sign", 1.0).value)
        self.yaw_offset_rad = float(
            self.declare_parameter("yaw_offset_rad", 0.0).value
        )
        self.initial_yaw_rad = float(
            self.declare_parameter("initial_yaw_rad", 0.0).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 100.0).value
        )
        self.publish_before_first_msg = bool(
            self.declare_parameter("publish_before_first_msg", True).value
        )

        self.latest_yaw_rad = self.initial_yaw_rad
        self.received_angle = False
        self.warned_bad_unit = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            ChassisOdom,
            self.gimbal_angle_topic,
            self.gimbal_angle_callback,
            20,
        )

        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            "gimbal_yaw_tf_publisher: %s -> %s from %s.gimbal_angle (%s)"
            % (
                self.parent_frame,
                self.child_frame,
                self.gimbal_angle_topic,
                self.yaw_unit,
            )
        )
        self.get_logger().warn(
            "ChassisOdom has no hardware timestamp; this TF is stamped with ROS receive time."
        )

    def gimbal_angle_callback(self, msg):
        self.latest_yaw_rad = self.convert_yaw(float(msg.gimbal_angle))
        self.received_angle = True
        self.publish_transform()

    def convert_yaw(self, raw_yaw):
        if self.yaw_unit == "deg":
            yaw_rad = math.radians(raw_yaw)
        elif self.yaw_unit == "rad":
            yaw_rad = raw_yaw
        else:
            yaw_rad = math.radians(raw_yaw)
            if not self.warned_bad_unit:
                self.get_logger().warn(
                    "Unknown yaw_unit '%s', falling back to degrees" % self.yaw_unit
                )
                self.warned_bad_unit = True
        return self.yaw_offset_rad + self.yaw_sign * yaw_rad

    def timer_callback(self):
        if self.received_angle or self.publish_before_first_msg:
            self.publish_transform()

    def publish_transform(self):
        qx, qy, qz, qw = yaw_to_quaternion(self.latest_yaw_rad)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = self.gimbal_axis_x
        tf_msg.transform.translation.y = self.gimbal_axis_y
        tf_msg.transform.translation.z = self.gimbal_axis_z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalYawTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
