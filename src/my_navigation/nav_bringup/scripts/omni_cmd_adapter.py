#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def yaw_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OmniCmdAdapter(Node):
    """
    Adapter for the omni chassis migration.

    Input command is interpreted in base_link_fake. The node keeps
    base_link_fake colocated with base_link, rotates the command into base_link,
    and republishes odometry in the fake control frame for NMPC feedback.
    """

    def __init__(self):
        super().__init__("omni_cmd_adapter")

        self.input_cmd_vel_topic = self.declare_parameter(
            "input_cmd_vel_topic", "/cmd_vel_fake"
        ).value
        self.output_cmd_vel_topic = self.declare_parameter(
            "output_cmd_vel_topic", "/cmd_vel"
        ).value
        self.source_odom_topic = self.declare_parameter(
            "source_odom_topic", "/Odometry"
        ).value
        self.fake_odom_topic = self.declare_parameter(
            "fake_odom_topic", "/Odometry/fake"
        ).value

        self.parent_frame = self.declare_parameter("parent_frame", "base_link").value
        self.fake_frame = self.declare_parameter("fake_frame", "base_link_fake").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value

        self.use_fake_vel = bool(self.declare_parameter("use_fake_vel", True).value)
        self.fake_vel_on = bool(self.declare_parameter("fake_vel_on", True).value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.publish_fake_odom = bool(
            self.declare_parameter("publish_fake_odom", True).value
        )
        self.publish_zero_on_timeout = bool(
            self.declare_parameter("publish_zero_on_timeout", True).value
        )

        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 50.0).value)
        self.cmd_timeout_sec = float(
            self.declare_parameter("cmd_timeout_sec", 0.2).value
        )
        self.fake_angular_speed_coefficient = float(
            self.declare_parameter("fake_angular_speed_coefficient", 1.0).value
        )
        self.fake_yaw = normalize_angle(
            float(self.declare_parameter("initial_fake_yaw", 0.0).value)
        )

        self.latest_cmd = Twist()
        self.last_cmd_time = None
        self.last_update_time = self.get_clock().now()
        self.last_delta_rate = 0.0
        self.last_fake_odom_yaw = None
        self.last_fake_odom_stamp = None
        self.last_published_timeout_zero = False

        qos_cmd = QoSProfile(depth=10)
        qos_sensor = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_vel_topic, 10)
        self.fake_odom_pub = self.create_publisher(
            Odometry, self.fake_odom_topic, qos_sensor
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.input_cmd_vel_topic, self.cmd_callback, qos_cmd
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.source_odom_topic, self.odom_callback, qos_sensor
        )

        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            "omni_cmd_adapter: %s(base_link_fake) -> %s(base_link), odom %s -> %s"
            % (
                self.input_cmd_vel_topic,
                self.output_cmd_vel_topic,
                self.source_odom_topic,
                self.fake_odom_topic,
            )
        )

    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.last_published_timeout_zero = False

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        cmd_valid = self.is_cmd_fresh(now)
        cmd = self.latest_cmd if cmd_valid else Twist()

        omega_for_fake = cmd.angular.z if cmd_valid else 0.0
        if self.use_fake_vel and self.fake_vel_on and dt > 0.0:
            self.last_delta_rate = (
                omega_for_fake * self.fake_angular_speed_coefficient
            )
            self.fake_yaw = normalize_angle(self.fake_yaw + self.last_delta_rate * dt)
        else:
            self.last_delta_rate = 0.0

        if self.publish_tf:
            self.publish_fake_tf(now)

        if cmd_valid or self.publish_zero_on_timeout:
            self.publish_transformed_cmd(cmd)
            if not cmd_valid and self.last_cmd_time is not None:
                if not self.last_published_timeout_zero:
                    self.get_logger().warn(
                        "cmd_vel timeout, publishing zero output command"
                    )
                    self.last_published_timeout_zero = True

    def is_cmd_fresh(self, now):
        if self.last_cmd_time is None:
            return False
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        return age <= max(0.0, self.cmd_timeout_sec)

    def current_delta(self):
        if self.use_fake_vel and self.fake_vel_on:
            return self.fake_yaw
        return 0.0

    def publish_fake_tf(self, now):
        qx, qy, qz, qw = yaw_to_quaternion(self.current_delta())

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.fake_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_transformed_cmd(self, cmd_fake):
        delta = self.current_delta()
        c = math.cos(delta)
        s = math.sin(delta)

        vx_fake = float(cmd_fake.linear.x)
        vy_fake = float(cmd_fake.linear.y)

        cmd_base = Twist()
        cmd_base.linear.x = vx_fake * c - vy_fake * s
        cmd_base.linear.y = vx_fake * s + vy_fake * c
        cmd_base.linear.z = cmd_fake.linear.z
        cmd_base.angular.x = cmd_fake.angular.x
        cmd_base.angular.y = cmd_fake.angular.y
        cmd_base.angular.z = cmd_fake.angular.z
        self.cmd_pub.publish(cmd_base)

    def odom_callback(self, msg):
        if not self.publish_fake_odom:
            return

        delta = self.current_delta()
        c = math.cos(delta)
        s = math.sin(delta)

        odom = Odometry()
        odom.header = msg.header
        if not odom.header.frame_id:
            odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.fake_frame

        odom.pose.pose.position = msg.pose.pose.position
        q_odom_base = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        q_base_fake = yaw_to_quaternion(delta)
        q_odom_fake = quaternion_normalize(
            quaternion_multiply(q_odom_base, q_base_fake)
        )
        odom.pose.pose.orientation.x = q_odom_fake[0]
        odom.pose.pose.orientation.y = q_odom_fake[1]
        odom.pose.pose.orientation.z = q_odom_fake[2]
        odom.pose.pose.orientation.w = q_odom_fake[3]
        odom.pose.covariance = msg.pose.covariance

        vx_base = float(msg.twist.twist.linear.x)
        vy_base = float(msg.twist.twist.linear.y)
        odom.twist.twist.linear.x = vx_base * c + vy_base * s
        odom.twist.twist.linear.y = -vx_base * s + vy_base * c
        odom.twist.twist.linear.z = msg.twist.twist.linear.z
        odom.twist.twist.angular.x = msg.twist.twist.angular.x
        odom.twist.twist.angular.y = msg.twist.twist.angular.y
        odom.twist.twist.angular.z = self.estimate_fake_yaw_rate(
            odom, float(msg.twist.twist.angular.z)
        )
        odom.twist.covariance = msg.twist.covariance

        self.fake_odom_pub.publish(odom)

    def estimate_fake_yaw_rate(self, odom, source_wz):
        yaw = yaw_from_quaternion(
            (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            )
        )

        stamp = Time.from_msg(odom.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        if self.last_fake_odom_yaw is None or self.last_fake_odom_stamp is None:
            self.last_fake_odom_yaw = yaw
            self.last_fake_odom_stamp = stamp
            return source_wz + self.last_delta_rate

        dt = (stamp - self.last_fake_odom_stamp).nanoseconds * 1e-9
        if dt <= 1e-4 or dt > 1.0:
            self.last_fake_odom_yaw = yaw
            self.last_fake_odom_stamp = stamp
            return source_wz + self.last_delta_rate

        yaw_rate = normalize_angle(yaw - self.last_fake_odom_yaw) / dt
        self.last_fake_odom_yaw = yaw
        self.last_fake_odom_stamp = stamp
        return yaw_rate


def main(args=None):
    rclpy.init(args=args)
    node = OmniCmdAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
