#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


class OmniStage3Check(Node):
    def __init__(self):
        super().__init__("omni_stage3_check")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.fake_frame = self.declare_parameter("fake_frame", "base_link_fake").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "livox_frame").value

        self.cloud_topic = self.declare_parameter(
            "cloud_registered_topic", "/cloud_registered"
        ).value
        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry").value
        self.fake_odom_topic = self.declare_parameter(
            "fake_odom_topic", "/Odometry/fake"
        ).value
        self.dynamic_layer_topic = self.declare_parameter(
            "dynamic_layer_topic", "/rog_map/map_2d"
        ).value
        self.check_duration_sec = float(
            self.declare_parameter("check_duration_sec", 5.0).value
        )

        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cloud = None
        self.latest_odom = None
        self.latest_fake_odom = None
        self.latest_dynamic_map = None

        self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_callback, qos_sensor
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_sensor)
        self.create_subscription(
            Odometry, self.fake_odom_topic, self.fake_odom_callback, qos_sensor
        )
        self.create_subscription(
            OccupancyGrid, self.dynamic_layer_topic, self.dynamic_map_callback, qos_sensor
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.done = False

        self.get_logger().info("Stage3 check started; waiting for TF and topics...")

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def fake_odom_callback(self, msg):
        self.latest_fake_odom = msg

    def dynamic_map_callback(self, msg):
        self.latest_dynamic_map = msg

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < self.check_duration_sec:
            return
        self.done = True
        self.report()

    def report(self):
        checks = []
        checks.append(self.check_tf(self.map_frame, self.odom_frame))
        checks.append(self.check_tf(self.odom_frame, self.base_frame))
        checks.append(self.check_tf(self.base_frame, self.fake_frame))
        checks.append(self.check_tf(self.base_frame, self.lidar_frame))
        checks.append(self.check_tf(self.map_frame, self.fake_frame))

        checks.append(self.check_topic(self.cloud_topic, self.latest_cloud))
        checks.append(self.check_topic(self.odom_topic, self.latest_odom))
        checks.append(self.check_topic(self.fake_odom_topic, self.latest_fake_odom))
        checks.append(self.check_topic(self.dynamic_layer_topic, self.latest_dynamic_map))

        if self.latest_cloud is not None:
            checks.append(
                self.check_value(
                    f"{self.cloud_topic}.header.frame_id",
                    self.latest_cloud.header.frame_id,
                    nonempty=True,
                )
            )
        if self.latest_odom is not None:
            checks.append(
                self.check_value(
                    f"{self.odom_topic}.child_frame_id",
                    self.latest_odom.child_frame_id,
                    expected=self.base_frame,
                )
            )
        if self.latest_fake_odom is not None:
            checks.append(
                self.check_value(
                    f"{self.fake_odom_topic}.child_frame_id",
                    self.latest_fake_odom.child_frame_id,
                    expected=self.fake_frame,
                )
            )
        if self.latest_dynamic_map is not None:
            checks.append(
                self.check_value(
                    f"{self.dynamic_layer_topic}.header.frame_id",
                    self.latest_dynamic_map.header.frame_id,
                    expected=self.odom_frame,
                )
            )

        passed = sum(1 for ok, _ in checks if ok)
        total = len(checks)
        for ok, text in checks:
            prefix = "OK" if ok else "FAIL"
            self.get_logger().info(f"[{prefix}] {text}")
        if passed == total:
            self.get_logger().info(f"Stage3 check passed ({passed}/{total})")
        else:
            self.get_logger().warn(f"Stage3 check incomplete ({passed}/{total})")

    def check_tf(self, target, source):
        try:
            self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            return True, f"TF {target} <- {source}"
        except Exception as exc:
            return False, f"TF {target} <- {source}: {exc}"

    @staticmethod
    def check_topic(topic, msg):
        return (msg is not None), f"topic {topic}"

    @staticmethod
    def check_value(name, actual, expected=None, nonempty=False):
        if nonempty:
            ok = bool(actual)
            return ok, f"{name} = {actual!r}"
        ok = actual == expected
        return ok, f"{name} = {actual!r}, expected {expected!r}"


def main(args=None):
    rclpy.init(args=args)
    node = OmniStage3Check()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
