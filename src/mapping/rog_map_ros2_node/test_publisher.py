#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Twist, Point, Quaternion
import numpy as np
import struct


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Publishers
        self.points_pub = self.create_publisher(PointCloud2, '/points_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_data)
        self.counter = 0
        
    def publish_data(self):
        """Publish test point cloud and odometry"""
        self.counter += 1
        
        # Publish point cloud
        cloud = self.create_point_cloud()
        self.points_pub.publish(cloud)
        
        # Publish odometry
        odom = self.create_odometry()
        self.odom_pub.publish(odom)
        
        self.get_logger().info(f'Published data {self.counter}')
        
    def create_point_cloud(self):
        """Create a simple point cloud"""
        # Generate random points in a small region
        num_points = 100
        points = np.random.rand(num_points, 3).astype(np.float32)
        points[:, 0] = points[:, 0] * 10 - 5  # x: -5 to 5
        points[:, 1] = points[:, 1] * 10 - 5  # y: -5 to 5
        points[:, 2] = points[:, 2] * 2       # z: 0 to 2
        
        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'base_link'
        
        cloud.height = 1
        cloud.width = num_points
        cloud.is_dense = True
        cloud.is_bigendian = False
        
        # Define fields
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud.point_step = 12  # 3 * float32
        cloud.row_step = cloud.point_step * num_points
        
        # Pack points data
        cloud.data = points.tobytes()
        
        return cloud
    
    def create_odometry(self):
        """Create odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Simple moving position
        x = np.sin(self.counter * 0.01) * 5
        y = np.cos(self.counter * 0.01) * 5
        
        odom.pose.pose = self.create_pose_stamped(x, y, 0)
        odom.pose.covariance = [0.0] * 36
        
        odom.twist.twist = Twist()
        odom.twist.covariance = [0.0] * 36
        
        return odom
    
    def create_pose_stamped(self, x, y, z):
        """Create a pose message"""
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    publisher = TestPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
