#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


def yaw_from_quaternion(qx, qy, qz, qw):
    """쿼터니언 → yaw(rad)"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """yaw(rad) → (x, y, z, w)"""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        self.last_x = None
        self.last_y = None
        self.last_yaw = None  # rad

        self.frame_id = 'odom'

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_cb,
            10
        )

        self.wp_pub = self.create_publisher(
            PoseStamped,
            '/waypoint',
            10
        )

        self.get_logger().info('WaypointNode: /odom + /imu/data -> /waypoint')

    def odom_cb(self, msg: Odometry):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        self.publish_if_ready()

    def imu_cb(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.last_yaw = yaw_from_quaternion(qx, qy, qz, qw)
        self.publish_if_ready()

    def publish_if_ready(self):
        if self.last_x is None or self.last_y is None or self.last_yaw is None:
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id

        ps.pose.position.x = self.last_x
        ps.pose.position.y = self.last_y
        ps.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(self.last_yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self.wp_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
