#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseMonitor(Node):
    def __init__(self):
        super().__init__('initial_pose_monitor')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.callback,
            10
        )
        self.get_logger().info('준비 완료 — RViz2에서 "2D Pose Estimate"를 클릭하세요.')

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_deg = math.degrees(yaw_rad)
        self.get_logger().info(
            f'초기 위치 수신 → x={x:.3f}  y={y:.3f}  yaw={yaw_deg:.1f}°  '
            f'(slam_toolbox가 자동으로 적용합니다)'
        )


def main():
    rclpy.init()
    node = InitialPoseMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
