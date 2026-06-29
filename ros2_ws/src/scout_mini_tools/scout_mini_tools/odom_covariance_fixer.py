#!/usr/bin/env python3
"""
scout_base가 공분산을 0으로 퍼블리시하는 문제를 해결하는 래퍼 노드.
/odom을 구독해서 적절한 공분산을 추가한 뒤 /odom_fixed로 퍼블리시.
EKF yaml에서 odom0: /odom_fixed 로 변경해서 사용.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

POSE_COV = [
    0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
]

TWIST_COV = [
    0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  1e6,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
]


class OdomCovarianceFixer(Node):
    def __init__(self):
        super().__init__('odom_covariance_fixer')
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom_fixed', 10)

    def callback(self, msg: Odometry):
        msg.pose.covariance = POSE_COV
        msg.twist.covariance = TWIST_COV
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomCovarianceFixer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
