# path_follower.py
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quaternion(q):
    # q: geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    # [-pi, pi]로 정규화
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # === 파라미터 ===
        # waypoint 파일 경로 (필수)
        self.declare_parameter('waypoint_file', '')
        # pure-pursuit 비슷한 lookahead 거리
        self.declare_parameter('lookahead_dist', 0.8)
        # 기본 선속도
        self.declare_parameter('linear_speed', 0.7)
        # heading 오차에 곱할 gain
        self.declare_parameter('k_ang', 1.5)
        # 최종 goal 근처 허용 오차
        self.declare_parameter('goal_tolerance', 0.3)

        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        if waypoint_file == '':
            # 기본값: 현재 디렉토리 기준 (예: ~/ros2_ws/waypoints_gazebo_floor5.yaml)
            waypoint_file = os.path.join(os.getcwd(), 'waypoints_gazebo_floor5_fixed.yaml')
            self.get_logger().warn(
                f'waypoint_file 파라미터가 비어 있어서, '
                f'현재 디렉토리 기준 {waypoint_file} 를 시도합니다.'
            )

        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # === waypoints 로드 ===
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f'waypoint 파일을 찾을 수 없습니다: {waypoint_file}')
            raise FileNotFoundError(waypoint_file)

        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)

        raw_wps = data.get('waypoints', [])
        if len(raw_wps) == 0:
            self.get_logger().error('waypoints 리스트가 비어 있습니다.')
            raise RuntimeError('empty waypoints')

        self.waypoints = [(float(x), float(y)) for x, y in raw_wps]
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {waypoint_file}')

        # 현재 타겟 인덱스
        self.current_idx = 0
        self.goal_reached = False

        # TF 버퍼/리스너 (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20 Hz 루프
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('PathFollower started.')

    def timer_callback(self):
        if self.goal_reached:
            ...
        try:
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', now
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Humble rclpy에는 warn_throttle이 없어서 그냥 warn만 사용
            self.get_logger().warn('TF (odom -> base_link) 를 아직 가져오지 못했습니다.')
            return


        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        # 최종 waypoint와의 거리
        goal_x, goal_y = self.waypoints[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        if goal_dist < self.goal_tolerance:
            self.get_logger().info('Goal reached! Stopping.')
            self.goal_reached = True
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # lookahead target 선택
        target_idx = self.current_idx
        for i in range(self.current_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - x, wy - y)
            if dist > self.lookahead_dist:
                target_idx = i
                break
        else:
            # 마지막 waypoint까지 도달한 경우
            target_idx = len(self.waypoints) - 1

        self.current_idx = target_idx
        tx, ty = self.waypoints[target_idx]

        # 타겟까지의 각도/거리 계산
        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error = normalize_angle(angle_to_target - yaw)
        dist_to_target = math.hypot(tx - x, ty - y)

        # 선속도 / 각속도 결정
        twist = Twist()

        # goal 근처에서는 속도 살짝 줄이기
        v = self.linear_speed
        if goal_dist < 2.0:
            v = max(0.2, self.linear_speed * 0.5)

        # heading error가 너무 크면 선속도 줄이기
        if abs(heading_error) > math.radians(60.0):
            v = 0.0

        twist.linear.x = v
        twist.angular.z = self.k_ang * heading_error

        self.cmd_pub.publish(twist)

        # 디버깅용 로그(너무 시끄러우면 주석 처리)
        self.get_logger().debug(
            f'idx={self.current_idx}, pos=({x:.2f},{y:.2f}), '
            f'target=({tx:.2f},{ty:.2f}), dist={dist_to_target:.2f}, '
            f'heading_err(deg)={math.degrees(heading_error):.1f}, v={v:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down PathFollower node')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
