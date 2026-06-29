#sm_path_follower.py
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
        super().__init__('sm_path_follower')

        # === 파라미터 ===
        # waypoint 파일 경로
        self.declare_parameter('waypoint_file', '')
        # pure-pursuit 비슷한 lookahead 거리
        self.declare_parameter('lookahead_dist', 0.8)
        # 기본 선속도
        self.declare_parameter('linear_speed', 0.7)
        # heading 오차에 곱할 gain
        self.declare_parameter('k_ang', 1.5)
        # ★ 최대 각속도 (rad/s) – 너무 과격하게 돌지 않게 제한
        self.declare_parameter('max_ang_vel', 1.2)
        # 최종 goal 근처 허용 오차
        self.declare_parameter('goal_tolerance', 0.3)
        # TF 프레임 (recorder와 동일하게)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # === 파라미터 값 읽기 ===
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        if waypoint_file == '':
            # ★ 기본 waypoint 파일 (필요하면 여기 smoothSpline 버전으로 바꿔도 됨)
            waypoint_file = os.path.join(
                os.path.expanduser('~'),
                'ros2_ws',
                'src',
                'scout_mini_tools',
                'waypoint',
                'waypoints_floor12_second_smoothSpline.yaml'
            )
            self.get_logger().warn(
                f'waypoint_file 파라미터가 비어 있어서, '
                f'기본 경로 {waypoint_file} 를 사용합니다.'
            )

        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)   # ★

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

        # recorder가 [x, y, yaw]로 저장하니까 그 형식에 맞게 읽기
        self.waypoints = []
        for wp in raw_wps:
            if len(wp) == 3:
                x, y, yaw = wp
            elif len(wp) == 2:
                # 혹시 예전 포맷([x, y])일 수도 있으니 fallback
                x, y = wp
                yaw = 0.0
            else:
                self.get_logger().warn(f"잘못된 waypoint 형식: {wp}, 건너뜁니다.")
                continue
            self.waypoints.append((float(x), float(y), float(yaw)))

        if len(self.waypoints) == 0:
            self.get_logger().error('유효한 waypoints가 없습니다.')
            raise RuntimeError('no valid waypoints')

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {waypoint_file}')

        # 현재 타겟 인덱스
        self.current_idx = 0
        self.goal_reached = False

        # TF 버퍼/리스너 (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ★ 제어 주기: 0.02s => 50Hz (조금 더 빠르게)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f'PathFollower started. global_frame={self.global_frame}, base_frame={self.base_frame}'
        )

    def timer_callback(self):
        if self.goal_reached:
            return

        try:
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,   # 'map'
                self.base_frame,     # 'base_link'
                now
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'TF ({self.global_frame} -> {self.base_frame}) 를 아직 가져오지 못했습니다.'
            )
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        # 최종 waypoint와의 거리 (x, y만 사용)
        goal_x, goal_y, _ = self.waypoints[-1]
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
            wx, wy, _ = self.waypoints[i]
            dist = math.hypot(wx - x, wy - y)
            if dist > self.lookahead_dist:
                target_idx = i
                break
        else:
            # 마지막 waypoint까지 도달한 경우
            target_idx = len(self.waypoints) - 1

        self.current_idx = target_idx
        tx, ty, _ = self.waypoints[target_idx]

        # 타겟까지의 각도/거리 계산
        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error = normalize_angle(angle_to_target - yaw)
        dist_to_target = math.hypot(tx - x, ty - y)

        heading_mag = abs(heading_error)

        # === 선속도 / 각속도 결정 ===
        twist = Twist()

        # ★ 1) heading error에 따른 부드러운 속도 스케일링
        v_nominal = self.linear_speed

        # heading_error가 클수록 속도를 줄이는 팩터 (대충 0.2 ~ 1.0 사이)
        slow_factor = 1.0 / (1.0 + 2.0 * heading_mag)  # 튜닝 가능
        v = v_nominal * slow_factor

        # ★ goal 근처에서는 추가 감속
        if goal_dist < 2.0:
            v *= 0.5
            v = max(0.2, v)

        # heading_error가 너무 크면 거의 정지해서 회전만
        if heading_mag > math.radians(80.0):
            v = 0.0

        # ★ 2) 각속도 = P제어 + saturation
        omega = self.k_ang * heading_error

        # 각속도 제한
        if omega > self.max_ang_vel:
            omega = self.max_ang_vel
        elif omega < -self.max_ang_vel:
            omega = -self.max_ang_vel

        twist.linear.x = v
        twist.angular.z = omega

        self.cmd_pub.publish(twist)

        # 디버깅용 로그(너무 시끄러우면 주석 처리)
        self.get_logger().debug(
            f'idx={self.current_idx}, pos=({x:.2f},{y:.2f}), '
            f'target=({tx:.2f},{ty:.2f}), dist={dist_to_target:.2f}, '
            f'heading_err(deg)={math.degrees(heading_error):.1f}, v={v:.2f}, w={omega:.2f}'
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