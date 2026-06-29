# sm_path_follower2.py
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
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('lookahead_dist', 0.8)
        self.declare_parameter('linear_speed', 0.7)
        self.declare_parameter('k_ang', 1.5)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # ### 추가: 최대 각속도 제한
        self.declare_parameter('max_ang_speed', 1.0)  # [rad/s] 정도로 시작

        # === 파라미터 값 읽기 ===
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        if waypoint_file == '':
            waypoint_file = os.path.join(
                os.path.expanduser('~'),
                'ros2_ws',
                'src',
                'scout_mini_tools',
                'waypoint',
                'waypoints_floor12_second.yaml'
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
        self.max_ang_speed = float(self.get_parameter('max_ang_speed').value)

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

        self.waypoints = []
        for wp in raw_wps:
            if len(wp) == 3:
                x, y, yaw = wp
            elif len(wp) == 2:
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

        # ### 추가: 경로 누적 길이(pre-compute) – path 상에서 lookahead 계산용
        self.cum_s = [0.0]
        for i in range(1, len(self.waypoints)):
            x_prev, y_prev, _ = self.waypoints[i - 1]
            x_i, y_i, _ = self.waypoints[i]
            ds = math.hypot(x_i - x_prev, y_i - y_prev)
            self.cum_s.append(self.cum_s[-1] + ds)

        self.current_idx = 0
        self.goal_reached = False

        # TF 버퍼/리스너 (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 제어 루프 (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f'PathFollower started. global_frame={self.global_frame}, base_frame={self.base_frame}'
        )

    def timer_callback(self):
        if self.goal_reached:
            return

        try:
            # ### 수정: latest transform 사용 (Time() 0초 → 가장 최근 TF)
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,   # 'map'
                self.base_frame,     # 'base_link'
                Time()               # 0 → latest
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

        # ### 수정 1: 현재 위치에서 가장 가까운 waypoint 찾기
        nearest_idx = 0
        nearest_dist = float('inf')
        for i, (wx, wy, _) in enumerate(self.waypoints):
            d = math.hypot(wx - x, wy - y)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        # ### 수정 2: 경로 상 거리 기준으로 lookahead target 선택
        target_s = self.cum_s[nearest_idx] + self.lookahead_dist
        target_idx = len(self.waypoints) - 1
        for i in range(nearest_idx, len(self.waypoints)):
            if self.cum_s[i] >= target_s:
                target_idx = i
                break

        self.current_idx = target_idx
        tx, ty, _ = self.waypoints[target_idx]

        # 타겟까지의 각도/거리 계산
        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error = normalize_angle(angle_to_target - yaw)
        dist_to_target = math.hypot(tx - x, ty - y)

        # === 속도 결정 ===
        twist = Twist()

        # ### 수정 3: heading error에 따라 선속도 연속적으로 줄이기
        # heading_error = 0일 때 100%, 약 80deg에서 0%로 줄어들도록
        heading_abs = abs(heading_error)
        heading_limit = math.radians(80.0)

        speed_scale = 1.0
        if heading_abs >= heading_limit:
            speed_scale = 0.0
        else:
            speed_scale = 1.0 - (heading_abs / heading_limit)

        v = self.linear_speed * speed_scale

        # goal 근처에서는 전체 속도 자체를 줄이기
        if goal_dist < 2.0:
            v = max(0.2, v * 0.5)

        twist.linear.x = max(0.0, v)  # 음수 속도는 여기선 사용 X

        # 각속도 = k * heading_error + saturation
        w = self.k_ang * heading_error
        if w > self.max_ang_speed:
            w = self.max_ang_speed
        elif w < -self.max_ang_speed:
            w = -self.max_ang_speed
        twist.angular.z = w

        self.cmd_pub.publish(twist)

        self.get_logger().debug(
            f'idx={self.current_idx}, pos=({x:.2f},{y:.2f}), '
            f'target=({tx:.2f},{ty:.2f}), dist={dist_to_target:.2f}, '
            f'heading_err(deg)={math.degrees(heading_error):.1f}, '
            f'v={twist.linear.x:.2f}, w={twist.angular.z:.2f}'
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
