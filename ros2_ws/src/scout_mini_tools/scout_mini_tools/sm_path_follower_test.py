# sm_path_follower_test.py
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
        super().__init__('sm_path_follower_test')

        # === 파라미터 ===
        # waypoint 파일 경로
        self.declare_parameter('waypoint_file', '')
        # 기준 lookahead 거리 (가변 lookahead의 중심값 역할)
        self.declare_parameter('lookahead_dist', 0.8)
        # 기준 선속도 (직선 구간에서의 최대 속도)  ★ 0.5 m/s 로 낮춰서 등속 느낌
        self.declare_parameter('linear_speed', 0.5)
        # Pure Pursuit gain (ω = v * k_ang * curvature_pp)
        self.declare_parameter('k_ang', 1.0)
        # 최대 각속도 (rad/s) – 너무 과격하게 돌지 않게 제한  ★ 1.0
        self.declare_parameter('max_ang_vel', 1.0)
        # 최종 goal 근처 허용 오차 (직선거리 기준)
        self.declare_parameter('goal_tolerance', 0.3)
        # TF 프레임 (recorder와 동일하게)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # === 종방향(속도) 제어용 추가 파라미터 ===
        # 곡률 기반 감속 강도 (클수록 커브에서 더 느리게 감속) ★ 0.25로 완만하게
        self.declare_parameter('kappa_v_gain', 0.25)
        # goal까지 남은 거리(m) 기준 추가 감속 시작 구간 ★ 0.7m 정도에서만 살짝 감속
        self.declare_parameter('slowdown_distance', 0.7)
        # 최소 선속도 (너무 느리게 안 가도록 바닥 속도) ★ 0.4 m/s
        self.declare_parameter('min_speed', 0.40)

        # 곡률 관련 추가 파라미터
        # 이 이하 곡률은 감속 안 함 (거의 직선 처리) ★ 0.22
        self.declare_parameter('kappa_deadband', 0.22)
        # 곡률 때문에 줄여도 이 비율 이하로는 안 떨어지게 ★ 0.9 (속도 최소 90% 유지)
        self.declare_parameter('min_curv_factor', 0.90)

        # === 파라미터 값 읽기 ===
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        if waypoint_file == '':
            # 기본 waypoint 파일 (smoothSpline + 5개 값 들어있는 버전)
            waypoint_file = os.path.join(
                os.path.expanduser('~'),
                'ros2_ws',
                'src',
                'scout_mini_tools',
                'waypoint',
                'waypoints_floor12_1127_smoothSpline_developed.yaml'
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
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)

        # 종방향 제어용 파라미터
        self.kappa_v_gain = float(self.get_parameter('kappa_v_gain').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.min_speed = float(self.get_parameter('min_speed').value)

        # 곡률 관련 파라미터
        self.kappa_deadband = float(self.get_parameter('kappa_deadband').value)
        self.min_curv_factor = float(self.get_parameter('min_curv_factor').value)

        # 가변 lookahead 거리 범위 설정 (기본값은 lookahead_dist 기준으로 스케일)
        self.min_lookahead = 0.5 * self.lookahead_dist
        self.max_lookahead = 1.5 * self.lookahead_dist
        self.current_lookahead = self.lookahead_dist  # 다음 사이클에서 사용할 Ld

        # === waypoints 로드 (x, y, yaw, curvature, cumlength) ===
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f'waypoint 파일을 찾을 수 없습니다: {waypoint_file}')
            raise FileNotFoundError(waypoint_file)

        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)

        raw_wps = data.get('waypoints', [])
        if len(raw_wps) == 0:
            self.get_logger().error('waypoints 리스트가 비어 있습니다.')
            raise RuntimeError('empty waypoints')

        # 1차로 x,y,yaw,(optional kappa,s)만 읽어두기
        tmp_points = []
        for wp in raw_wps:
            if len(wp) < 2:
                self.get_logger().warn(f"잘못된 waypoint 형식(2개 미만): {wp}, 건너뜁니다.")
                continue

            x = float(wp[0])
            y = float(wp[1])
            yaw = float(wp[2]) if len(wp) >= 3 else 0.0

            if len(wp) >= 5:
                kappa = float(wp[3])
                s_val = float(wp[4])
            else:
                kappa = None
                s_val = None

            tmp_points.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'kappa': kappa,
                's': s_val,
            })

        if len(tmp_points) == 0:
            self.get_logger().error('유효한 waypoints가 없습니다.')
            raise RuntimeError('no valid waypoints')

        # 누적거리(s) 없으면 x,y로부터 계산해서 채움
        xs = [p['x'] for p in tmp_points]
        ys = [p['y'] for p in tmp_points]
        s_list = [0.0]
        for i in range(1, len(xs)):
            ds = math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
            s_list.append(s_list[-1] + ds)

        # 최종 waypoints 리스트 구성
        self.waypoints = []
        for i, p in enumerate(tmp_points):
            kappa = p['kappa'] if p['kappa'] is not None else 0.0
            s_val = p['s'] if p['s'] is not None else s_list[i]

            self.waypoints.append({
                'x': p['x'],
                'y': p['y'],
                'yaw': p['yaw'],
                'kappa': kappa,
                's': s_val,
            })

        self.goal_s = self.waypoints[-1]['s']

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints (x,y,yaw,kappa,s) from {waypoint_file}'
        )

        # 현재 타겟 인덱스
        self.current_idx = 0
        self.goal_reached = False

        # TF 버퍼/리스너 (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 제어 주기: 0.02s => 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f'PathFollower started. global_frame={self.global_frame}, base_frame={self.base_frame}'
        )

    def timer_callback(self):
        if self.goal_reached:
            return

        # === 현재 로봇 pose (map -> base_link) ===
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

        # === goal까지 남은 거리(직선거리) ===
        goal_x = self.waypoints[-1]['x']
        goal_y = self.waypoints[-1]['y']
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        if goal_dist < self.goal_tolerance:
            self.get_logger().info('Goal reached! Stopping.')
            self.goal_reached = True
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # === 현재 경로 상 진행 정도 (s_current) ===
        s_current = self.waypoints[self.current_idx]['s']
        remaining_s = max(0.0, self.goal_s - s_current)

        # === 가변 lookahead 거리 사용 (이전 스텝에서 업데이트한 값) ===
        Ld = self.current_lookahead

        # === lookahead target 선택 === (pure pursuit용 목표점 선택)
        target_idx = self.current_idx
        for i in range(self.current_idx, len(self.waypoints)):
            wx = self.waypoints[i]['x']
            wy = self.waypoints[i]['y']
            dist = math.hypot(wx - x, wy - y)
            if dist > Ld:
                target_idx = i
                break
        else:
            # 마지막 waypoint까지 도달한 경우
            target_idx = len(self.waypoints) - 1

        self.current_idx = target_idx
        tx = self.waypoints[target_idx]['x']
        ty = self.waypoints[target_idx]['y']
        kappa_target = self.waypoints[target_idx]['kappa']

        # === 횡방향 제어: Pure Pursuit ===
        dx = tx - x
        dy = ty - y
        # base_link 기준 전방 x, 좌측 y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        Ld_actual = math.hypot(local_x, local_y)
        if Ld_actual < 1e-6:
            curvature_pp = 0.0
        else:
            # Pure Pursuit 곡률: k_pp = 2 * y / Ld^2
            curvature_pp = 2.0 * local_y / (Ld_actual ** 2)

        # 참고용 heading error
        angle_to_target = math.atan2(ty - y, tx - x)
        heading_error = normalize_angle(angle_to_target - yaw)
        heading_mag = abs(heading_error)

        # === 종방향 제어: 선속도 결정 (곡률은 여기에서만 사용) ===
        v_nominal = self.linear_speed

        # 1) 곡률 기반 감속 (커브에서만 살짝)
        kappa_mag = abs(kappa_target)

        if kappa_mag <= self.kappa_deadband:
            # 거의 직선 → 감속 없음
            curv_factor = 1.0
        else:
            # deadband 이상인 부분만 반영
            kappa_eff = kappa_mag - self.kappa_deadband
            curv_factor = 1.0 / (1.0 + self.kappa_v_gain * kappa_eff)
            # 너무 많이 줄이지 않도록 하한 설정
            curv_factor = max(self.min_curv_factor, curv_factor)

        # 2) goal 근처 감속 (누적거리 기반)
        if self.slowdown_distance > 0.0:
            slow_factor = min(1.0, remaining_s / self.slowdown_distance)
        else:
            slow_factor = 1.0

        v = v_nominal * curv_factor * slow_factor

        # 3) 최소/최대 속도 보장
        v = max(self.min_speed, min(v, v_nominal))

        # 4) heading error가 너무 크면 속도 줄이기 / 정지 회전
        if heading_mag > math.radians(80.0):
            v = 0.0

        # === Pure Pursuit로 각속도 계산 (횡방향) ===
        omega = v * curvature_pp * self.k_ang

        # 각속도 제한
        if omega > self.max_ang_vel:
            omega = self.max_ang_vel
        elif omega < -self.max_ang_vel:
            omega = -self.max_ang_vel

        # === cmd_vel publish ===
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_pub.publish(twist)

        # === 다음 스텝을 위한 가변 Lookahead 업데이트 ===
        self.current_lookahead = self._update_lookahead(v, remaining_s)

        # 디버깅용 로그
        self.get_logger().debug(
            f'idx={self.current_idx}, pos=({x:.2f},{y:.2f}), '
            f'target=({tx:.2f},{ty:.2f}), Ld={Ld:.2f}, '
            f'heading_err(deg)={math.degrees(heading_error):.1f}, '
            f'v={v:.2f}, w={omega:.2f}, kappa_path={kappa_target:.3f}, '
            f'rem_s={remaining_s:.2f}'
        )

    def _update_lookahead(self, v, remaining_s):
        """
        가변 Lookahead Distance 업데이트 로직
        - 속도(v)가 클수록 lookahead를 크게
        - goal에 가까워질수록 lookahead를 작게
        """
        # 속도 기반 normalize (0~1)
        v_ref = max(self.linear_speed, 1e-3)
        alpha_v = max(0.0, min(1.0, abs(v) / v_ref))

        # goal까지 남은 거리 기반 normalize (0~1)
        if self.slowdown_distance > 0.0:
            alpha_s = max(0.0, min(1.0, remaining_s / self.slowdown_distance))
        else:
            alpha_s = 1.0

        # 둘을 적당히 섞어서 가중합 (속도 0.7, 거리 0.3)
        alpha = 0.7 * alpha_v + 0.3 * alpha_s

        Ld = self.min_lookahead + alpha * (self.max_lookahead - self.min_lookahead)
        return max(self.min_lookahead, min(Ld, self.max_lookahead))


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
