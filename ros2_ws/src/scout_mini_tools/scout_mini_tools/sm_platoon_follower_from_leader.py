# sm_platoon_follower_from_leader.py
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

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


class PlatoonFollower(Node):
    """
    follower용 path follower.

    - Leader:
      * 미리 준비된 spline path를 따라 주행 (sm_path_follower_test4.py)
      * waypoint_node + bridge로 PoseStamped(/leader/waypoint) 발행

    - Follower:
      * /leader/waypoint (PoseStamped, [x,y,yaw])를 받아
      * Pure Pursuit + 거리 기반 ACC로 리더를 따라감
      * E-STOP, AEB, OA(LiDAR 기반) 구조는 leader 코드와 최대한 유사하게 유지
    """

    def __init__(self):
        super().__init__('sm_platoon_follower')

        # === 파라미터 ===
        # 리더 waypoint 토픽 이름
        self.declare_parameter('leader_waypoint_topic', '/leader/waypoint')

        # TF 프레임
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # 기준 속도 및 각속도
        self.declare_parameter('linear_speed', 0.5)     # [m/s] 직선부 최대 속도
        self.declare_parameter('min_speed', 0.10)       # [m/s] 너무 느리지 않게 하기 위한 최소 속도
        self.declare_parameter('max_ang_vel', 1.0)      # [rad/s] 최대 yaw rate
        self.declare_parameter('k_ang', 1.0)            # Pure Pursuit gain

        # 리더와의 거리 제어 (간단 ACC)
        self.declare_parameter('desired_gap', 1.0)      # [m] 리더와 유지하고 싶은 거리
        self.declare_parameter('kp_dist', 0.8)          # 거리 오차 -> 속도 변환 gain
        self.declare_parameter('min_stop_distance', 0.4)  # [m] 이 이내면 정지

        # Pure Pursuit용 lookahead
        self.declare_parameter('min_lookahead', 0.5)
        self.declare_parameter('max_lookahead', 2.0)

        # heading error가 너무 크면 제자리 회전
        self.declare_parameter('heading_stop_deg', 80.0)

        # === Obstacle Avoidance 관련 파라미터 (leader 코드와 통일) ===
        self.declare_parameter('oa_trigger_distance', 1.4)
        self.declare_parameter('oa_clear_distance', 0.7)
        self.declare_parameter('oa_max_omega', 0.5)
        self.declare_parameter('oa_v_slow_gain', 0.5)
        self.declare_parameter('oa_fov_deg', 30.0)

        # === 파라미터 값 읽기 ===
        self.leader_waypoint_topic = (
            self.get_parameter('leader_waypoint_topic')
            .get_parameter_value().string_value
        )

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)
        self.k_ang = float(self.get_parameter('k_ang').value)

        self.desired_gap = float(self.get_parameter('desired_gap').value)
        self.kp_dist = float(self.get_parameter('kp_dist').value)
        self.min_stop_distance = float(self.get_parameter('min_stop_distance').value)

        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)

        self.heading_stop_deg = float(self.get_parameter('heading_stop_deg').value)

        # OA 파라미터
        self.oa_trigger_distance = float(self.get_parameter('oa_trigger_distance').value)
        self.oa_clear_distance = float(self.get_parameter('oa_clear_distance').value)
        self.oa_max_omega = float(self.get_parameter('oa_max_omega').value)
        self.oa_v_slow_gain = float(self.get_parameter('oa_v_slow_gain').value)
        self.oa_fov = math.radians(
            float(self.get_parameter('oa_fov_deg').value)
        )

        # === 내부 상태 ===
        self.last_leader_pose = None   # PoseStamped
        self.last_leader_time = None

        # OA용 LiDAR 전방 최소 거리
        self.oa_front_dist = math.inf

        # TF 버퍼/리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === E-STOP ===
        self.estop_active = False
        self.create_subscription(
            Bool,
            '/scout_mini/e_stop',
            self.estop_callback,
            10
        )

        # === AEB 스케일 ===
        self.aeb_scale = 1.0
        self.create_subscription(
            Float32,
            '/scout_mini/aeb_scale',
            self.aeb_callback,
            10
        )

        # === OA용 LiDAR 구독 ===
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback_oa,
            qos_profile_sensor_data
        )

        # === Leader waypoint 구독 ===
        self.create_subscription(
            PoseStamped,
            self.leader_waypoint_topic,
            self.leader_waypoint_callback,
            10
        )

        # 제어 주기: 0.02s => 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(
            f'PlatoonFollower started. '
            f'leader_waypoint_topic={self.leader_waypoint_topic}, '
            f'global_frame={self.global_frame}, base_frame={self.base_frame}'
        )

    # === 콜백들 ===
    def estop_callback(self, msg: Bool):
        if msg.data and not self.estop_active:
            self.get_logger().warn('E-STOP 활성화: 로봇을 정지합니다.')
        elif not msg.data and self.estop_active:
            self.get_logger().info('E-STOP 해제.')
        self.estop_active = msg.data

    def aeb_callback(self, msg: Float32):
        s = float(msg.data)
        if s < 0.0:
            s = 0.0
        elif s > 1.0:
            s = 1.0
        self.aeb_scale = s

    def leader_waypoint_callback(self, msg: PoseStamped):
        self.last_leader_pose = msg
        self.last_leader_time = self.get_clock().now()

    # === OA용 LiDAR 콜백 (leader 코드와 동일 로직) ===
    def laser_callback_oa(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        d_min = math.inf

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r <= 0.05:
                continue

            # LiDAR 프레임 각도
            angle = angle_min + i * angle_inc

            # base_link 기준으로 180도 회전 보정
            angle += math.pi
            if angle > math.pi:
                angle -= 2.0 * math.pi
            elif angle < -math.pi:
                angle += 2.0 * math.pi

            # 전방 ± oa_fov 안만 사용
            if -self.oa_fov <= angle <= self.oa_fov:
                if r < d_min:
                    d_min = r

        self.oa_front_dist = d_min

    # === OA 강도 계산 (0~1) ===
    def _compute_oa_alpha(self):
        d = self.oa_front_dist

        # AEB가 거의 풀브레이크 상태(0.2 이하)이면 OA는 굳이 안 넣음
        if self.aeb_scale < 0.2:
            return 0.0

        if math.isinf(d):
            return 0.0
        if d >= self.oa_trigger_distance:
            return 0.0
        if d <= self.oa_clear_distance:
            return 1.0

        # trigger_distance ~ clear_distance 사이를 0~1로 선형 매핑
        return (self.oa_trigger_distance - d) / (
            self.oa_trigger_distance - self.oa_clear_distance
        )

    # === 메인 제어 루프 ===
    def timer_callback(self):
        # 최상단: E-STOP
        if self.estop_active:
            self._publish_stop()
            return

        # Leader waypoint가 아직 없음
        if self.last_leader_pose is None:
            self.get_logger().debug('Leader waypoint 없음. 대기 중.')
            self._publish_stop()
            return

        # Leader 정보가 너무 오래됐으면 정지
        now = self.get_clock().now()
        if self.last_leader_time is not None:
            dt = (now - self.last_leader_time).nanoseconds * 1e-9
            if dt > 1.0:
                self.get_logger().warn(
                    'Leader waypoint가 1초 이상 갱신되지 않았습니다. 정지합니다.'
                )
                self._publish_stop()
                return

        # follower 현재 pose (map -> base_link)
        try:
            tf_now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                tf_now
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'TF ({self.global_frame} -> {self.base_frame}) 를 아직 가져오지 못했습니다.'
            )
            self._publish_stop()
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        # 리더 pose (global_frame 기준이라고 가정)
        lp = self.last_leader_pose.pose
        lx = lp.position.x
        ly = lp.position.y
        lq = lp.orientation
        lyaw = yaw_from_quaternion(lq)

        # follower -> leader 벡터
        dx = lx - x
        dy = ly - y
        dist = math.hypot(dx, dy)

        # 너무 가까우면 (추돌 방지)
        if dist < self.min_stop_distance:
            self.get_logger().debug(
                f'Leader와 너무 가까움(dist={dist:.2f} < {self.min_stop_distance:.2f}). 정지.'
            )
            self._publish_stop()
            return

        # follower 로컬 좌표계에서 리더 위치
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # === 종방향 제어: 거리 기반 간단 ACC ===
        e_d = dist - self.desired_gap  # 거리 오차

        if e_d <= 0.0:
            v = 0.0
        else:
            v_cmd = self.kp_dist * e_d
            v = max(self.min_speed, min(self.linear_speed, v_cmd))

        # AEB 스케일 적용
        v *= self.aeb_scale

        # === OA 강도 계산 및 속도 감속 ===
        oa_alpha = self._compute_oa_alpha()
        if oa_alpha > 0.0:
            v *= (1.0 - self.oa_v_slow_gain * oa_alpha)

        # === Pure Pursuit 스타일 횡방향 제어: 리더를 lookahead target으로 ===
        # 리더까지의 거리 dist를 기반으로 lookahead 설정
        Ld = max(self.min_lookahead, min(dist, self.max_lookahead))

        Ld_actual = math.hypot(local_x, local_y)
        if Ld_actual < 1e-6:
            curvature_pp = 0.0
        else:
            curvature_pp = 2.0 * local_y / (Ld_actual ** 2)

        angle_to_leader = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_leader - yaw)
        heading_mag = abs(heading_error)

        # heading error가 너무 크면 (거의 반대 방향) 제자리 회전 위주
        if heading_mag > math.radians(self.heading_stop_deg):
            v = 0.0

        omega = v * curvature_pp * self.k_ang

        # OA 조향 편향: 항상 오른쪽(-) 방향으로 회피
        if oa_alpha > 0.0:
            omega += - self.oa_max_omega * oa_alpha

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

        self.get_logger().debug(
            f'dist={dist:.2f}, e_d={e_d:.2f}, v={v:.2f}, w={omega:.2f}, '
            f'heading_err(deg)={math.degrees(heading_error):.1f}, '
            f'oa_d={self.oa_front_dist:.2f}, oa_alpha={oa_alpha:.2f}'
        )

    def _publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PlatoonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down PlatoonFollower node')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
