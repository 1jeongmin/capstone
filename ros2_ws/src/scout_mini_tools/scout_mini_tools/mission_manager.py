# mission_manager.py
"""
5층 한 바퀴 배달/회수 미션 오케스트레이터.

수행 순서:
  START 출발
    → 523호(waypoint1) 도착 → 정지 → 로봇팔 '배달'(/start_delivery → /delivery_done)
    → 다시 출발
    → 521호(waypoint2) 도착 → 정지 → 로봇팔 '회수'(/start_recover → /recover_done)
    → 다시 출발
    → (이 구간에서 OA 노드가 정적/동적 장애물을 자동 회피)
    → GOAL 도착 → 정지

동작 방식:
  - 로봇 위치는 TF (map -> base_link) 로 추적한다. (path_follower 와 동일한 소스)
  - 정지/재개는 /mission/pause (Bool) 로 path_follower 를 제어한다.
      * e-stop 노드가 /scout_mini/e_stop 을 센서 주기로 계속 덮어쓰기 때문에
        그 토픽을 재사용하지 않고 별도의 pause 토픽을 둔다.
  - 팔 동작은 토픽으로 트리거하고 완료 토픽을 기다린다.
      * 배달: publish /start_delivery(Bool true) → wait /delivery_done(Bool true)
      * 회수: publish /start_recover (Bool true) → wait /recover_done (Bool true)
  - 정지 지점(wp1/wp2)·목적지(goal) 좌표와 도달 허용 오차는 파라미터로 지정한다.

토픽:
  발행: /mission/pause   (Bool)   — true 면 path_follower 정지
  발행: /mission/state   (String) — 현재 미션 상태(모니터링용)
  발행: /start_delivery  (Bool)   — 배달 시퀀스 트리거 (arm_delivery)
  발행: /start_recover   (Bool)   — 회수 시퀀스 트리거 (arm_recover)
  구독: /delivery_done   (Bool)   — 배달 완료 신호
  구독: /recover_done    (Bool)   — 회수 완료 신호
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool, String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


# === 미션 상태 ===
S_DRIVE_TO_WP1 = 'DRIVE_TO_WP1'   # 출발 → 523호로 주행
S_ARM_DELIVERY = 'ARM_DELIVERY'   # 523호 정지, 배달 시퀀스
S_DRIVE_TO_WP2 = 'DRIVE_TO_WP2'   # 523호 → 521호로 주행
S_ARM_RECOVER  = 'ARM_RECOVER'    # 521호 정지, 회수 시퀀스
S_DRIVE_TO_GOAL = 'DRIVE_TO_GOAL'  # 521호 → 목적지로 주행 (이 구간 OA 회피)
S_DONE          = 'DONE'           # 목적지 도착, 미션 종료


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # === 정지 지점 / 목적지 좌표 (map 프레임) ===
        # 기본값은 rviz 에서 클릭한 523호/521호 위치, goal 은 경로 마지막점.
        # 이 좌표는 복도 경로에서 ~1m 떨어진 '방 위치' 라서 정확히 통과하지 않는다.
        # 따라서 '최근접 통과(closest approach)' 로 정지 지점을 판정한다.
        self.declare_parameter('wp1_x', 1.45987)    # 523호 위치 x
        self.declare_parameter('wp1_y', 19.7243)   # 523호 위치 y
        self.declare_parameter('wp2_x', -3.68206)  # 521호 위치 x
        self.declare_parameter('wp2_y', 19.8737)   # 521호 위치 y
        self.declare_parameter('goal_x', 2.86)     # 최종 목적지 x (경로 마지막점)
        self.declare_parameter('goal_y', 4.52)     # 최종 목적지 y

        # 포착 반경(capture radius): 이 반경 안에 들어오면 최근접 추적 시작.
        # 경로~방 최단거리(약 0.9~1.1m)보다 넉넉하게 잡는다.
        self.declare_parameter('wp1_capture', 1.6)
        self.declare_parameter('wp2_capture', 1.6)
        # 최근접 통과 판정 히스테리시스: 최소거리 대비 이만큼 멀어지면 '통과'로 본다.
        self.declare_parameter('approach_hysteresis', 0.15)
        # 목적지는 경로 끝점이라 정확히 도달 → 단순 거리 판정.
        self.declare_parameter('goal_tol', 0.4)

        # TF 프레임 (path_follower 와 동일)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # 팔 트리거 재발행 주기 (완료 신호를 받을 때까지 메시지 유실 대비 반복 발행)
        self.declare_parameter('arm_trigger_period', 2.0)

        self.wp1 = (float(self.get_parameter('wp1_x').value),
                    float(self.get_parameter('wp1_y').value))
        self.wp2 = (float(self.get_parameter('wp2_x').value),
                    float(self.get_parameter('wp2_y').value))
        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value))
        self.wp1_capture = float(self.get_parameter('wp1_capture').value)
        self.wp2_capture = float(self.get_parameter('wp2_capture').value)
        self.approach_hysteresis = float(self.get_parameter('approach_hysteresis').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.arm_trigger_period = float(self.get_parameter('arm_trigger_period').value)

        # === TF ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === 발행 ===
        self.pause_pub = self.create_publisher(Bool, '/mission/pause', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.delivery_trig_pub = self.create_publisher(Bool, '/start_delivery', 10)
        self.recover_trig_pub = self.create_publisher(Bool, '/start_recover', 10)

        # === 구독 ===
        self.create_subscription(Bool, '/delivery_done', self._cb_delivery_done, 10)
        self.create_subscription(Bool, '/recover_done', self._cb_recover_done, 10)

        # === 내부 상태 ===
        self.state = S_DRIVE_TO_WP1
        self.delivery_done = False
        self.recover_done = False
        self._arm_trigger_sent_t = None  # 마지막 트리거 발행 시각(sec)

        # 최근접 통과 추적용
        self._captured = False           # capture 반경 안에 들어왔는가
        self._min_dist = float('inf')    # 추적 중 본 최소 거리

        self.get_logger().info(
            'MissionManager 시작.\n'
            f'  wp1(523호)={self.wp1} capture={self.wp1_capture}\n'
            f'  wp2(521호)={self.wp2} capture={self.wp2_capture}\n'
            f'  goal={self.goal} tol={self.goal_tol}\n'
            f'  approach_hysteresis={self.approach_hysteresis}'
        )

        # 10Hz 미션 루프
        self.timer = self.create_timer(0.1, self.timer_callback)

    # === 콜백 ===
    def _cb_delivery_done(self, msg: Bool):
        if msg.data and self.state == S_ARM_DELIVERY:
            self.get_logger().info('/delivery_done 수신 → 배달 완료')
            self.delivery_done = True

    def _cb_recover_done(self, msg: Bool):
        if msg.data and self.state == S_ARM_RECOVER:
            self.get_logger().info('/recover_done 수신 → 회수 완료')
            self.recover_done = True

    # === 보조 ===
    def _reset_approach(self):
        """새 정지 지점 주행 시작 시 최근접 추적 상태 초기화."""
        self._captured = False
        self._min_dist = float('inf')

    def _passed_closest(self, dist, capture):
        """
        정지 지점 최근접 통과 판정.
        - capture 반경 안에 들어오면 추적 시작(_captured=True)하며 최소거리 갱신.
        - 추적 중 거리가 (최소거리 + 히스테리시스) 보다 커지면 = 최근접점을 지났다 → True.
        """
        if dist <= capture:
            self._captured = True
        if self._captured:
            if dist < self._min_dist:
                self._min_dist = dist
            elif dist > self._min_dist + self.approach_hysteresis:
                return True
        return False

    def _publish_pause(self, value: bool):
        msg = Bool()
        msg.data = value
        self.pause_pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _dist(self, pose, target):
        return math.hypot(pose[0] - target[0], pose[1] - target[1])

    def _trigger_arm_periodic(self, publisher):
        """완료 신호 받을 때까지 트리거 메시지를 주기적으로 재발행."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self._arm_trigger_sent_t is None or
                now - self._arm_trigger_sent_t >= self.arm_trigger_period):
            msg = Bool()
            msg.data = True
            publisher.publish(msg)
            self._arm_trigger_sent_t = now

    # === 메인 루프 ===
    def timer_callback(self):
        self._publish_state()

        if self.state == S_DONE:
            self._publish_pause(True)   # 목적지에서 계속 정지 유지
            return

        pose = self._get_pose()
        if pose is None:
            self.get_logger().warn(
                f'TF ({self.global_frame} -> {self.base_frame}) 대기 중...',
                throttle_duration_sec=2.0)
            # 위치를 모르면 안전하게 정지
            self._publish_pause(True)
            return

        # --- 523호로 주행 ---
        if self.state == S_DRIVE_TO_WP1:
            self._publish_pause(False)
            if self._passed_closest(self._dist(pose, self.wp1), self.wp1_capture):
                self.get_logger().info(
                    f'523호(wp1) 최근접 통과(min={self._min_dist:.2f}m) → 정지, 배달 시작')
                self._publish_pause(True)
                self._arm_trigger_sent_t = None
                self.delivery_done = False
                self.state = S_ARM_DELIVERY
            return

        # --- 523호 정지, 배달 ---
        if self.state == S_ARM_DELIVERY:
            self._publish_pause(True)  # 정지 유지
            if self.delivery_done:
                self.get_logger().info('배달 완료 → 521호로 출발')
                self._reset_approach()
                self.state = S_DRIVE_TO_WP2
            else:
                self._trigger_arm_periodic(self.delivery_trig_pub)
            return

        # --- 521호로 주행 ---
        if self.state == S_DRIVE_TO_WP2:
            self._publish_pause(False)
            if self._passed_closest(self._dist(pose, self.wp2), self.wp2_capture):
                self.get_logger().info(
                    f'521호(wp2) 최근접 통과(min={self._min_dist:.2f}m) → 정지, 회수 시작')
                self._publish_pause(True)
                self._arm_trigger_sent_t = None
                self.recover_done = False
                self.state = S_ARM_RECOVER
            return

        # --- 521호 정지, 회수 ---
        if self.state == S_ARM_RECOVER:
            self._publish_pause(True)  # 정지 유지
            if self.recover_done:
                self.get_logger().info('회수 완료 → 목적지로 출발 (OA 회피 구간)')
                self.state = S_DRIVE_TO_GOAL
            else:
                self._trigger_arm_periodic(self.recover_trig_pub)
            return

        # --- 목적지로 주행 (이 구간에서 OA 노드가 장애물 자동 회피) ---
        if self.state == S_DRIVE_TO_GOAL:
            self._publish_pause(False)
            if self._dist(pose, self.goal) <= self.goal_tol:
                self.get_logger().info('🎉 목적지(goal) 도착 → 미션 종료')
                self._publish_pause(True)
                self.state = S_DONE
            return


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down MissionManager node')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
