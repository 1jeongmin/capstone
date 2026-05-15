import math
import yaml
import os
import csv   # CSV 저장을 위해 추가
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # 파라미터: 최소 간격 (m)
        self.declare_parameter('min_dist', 0.5)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.min_dist = self.get_parameter('min_dist').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # tf2 버퍼 & 리스너 (sim_time 사용하니까 queue_time 크게)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []  # 각 원소: [x, y, yaw]
        self.last_x = None
        self.last_y = None

        # 10 Hz로 pose 체크
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f"WaypointRecorder started. "
            f"min_dist = {self.min_dist} m, "
            f"global_frame = {self.global_frame}, "
            f"base_frame = {self.base_frame}"
        )

    def timer_callback(self):
        try:
            # 최신 transform: map -> base_link
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                now
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = self._quat_to_yaw(transform.transform.rotation)

            if self.last_x is None:
                # 첫 번째 포인트는 무조건 저장
                self._add_waypoint(x, y, yaw)
                self.last_x = x
                self.last_y = y
                return

            dist = math.hypot(x - self.last_x, y - self.last_y)

            if dist >= self.min_dist:
                self._add_waypoint(x, y, yaw)
                self.last_x = x
                self.last_y = y

        except (LookupException, ConnectivityException, ExtrapolationException):
            # tf 아직 준비 안 됐을 때
            self.get_logger().warn(
                f'TF ({self.global_frame} -> {self.base_frame}) 를 아직 가져오지 못했습니다.'
            )
            return

    def _quat_to_yaw(self, q):
        """geometry_msgs/Quaternion -> yaw (rad)"""
        # q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _add_waypoint(self, x, y, yaw):
        # [x, y, yaw] 형태로 저장
        self.waypoints.append([float(x), float(y), float(yaw)])
        self.get_logger().info(
            f"Add waypoint #{len(self.waypoints)-1}: "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

    def save_waypoints(self, filename='waypoints_5152332.yaml'):
        """
        YAML + CSV 두 가지 형식으로 저장
        - YAML : sm_path_follower에서 바로 사용
        - CSV  : MATLAB에서 smoothPathSpline용으로 사용
        """
        waypoint_dir = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws3',
            'src',
            'scout_mini_tools',
            'waypoint'
        )
        os.makedirs(waypoint_dir, exist_ok=True)

        # --- YAML 저장 ---
        full_yaml_path = os.path.join(waypoint_dir, filename)
        data = {'waypoints': self.waypoints}
        with open(full_yaml_path, 'w') as f_yaml:
            yaml.dump(data, f_yaml)

        self.get_logger().info(
            f"Saved {len(self.waypoints)} waypoints to {full_yaml_path}"
        )

        # --- CSV 저장 (같은 이름으로 확장자만 .csv) ---
        base_name, _ = os.path.splitext(filename)
        csv_filename = base_name + '.csv'
        full_csv_path = os.path.join(waypoint_dir, csv_filename)

        with open(full_csv_path, 'w', newline='') as f_csv:
            writer = csv.writer(f_csv)
            # 헤더 (MATLAB에서 readmatrix로 읽을 거면 생략해도 되지만, 보기 좋게 넣자)
            writer.writerow(['x', 'y', 'yaw'])
            for wp in self.waypoints:
                writer.writerow(wp)

        self.get_logger().info(
            f"Also saved CSV waypoints to {full_csv_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 파일로 저장
        node.save_waypoints('waypoints_5152332.yaml')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



###########################################################################################
# import math
# import yaml
# import os
# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time

# from tf2_ros import (Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException)

# class WaypointRecorder(Node):
#     def __init__(self):
#         super().__init__('waypoint_recorder')

#         # 파라미터: 최소 간격 (m)
#         self.declare_parameter('min_dist', 0.5)
#         self.declare_parameter('global_frame', 'map')
#         self.declare_parameter('base_frame', 'base_link')

#         self.min_dist = self.get_parameter('min_dist').value
#         self.global_frame = self.get_parameter('global_frame').value
#         self.base_frame = self.get_parameter('base_frame').value

#         # tf2 버퍼 & 리스너 (sim_time 사용하니까 queue_time 크게)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.waypoints = []
#         self.last_x = None
#         self.last_y = None

#         # 10 Hz로 pose 체크
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.get_logger().info(
#             f"WaypointRecorder started. "
#             f"min_dist = {self.min_dist} m, "
#             f"global_frame = {self.global_frame}, "
#             f"base_frame = {self.base_frame}"
#         )

#     def timer_callback(self):
#         try:
#             # 최신 transform: map -> base_link
#             now = Time()
#             transform = self.tf_buffer.lookup_transform(
#                 self.global_frame,
#                 self.base_frame,
#                 now
#             )

#             x = transform.transform.translation.x
#             y = transform.transform.translation.y
#             yaw = self._quat_to_yaw(transform.transform.rotation)

#             if self.last_x is None:
#                 # 첫 번째 포인트는 무조건 저장
#                 self._add_waypoint(x, y, yaw)
#                 self.last_x = x
#                 self.last_y = y
#                 return

#             dist = math.hypot(x - self.last_x, y - self.last_y)

#             if dist >= self.min_dist:
#                 self._add_waypoint(x, y, yaw)
#                 self.last_x = x
#                 self.last_y = y

#         except (LookupException, ConnectivityException, ExtrapolationException):
#             # tf 아직 준비 안 됐을 때
#             self.get_logger().warn(
#                 f'TF ({self.global_frame} -> {self.base_frame}) 를 아직 가져오지 못했습니다.'
#             )
#             return

#     def _quat_to_yaw(self, q):
#         """geometry_msgs/Quaternion -> yaw (rad)"""
#         # q.x, q.y, q.z, q.w
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     def _add_waypoint(self, x, y, yaw):
#         # [x, y, yaw] 형태로 저장
#         self.waypoints.append([float(x), float(y), float(yaw)])
#         self.get_logger().info(
#             f"Add waypoint #{len(self.waypoints)-1}: "
#             f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
#         )
    
#     def save_waypoints(self, filename='waypoints_floor12_1126.yaml'):
#         waypoint_dir = os.path.join(
#             os.path.expanduser('~'),
#             'ros2_ws',
#             'src',
#             'scout_mini_tools',
#             'waypoint'
#         )
#         os.makedirs(waypoint_dir, exist_ok=True)

#         full_path = os.path.join(waypoint_dir, filename)
        
#         data = {'waypoints': self.waypoints}
#         with open(full_path, 'w') as f:   # full_path 사용
#             yaml.dump(data, f)
#         self.get_logger().info(
#             f"Saved {len(self.waypoints)} waypoints to {full_path}"  # 로그도 full_path
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointRecorder()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 종료 시 파일로 저장
#         node.save_waypoints('waypoints_floor12_1126.yaml')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()