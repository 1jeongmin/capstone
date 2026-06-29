#!/usr/bin/env python3
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def rpy_to_quaternion(roll, pitch, yaw):
    """
    roll, pitch, yaw [rad] -> x, y, z, w
    ROS 표준 (X:roll, Y:pitch, Z:yaw) 기준.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def parse_ebimu_line(line: bytes):
    """
    EBRCV24GV6 ASCII 한 줄을 파싱해서 roll/pitch/yaw(deg)를 뽑는다.

    현재 출력 포맷 (네가 보여준 예시 기준):
      EulerAngles + Battery
      예) "100-0,0.36,-0.45,-175.10,100\\r\\n"
          CH-ID, Roll, Pitch, Yaw, Battery
    """

    try:
        text = line.decode("ascii").strip()
    except UnicodeDecodeError:
        return None

    if not text:
        return None

    # 설정 응답("<ok>") 같은 건 무시
    if text.startswith("<") and text.endswith(">"):
        return None

    # 쉼표 기준 split
    parts = text.split(",")
    if len(parts) < 4:
        return None

    # 첫 번째 항목: "CH-ID" (예: "100-0")
    ch_id = parts[0]
    if "-" not in ch_id:
        return None  # 예상 포맷 아니면 버림

    # 둘째, 셋째, 넷째: Roll, Pitch, Yaw (deg)
    try:
        roll_deg = float(parts[1])
        pitch_deg = float(parts[2])
        yaw_deg = float(parts[3])
    except ValueError:
        return None

    # 마지막 값은 배터리(%)인 경우가 많음. 지금은 쓰지 않지만 참고로 파싱
    battery = None
    if len(parts) >= 5:
        try:
            battery = float(parts[-1])
        except ValueError:
            battery = None

    return {
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "battery": battery,
    }


class EbimuImuPublisher(Node):
    def __init__(self):
        super().__init__("ebimu_imu_publisher")

        # 파라미터 선언 (나중에 launch에서 바꾸고 싶으면 사용)
        self.declare_parameter("port", "/dev/ebimu")   # udev에서 만든 심볼릭 링크
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("frame_id", "imu_link")  # URDF에서 만든 imu_link 이름

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # 시리얼 포트 오픈
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
            )
            self.get_logger().info(f"Opened EBIMU receiver on {port} @ {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        # ROS 퍼블리셔
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)

        # 100Hz 정도 (EBMotion 기본 100Hz 출력이면 맞춰줌)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # ★ 지금은 EBMotion에서 이미 설정을 해둔 상태라 굳이 안 건드려도 됨.
        #   나중에 ROS 노드에서 직접 설정하고 싶으면 아래 함수처럼 명령 보내면 됨.
        # self.configure_receiver()

    def configure_receiver(self):
        """
        (선택사항) 수신기 출력 설정을 ROS에서 직접 보내고 싶을 때 사용.
        EBRCV24GV6 매뉴얼의 soc/sof/sog/soa/... 명령 참고.
        """
        cmds = [
            b"<soc1>",  # ASCII 출력
            b"<sof1>",  # Euler Angles mode (Roll,Pitch,Yaw)
            # 아래부터는 나중에 자이로/가속도도 쓰고 싶을 때:
            # b"<sog1>",   # Gyro 출력 ON (x,y,z, 단위: dps)
            # b"<soa2>",   # 가속도 Local, 중력제거 (x,y,z, 단위: g)
            # b"<som0>",   # Magneto OFF
            # b"<sod0>",   # Distance OFF
            # b"<sot0>",   # Temperature OFF
            # b"<sob1>",   # Battery ON
            # b"<sots0>",  # Timestamp OFF
        ]
        for cmd in cmds:
            try:
                self.ser.write(cmd)
                self.ser.flush()
                self.get_logger().info(f"Sent config cmd: {cmd}")
                self.ser.readline()  # "<ok>" 응답 한 줄 버리기
            except serial.SerialException as e:
                self.get_logger().warn(f"Config cmd {cmd} failed: {e}")
                break

    def timer_callback(self):
        try:
            line = self.ser.readline()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            return

        if not line:
            return

        parsed = parse_ebimu_line(line)
        if parsed is None:
            return

        roll_rad = math.radians(parsed["roll_deg"])
        pitch_rad = math.radians(parsed["pitch_deg"])
        yaw_rad = math.radians(parsed["yaw_deg"])

        qx, qy, qz, qw = rpy_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation (쿼터니언)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # orientation_covariance: 0 으로 두면 "covariance unknown" 의미
        msg.orientation_covariance = [0.0] * 9

        # 지금은 자이로/가속도가 없으므로 값은 0, covariance[0] = -1 로 "측정 없음" 표시
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.angular_velocity_covariance = [-1.0] + [0.0] * 8

        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EbimuImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
