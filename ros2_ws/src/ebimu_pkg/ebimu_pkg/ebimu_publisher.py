# When the message "Permission denied: /dev/ttyx" appears,
# change the permission settings on your serial_port
# eg. "sudo chmod 666 /dev/ttyUSB0"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

COM_PORT = "/dev/ebimu"
BAUDRATE = 921600

try:
    ser = serial.Serial(port=COM_PORT, baudrate=BAUDRATE)
except Exception as e:
    print("Serial port error:", e)
    ser = None


class EbimuPublisher(Node):

    def __init__(self):
        super().__init__("ebimu_publisher")
        qos_profile = QoSProfile(depth=10)

        self.publisher = self.create_publisher(String, "ebimu_data", qos_profile)
        timer_period = 0.0005
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # 시리얼이 안 열려 있으면 그냥 리턴
        if ser is None or not ser.is_open:
            self.get_logger().warn("Serial port not available")
            return

        msg = String()
        ser_data = ser.readline()
        msg.data = ser_data.decode("utf-8", errors="ignore")
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    print("Starting ebimu_publisher..")

    node = EbimuPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
