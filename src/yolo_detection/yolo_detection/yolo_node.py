import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        self.declare_parameter('model', 'yolov8m.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda')

        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.conf = self.get_parameter('confidence').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model loaded. Running on: {self.device}')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(Image, '/yolo/detection_image', 10)

        self.get_logger().info('YOLO Detection Node started.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(cv_image, conf=self.conf, device=self.device, verbose=False)

        annotated = results[0].plot()

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
