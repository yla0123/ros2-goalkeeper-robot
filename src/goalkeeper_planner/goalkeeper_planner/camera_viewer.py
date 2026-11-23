import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self._bridge = CvBridge()

        self._image_subscriber = self.create_subscription(Image, '/goalkeeper/camera/image',       self._image_callback, qos_profile_sensor_data)

        self.get_logger().info('Camera viewer started')

    def _image_callback(self, msg):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow('Goalkeeper Camera', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()