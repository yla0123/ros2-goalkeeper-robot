import cv2
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Camera(Node):

    def __init__(self):
        super().__init__('camera')

        self._bridge = CvBridge()

        self._image_subscriber = self.create_subscription(Image, '/goalkeeper/camera/image',       self._image_callback, qos_profile_sensor_data)

        self.get_logger().info('Camera started')

    def _image_callback(self, msg):

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
        detection = self._detect_ball(frame)

        if detection is not None:

            x, y, w, h = detection
            u = x + w // 2
            v = y + h // 2

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow('Goalkeeper Camera', frame)
        cv2.waitKey(1)

    def _detect_ball(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White masks
        lower = (0, 0, 110)
        upper = (179, 60, 255)
        mask = cv2.inRange(hsv, lower, upper)
        #cv2.imshow('White Mask', mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []

        for contour in contours:
            # Area filtering
            area = cv2.contourArea(contour)
            if area < 3:
                continue

            # Aspect ratio filtering
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h

            # the detect area should look like square
            if not 0.6 <= aspect_ratio <= 1.6:
                continue

            # Circularity filtering
            perimeter = cv2.arcLength(contour, True)

            if perimeter == 0:
                continue

            circularity = 4 * math.pi * area / perimeter**2

            if circularity < 0.5:
                continue

            candidates.append(contour)

        if not candidates:
            return None

        ball = max(candidates, key=cv2.contourArea)

        return cv2.boundingRect(ball)
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Camera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()