import cv2
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer

class Camera(Node):

    def __init__(self):
        super().__init__('camera')

        self._bridge = CvBridge()

        self._rgb_subscriber = Subscriber(self, Image,
            '/goalkeeper/camera/image',
            qos_profile=qos_profile_sensor_data
        )

        self._depth_subscriber = Subscriber(self, Image,
            '/goalkeeper/camera/depth_image',
            qos_profile=qos_profile_sensor_data
        )

        # Synchronize rgb camera and depth camera by timestamp
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size=5,
            slop=0.01
        )

        self._sync.registerCallback(self._image_callback)
        self.get_logger().info('Camera started')

        self._reference_depth = None
        self._depth_threshold = 0.1  # meters

    def _image_callback(self, rgb_msg, depth_msg):

        frame = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8').copy()
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        mask = self._get_foreground_mask(depth)
        detection = self._depth_ball_detect(mask)
        #cv2.imshow('Depth Foreground Mask', mask)

        if detection is not None:
            x, y, w, h = detection
            u = x + w // 2
            v = y + h // 2
            z = self._get_ball_depth(depth, u, v)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if z is not None:
                label = f'Ball: {z:.2f} m'
            else:
                label = 'Ball: invalid depth'

            cv2.putText(frame,
                label,
                (x, max(y - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 0),
                1
            ) 
        cv2.imshow('Goalkeeper Camera', frame)
        cv2.waitKey(1)

    def _hsv_ball_detect(self, frame):

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
    
    def _depth_ball_detect(self, mask):

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
    
    def _get_ball_depth(self, depth, u, v):

        h, w = depth.shape

        # Extract a 3x3 region around the ball center
        region = depth[
            max(0, v - 1):min(h, v + 2),
            max(0, u - 1):min(w, u + 2)
        ]

        # Remove invalid depth values
        valid = region[np.isfinite(region) & (region > 0)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def _get_foreground_mask(self, depth):

        # Capture the first depth frame as background
        if self._reference_depth is None:
            self._reference_depth = depth.copy()

            return np.zeros(depth.shape, dtype=np.uint8)

        reference = self._reference_depth

        # Valid depth measurements
        valid_current = np.isfinite(depth) & (depth > 0)
        valid_reference = np.isfinite(reference) & (reference > 0)

        # Case 1: Both current and reference have valid depth
        both_valid = valid_current & valid_reference

        foreground = np.zeros(depth.shape, dtype=bool)

        foreground[both_valid] = (
            reference[both_valid] - depth[both_valid] > self._depth_threshold
        )

        # Case 2: An object appears where there was no surface
        foreground |= valid_current & ~valid_reference

        # Convert boolean array into OpenCV binary mask
        mask = foreground.astype(np.uint8) * 255
        return mask
    
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