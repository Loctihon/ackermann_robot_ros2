import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import numpy as np


class GazeboLaneViewer(Node):

    def __init__(self):
        super().__init__('gazebo_lane_viewer')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/camera/debug_image',
            10
        )

        # ==========================
        # THÔNG SỐ NHẬN DIỆN
        # ==========================

        self.roi_start_ratio = 0.1

        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

        self.min_contour_area = 500

        self.offset_to_center = 260

        self.get_logger().info(
            "Lane Viewer Started"
        )

    def ros_image_to_cv2(self, msg):

        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img.copy()

        elif msg.encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            return img.copy()

        elif msg.encoding == 'rgba8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 4))
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            return img.copy()

        elif msg.encoding == 'bgra8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 4))
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            return img.copy()

        else:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img.copy()

    def publish_debug_image(self, img):

        debug_msg = Image()

        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "camera_link"

        debug_msg.height = img.shape[0]
        debug_msg.width = img.shape[1]

        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = 0
        debug_msg.step = img.shape[1] * 3
        debug_msg.data = img.tobytes()

        self.debug_pub.publish(debug_msg)

    def image_callback(self, msg):

        try:
            frame = self.ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        h, w, _ = frame.shape

        debug_view = frame.copy()

        roi_y1 = int(h * self.roi_start_ratio)

        roi = debug_view[roi_y1:h, :]

        cv2.rectangle(
            debug_view,
            (0, roi_y1),
            (w - 1, h - 1),
            (0, 255, 255),
            2
        )

        cv2.line(
            debug_view,
            (w // 2, 0),
            (w // 2, h),
            (255, 0, 0),
            2
        )

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        mask = cv2.inRange(
            hsv,
            self.lower_yellow,
            self.upper_yellow
        )

        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid_contours = [
            c for c in contours
            if cv2.contourArea(c) > self.min_contour_area
        ]

        center_lane = None

        if len(valid_contours) > 0:

            contour_centers = []

            for c in valid_contours:

                M = cv2.moments(c)

                if M['m00'] > 0:

                    cx = int(M['m10'] / M['m00'])

                    contour_centers.append(
                        (cx, c)
                    )

            if len(contour_centers) > 0:

                contour_centers.sort(
                    key=lambda x: x[0]
                )

                cx_left, contour = contour_centers[0]

                cv2.drawContours(
                    roi,
                    [contour],
                    -1,
                    (0, 255, 0),
                    3
                )

                center_lane = (
                    cx_left +
                    self.offset_to_center
                )

        if center_lane is not None:

            error = center_lane - (w / 2)

            cv2.circle(
                roi,
                (
                    int(center_lane),
                    roi.shape[0] // 2
                ),
                10,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                debug_view,
                f"LINE DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug_view,
                f"ERROR = {error:.1f}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

        else:

            cv2.putText(
                debug_view,
                "NO LINE",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        self.publish_debug_image(
            np.ascontiguousarray(debug_view)
        )


def main(args=None):

    rclpy.init(args=args)

    node = GazeboLaneViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

