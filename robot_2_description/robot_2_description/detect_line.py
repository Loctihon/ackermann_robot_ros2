import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np


class GazeboLaneKeeper(Node):
    def __init__(self):
        super().__init__('gazebo_lane_keeper')

        # Nhận ảnh camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Lắng nghe lệnh AI từ laptop truyền về
        self.ai_sub = self.create_subscription(
            Twist,
            '/ai_override_cmd',
            self.ai_callback,
            10
        )

        # Publish ảnh debug để xem trên RViz/laptop
        self.debug_pub = self.create_publisher(
            Image,
            '/camera/debug_image',
            10
        )

        # Publish lệnh chạy xe
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================
        # THÔNG SỐ TINH CHỈNH THỊ GIÁC
        # ==========================

        # Lấy vùng nhìn line: 2/3 dưới ảnh
        # Nếu muốn nhìn xa hơn: đổi thành 0.5
        # Nếu muốn nhìn sát đầu xe hơn: đổi thành 0.75
        self.roi_start_ratio = 2 / 3

        # Dải màu vàng trong HSV
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

        # Lọc nhiễu contour
        self.min_contour_area = 500

        # Offset từ line trái sang tâm làn
        self.offset_to_center = 260

        # Đếm số frame chỉ thấy 1 line
        self.one_line_counter = 0
        self.max_one_line_frames = 90

        # Hệ số đánh lái
        self.steer_kp = -0.0015

        # Ngưỡng cho phép đi thẳng
        self.center_deadband = 15

        # Tốc độ chạy bình thường
        self.normal_speed = 0.3

        # Tốc độ khi mất line / fail-safe
        self.failsafe_speed = 0.2

        # ==========================
        # AI OVERRIDE
        # ==========================

        self.ai_override_active = False
        self.ai_twist = Twist()
        self.last_ai_time = self.get_clock().now()

        self.get_logger().info(
            "Node bám line + debug full ảnh + AI override đã khởi động."
        )

    def ai_callback(self, msg):
        self.ai_twist = msg
        self.ai_override_active = True
        self.last_ai_time = self.get_clock().now()

    def ros_image_to_cv2(self, msg):
        """
        Chuyển sensor_msgs/Image sang OpenCV BGR.
        Không dùng cv_bridge để tránh thiếu thư viện.
        """

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
            self.get_logger().warn(
                f"Encoding camera chưa hỗ trợ rõ: {msg.encoding}. Thử đọc như rgb8."
            )
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img.copy()

    def publish_debug_image(self, debug_view):
        debug_view = np.ascontiguousarray(debug_view)

        debug_msg = Image()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "camera_link"
        debug_msg.height = debug_view.shape[0]
        debug_msg.width = debug_view.shape[1]
        debug_msg.encoding = 'bgr8'
        debug_msg.is_bigendian = 0
        debug_msg.step = debug_view.shape[1] * 3
        debug_msg.data = debug_view.tobytes()

        self.debug_pub.publish(debug_msg)

    def image_callback(self, msg):
        # ==========================================
        # 1. KIỂM TRA AI OVERRIDE
        # ==========================================

        current_time = self.get_clock().now()

        if self.ai_override_active:
            dt_ns = (current_time - self.last_ai_time).nanoseconds

            # Sau 1 giây không có lệnh AI mới thì nhả quyền lại cho bám line
            if dt_ns > 1e9:
                self.ai_override_active = False

        if self.ai_override_active:
            self.publisher.publish(self.ai_twist)
            return

        # ==========================================
        # 2. NHẬN ẢNH CAMERA
        # ==========================================

        try:
            cv_image = self.ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Lỗi chuyển ảnh ROS sang OpenCV: {e}")
            return

        h, w, _ = cv_image.shape

        # debug_view là ảnh full camera để gửi về laptop
        debug_view = cv_image.copy()

        # ==========================================
        # 3. CẮT ROI ĐỂ XỬ LÝ, NHƯNG DEBUG VẪN FULL ẢNH
        # ==========================================

        roi_y1 = int(h * self.roi_start_ratio)
        roi_y2 = h

        roi = debug_view[roi_y1:roi_y2, 0:w]

        roi_h = roi.shape[0]
        roi_mid_h = roi_h // 2

        # Vẽ khung ROI lên ảnh full
        cv2.rectangle(
            debug_view,
            (0, roi_y1),
            (w - 1, roi_y2 - 1),
            (0, 255, 255),
            2
        )

        # Vẽ đường tâm ảnh full
        cv2.line(
            debug_view,
            (w // 2, 0),
            (w // 2, h),
            (255, 0, 0),
            2
        )

        # ==========================================
        # 4. XỬ LÝ MÀU VÀNG
        # ==========================================

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(
            hsv,
            self.lower_yellow,
            self.upper_yellow
        )

        # Làm sạch nhiễu nhẹ
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid_contours = [
            c for c in contours
            if cv2.contourArea(c) > self.min_contour_area
        ]

        # ==========================================
        # 5. TÍNH TÂM LÀN
        # ==========================================

        center_lane = None

        if len(valid_contours) >= 2:
            self.one_line_counter = 0
        elif len(valid_contours) == 1:
            self.one_line_counter += 1
        else:
            self.one_line_counter = 0

        if len(valid_contours) > 0:
            contour_centers = []

            for c in valid_contours:
                M = cv2.moments(c)

                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    contour_centers.append((cx, c))

            if len(contour_centers) > 0:
                contour_centers.sort(key=lambda item: item[0])

                # Lấy line ngoài cùng bên trái
                cx_leftmost, best_contour = contour_centers[0]

                # Vẽ contour lên ROI, vì ROI là view của debug_view nên sẽ hiện trên ảnh full
                cv2.drawContours(
                    roi,
                    [best_contour],
                    -1,
                    (0, 255, 0),
                    3
                )

                center_lane = cx_leftmost + self.offset_to_center

        # ==========================================
        # 6. TẠO LỆNH ĐIỀU KHIỂN
        # ==========================================

        twist = Twist()

        if center_lane is not None:
            # Vẽ tâm làn trong ROI
            cv2.circle(
                roi,
                (int(center_lane), roi_mid_h),
                10,
                (0, 0, 255),
                -1
            )

            error = center_lane - (w / 2)

            # Ghi thông tin lên ảnh debug
            cv2.putText(
                debug_view,
                f"error: {error:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug_view,
                f"contours: {len(valid_contours)}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )

            if self.one_line_counter > self.max_one_line_frames:
                twist.linear.x = self.failsafe_speed
                twist.angular.z = -0.5

                cv2.putText(
                    debug_view,
                    "MODE: ONE LINE FAILSAFE",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2
                )
            else:
                if abs(error) < self.center_deadband:
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = float(error) * self.steer_kp

                twist.linear.x = self.normal_speed

                cv2.putText(
                    debug_view,
                    "MODE: LINE KEEPING",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2
                )

        else:
            # Không thấy line thì cho chạy chậm và cua tìm line
            twist.linear.x = self.failsafe_speed
            twist.angular.z = -0.4

            cv2.putText(
                debug_view,
                "MODE: NO LINE SEARCH",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2
            )

        # ==========================================
        # 7. PUBLISH LỆNH VÀ DEBUG IMAGE
        # ==========================================

        self.publisher.publish(twist)

        cv2.putText(
            debug_view,
            f"vx: {twist.linear.x:.2f}  wz: {twist.angular.z:.2f}",
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2
        )

        self.publish_debug_image(debug_view)


def main(args=None):
    rclpy.init(args=args)

    node = GazeboLaneKeeper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
