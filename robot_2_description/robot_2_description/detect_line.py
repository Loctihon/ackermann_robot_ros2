#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class GazeboLaneKeeper(Node):
    def __init__(self):
        super().__init__('gazebo_lane_keeper')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # 🔥 THÊM VÀO: Lắng nghe lệnh AI từ Laptop truyền về
        self.ai_sub = self.create_subscription(Twist, '/ai_override_cmd', self.ai_callback, 10)
        
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.offset_to_center = 260 
        self.one_line_counter = 0
        self.max_one_line_frames = 90 
        
        # 🔥 THÊM VÀO: Biến cờ cướp quyền vô lăng
        self.ai_override_active = False
        self.ai_twist = Twist()
        self.last_ai_time = self.get_clock().now()
        
        self.get_logger().info("🔥 Node Bám Lề + FAIL-SAFE + LẮNG NGHE ĐẠI NÃO đã sẵn sàng!")

    def ai_callback(self, msg):
        # Khi Laptop thấy biển báo, nó gửi lệnh Twist xuống đây
        self.ai_twist = msg
        self.ai_override_active = True
        self.last_ai_time = self.get_clock().now()

    def image_callback(self, msg):
        # ==========================================
        # 1. KIỂM TRA QUYỀN ĐIỀU KHIỂN CỦA LAPTOP (AI OVERRIDE)
        # ==========================================
        current_time = self.get_clock().now()
        # Nếu trong 1 giây qua không có lệnh AI nào mới -> Hết hiệu lực biển báo -> Tự lái tiếp
        if self.ai_override_active and (current_time - self.last_ai_time).nanoseconds > 1e9:
            self.ai_override_active = False

        if self.ai_override_active:
            # LAPTOP ĐANG CƯỚP QUYỀN! Bắn thẳng lệnh của Laptop xuống bánh xe
            self.publisher.publish(self.ai_twist)
            # THOÁT HÀM LUÔN, KHÔNG THÈM TÍNH TOÁN BÁM LINE NỮA
            return 
            
        # ==========================================
        # 2. CHẠY BÁM LINE BÌNH THƯỜNG (Nếu Laptop im lặng)
        # ==========================================
        cv_image = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        h, w, _ = cv_image.shape
        roi = cv_image[int(h * 2/3):h, 0:w]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > 500]

        center_lane = None
        roi_mid_h = int((h/3) / 2) 

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
                    cx = int(M['m10']/M['m00'])
                    contour_centers.append((cx, c))
            
            if len(contour_centers) > 0:
                contour_centers.sort(key=lambda item: item[0])
                cx_leftmost, best_contour = contour_centers[0]
                cv2.drawContours(roi, [best_contour], -1, (0, 255, 0), 3)
                center_lane = cx_leftmost + self.offset_to_center

        twist = Twist()
        
        if center_lane is not None:
            cv2.circle(roi, (center_lane, roi_mid_h), 10, (0, 0, 255), -1)
            error = center_lane - (w / 2)
            
            if self.one_line_counter > self.max_one_line_frames:
                twist.linear.x = 0.2
                twist.angular.z = -0.5 
            else:
                if abs(error) < 15:
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = float(error) * -0.0015
                twist.linear.x = 0.3  
        else:
            twist.linear.x = 0.2
            twist.angular.z = -0.4 
            
        self.publisher.publish(twist)
        
        # Gửi ảnh Debug
        cv2.line(roi, (w//2, 0), (w//2, int(h/3)), (255, 0, 0), 2) 
        debug_msg = Image()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "camera_link"
        debug_msg.height = roi.shape[0]
        debug_msg.width = roi.shape[1]
        debug_msg.encoding = 'bgr8'
        debug_msg.is_bigendian = 0
        debug_msg.step = roi.shape[1] * 3
        debug_msg.data = roi.tobytes()
        self.debug_pub.publish(debug_msg)

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