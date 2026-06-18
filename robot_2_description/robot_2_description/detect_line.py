import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# Đã cắt bỏ cv_bridge để chống lỗi SystemError trên Jetson
import cv2
import numpy as np

class GazeboLaneKeeper(Node):
    def __init__(self):
        super().__init__('gazebo_lane_keeper')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Đã xóa self.br = CvBridge()
        
        self.offset_to_center = 260 
        
        # ==========================================
        # 🔥 ĐÃ KHÔI PHỤC: BỘ ĐẾM ÉP CUA
        # ==========================================
        self.one_line_counter = 0
        self.max_one_line_frames = 90 # Hạ xuống 30 frame (~1 giây) để cua cho lẹ
        
        self.get_logger().info("🔥 Node Bám Lề Trái + Fail-safe Cua Phải đã sẵn sàng (No cv_bridge)!")

    def image_callback(self, msg):
        # ==========================================
        # ĐỌC ẢNH TRỰC TIẾP BẰNG NUMPY (Nhanh & Không crash)
        # ==========================================
        cv_image = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
        # Convert từ chuẩn màu ROS sang OpenCV
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        h, w, _ = cv_image.shape
        
        # Cắt ROI sát mũi xe để chống cua sớm
        roi = cv_image[int(h * 2/3):h, 0:w]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > 500]

        center_lane = None
        roi_mid_h = int((h/3) / 2) # Tính lại tọa độ Y để vẽ chấm đỏ cho khớp ROI

        # ĐẾM SỐ LƯỢNG VẠCH ĐỂ KÍCH HOẠT FAIL-SAFE
        if len(valid_contours) >= 2:
            self.one_line_counter = 0 # Thấy 2 vạch -> Đường thẳng -> Reset đếm
        elif len(valid_contours) == 1:
            self.one_line_counter += 1 # Chỉ thấy 1 vạch (Vào ngã tư/cua) -> Cộng dồn
        else:
            self.one_line_counter = 0 # Mù tịt -> Reset đếm để tránh lỗi

        if len(valid_contours) > 0:
            # VẪN GIỮ TUYỆT CHIÊU: CHỈ LẤY VẠCH TRÁI NHẤT
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
                
                # Tâm ảo: Lấy vạch trái đẩy vô giữa
                center_lane = cx_leftmost + self.offset_to_center

        twist = Twist()
        
        if center_lane is not None:
            cv2.circle(roi, (center_lane, roi_mid_h), 10, (0, 0, 255), -1)
            error = center_lane - (w / 2)
            
            # ==========================================
            # KIỂM TRA FAIL-SAFE ÉP CUA
            # ==========================================
            if self.one_line_counter > self.max_one_line_frames:
                twist.linear.x = 0.2
                # Số ÂM là cua PHẢI. Càng âm càng cua gắt.
                twist.angular.z = -0.5 
                self.get_logger().warn(f"Chỉ thấy 1 line quá {self.max_one_line_frames} frames! Đang CUA PHẢI...")
            else:
                # BÁM LINE BÌNH THƯỜNG
                if abs(error) < 15:
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = float(error) * -0.0015
                twist.linear.x = 0.3  
            
        else:
            # Mù hoàn toàn -> Lượn vòng tìm line (Xoay sang Phải)
            twist.linear.x = 0.2
            twist.angular.z = -0.4 
            self.get_logger().warn("Mất sạch vạch! Xoay vòng qua PHẢI tìm lại...")
            
        self.publisher.publish(twist)
        
        cv2.line(roi, (w//2, 0), (w//2, int(h/3)), (255, 0, 0), 2) 
        
        # ==========================================
        # ĐÓNG GÓI ẢNH THỦ CÔNG GỬI VỀ LAPTOP
        # ==========================================
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