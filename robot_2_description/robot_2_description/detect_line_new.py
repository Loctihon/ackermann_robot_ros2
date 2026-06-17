import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class GazeboLaneKeeper(Node):
    def __init__(self):
        super().__init__('gazebo_lane_keeper')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.br = CvBridge()
        
        self.offset_to_center = 150 
        self.one_line_counter = 0
        self.max_one_line_frames = 60 
        
        self.get_logger().info("🚀 Node Lane Keeper: Thuật toán HoughLines đã được chống nhiễu & Smoothing!")

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape
        roi = cv_image[int(h * 0.5):h, 0:w]
        roi_h, roi_w, _ = roi.shape
        
        # 1. BỘ LỌC MÀU
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
        mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 30, 255]))
        mask = cv2.bitwise_or(mask_yellow, mask_white)

        # 2. CẠNH CANNY & HOUGH LINES
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=40, maxLineGap=10)

        left_lines = []
        right_lines = []

        # 3. LỌC NHIỄU & TÍNH TOÁN PHƯƠNG TRÌNH ĐƯỜNG THẲNG
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 == x2: continue # Bỏ qua đường thẳng đứng tuyệt đối
                
                slope = (y2 - y1) / (x2 - x1)
                
                # FILTER 1: Bỏ qua đường kẻ ngang (Crosswalk hoặc nhiễu)
                if abs(slope) < 0.5:
                    continue
                    
                intercept = y1 - slope * x1
                
                # FILTER 2: Phân loại Trái/Phải bằng Slope và Không gian
                if slope < -0.5 and x1 < roi_w * 0.6 and x2 < roi_w * 0.6:
                    left_lines.append((slope, intercept))
                elif slope > 0.5 and x1 > roi_w * 0.4 and x2 > roi_w * 0.4:
                    right_lines.append((slope, intercept))

        # 4. TRUNG BÌNH HÓA (SMOOTHING) ĐỂ TÌM TÂM CHUẨN
        target_y = roi_h // 2 # Chúng ta muốn tính tọa độ X ngay tại trục ngang giữa màn hình
        left_x_target = None
        right_x_target = None

        if len(left_lines) > 0:
            avg_slope, avg_intercept = np.mean(left_lines, axis=0)
            left_x_target = int((target_y - avg_intercept) / avg_slope)
            # Vẽ đường Line Đại Diện (Xanh Dương) để debug
            cv2.line(roi, (int((roi_h - avg_intercept)/avg_slope), roi_h), 
                          (int((0 - avg_intercept)/avg_slope), 0), (255, 0, 0), 4)

        if len(right_lines) > 0:
            avg_slope, avg_intercept = np.mean(right_lines, axis=0)
            right_x_target = int((target_y - avg_intercept) / avg_slope)
            # Vẽ đường Line Đại Diện (Xanh Lục) để debug
            cv2.line(roi, (int((roi_h - avg_intercept)/avg_slope), roi_h), 
                          (int((0 - avg_intercept)/avg_slope), 0), (0, 255, 0), 4)

        # 5. LOGIC NÉ TRÁI - BÁM PHẢI
        center_lane = None
        
        if left_x_target is not None and right_x_target is not None:
            self.one_line_counter = 0
            road_width = right_x_target - left_x_target
            self.offset_to_center = road_width // 2
            
            # Ép xe đi lệch sang phải (cách lề phải 35% mặt đường)
            center_lane = right_x_target - int(road_width * 0.35)
            
        elif left_x_target is not None:
            self.one_line_counter += 1
            # Ép xe né sang phải dựa vào lề trái
            center_lane = left_x_target + int(self.offset_to_center * 1.3)
            
        elif right_x_target is not None:
            self.one_line_counter = 0 
            # Bám cua phải an toàn
            center_lane = right_x_target - int(self.offset_to_center * 0.7)
            
        else:
            self.one_line_counter += 1

        # ==========================================
        # BƠM LỆNH XUỐNG BÁNH XE
        # ==========================================
        twist = Twist()
        
        if self.one_line_counter > self.max_one_line_frames:
            twist.linear.x = 0.2
            twist.angular.z = -0.65 # Ép cua phải ngã tư
            
        elif center_lane is not None:
            # Chấm đỏ là hồng tâm xe nhắm tới
            cv2.circle(roi, (center_lane, target_y), 8, (0, 0, 255), -1)
            error = center_lane - (roi_w / 2)
            
            if abs(error) < 15:
                twist.angular.z = 0.0
            else:
                twist.angular.z = float(error) * -0.0015
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.1
            twist.angular.z = -0.4 
            
        self.publisher.publish(twist)
        
        cv2.line(roi, (roi_w//2, 0), (roi_w//2, roi_h), (255, 0, 0), 2)
        cv2.imshow("Mat Than (Canny Edges)", edges)
        cv2.imshow("Goc nhin AI", roi)
        cv2.waitKey(1)

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