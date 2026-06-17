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
        
        self.offset_to_center = 260 
        self.one_line_counter = 0
        # Ngưỡng 30 frames (~1 giây ở 30fps)
        self.max_one_line_frames = 30 
        
        self.get_logger().info("🔥 Node Lane Keeper: Chế độ Ưu tiên Cua Phải!")

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape
        roi = cv_image[int(h * 0.6):h, 0:w]
        roi_h, roi_w, _ = roi.shape
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = sorted([c for c in contours if cv2.contourArea(c) > 500], key=cv2.contourArea, reverse=True)

        center_lane = None
        
        # ==========================================
        # LOGIC ƯU TIÊN: BÁM 2 LỀ -> ĐI GIỮA
        # CHỈ THẤY 1 LỀ TRÁI -> ĐI THẲNG (KHÔNG BÁM)
        # ==========================================
        if len(valid_contours) >= 2:
            self.one_line_counter = 0
            cx1 = int(cv2.moments(valid_contours[0])['m10'] / cv2.moments(valid_contours[0])['m00'])
            cx2 = int(cv2.moments(valid_contours[1])['m10'] / cv2.moments(valid_contours[1])['m00'])
            center_lane = (min(cx1, cx2) + max(cx1, cx2)) // 2
            cv2.drawContours(roi, [valid_contours[0], valid_contours[1]], -1, (0, 255, 0), 2)
            
        elif len(valid_contours) == 1:
            cx = int(cv2.moments(valid_contours[0])['m10'] / cv2.moments(valid_contours[0])['m00'])
            # Nếu thấy lề TRÁI (cx < roi_w/2), ta đi thẳng (center_lane = trung tâm ảnh)
            if cx < (roi_w / 2):
                center_lane = roi_w // 2 
                self.one_line_counter += 1
            else:
                # Nếu thấy lề PHẢI hoặc không thấy lề trái, tăng counter để ép cua
                self.one_line_counter += 1
                center_lane = cx - self.offset_to_center

        # ==========================================
        # ĐIỀU KHIỂN
        # ==========================================
        twist = Twist()
        
        # FAIL-SAFE: Cua phải dứt khoát
        if self.one_line_counter > self.max_one_line_frames:
            twist.linear.x = 0.1
            twist.angular.z = -0.6 # Cua phải
        elif center_lane is not None:
            error = center_lane - (roi_w / 2)
            if abs(error) < 15:
                twist.angular.z = 0.0
            else:
                twist.angular.z = float(error) * -0.002
            twist.linear.x = 0.3
        else:
            twist.linear.x = 0.1
            twist.angular.z = -0.4 # Tìm đường sang phải
            
        self.publisher.publish(twist)
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