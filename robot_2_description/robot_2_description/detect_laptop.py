#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from ultralytics import YOLO

class LaptopYoloBrain(Node):
    def __init__(self):
        super().__init__('laptop_yolo_brain')
        # Lấy ảnh trực tiếp từ luồng thô của Jetson truyền qua Wi-Fi
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # Phát lệnh ghi đè xuống Jetson
        self.override_pub = self.create_publisher(Twist, '/ai_override_cmd', 10)
        
        self.get_logger().info("🧠 [LAPTOP] Trung vệ AI RTX 3050 đã lên tiếng! Đang quét biển báo...")
        
        # ĐƯỜNG DẪN TỚI FILE .PT TRÊN LAPTOP CỦA BRO
        model_path = '/home/loc/rb2_ws/src/robot_2_description/robot_2_description/best_robo2.pt'
        self.model = YOLO(model_path, task='detect') 
        self.id_to_label = {
            0: '50',   1: 'G',    2: 'P',    3: 'R', 
            4: 'SM',   5: 'S',    6: 'TR',   7: 'Y'
        }

    def image_callback(self, msg):
        try:
            # Đọc ảnh từ sóng Wi-Fi
            cv_image = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Quật YOLO hết công suất bằng GPU Laptop
            results = self.model(cv_image, verbose=False, conf=0.25)
            annotated_frame = results[0].plot() 
            
            max_area = 0
            primary_label = None
            emergency_stop = False

            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    class_id = int(box.cls[0].item())
                    if class_id in self.id_to_label:
                        label = self.id_to_label[class_id]
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        area = (x2 - x1) * (y2 - y1)
                        
                        if label == 'SM':
                            emergency_stop = True
                            break
                        if area > max_area:
                            max_area = area
                            primary_label = label

            # Nếu phát hiện biển báo, bắn lệnh Twist qua Wi-Fi xuống Jetson
            if emergency_stop or primary_label is not None:
                twist = Twist()
                if emergency_stop:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().warn("🚨 LỆNH TỪ LAPTOP: Gặp Spiderman! Phanh khẩn cấp!")
                elif primary_label in ['R', 'P']:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif primary_label in ['G', 'S']:
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0 
                elif primary_label == 'Y':
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0
                elif primary_label == '50':
                    twist.linear.x = 0.6
                elif primary_label == 'TR':
                    twist.linear.x = 0.2
                    twist.angular.z = -0.65 
                    self.get_logger().info("↪️ LỆNH TỪ LAPTOP: Bẻ lái sang phải!")

                self.override_pub.publish(twist)
            
            # Mở cửa sổ xem trực tiếp kết quả YOLO trên Laptop
            cv2.imshow("Laptop AI View", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Lỗi: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaptopYoloBrain()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()