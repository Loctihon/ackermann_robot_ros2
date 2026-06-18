#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from ultralytics import YOLO

class DetectLineNode(Node):
    def __init__(self):
        super().__init__('detect_line_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ĐÃ CẮT BỎ CV_BRIDGE ĐỂ CỨU JETSON
        
        self.offset_to_center = 260 
        self.one_line_counter = 0
        self.max_one_line_frames = 90 
        
        # ==========================================
        # 🧠 NÃO BỘ YOLO AI (CHẾ ĐỘ TEST NHANH)
        # ==========================================
        self.get_logger().info("🔥 Đang nạp mô hình Test (.pt hoặc .onnx)...")
        
        # Chỉ thẳng đường dẫn tuyệt đối vào file .pt hoặc .onnx của bro
        test_model_path = '/root/rb2_ws/src/robot_2_description/robot_2_description/best.onnx' 
        
        # Nạp mô hình
        self.model = YOLO(test_model_path, task='detect') 
        
        self.id_to_label = {
            0: '50',   1: 'G',    2: 'P',    3: 'R', 
            4: 'SM',   5: 'S',    6: 'TR',   7: 'Y'
        }

    def image_callback(self, msg):
        try:
            # ==========================================
            # 1. ĐỌC ẢNH BẰNG NUMPY (CHỐNG LỖI JETSON)
            # ==========================================
            cv_image = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            h, w, _ = cv_image.shape
            
            # ==========================================
            # 2. CHẠY YOLO TRÊN ẢNH GỐC TOÀN CẢNH
            # ==========================================
            results = self.model(cv_image, imgsz=640, verbose=False, conf=0.25)
            # Lấy ảnh có vẽ sẵn khung Bounding Box của YOLO để tí gửi về Laptop
            annotated_frame = results[0].plot() 
            
            # ==========================================
            # 3. THUẬT TOÁN BÁM LỀ (CONTOUR)
            # ==========================================
            roi_start_y = int(h * 2/3)
            roi = cv_image[roi_start_y:h, 0:w]
            
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [c for c in contours if cv2.contourArea(c) > 500]

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
                        cx = int(M['m10']/M['m00'])
                        contour_centers.append((cx, c))
                
                if len(contour_centers) > 0:
                    contour_centers.sort(key=lambda item: item[0])
                    cx_leftmost, _ = contour_centers[0]
                    center_lane = cx_leftmost + self.offset_to_center

            # TÍNH TOÁN LỆNH CHẠY CƠ BẢN TỪ LÀN ĐƯỜNG
            twist = Twist()
            
            if center_lane is not None:
                # Vẽ chấm đỏ bám làn lên ảnh Full YOLO (Cộng bù Y để khớp tọa độ)
                target_y_full_img = roi_start_y + int((h/3) / 2)
                cv2.circle(annotated_frame, (center_lane, target_y_full_img), 10, (0, 0, 255), -1)
                error = center_lane - (w / 2)
                
                if self.one_line_counter > self.max_one_line_frames:
                    twist.linear.x = 0.2
                    twist.angular.z = -0.5 
                else:
                    twist.angular.z = 0.0 if abs(error) < 15 else float(error) * -0.0015
                    twist.linear.x = 0.3  
            else:
                twist.linear.x = 0.2
                twist.angular.z = -0.4 
                
            # ==========================================
            # 4. YOLO CƯỚP QUYỀN ĐIỀU KHIỂN (GHI ĐÈ)
            # ==========================================
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

            if emergency_stop:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn("🚨 KHẨN CẤP: Gặp Spiderman! Phanh gấp!")
                
            elif primary_label is not None:
                if primary_label in ['R', 'P']:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif primary_label in ['G', 'S']:
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0 # Ép thẳng lái
                elif primary_label == 'Y':
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0
                elif primary_label == '50':
                    twist.linear.x = 0.6
                elif primary_label == 'TR':
                    twist.linear.x = 0.2
                    twist.angular.z = -0.65 

            # Bắn lệnh xuống Node bánh xe
            self.publisher.publish(twist)
            
            # ==========================================
            # 5. ĐÓNG GÓI ẢNH FULL BẮN VỀ LAPTOP
            # ==========================================
            # Vẽ thêm cái sọc xanh phân tách vùng ROI
            cv2.line(annotated_frame, (w//2, roi_start_y), (w//2, h), (255, 0, 0), 2) 
            
            debug_msg = Image()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = "camera_link"
            debug_msg.height = annotated_frame.shape[0]
            debug_msg.width = annotated_frame.shape[1]
            debug_msg.encoding = 'bgr8'
            debug_msg.is_bigendian = 0
            debug_msg.step = annotated_frame.shape[1] * 3
            debug_msg.data = annotated_frame.tobytes()
            
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Lỗi Frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()