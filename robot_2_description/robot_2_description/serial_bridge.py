#!/usr/bin/env python3
import rclcpp
from rclcpp.node import Node
from std_msgs.msg import Float64MultiArray
import socket
import time

class BluetoothBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Địa chỉ MAC của con HC-05 của bro
        self.bd_addr = "00:23:11:A0:38:24"
        self.port = 1 # Kênh RFCOMM mặc định
        
        self.sock = None
        self.connect_to_hc05()

        # Khởi tạo giá trị ban đầu an toàn
        self.v_left = 0.0
        self.v_right = 0.0
        self.steer = 0.0

        # Đăng ký nhận dữ liệu từ các Topic đầu ra của node Ackermann C++
        self.sub_pos = self.create_subscription(Float64MultiArray, '/forward_position_controller/commands', self.pos_cb, 10)
        self.sub_vel = self.create_subscription(Float64MultiArray, '/forward_velocity_controller/commands', self.vel_cb, 10)
        
        # Vòng lặp gửi dữ liệu định kỳ 50Hz (20ms/lần) khớp hoàn toàn với update_rate của STM32
        self.timer = self.create_timer(0.02, self.send_to_stm32)

    def connect_to_hc05(self):
        try:
            self.get_logger().info(f"Đang kết nối Socket tới HC-05 [{self.bd_addr}]...")
            self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.sock.connect((self.bd_addr, self.port))
            self.get_logger().info("🔥 KẾT NỐI BLUETOOTH THÀNH CÔNG! Thao tác xe sẵn sàng.")
        except Exception as e:
            self.get_logger().error(f"Kết nối thất bại: {e}. Đang thử lại sau 2 giây...")
            time.sleep(2)
            self.connect_to_hc05()

    def pos_cb(self, msg):
        if len(msg.data) > 0:
            self.steer = msg.data[0] # Lấy góc lái (Radian) của bánh trước từ hệ Ackermann

    def vel_cb(self, msg):
        if len(msg.data) >= 2:
            self.v_left = msg.data[0]  # Vận tốc góc bánh sau TRÁI (rad/s)
            self.v_right = msg.data[1] # Vận tốc góc bánh sau PHẢI (rad/s)

    def send_to_stm32(self):
        if self.sock is None:
            return
            
        # KHỚP LOGIC STM32: Tính vận tốc trung bình tổng của xe từ hệ vi sai ảo
        v_mean = (self.v_left + self.v_right) / 2.0
        
        # 1. Quy đổi Vận tốc góc -> Giá trị điều khiển thô (-255 đến 255) giống hệt file C++ cũ
        target_v_stm = int(v_mean * 15.0)
        target_v_stm = max(-255, min(255, target_v_stm)) # Ép dải thô giới hạn MAX_TARGET_VELOCITY

        # 2. Quy đổi Góc bẻ lái Radian -> Xung PWM Servo (1000 đến 2000, giữa là 1500)
        target_servo_stm = 1500 + int(self.steer * 400.0)
        target_servo_stm = max(1000, min(2000, target_servo_stm))

        # Đóng gói chuỗi String khít khịt với hàm sscanf(parse_buffer, "V:%d,S:%d") dưới STM32
        tx_str = f"V:{target_v_stm},S:{target_servo_stm}\n"
        
        try:
            # Bắn thẳng dữ liệu qua Bluetooth Socket
            self.sock.send(tx_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Mất kết nối với mạch: {e}. Đang tự động kết nối lại...")
            self.connect_to_hc05()

def main(args=None):
    rclcpp.init(args=args)
    node = BluetoothBridge()
    try:
        rclcpp.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock:
            node.sock.close()
        node.destroy_node()
        rclcpp.shutdown()

if __name__ == '__main__':
    main()