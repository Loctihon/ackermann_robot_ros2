#include <chrono>
#include <string>
#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
// Thư viện serial thuần của Linux để nói chuyện với STM32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge() : Node("serial_bridge")
  {
    // Khởi tạo giá trị ban đầu an toàn
    latest_v_left_ = 0.0;
    latest_v_right_ = 0.0;
    latest_steer_ = 0.0;

    // 1. Cấu hình cổng Serial kết nối STM32
    serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Không thể mở cổng Serial /dev/ttyUSB0! Kiểm tra dây cáp.");
    } else {
      struct termios options;
      tcgetattr(serial_port_, &options);
      cfsetispeed(&options, B115200); // Tốc độ baudrate 115200 trùng với USART1 STM32
      cfsetospeed(&options, B115200);
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      
      // Cấu hình chế độ Non-blocking cho cổng Serial
      options.c_cc[VMIN] = 0;
      options.c_cc[VTIME] = 0;
      
      tcsetattr(serial_port_, TCSANOW, &options);
      RCLCPP_INFO(this->get_logger(), "Kết nối thành công tới STM32 tại cổng /dev/ttyUSB0");
    }

    // 2. Đăng ký nhận dữ liệu đã tính toán từ node Động học Ackermann
    sub_pos_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10, std::bind(&SerialBridge::pos_pos_cb, this, std::placeholders::_1));
    
    sub_vel_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10, std::bind(&SerialBridge::vel_vel_cb, this, std::placeholders::_1));

    // Vòng lặp gửi data xuống STM32 chu kỳ 20ms (50Hz) cho mượt
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SerialBridge::send_to_stm32, this));
  }

  ~SerialBridge() {
    if (serial_port_ >= 0) {
      close(serial_port_);
      RCLCPP_INFO(this->get_logger(), "Đã đóng cổng Serial an toàn.");
    }
  }

private:
  void pos_pos_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if(!msg->data.empty()) {
      latest_steer_ = msg->data[0]; // Lấy góc lái (Radian) của bánh trước
    }
  }

  void vel_vel_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Ép điều kiện kiểm tra mảng phải có đủ ít nhất 2 phần tử bánh sau
    if(msg->data.size() >= 2) {
      latest_v_left_ = msg->data[0];  // Vận tốc góc bánh sau TRÁI (rad/s)
      latest_v_right_ = msg->data[1]; // Vận tốc góc bánh sau PHẢI (rad/s)
    }
  }

  void send_to_stm32()
  {
    if (serial_port_ < 0) return;

    // --- SỬA LỖI VI SAI: Tính vận tốc trung bình thực tế của xe ---
    double v_mean = (latest_v_left_ + latest_v_right_) / 2.0;

    // --- QUY ĐỔI ĐƠN VỊ TỪ ROS 2 SANG STM32 ---
    
    // 1. Quy đổi Vận tốc góc trung bình (rad/s) -> Giá trị điều khiển motor thô từ -255 đến 255
    int target_v_stm = static_cast<int>(v_mean * 15.0); 
    target_v_stm = std::clamp(target_v_stm, -255, 255);

    // 2. Quy đổi Góc bẻ lái Radian -> Xung PWM Servo (1000 đến 2000, giữa là 1500)
    int target_servo_stm = 1500 + static_cast<int>(latest_steer_ * 400.0);
    target_servo_stm = std::clamp(target_servo_stm, 1000, 2000);

    // Đóng gói thành chuỗi String đúng định dạng STM32 đang đợi đọc: "V:%d,S:%d\n"
    std::string tx_str = "V:" + std::to_string(target_v_stm) + ",S:" + std::to_string(target_servo_stm) + "\n";

    // Bắn thẳng xuống UART sang mạch STM32
    write(serial_port_, tx_str.c_str(), tx_str.length());
  }

  int serial_port_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pos_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  double latest_v_left_;
  double latest_v_right_;
  double latest_steer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridge>());
  rclcpp::shutdown();
  return 0;
}