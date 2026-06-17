#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class AckermannKinematics : public rclcpp::Node
{
public:
  AckermannKinematics() : Node("ackermann_kinematics")
  {
    // Đăng ký nhận lệnh từ topic cmd_vel (Cả tay cầm và Nav2 tự hành đều đổ vào đây)
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&AckermannKinematics::cmd_cb, this, _1));

    // Đăng ký phát lệnh xuống ros2_control
    pub_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);
    pub_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);

    // Thông số hình học của xe robot
    wheelbase_ = 0.200;
    track_width_ = 0.166;
    wheel_radius_ = 0.035;
    max_steer_ = 0.785; // Giới hạn góc bẻ lái cơ khí (~34 độ)
    
    RCLCPP_INFO(this->get_logger(), "C++ Ackermann Kinematics Node chuẩn hóa cho TỰ HÀNH đã khởi động.");
  }

private:
  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v = msg->linear.x;
    double w = msg->angular.z;

    double steering_angle = 0.0;

    // TOÁN HỌC HÌNH HỌC ACKERMANN CHUẨN TOÀN CẦU
    if (std::abs(v) > 0.005) {
      // Chia trực tiếp cho v để hệ thống tự đảo dấu góc vô lăng khi đi lùi (v < 0)
      // Giúp bộ điều hướng Nav2 bo cua lùi chuồng chuẩn xác, không bị crash
      steering_angle = std::atan((wheelbase_ * w) / v);
    } 
    else if (std::abs(w) > 0.005) {
      // Nếu xe đứng im tại chỗ mà gạt bẻ lái: Ép góc theo hướng w của tay cầm
      steering_angle = (w > 0.0) ? max_steer_ : -max_steer_;
    }

    // Khóa cứng giới hạn góc bẻ lái bảo vệ Servo ảo
    steering_angle = std::clamp(steering_angle, -max_steer_, max_steer_);

    // TÍNH TOÁN VẬN TỐC TUYẾN TÍNH CHO 2 BÁNH SAU CHỦ ĐỘNG
    double v_left = v - (w * track_width_ / 2.0);
    double v_right = v + (w * track_width_ / 2.0);

    // Chuyển đổi sang vận tốc góc (rad/s) cho các bánh sau
    double w_left = v_left / wheel_radius_;
    double w_right = v_right / wheel_radius_;

    // XUẤT LỆNH ĐỒNG BỘ GÓC LÁI (Bánh trước)
    auto pos_msg = std_msgs::msg::Float64MultiArray();
    pos_msg.data = {steering_angle, steering_angle}; 

    // XUẤT LỆNH ĐỒNG BỘ VẬN TỐC QUAY (Bánh sau)
    auto vel_msg = std_msgs::msg::Float64MultiArray();
    // ĐÃ XÓA DẤU TRỪ: Trả về dấu dương thuận theo file URDF mới đã lật trục Y
    vel_msg.data = {w_left, w_right}; 

    pub_pos_->publish(pos_msg);
    pub_vel_->publish(vel_msg);
  }

  // Khai báo các biến con trỏ và thông số
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_vel_;

  double wheelbase_;
  double track_width_;
  double wheel_radius_;
  double max_steer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannKinematics>());
  rclcpp::shutdown();
  return 0;
}