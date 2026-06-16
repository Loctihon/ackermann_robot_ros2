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
    // Đăng ký nhận lệnh từ topic cmd_vel
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&AckermannKinematics::cmd_cb, this, _1));

    // Đăng ký phát lệnh xuống ros2_control
    pub_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);
    pub_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);

    // Thông số xe (giữ nguyên như bản Python)
    wheelbase_ = 0.200;
    track_width_ = 0.166;
    wheel_radius_ = 0.035;
    max_steer_ = 0.6;
    
    RCLCPP_INFO(this->get_logger(), "C++ Ackermann Kinematics Node has been started.");
  }

private:
  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v = msg->linear.x;
    double w = msg->angular.z;

    // 1. TÍNH GÓC BẺ LÁI
    double steering_angle = 0.0;
    if (v != 0.0) {
      steering_angle = std::atan((wheelbase_ * w) / v);
    } else if (w != 0.0) {
      // Bẻ kịch lái nếu đứng im mà muốn xoay
      steering_angle = std::copysign(max_steer_, w);
    }

    // Ép giới hạn góc bẻ lái vào mốc +-0.6
    steering_angle = std::clamp(steering_angle, -max_steer_, max_steer_);

    // 2. TÍNH TỐC ĐỘ 2 BÁNH SAU (rad/s)
    double v_left = v - (w * track_width_ / 2.0);
    double v_right = v + (w * track_width_ / 2.0);

    double w_left = v_left / wheel_radius_;
    double w_right = v_right / wheel_radius_;

    // 3. XUẤT LỆNH
    auto pos_msg = std_msgs::msg::Float64MultiArray();
    pos_msg.data = {steering_angle, steering_angle}; // [left_steer, right_steer]

    auto vel_msg = std_msgs::msg::Float64MultiArray();
    vel_msg.data = {w_left, w_right}; // [left_rear_vel, right_rear_vel]

    pub_pos_->publish(pos_msg);
    pub_vel_->publish(vel_msg);
  }

  // Khai báo các biến con trỏ 
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