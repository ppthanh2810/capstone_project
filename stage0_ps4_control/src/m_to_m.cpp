#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class MtoM : public rclcpp::Node
{
public:
  MtoM() : Node("m_to_m")
  {
    motion_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        x_ = msg->linear.x;
        z_ = msg->angular.z;
      });

    percent_sub_ = create_subscription<std_msgs::msg::Float32>("/PT/percent", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { 
        percent_ = msg->data; 
    });

    move_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/PT/move", 10);

    timer_ = create_wall_timer(100ms, std::bind(&MtoM::publishRpm, this));

    RCLCPP_INFO(get_logger(), "m_to_m started");
  }

private:
  void publishRpm()
  {
    double x = (percent_ >= 50.0) ? 0.0 : x_;
    double z = z_;

    double left, right;

    if (x >= 0.0) {
      left  = (x - wheel_distance_ / 2.0 * z) / wheel_radius_;
      right = (x + wheel_distance_ / 2.0 * z) / wheel_radius_;
    } else {
      left  = (x + wheel_distance_ / 2.0 * z) / wheel_radius_;
      right = (x - wheel_distance_ / 2.0 * z) / wheel_radius_;
    }

    constexpr double rpm_factor = 60.0 / (2.0 * 3.141592653589793);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      left * rpm_factor,
      -right * rpm_factor
    };

    move_pub_->publish(msg);
  }

  static constexpr double wheel_radius_ = 0.0535;
  static constexpr double wheel_distance_ = 0.34;

  double x_ = 0.0;
  double z_ = 0.0;
  double percent_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr percent_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr move_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MtoM>());
  rclcpp::shutdown();
  return 0;
}
