
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "zlac8015d_hw/zlac8015d.hpp"

using namespace std::chrono_literals;

class zlac8015d_move : public rclcpp::Node
{
public:
  zlac8015d_move()
  : Node("zlac8015d_move"),
    motors_("/dev/ttyUSB2")
  {
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0535);
    wheel_distance_ = declare_parameter<double>("wheel_distance", 0.34);

    motors_.disableMotor();
    motors_.setAccelTime(500, 500);
    motors_.setDecelTime(500, 500);
    motors_.setMode(3);
    motors_.setRpm(0, 0);
    motors_.enableMotor();

    motion_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_out", 1,
      std::bind(
        &zlac8015d_move::motionCallback,
        this,
        std::placeholders::_1));

    feedback_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wheel_feedback", 10);

    feedback_timer_ = create_wall_timer(50ms,std::bind(&zlac8015d_move::readFeedback, this));

    watchdog_timer_ = create_wall_timer(100ms,std::bind(&zlac8015d_move::checkWatchdog, this));

    last_command_time_ = now();

    RCLCPP_INFO(get_logger(), "zlac8015d_move started");
  }

  ~zlac8015d_move()
  {
    try {
      motors_.setRpm(0, 0);
    } catch (...) {
    }

    try {
      motors_.disableMotor();
    } catch (...) {
    }
  }

private:

  // ============================================================
  // MOTION CONTROL
  // ============================================================

  void motionCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double x = msg->linear.x;
    double z = msg->angular.z;

    double left = (x - wheel_distance_ / 2.0 * z) / wheel_radius_;

    double right = (x + wheel_distance_ / 2.0 * z) / wheel_radius_;

    double left_rpm = left * rpm_factor_;
    double right_rpm = -right * rpm_factor_;

    try {
      motors_.setRpm(
        static_cast<int>(std::lround(left_rpm)),
        static_cast<int>(std::lround(right_rpm)));

      last_command_time_ = now();
      command_received_ = true;
      watchdog_stopped_ = false;

    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Set RPM failed: %s",
        e.what());
    }
  }

  // ============================================================
  // WATCHDOG
  // ============================================================

  void checkWatchdog()
  {
    if (!command_received_) return;

    double elapsed =
      (now() - last_command_time_).seconds();

    if (elapsed > command_timeout_ && !watchdog_stopped_) {
      try {
        motors_.setRpm(0, 0);

        watchdog_stopped_ = true;

        RCLCPP_WARN(
          get_logger(),
          "Command timeout. Motors stopped.");

      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "Watchdog stop failed: %s",
          e.what());
      }
    }
  }

  // ============================================================
  // MOTOR FEEDBACK
  // ============================================================

  void readFeedback()
  {
    try {
      auto [left_rpm_raw, right_rpm_raw] =
        motors_.getRpm();

      auto [left_travelled_raw, right_travelled_raw] =
        motors_.getWheelsTravelled();

      double left_rpm =
        static_cast<double>(left_rpm_raw);

      double right_rpm =
        -static_cast<double>(right_rpm_raw);

      double left_travelled =
        static_cast<double>(left_travelled_raw);

      double right_travelled =
        -static_cast<double>(right_travelled_raw);

      if (!std::isfinite(left_rpm) ||
          !std::isfinite(right_rpm) ||
          !std::isfinite(left_travelled) ||
          !std::isfinite(right_travelled)) {

        RCLCPP_WARN(
          get_logger(),
          "Invalid motor feedback.");

        return;
      }

      std_msgs::msg::Float64MultiArray feedback;

      feedback.data = {left_rpm,right_rpm,left_travelled,right_travelled};

      feedback_pub_->publish(feedback);

    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Read feedback failed: %s",
        e.what());
    }
  }

  // ============================================================
  // PARAMETERS
  // ============================================================

  static constexpr double pi_ = 3.14159265358979323846;

  static constexpr double rpm_factor_ = 30.0 / pi_;

  static constexpr double command_timeout_ = 0.5;

  double wheel_radius_ = 0.0535;
  double wheel_distance_ = 0.34;

  bool command_received_ = false;
  bool watchdog_stopped_ = false;

  rclcpp::Time last_command_time_;

  zlac8015d_hw::Controller motors_;

  // ============================================================
  // ROS2 INTERFACES
  // ============================================================

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr motion_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<zlac8015d_move>());

  rclcpp::shutdown();

  return 0;
}