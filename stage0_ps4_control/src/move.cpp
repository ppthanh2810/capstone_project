#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "zlac8015d_hw/zlac8015d.hpp"

using namespace std::chrono_literals;

class Move : public rclcpp::Node
{
public:
  Move() : Node("move"), motors_("/dev/ttyUSB1")
  {
    motors_.disableMotor();
    motors_.setAccelTime(500, 500);
    motors_.setDecelTime(500, 500);
    motors_.setMode(3);
    motors_.setRpm(0, 0);
    motors_.enableMotor();

    move_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/PT/move", 1, std::bind(&Move::moveCallback, this, std::placeholders::_1));

    rpm_timer_ = create_wall_timer(200ms, std::bind(&Move::readRpm, this));
    watchdog_timer_ = create_wall_timer(100ms, std::bind(&Move::checkWatchdog, this));

    last_move_time_ = now();

    RCLCPP_INFO(get_logger(), "move started");
  }

  ~Move()
  {
    try { motors_.setRpm(0, 0); } catch (...) {}
    try { motors_.disableMotor(); } catch (...) {}
  }

private:
  void moveCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "Invalid /PT/move message");
      return;
    }

    int left_rpm = static_cast<int>(msg->data[0]);
    int right_rpm = static_cast<int>(msg->data[1]);

    motors_.setRpm(left_rpm, right_rpm);

    last_move_time_ = now();
    move_received_ = true;
    watchdog_stopped_ = false;
  }

  void checkWatchdog()
  {
    if (!move_received_) return;

    double elapsed = (now() - last_move_time_).seconds();

    if (elapsed > command_timeout_ && !watchdog_stopped_) {
      motors_.setRpm(0, 0);
      watchdog_stopped_ = true;
      RCLCPP_WARN(get_logger(), "Move command timeout. Motors stopped.");
    }
  }

  void readRpm()
  {
    try {
      auto [left_rpm, right_rpm] = motors_.getRpm();

      RCLCPP_INFO(get_logger(), "RPM Left: %.4f | RPM Right: %.4f",
                  static_cast<double>(left_rpm), static_cast<double>(right_rpm));
    }
    catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Get RPM failed: %s", e.what());
    }
  }

  static constexpr double command_timeout_ = 0.5;

  bool move_received_ = false;
  bool watchdog_stopped_ = false;

  rclcpp::Time last_move_time_;

  zlac8015d_hw::Controller motors_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr move_sub_;
  rclcpp::TimerBase::SharedPtr rpm_timer_, watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Move>());
  rclcpp::shutdown();
  return 0;
}