#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "zlac8015d_hw/zlac8015d.hpp"

class Move : public rclcpp::Node
{
public:
  Move() : Node("move"), motors_("/dev/ttyUSB1")
  {
    motors_.disableMotor();
    motors_.setAccelTime(500, 500);
    motors_.setDecelTime(500, 500);
    motors_.setMode(3);
    motors_.enableMotor();

    move_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>( 
        "/PT/move", 10, std::bind(&Move::moveCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Move::readRpm, this));

    RCLCPP_INFO(get_logger(), "move started");
  }

  ~Move()
  {
    motors_.setRpm(0, 0);
    motors_.disableMotor();
  }

private:
  void moveCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2)
      return;

    motors_.setRpm(
      static_cast<int>(msg->data[0]),
      static_cast<int>(msg->data[1]));

    auto [left_rpm, right_rpm] = motors_.getRpm();
  }

  void readRpm()
  {
    try {
      auto [left_rpm, right_rpm] = motors_.getRpm();

      RCLCPP_INFO(
        get_logger(),
        "RPM Left: %.4f | RPM Right: %.4f",
        left_rpm,
        right_rpm);
    }
    catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Get RPM failed: %s", e.what());
    }
  }

  zlac8015d_hw::Controller motors_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr move_sub_;
    
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Move>());
  rclcpp::shutdown();
  return 0;
}