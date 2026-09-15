#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "zlac8015d_hardware/zlac8015d.hpp"

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
  }

  zlac8015d_hardware::Controller motors_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr move_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Move>());
  rclcpp::shutdown();
  return 0;
}