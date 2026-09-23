#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_bno055/bno055.hpp"

using namespace std::chrono_literals;

class Bno055Node : public rclcpp::Node {
public:
  Bno055Node() : Node("bno055_node")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    serial_ = std::make_unique<imu_bno055::Bno055>(port);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    timer_ = create_wall_timer(5ms, [this] { poll(); });
    RCLCPP_INFO(get_logger(), "Waiting for BNO055 UART on %s", port.c_str());
  }

private:
  void poll()
  {
    if (!serial_->connected()) {
      const auto now = std::chrono::steady_clock::now();
      if (now < next_retry_) return;
      next_retry_ = now + 1s;
      if (!serial_->openPort()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Cannot open %s (check USB-UART, port and dialout permission)", serial_->port().c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "Opened %s at 115200 8N1", serial_->port().c_str());
    }

    imu_bno055::ImuSample s;
    if (!serial_->readLatest(s)) return;

    constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();  // host receipt time; ESP32 millis() is NOT ROS epoch time
    msg.header.frame_id = frame_id_;
    msg.orientation.w = s.qw;
    msg.orientation.x = s.qx;
    msg.orientation.y = s.qy;
    msg.orientation.z = s.qz;
    msg.angular_velocity.x = s.gx * deg_to_rad;
    msg.angular_velocity.y = s.gy * deg_to_rad;
    msg.angular_velocity.z = s.gz * deg_to_rad;
    msg.linear_acceleration.x = s.ax;
    msg.linear_acceleration.y = s.ay;
    msg.linear_acceleration.z = s.az;
    // Covariance arrays remain zero: values are not supplied by ESP32 firmware.
    imu_pub_->publish(msg);
  }

  std::string frame_id_;
  std::unique_ptr<imu_bno055::Bno055> serial_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point next_retry_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bno055Node>());
  rclcpp::shutdown();
  return 0;
}