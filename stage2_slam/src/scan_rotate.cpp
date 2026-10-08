// Xoay dữ liệu LaserScan quanh trục Z của LiDAR một góc angle_offset (mặc định pi).
//
// Driver sllidar_ros2 phát góc = pi - góc thiết bị, nên với RPLIDAR C1 lắp vạch 0° hướng ra trước,
// góc 0 của /scan_raw hướng về phía SAU robot. Node này cộng angle_offset vào angle_min/angle_max
// (giữ nguyên thứ tự ranges/intensities -> không nội suy, không sai số) và đổi frame_id,
// để laser_frame (= lidar_link) cùng hướng base_footprint: X trước, Y trái, Z lên.
//
// Sau khi xoay pi: angle_min/max = [0, 2pi] (sensor_msgs/LaserScan không giới hạn khoảng góc).

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanRotate : public rclcpp::Node
{
public:
  ScanRotate() : Node("scan_rotate")
  {
    angle_offset_ = declare_parameter<double>("angle_offset", M_PI);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_frame");

    // Cùng QoS với driver sllidar (reliable, keep last 10) để laser_filters/RViz subscribe được.
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_out", rclcpp::QoS(rclcpp::KeepLast(10)));

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_in", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        msg->angle_min += angle_offset_;
        msg->angle_max += angle_offset_;
        msg->header.frame_id = frame_id_;
        pub_->publish(std::move(msg));
      });

    RCLCPP_INFO(
      get_logger(), "scan_rotate | angle_offset=%.4f rad | frame_id=%s",
      angle_offset_, frame_id_.c_str());
  }

private:
  double angle_offset_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRotate>());
  rclcpp::shutdown();
  return 0;
}
