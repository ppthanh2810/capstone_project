// lidar_sim_adapter: thay driver sllidar_ros2 (RPLIDAR C1) trong Gazebo.
//
//   /sim/scan (gpu_lidar, lidar_link, góc 0 -> 2pi tính từ +X) ─► /scan_raw (quy ước sllidar_ros2)
//
// Quy ước của sllidar_node (angle_compensate, 720 tia, chế độ thiết bị 0 -> 360°):
//   angle_min = -pi, angle_max = +pi, angle_increment = 2pi/(N-1), frame_id "laser_raw",
//   góc 0 của /scan_raw hướng ra SAU robot (ray i nằm ở hướng (angle_min + i*inc) + pi trong laser_frame),
//   tia không đo được = +inf, range_min 0.05, range_max = max_distance của thiết bị.
// Gazebo quét 0 -> 2pi nên tia i ở hướng i*inc trong lidar_link = (−pi + i*inc) + pi
//   -> chỉ cần trừ pi vào angle_min/max, GIỮ NGUYÊN thứ tự ranges (không nội suy).
// scan_rotate (stage2, giữ nguyên) cộng lại pi -> /scan trong laser_frame, góc [0, 2pi].
//
// Giá trị ngoài [range_min, range_max] hoặc không hữu hạn -> +inf (driver thật phát +inf khi dist = 0).
// Stamp giữ của Gazebo (sim time lúc chụp). scan_time / time_increment tính từ chu kỳ thật giữa 2 scan
// (gpu_lidar quét tức thời nên để 0; giữ cho giống driver).

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSimAdapter : public rclcpp::Node
{
public:
  LidarSimAdapter()
  : Node("lidar_sim_adapter")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_raw");
    range_min_ = declare_parameter<double>("range_min", 0.05);   // hard-code trong sllidar_node
    range_max_ = declare_parameter<double>("range_max", 16.0);   // max_distance C1 (log driver)
    const auto input_topic = declare_parameter<std::string>("input_topic", "/sim/scan");

    // Cùng QoS với publisher của sllidar_node (reliable, keep last 10)
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_raw", rclcpp::QoS(rclcpp::KeepLast(10)));
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarSimAdapter::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "LIDAR SIM ADAPTER | %s -> /scan_raw | frame_id=%s | range %.2f..%.2f m",
      input_topic.c_str(), frame_id_.c_str(), range_min_, range_max_);
  }

private:
  void scanCallback(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    const size_t n = msg->ranges.size();
    if (n < 2) return;

    // Cấu hình sensor sai (không phải quét 0 -> 2pi) thì quy ước góc sẽ lệch -> báo một lần.
    if (!warned_ && (std::abs(msg->angle_min) > 1e-3 || std::abs(msg->angle_max - 2.0 * M_PI) > 1e-3)) {
      RCLCPP_WARN(
        get_logger(), "gpu_lidar quét [%.4f, %.4f] rad, cần [0, 2pi] (xem urdf/sim_sensors.xacro)",
        msg->angle_min, msg->angle_max);
      warned_ = true;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    double scan_time = 0.0;
    if (last_stamp_.nanoseconds() > 0 && stamp > last_stamp_) {
      scan_time = (stamp - last_stamp_).seconds();
    }
    last_stamp_ = stamp;

    msg->header.frame_id = frame_id_;
    msg->angle_min -= static_cast<float>(M_PI);
    msg->angle_max -= static_cast<float>(M_PI);
    msg->angle_increment = (msg->angle_max - msg->angle_min) / static_cast<float>(n - 1);
    msg->scan_time = static_cast<float>(scan_time);
    msg->time_increment = static_cast<float>(scan_time / static_cast<double>(n - 1));
    msg->range_min = static_cast<float>(range_min_);
    msg->range_max = static_cast<float>(range_max_);
    for (auto & r : msg->ranges) {
      if (!std::isfinite(r) || r < range_min_ || r > range_max_) {
        r = std::numeric_limits<float>::infinity();
      }
    }
    pub_->publish(std::move(msg));
  }

  std::string frame_id_;
  double range_min_;
  double range_max_;
  bool warned_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
