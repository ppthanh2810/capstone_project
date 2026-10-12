// Node ROS 2 bọc ScanMatchInitializer.
//
//   initialpose_guess (PoseWithCovarianceStamped, frame map) ─┐
//   map (OccupancyGrid, transient_local) ─────────────────────┼─> CSM ─> scan_match/candidate
//   scan_filtered (LaserScan, sensor QoS) + TF ───────────────┘        scan_match/status (DiagnosticStatus)
//                                                                       initialpose (chỉ khi ACCEPTED)
//
// Node KHÔNG publish TF. AMCL vẫn là nguồn duy nhất của map -> odom.
// Mỗi guess -> tối đa MỘT /initialpose; không publish lặp, có giới hạn tần suất.

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "stage2_slam/scan_match_initializer.hpp"

namespace stage2_slam
{

// Chuyển đổi msg <-> struct lõi (dùng chung cho node và monitor).
GridMap toGridMap(const nav_msgs::msg::OccupancyGrid & msg);
ScanData toScanData(const sensor_msgs::msg::LaserScan & msg);
double yawFromQuaternion(double x, double y, double z, double w);

class ScanMatchInitializerNode : public rclcpp::Node
{
public:
  explicit ScanMatchInitializerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGuess(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onTimer();
  void pollAmclState();
  void handleResult(const SessionResult & result);
  void publishStatus(
    MatchStatus status, const std::string & reason, const SessionResult * result = nullptr);
  std::optional<Pose2D> lookup2D(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    std::string & error);

  MatcherConfig matcher_cfg_;
  SessionConfig session_cfg_;
  std::string global_frame_, base_frame_, odom_frame_;
  double tf_timeout_{0.1};
  double min_publish_interval_{1.0};
  bool publish_initialpose_{true};
  bool require_amcl_active_{true};
  double amcl_state_max_age_{3.0};

  std::shared_ptr<ScoreField> field_;
  std::string map_error_{"no map received"};
  std::optional<InitializationSession> session_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  bool published_once_{false};

  std::optional<bool> amcl_active_;
  rclcpp::Time amcl_state_time_{0, 0, RCL_ROS_TIME};
  bool amcl_request_pending_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr guess_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr candidate_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr amcl_state_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr amcl_timer_;
};

}  // namespace stage2_slam
