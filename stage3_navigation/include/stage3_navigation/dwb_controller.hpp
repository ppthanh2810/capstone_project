// =====================================================================================================
// dwb_controller.hpp — Bộ điều khiển cục bộ Dynamic Window TỰ VIẾT (plugin nav2_core::Controller)
// =====================================================================================================
//
// Đây là cài đặt riêng của dự án theo nguyên lý DWA (Dynamic Window Approach, Fox/Burgard/Thrun 1997),
// KHÔNG dùng lại code của dwb_core (Nav2). Plugin được controller_server của Nav2 nạp qua pluginlib:
//
//   nav2_params.yaml:  controller_server.ros__parameters.CustomDWB.plugin: "stage3_navigation::DwbController"
//
// Phân công trách nhiệm (Nav2 Humble 1.1.20):
//   controller_server: action FollowPath, local_costmap, odometry, goal/progress checker, lifecycle, publish
//     cmd_vel; bắt PlannerException -> lệnh 0 trong failure_tolerance rồi abort + lệnh 0. Khi local costmap
//     KHÔNG current (scan cũ), server chờ trong vòng lặp và KHÔNG gọi plugin, KHÔNG publish gì (cũng không
//     publish 0) -> lệnh cuối hết hạn ở twist_mux (timeout 0.5 s) / watchdog của driver (0.5 s).
//   plugin này: mỗi chu kỳ trả lời "lệnh (v, ω) tiếp theo là gì?" và TỪ CHỐI (ném PlannerException) khi dữ
//     liệu đầu vào không đủ mới/hợp lệ: pose (TF odom->base) cũ, TF map->odom cũ, odometry ngừng, path/pose
//     NaN, robot lệch đường quá xa, không có quỹ đạo chấp nhận được.
//
// Toán (dynamic window, mô phỏng, va chạm + phanh, bám đường, chống dao động) nằm ở dwa_core.hpp (không phụ
// thuộc ROS node -> unit test + mô phỏng kín vòng). File này chỉ là lớp vỏ ROS:
//   1. setPlan: kiểm tra path (frame, NaN, quaternion đích) -> PathTracker (frame map).
//   2. computeVelocityCommands:
//      a. kiểm tra pose/velocity hữu hạn, tuổi pose, odometry watchdog;
//      b. tra TF map->odom MỚI NHẤT một lần (chờ tối đa transform_tolerance, tuổi <= max_tf_age), dùng CHUNG
//         cho đường cục bộ và đích (không dùng stamp cũ của pose cuối path);
//      c. tiến trình dọc đường (PathTracker, frame map) -> đường cục bộ -> frame odom;
//      d. dung sai đích lấy từ GoalChecker đang dùng (getTolerances) -> đồng bộ với điều kiện kết thúc action;
//      e. KHÓA costmap chỉ trong lúc cắt đường ở mép costmap + DwaCore::compute (không tra TF, không publish
//         trong khóa);
//      f. publish /local_plan, /<plugin>/local_path; marker + trạng thái chẩn đoán có giới hạn tần suất.
//
// Mọi tham số nằm dưới <tên plugin>.* và chỉnh được khi đang chạy (kiểm tra NGUYÊN TỬ: một tham số sai -> cả
// lô bị từ chối, giữ nguyên tham số cũ):
//   ros2 param set /controller_server CustomDWB.max_vel_x 0.12
// =====================================================================================================

#ifndef STAGE3_NAVIGATION__DWB_CONTROLLER_HPP_
#define STAGE3_NAVIGATION__DWB_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "stage3_navigation/dwa_core.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace stage3_navigation
{

class DwbController : public nav2_core::Controller
{
public:
  DwbController() = default;
  ~DwbController() override = default;

  // ---- Vòng đời plugin (controller_server gọi theo lifecycle của nó) ----
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // controller_server gọi khi nhận FollowPath mới hoặc goal thay thế (preempt). Path không hợp lệ ->
  // PlannerException (server abort + lệnh 0).
  void setPlan(const nav_msgs::msg::Path & path) override;

  // Hàm chính, gọi mỗi chu kỳ điều khiển. pose: frame local costmap (odom), stamp = stamp TF;
  // velocity: /odometry/filtered đã qua ngưỡng min_*_velocity_threshold của server.
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // Humble: speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT (0.0) = BỎ giới hạn (không phải "dừng");
  // percentage: % của max_vel_x danh nghĩa; ngược lại m/s. Giá trị âm / NaN bị bỏ qua (giữ giới hạn cũ).
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ---------------- Tham số ----------------
  void declareAndLoadParams();
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);
  // Bản sao nhất quán cho MỘT chu kỳ + max_vel_x sau speed limit.
  DwbParams snapshotParams(double & max_vel_x_eff);

  // ---------------- Hỗ trợ ----------------
  void resetState();
  // Ghi lý do, publish trạng thái BLOCKED và ném PlannerException (controller_server phát lệnh 0).
  [[noreturn]] void fail(const std::string & reason, const DwbParams & p);
  CollisionModel makeCollisionModel(const DwbParams & p, std::string & mode_name) const;
  GoalTolerance goalTolerance(const DwbParams & p, nav2_core::GoalChecker * goal_checker);
  bool debugDue(const DwbParams & p, bool force);
  void publishState(
    const DwbParams & p, const ControlOutput * out, const std::string & error, double compute_ms);
  void publishMarkers(const ControlOutput & out, const std::string & frame, int stride);
  void publishPaths(
    const std::vector<Pose2D> & local_path, const Trajectory * chosen, const std::string & frame);
  void clearDebug();

  // ---------------- Dữ liệu ----------------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;                                        // tên plugin, vd. "CustomDWB"
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("DwbController")};
  rclcpp::Clock::SharedPtr clock_;
  double controller_frequency_{10.0};

  // Tham số: đọc/ghi từ luồng executor (param callback, speed limit) và luồng action (compute).
  std::mutex params_mutex_;
  DwbParams params_;
  enum class LimitMode { NONE, PERCENT, ABSOLUTE };
  LimitMode limit_mode_{LimitMode::NONE};
  double limit_value_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Trạng thái theo plan (luồng action + lifecycle). Thứ tự khóa: state_mutex_ -> params_mutex_ -> costmap.
  std::mutex state_mutex_;
  PathTracker tracker_;
  std::string plan_frame_;
  DwaCore core_;
  Velocity2D last_cmd_;
  bool have_last_cmd_{false};
  std::chrono::steady_clock::time_point last_compute_;
  bool have_last_compute_{false};
  Phase last_phase_{Phase::IDLE};
  bool warned_tolerance_mismatch_{false};
  // Thống kê thời gian tính (ms) từ lúc activate.
  double compute_ms_max_{0.0};
  long cycles_{0};
  long over_budget_{0};      // > 80 % chu kỳ danh nghĩa
  long missed_deadline_{0};  // khoảng giữa hai lần gọi > 1.5 chu kỳ
  std::chrono::steady_clock::time_point last_debug_;
  bool have_last_debug_{false};
  PathTracker::Result last_track_;

  // Odometry watchdog (controller_server Humble không kiểm tra tuổi odometry).
  std::mutex odom_mutex_;
  std::string odom_topic_{"odom"};
  rclcpp::Time last_odom_rx_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  markers_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticStatus>>
  state_pub_;
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__DWB_CONTROLLER_HPP_
