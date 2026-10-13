// =====================================================================================================
// dwb_controller.hpp — Bộ điều khiển cục bộ Dynamic Window TỰ VIẾT (plugin nav2_core::Controller)
// =====================================================================================================
//
// Đây là cài đặt riêng của dự án theo nguyên lý DWA (Dynamic Window Approach), KHÔNG dùng lại code của
// dwb_core (Nav2). Plugin được controller_server của Nav2 nạp qua pluginlib, giống hệt DWB:
//
//   nav2_params.yaml:  controller_server.ros__parameters.CustomDWB.plugin: "stage3_navigation::DwbController"
//
// controller_server lo phần "khung": action FollowPath, local_costmap, odometry, goal/progress checker,
// lifecycle, publish cmd_vel. Plugin này CHỈ trả lời một câu hỏi mỗi chu kỳ (10 Hz):
//   "Với pose + vận tốc hiện tại của robot và đường A* đã cho, lệnh (v, ω) tiếp theo là gì?"
//
// Luồng xử lý mỗi chu kỳ (computeVelocityCommands):
//   1. Chuyển đường A* (frame map) sang frame của local costmap (odom), tìm điểm gần robot nhất,
//      cắt ra "đường cục bộ" phía trước robot và chọn "cà rốt" (carrot) cách robot carrot_distance
//      dọc theo đường.
//   2. Chọn pha:
//        ROTATE_TO_GOAL : đã tới vị trí đích (trong xy_goal_tolerance) -> chỉ xoay tại chỗ về hướng đích.
//        ROTATE_TO_PATH : hướng robot lệch quá rotate_to_heading_angle so với cà rốt -> xoay tại chỗ.
//        FOLLOW_PATH    : tìm kiếm DWA (bước 3-5).
//   3. Dynamic window: tập (v, ω) đạt được trong MỘT chu kỳ điều khiển từ vận tốc hiện tại, theo giới hạn
//      gia tốc (acc/decel) và giới hạn vận tốc tuyệt đối.
//   4. Lấy mẫu vx_samples x vtheta_samples cặp (v, ω), mô phỏng mỗi cặp (giữ hằng) trong sim_time giây.
//   5. Loại quỹ đạo va chạm (ô costmap >= INSCRIBED), chấm điểm phần còn lại bằng hàm chi phí có trọng số,
//      chọn quỹ đạo chi phí nhỏ nhất. Không còn quỹ đạo hợp lệ -> ném PlannerException để controller_server
//      dừng robot (failure_tolerance, rồi abort + vận tốc 0).
//
// Mọi tham số nằm dưới namespace <tên plugin> (vd. CustomDWB.max_vel_x) và chỉnh được khi đang chạy:
//   ros2 param set /controller_server CustomDWB.max_vel_x 0.12
// =====================================================================================================

#ifndef STAGE3_NAVIGATION__DWB_CONTROLLER_HPP_
#define STAGE3_NAVIGATION__DWB_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace stage3_navigation
{

// Một pose 2D (x, y, θ) trong frame của local costmap.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// Một quỹ đạo ứng viên: cặp (v, ω) giữ hằng trong sim_time + các pose mô phỏng + điểm số.
struct Trajectory
{
  double v{0.0};                 // m/s
  double w{0.0};                 // rad/s
  std::vector<Pose2D> poses;     // pose sau mỗi bước sim_granularity
  bool valid{true};              // false: va chạm / ra ngoài costmap / ô unknown
  double total_cost{0.0};        // tổng có trọng số (nhỏ hơn = tốt hơn)
  // Các thành phần chi phí (chưa nhân trọng số) - để debug / in log.
  double obstacle_cost{0.0};
  double path_cost{0.0};
  double carrot_cost{0.0};
  double heading_cost{0.0};
  double speed_cost{0.0};
};

// Toàn bộ tham số của thuật toán (đọc từ YAML, đổi được bằng ros2 param set).
struct DwbParams
{
  // ---- Giới hạn động học (robot vi sai: không có vy) ----
  double max_vel_x{0.15};               // m/s, vận tốc tiến tối đa
  double min_vel_x{0.0};                // m/s, 0 = KHÔNG lùi (phía sau không có LiDAR)
  double max_vel_theta{0.5};            // rad/s, |ω| tối đa
  double min_in_place_vel_theta{0.1};   // rad/s, |ω| tối thiểu khi xoay tại chỗ (thắng ma sát tĩnh)
  double acc_lim_x{0.5};                // m/s², tăng tốc tịnh tiến tối đa
  double decel_lim_x{0.5};              // m/s², giảm tốc tịnh tiến tối đa (giá trị dương)
  double acc_lim_theta{1.0};            // rad/s², gia tốc góc tối đa (cả tăng và giảm)

  // ---- Lấy mẫu + mô phỏng ----
  int vx_samples{20};                   // số mẫu v trong dynamic window
  int vtheta_samples{21};               // số mẫu ω (lẻ -> có mẫu ω = 0 khi cửa sổ đối xứng)
  double sim_time{1.7};                 // s, thời gian mô phỏng mỗi quỹ đạo
  double sim_granularity{0.05};         // s, bước tích phân

  // ---- Đường cục bộ + cà rốt ----
  double local_plan_length{1.5};        // m, độ dài đường A* phía trước robot được xét
  double carrot_distance{0.6};          // m, điểm "cà rốt" dọc đường A* mà robot hướng tới
  double closest_search_distance{1.0};  // m, chỉ tìm điểm gần nhất trong đoạn này phía trước (chống nhảy)

  // ---- Các pha xoay tại chỗ ----
  double rotate_to_heading_angle{0.8};  // rad, lệch hướng > ngưỡng -> xoay tại chỗ trước khi đi
  double rotate_gain{1.5};              // ω = gain * sai số góc (rồi giới hạn)
  double xy_goal_tolerance{0.15};       // m, vào vùng này -> chỉ xoay về hướng đích
  double yaw_goal_tolerance{0.25};      // rad, sai số hướng chấp nhận ở đích

  // ---- Trọng số hàm chi phí (tổng = Σ scale * thành phần) ----
  double path_distance_scale{4.0};      // khoảng cách cuối quỹ đạo -> đường A* (m)
  double carrot_distance_scale{2.0};    // khoảng cách cuối quỹ đạo -> cà rốt (m)
  double heading_scale{1.0};            // lệch hướng cuối quỹ đạo so với hướng tới cà rốt (0..1)
  double obstacle_scale{0.5};           // chi phí costmap lớn nhất dọc quỹ đạo (0..1)
  double speed_scale{0.3};              // (max_vel_x - v) / max_vel_x (0..1): khuyến khích đi nhanh

  // ---- An toàn / khác ----
  bool unknown_is_obstacle{true};       // ô NO_INFORMATION coi như vật cản
  double transform_tolerance{0.3};      // s, TF map -> odom được phép cũ bao nhiêu
  bool publish_trajectories{true};      // publish MarkerArray các quỹ đạo ứng viên (RViz)
  int trajectory_marker_stride{4};      // chỉ vẽ 1 trên N quỹ đạo để RViz nhẹ
};

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

  // controller_server gọi khi nhận FollowPath mới (đường A* trong frame map).
  void setPlan(const nav_msgs::msg::Path & path) override;

  // Hàm chính, gọi mỗi chu kỳ điều khiển. pose: trong frame local costmap (odom);
  // velocity: vận tốc hiện tại từ odometry (/odometry/filtered).
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // Giới hạn tốc độ từ bên ngoài (speed_limit topic của Nav2). percentage: % của max_vel_x.
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ---------------- 1. Đường A* -> đường cục bộ ----------------
  // Chuyển đường A* sang frame costmap, cập nhật closest_index_, trả về đường cục bộ phía trước robot.
  std::vector<Pose2D> buildLocalPath(const Pose2D & robot, const std::string & costmap_frame);
  // Điểm trên đường cục bộ cách đầu đường (gần robot nhất) carrot_distance mét.
  Pose2D pickCarrot(const std::vector<Pose2D> & local_path, double carrot_distance) const;

  // ---------------- 2-5. DWA ----------------
  // Tìm lệnh tốt nhất trong dynamic window. Ném PlannerException nếu không có quỹ đạo hợp lệ.
  Trajectory searchDynamicWindow(
    const DwbParams & p, const Pose2D & robot, const geometry_msgs::msg::Twist & velocity,
    const std::vector<Pose2D> & local_path, const Pose2D & carrot, double dt);
  // Mô phỏng (v, ω) giữ hằng trong sim_time theo mô hình vi sai (unicycle).
  Trajectory simulate(const DwbParams & p, const Pose2D & start, double v, double w) const;
  // Kiểm tra va chạm + tính các thành phần chi phí, ghi vào traj.
  void scoreTrajectory(
    const DwbParams & p, Trajectory & traj, const std::vector<Pose2D> & local_path,
    const Pose2D & carrot) const;

  // ---------------- Xoay tại chỗ ----------------
  // Lệnh xoay tại chỗ về sai số góc 'error', tôn trọng giới hạn gia tốc; v giảm dần về 0.
  geometry_msgs::msg::Twist rotateInPlace(
    const DwbParams & p, double error, const geometry_msgs::msg::Twist & velocity, double dt,
    const Pose2D & robot) const;

  // ---------------- Tiện ích ----------------
  // Chi phí ô costmap tại (x, y); trả về NO_INFORMATION nếu ngoài costmap.
  unsigned char costAt(double x, double y) const;
  // Ô có làm robot (tròn, tâm tại x,y) va chạm không: cost >= INSCRIBED, ngoài map, hoặc unknown.
  bool isInCollision(const DwbParams & p, double x, double y) const;
  // Khoảng cách từ điểm tới đường gấp khúc (polyline).
  static double distanceToPath(double x, double y, const std::vector<Pose2D> & path);
  static double normalizeAngle(double a);

  // ---------------- Tham số ----------------
  void declareAndLoadParams();
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);
  DwbParams snapshotParams();
  static std::string validate(const DwbParams & p);   // "" = hợp lệ

  // ---------------- Hiển thị ----------------
  void publishDebug(
    const std::vector<Trajectory> & trajectories, const Trajectory * best,
    const std::vector<Pose2D> & local_path, const Pose2D & carrot, const std::string & frame);

  // ---------------- Dữ liệu ----------------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;                                        // tên plugin, vd. "CustomDWB"
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("DwbController")};
  rclcpp::Clock::SharedPtr clock_;

  std::mutex params_mutex_;
  DwbParams params_;
  double speed_limit_x_{0.0};                               // 0 = không giới hạn ngoài
  double controller_frequency_{10.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  nav_msgs::msg::Path global_plan_;                         // đường A* gốc (frame map)
  std::size_t closest_index_{0};                            // chỉ số điểm gần robot nhất (chỉ tăng)
  bool goal_rotation_latched_{false};                       // đã vào pha ROTATE_TO_GOAL (giữ tới hết)
  bool rotating_to_path_{false};                            // đang ở pha ROTATE_TO_PATH (có trễ)

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  markers_pub_;
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__DWB_CONTROLLER_HPP_
