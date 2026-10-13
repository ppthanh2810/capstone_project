// =====================================================================================================
// dwb_controller.cpp — Cài đặt bộ điều khiển Dynamic Window tự viết (xem tổng quan ở dwb_controller.hpp)
// =====================================================================================================
//
// Ký hiệu dùng trong chú thích:
//   v  : vận tốc tịnh tiến (m/s, trục x của base_footprint)      ω : vận tốc góc (rad/s, quanh trục z)
//   v0, ω0 : vận tốc hiện tại (từ /odometry/filtered)            dt : chu kỳ điều khiển = 1 / controller_frequency
//   costmap : local costmap (frame odom, rolling 2 x 2 m) do controller_server quản lý
//
// Giá trị ô costmap (nav2_costmap_2d):
//   0 = trống, 1..252 = gần vật cản (inflation, càng lớn càng gần), 253 = INSCRIBED (tâm robot ở đây thì
//   thân robot chạm vật cản), 254 = LETHAL (chính ô vật cản), 255 = NO_INFORMATION (chưa biết).
// Robot tròn => chỉ cần xét ô dưới TÂM robot: cost >= 253 nghĩa là va chạm (bán kính nội tiếp đã gồm
// footprint_padding của local costmap).
// =====================================================================================================

#include "stage3_navigation/dwb_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace stage3_navigation
{

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::MAX_NON_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

// =====================================================================================================
// VÒNG ĐỜI PLUGIN
// =====================================================================================================

void DwbController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("DwbController: parent node expired");
  }
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declareAndLoadParams();

  // controller_frequency do controller_server khai báo -> dùng để tính dt của dynamic window.
  if (node->has_parameter("controller_frequency")) {
    controller_frequency_ = node->get_parameter("controller_frequency").as_double();
  }

  // Publisher để xem trên RViz (tên tương đối -> /local_plan, /<plugin>/local_path, /<plugin>/trajectories).
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_path", 1);
  markers_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/trajectories", 1);

  // Cho phép chỉnh tham số khi đang chạy: ros2 param set /controller_server CustomDWB.<tên> <giá trị>
  param_cb_ = node->add_on_set_parameters_callback(
    std::bind(&DwbController::onParamChange, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "[%s] DwbController (self-written dynamic window) configured: max_vel_x %.2f, "
    "max_vel_theta %.2f, samples %dx%d, sim_time %.2f s, dt %.3f s", name_.c_str(),
    params_.max_vel_x, params_.max_vel_theta, params_.vx_samples, params_.vtheta_samples,
    params_.sim_time, 1.0 / controller_frequency_);
}

void DwbController::cleanup()
{
  local_plan_pub_.reset();
  local_path_pub_.reset();
  markers_pub_.reset();
  param_cb_.reset();
  global_plan_.poses.clear();
}

void DwbController::activate()
{
  local_plan_pub_->on_activate();
  local_path_pub_->on_activate();
  markers_pub_->on_activate();
}

void DwbController::deactivate()
{
  local_plan_pub_->on_deactivate();
  local_path_pub_->on_deactivate();
  markers_pub_->on_deactivate();
}

void DwbController::setPlan(const nav_msgs::msg::Path & path)
{
  // Đường mới (FollowPath mới): bắt đầu lại từ đầu đường, bỏ trạng thái các pha xoay.
  global_plan_ = path;
  closest_index_ = 0;
  goal_rotation_latched_ = false;
  rotating_to_path_ = false;
}

void DwbController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Nav2 gửi 0 (hoặc NO_SPEED_LIMIT) khi bỏ giới hạn.
  std::lock_guard<std::mutex> lock(params_mutex_);
  if (speed_limit <= 0.0 || (percentage && speed_limit >= 100.0)) {
    speed_limit_x_ = 0.0;
  } else if (percentage) {
    speed_limit_x_ = params_.max_vel_x * speed_limit / 100.0;
  } else {
    speed_limit_x_ = speed_limit;
  }
}

// =====================================================================================================
// HÀM CHÍNH — gọi mỗi chu kỳ điều khiển
// =====================================================================================================

geometry_msgs::msg::TwistStamped DwbController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker: controller_server tự kiểm tra đã tới đích*/)
{
  // Khóa costmap: không cho lớp obstacle/inflation cập nhật trong lúc đang đọc.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // Bản sao tham số cho chu kỳ này (an toàn khi người dùng đổi tham số giữa chừng).
  DwbParams p = snapshotParams();
  const double dt = 1.0 / std::max(controller_frequency_, 1.0);
  const std::string frame = pose.header.frame_id;   // = frame của local costmap (odom)

  const Pose2D robot{pose.pose.position.x, pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation)};

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = clock_->now();

  // ---------- Bước 1: đường A* -> đường cục bộ + đích cuối ----------
  const std::vector<Pose2D> local_path = buildLocalPath(robot, frame);

  geometry_msgs::msg::PoseStamped goal_in;
  geometry_msgs::msg::PoseStamped goal_out;
  goal_in = global_plan_.poses.back();
  goal_in.header.frame_id = global_plan_.header.frame_id;
  try {
    goal_out = tf_->transform(goal_in, frame, tf2::durationFromSec(p.transform_tolerance));
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::PlannerException(
            std::string(
              "DwbController: cannot transform goal: ") + ex.what());
  }
  const Pose2D goal{goal_out.pose.position.x, goal_out.pose.position.y,
    tf2::getYaw(goal_out.pose.orientation)};
  const double dist_to_goal = std::hypot(goal.x - robot.x, goal.y - robot.y);

  // ---------- Bước 2a: pha ROTATE_TO_GOAL ----------
  // Đã tới vị trí đích: dừng tịnh tiến, chỉ xoay về hướng Goal Pose. "Latch" để không quay lại đi tiếp khi
  // robot hơi trôi ra ngoài vùng dung sai trong lúc xoay.
  if (goal_rotation_latched_ || dist_to_goal < p.xy_goal_tolerance) {
    if (!goal_rotation_latched_) {
      RCLCPP_INFO(
        logger_, "[%s] at goal position (%.2f m): rotating to goal heading", name_.c_str(),
        dist_to_goal);
    }
    goal_rotation_latched_ = true;
    const double err = normalizeAngle(goal.theta - robot.theta);
    if (std::fabs(err) < p.yaw_goal_tolerance) {
      // Đã đúng hướng: lệnh 0, goal checker của controller_server sẽ kết thúc FollowPath.
      cmd.twist = geometry_msgs::msg::Twist();
    } else {
      cmd.twist = rotateInPlace(p, err, velocity, dt, robot);
    }
    publishDebug({}, nullptr, local_path, goal, frame);
    return cmd;
  }

  // ---------- Bước 2b: pha ROTATE_TO_PATH ----------
  // Cà rốt nằm lệch quá nhiều so với hướng robot (vd. lúc bắt đầu, đường đi ngược hướng robot): xoay tại
  // chỗ trước. Dùng trễ (hysteresis): vào pha khi lệch > ngưỡng, ra khi lệch < ngưỡng/2 -> không rung.
  const Pose2D carrot = pickCarrot(local_path, p.carrot_distance);
  const double carrot_dist = std::hypot(carrot.x - robot.x, carrot.y - robot.y);
  if (carrot_dist > 0.05) {
    const double heading_err =
      normalizeAngle(std::atan2(carrot.y - robot.y, carrot.x - robot.x) - robot.theta);
    const double exit_angle = 0.5 * p.rotate_to_heading_angle;
    if (std::fabs(heading_err) > p.rotate_to_heading_angle ||
      (rotating_to_path_ && std::fabs(heading_err) > exit_angle))
    {
      if (!rotating_to_path_) {
        RCLCPP_INFO(
          logger_, "[%s] heading error %.2f rad: rotating in place toward the path",
          name_.c_str(), heading_err);
      }
      rotating_to_path_ = true;
      cmd.twist = rotateInPlace(p, heading_err, velocity, dt, robot);
      publishDebug({}, nullptr, local_path, carrot, frame);
      return cmd;
    }
  }
  rotating_to_path_ = false;

  // ---------- Bước 3-5: pha FOLLOW_PATH (tìm kiếm Dynamic Window) ----------
  const Trajectory best = searchDynamicWindow(p, robot, velocity, local_path, carrot, dt);
  cmd.twist.linear.x = best.v;
  cmd.twist.angular.z = best.w;
  RCLCPP_DEBUG(
    logger_,
    "[%s] best v=%.3f w=%.3f cost=%.3f (obst %.2f path %.2f carrot %.2f head %.2f spd %.2f)",
    name_.c_str(), best.v, best.w, best.total_cost, best.obstacle_cost, best.path_cost,
    best.carrot_cost, best.heading_cost, best.speed_cost);
  return cmd;
}

// =====================================================================================================
// BƯỚC 1 — ĐƯỜNG CỤC BỘ
// =====================================================================================================

std::vector<Pose2D> DwbController::buildLocalPath(
  const Pose2D & robot, const std::string & costmap_frame)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("DwbController: received an empty plan");
  }

  // Một transform map -> odom cho cả đường (thay vì tra TF cho từng pose).
  geometry_msgs::msg::TransformStamped tf_plan;
  try {
    tf_plan = tf_->lookupTransform(
      costmap_frame, global_plan_.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(snapshotParams().transform_tolerance));
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::PlannerException(
            std::string(
              "DwbController: cannot transform plan to ") + costmap_frame + ": " + ex.what());
  }
  const auto to_local = [&](std::size_t i) {
      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(global_plan_.poses[i], out, tf_plan);
      return Pose2D{out.pose.position.x, out.pose.position.y, tf2::getYaw(out.pose.orientation)};
    };
  const auto seg_len = [&](std::size_t i) {   // độ dài đoạn (i-1, i), tính trong frame map (bất biến)
      const auto & a = global_plan_.poses[i - 1].pose.position;
      const auto & b = global_plan_.poses[i].pose.position;
      return std::hypot(b.x - a.x, b.y - a.y);
    };
  const DwbParams p = snapshotParams();
  const std::size_t n = global_plan_.poses.size();

  // (a) Tìm điểm gần robot nhất, CHỈ trong closest_search_distance phía trước closest_index_ cũ.
  //     closest_index_ chỉ tăng -> robot không "nhảy" sang đoạn đường khác khi đường vòng lại gần nhau.
  closest_index_ = std::min(closest_index_, n - 1);
  std::size_t best_i = closest_index_;
  Pose2D q = to_local(best_i);
  double best_d = std::hypot(q.x - robot.x, q.y - robot.y);
  double along = 0.0;
  for (std::size_t i = closest_index_ + 1; i < n; ++i) {
    along += seg_len(i);
    if (along > p.closest_search_distance) {
      break;
    }
    q = to_local(i);
    const double d = std::hypot(q.x - robot.x, q.y - robot.y);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  closest_index_ = best_i;

  // (b) Cắt đường cục bộ: từ điểm gần nhất đi tới local_plan_length mét, dừng ở mép costmap.
  std::vector<Pose2D> local_path;
  along = 0.0;
  for (std::size_t i = closest_index_; i < n; ++i) {
    if (i > closest_index_) {
      along += seg_len(i);
      if (along > p.local_plan_length) {
        break;
      }
    }
    const Pose2D pt = to_local(i);
    unsigned int mx, my;
    if (!costmap_->worldToMap(pt.x, pt.y, mx, my)) {
      break;   // ra khỏi local costmap
    }
    local_path.push_back(pt);
  }
  if (local_path.empty()) {
    // Điểm gần nhất đã nằm ngoài costmap: robot ở quá xa đường.
    throw nav2_core::PlannerException(
            "DwbController: no part of the plan lies inside the local costmap (robot too far from path)");
  }
  return local_path;
}

Pose2D DwbController::pickCarrot(
  const std::vector<Pose2D> & local_path,
  double carrot_distance) const
{
  // Đi dọc đường cục bộ, lấy điểm đầu tiên có quãng đường tích lũy >= carrot_distance.
  // Đường cục bộ ngắn hơn (gần đích / mép costmap) -> lấy điểm cuối.
  // Lý do dùng cà rốt GẦN thay vì điểm cuối đường cục bộ: ở đầu tường, điểm cuối có thể nằm SAU tường,
  // kéo robot thẳng vào tường (lỗi đã gặp với DWB khi local costmap 3 x 3 m).
  double along = 0.0;
  for (std::size_t i = 1; i < local_path.size(); ++i) {
    along += std::hypot(
      local_path[i].x - local_path[i - 1].x, local_path[i].y - local_path[i - 1].y);
    if (along >= carrot_distance) {
      return local_path[i];
    }
  }
  return local_path.back();
}

// =====================================================================================================
// BƯỚC 3-5 — DYNAMIC WINDOW: LẤY MẪU, MÔ PHỎNG, CHẤM ĐIỂM
// =====================================================================================================

Trajectory DwbController::searchDynamicWindow(
  const DwbParams & p, const Pose2D & robot, const geometry_msgs::msg::Twist & velocity,
  const std::vector<Pose2D> & local_path, const Pose2D & carrot, double dt)
{
  // ---------- Bước 3: dynamic window ----------
  // Trong một chu kỳ dt, từ v0 chỉ đạt được [v0 - decel*dt, v0 + acc*dt]; giao với [min_vel_x, max_vel_x].
  const double v0 = velocity.linear.x;
  const double w0 = velocity.angular.z;
  double v_lo = std::max(p.min_vel_x, v0 - p.decel_lim_x * dt);
  double v_hi = std::min(p.max_vel_x, v0 + p.acc_lim_x * dt);
  double w_lo = std::max(-p.max_vel_theta, w0 - p.acc_lim_theta * dt);
  double w_hi = std::min(p.max_vel_theta, w0 + p.acc_lim_theta * dt);
  // v0 vượt giới hạn (vd. vừa giảm max_vel_x / speed limit): cửa sổ rỗng -> dùng mức thấp nhất đạt được.
  if (v_lo > v_hi) {
    v_hi = v_lo = std::max(p.min_vel_x, std::min(v_lo, p.max_vel_x));
  }
  if (w_lo > w_hi) {
    const double w = std::clamp(w0, -p.max_vel_theta, p.max_vel_theta);
    w_lo = w_hi = w;
  }

  // ---------- Bước 4: lấy mẫu đều (gồm cả hai đầu mút) + mô phỏng ----------
  const auto sample = [](double lo, double hi, int k, int n) {
      return n <= 1 ? 0.5 * (lo + hi) : lo + (hi - lo) * static_cast<double>(k) / (n - 1);
    };
  std::vector<Trajectory> trajectories;
  trajectories.reserve(static_cast<std::size_t>(p.vx_samples * p.vtheta_samples));
  for (int i = 0; i < p.vx_samples; ++i) {
    const double v = sample(v_lo, v_hi, i, p.vx_samples);
    for (int j = 0; j < p.vtheta_samples; ++j) {
      const double w = sample(w_lo, w_hi, j, p.vtheta_samples);
      Trajectory t = simulate(p, robot, v, w);
      // ---------- Bước 5: kiểm tra va chạm + chấm điểm ----------
      scoreTrajectory(p, t, local_path, carrot);
      trajectories.push_back(std::move(t));
    }
  }

  // Chọn quỹ đạo hợp lệ có tổng chi phí nhỏ nhất.
  const Trajectory * best = nullptr;
  for (const auto & t : trajectories) {
    if (t.valid && (!best || t.total_cost < best->total_cost)) {
      best = &t;
    }
  }
  publishDebug(trajectories, best, local_path, carrot, costmap_ros_->getGlobalFrameID());

  if (!best) {
    // Mọi quỹ đạo đều va chạm: controller_server sẽ thử lại trong failure_tolerance rồi abort + dừng.
    throw nav2_core::PlannerException(
            "DwbController: no collision-free trajectory in the dynamic window");
  }
  return *best;
}

Trajectory DwbController::simulate(
  const DwbParams & p, const Pose2D & start, double v,
  double w) const
{
  // Mô hình robot vi sai (unicycle), tích phân Euler với bước sim_granularity:
  //   x += v cosθ Δt,  y += v sinθ Δt,  θ += ω Δt
  Trajectory t;
  t.v = v;
  t.w = w;
  const int steps = std::max(1, static_cast<int>(std::ceil(p.sim_time / p.sim_granularity)));
  t.poses.reserve(static_cast<std::size_t>(steps));
  Pose2D s = start;
  for (int k = 0; k < steps; ++k) {
    s.x += v * std::cos(s.theta) * p.sim_granularity;
    s.y += v * std::sin(s.theta) * p.sim_granularity;
    s.theta = normalizeAngle(s.theta + w * p.sim_granularity);
    t.poses.push_back(s);
  }
  return t;
}

void DwbController::scoreTrajectory(
  const DwbParams & p, Trajectory & t, const std::vector<Pose2D> & local_path,
  const Pose2D & carrot) const
{
  // (1) Va chạm + chi phí vật cản: duyệt mọi pose; một pose va chạm -> cả quỹ đạo bị loại.
  unsigned char max_cost = 0;
  for (const auto & s : t.poses) {
    if (isInCollision(p, s.x, s.y)) {
      t.valid = false;
      return;
    }
    max_cost = std::max(max_cost, costAt(s.x, s.y));
  }
  if (max_cost == NO_INFORMATION) {
    max_cost = MAX_NON_OBSTACLE;   // unknown được phép (unknown_is_obstacle:=false): coi như rất gần
  }

  const Pose2D & end = t.poses.back();
  // (2) obstacle: 0 (xa vật cản) .. 1 (sát vùng nội tiếp). Giữ khoảng cách với vật cản.
  t.obstacle_cost = static_cast<double>(max_cost) / static_cast<double>(MAX_NON_OBSTACLE);
  // (3) path: khoảng cách (m) từ cuối quỹ đạo tới đường A* cục bộ. Bám đường.
  t.path_cost = distanceToPath(end.x, end.y, local_path);
  // (4) carrot: khoảng cách (m) từ cuối quỹ đạo tới cà rốt. Tiến dọc đường về đích.
  t.carrot_cost = std::hypot(carrot.x - end.x, carrot.y - end.y);
  // (5) heading: lệch hướng ở cuối quỹ đạo so với hướng tới cà rốt, chuẩn hóa 0..1. Hướng đầu đúng đường.
  const double to_carrot = std::atan2(carrot.y - end.y, carrot.x - end.x);
  t.heading_cost = (t.carrot_cost > 0.05) ?
    std::fabs(normalizeAngle(to_carrot - end.theta)) / M_PI : 0.0;
  // (6) speed: 0 khi đi nhanh nhất, 1 khi đứng yên. Tránh "đứng yên cũng được điểm tốt".
  t.speed_cost = p.max_vel_x > 1e-6 ? (p.max_vel_x - t.v) / p.max_vel_x : 0.0;

  // Tổng có trọng số. Đổi trọng số = đổi "tính cách" robot (bám đường chặt / né vật cản xa / đi nhanh).
  t.total_cost =
    p.obstacle_scale * t.obstacle_cost +
    p.path_distance_scale * t.path_cost +
    p.carrot_distance_scale * t.carrot_cost +
    p.heading_scale * t.heading_cost +
    p.speed_scale * t.speed_cost;
}

// =====================================================================================================
// XOAY TẠI CHỖ (pha ROTATE_TO_PATH và ROTATE_TO_GOAL)
// =====================================================================================================

geometry_msgs::msg::Twist DwbController::rotateInPlace(
  const DwbParams & p, double error, const geometry_msgs::msg::Twist & velocity, double dt,
  const Pose2D & robot) const
{
  // Robot tròn xoay quanh tâm thì thân không quét thêm vùng mới -> chỉ cần tâm hiện tại không va chạm.
  // (Giả định tâm xoay = tâm thân tròn: CHƯA kiểm chứng trên robot.)
  if (isInCollision(p, robot.x, robot.y)) {
    throw nav2_core::PlannerException("DwbController: robot is in collision, cannot rotate in place");
  }
  // ω mong muốn tỉ lệ sai số góc, giới hạn |ω| <= max_vel_theta và >= min_in_place_vel_theta.
  double w_des = std::clamp(p.rotate_gain * error, -p.max_vel_theta, p.max_vel_theta);
  if (std::fabs(w_des) < p.min_in_place_vel_theta) {
    w_des = std::copysign(p.min_in_place_vel_theta, error);
  }
  // Tôn trọng gia tốc góc: |ω - ω0| <= acc_lim_theta * dt.
  const double w0 = velocity.angular.z;
  geometry_msgs::msg::Twist out;
  out.angular.z = std::clamp(w_des, w0 - p.acc_lim_theta * dt, w0 + p.acc_lim_theta * dt);
  // Tịnh tiến giảm dần về 0 theo decel_lim_x (không phanh gấp).
  out.linear.x = std::max(0.0, velocity.linear.x - p.decel_lim_x * dt);
  return out;
}

// =====================================================================================================
// TIỆN ÍCH
// =====================================================================================================

unsigned char DwbController::costAt(double x, double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return NO_INFORMATION;
  }
  return costmap_->getCost(mx, my);
}

bool DwbController::isInCollision(const DwbParams & p, double x, double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return true;   // ra ngoài local costmap: không biết gì -> không đi
  }
  const unsigned char c = costmap_->getCost(mx, my);
  if (c == NO_INFORMATION) {
    return p.unknown_is_obstacle;
  }
  return c >= INSCRIBED_INFLATED_OBSTACLE;   // 253 (nội tiếp) hoặc 254 (vật cản)
}

double DwbController::distanceToPath(double x, double y, const std::vector<Pose2D> & path)
{
  // Khoảng cách nhỏ nhất từ (x, y) tới các đoạn thẳng nối các điểm liên tiếp của đường.
  if (path.size() == 1) {
    return std::hypot(path[0].x - x, path[0].y - y);
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ax = path[i - 1].x, ay = path[i - 1].y;
    const double dx = path[i].x - ax, dy = path[i].y - ay;
    const double len2 = dx * dx + dy * dy;
    double u = len2 > 1e-12 ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
    u = std::clamp(u, 0.0, 1.0);
    best = std::min(best, std::hypot(ax + u * dx - x, ay + u * dy - y));
  }
  return best;
}

double DwbController::normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));   // về (-π, π]
}

// =====================================================================================================
// THAM SỐ
// =====================================================================================================

void DwbController::declareAndLoadParams()
{
  auto node = node_.lock();
  DwbParams d;   // giá trị mặc định (xem dwb_controller.hpp)
  const auto decl = [&](const std::string & key, const rclcpp::ParameterValue & def) {
      nav2_util::declare_parameter_if_not_declared(node, name_ + "." + key, def);
      return node->get_parameter(name_ + "." + key);
    };
  DwbParams p;
  p.max_vel_x = decl("max_vel_x", rclcpp::ParameterValue(d.max_vel_x)).as_double();
  p.min_vel_x = decl("min_vel_x", rclcpp::ParameterValue(d.min_vel_x)).as_double();
  p.max_vel_theta = decl("max_vel_theta", rclcpp::ParameterValue(d.max_vel_theta)).as_double();
  p.min_in_place_vel_theta =
    decl("min_in_place_vel_theta", rclcpp::ParameterValue(d.min_in_place_vel_theta)).as_double();
  p.acc_lim_x = decl("acc_lim_x", rclcpp::ParameterValue(d.acc_lim_x)).as_double();
  p.decel_lim_x = decl("decel_lim_x", rclcpp::ParameterValue(d.decel_lim_x)).as_double();
  p.acc_lim_theta = decl("acc_lim_theta", rclcpp::ParameterValue(d.acc_lim_theta)).as_double();
  p.vx_samples = decl("vx_samples", rclcpp::ParameterValue(d.vx_samples)).as_int();
  p.vtheta_samples = decl("vtheta_samples", rclcpp::ParameterValue(d.vtheta_samples)).as_int();
  p.sim_time = decl("sim_time", rclcpp::ParameterValue(d.sim_time)).as_double();
  p.sim_granularity =
    decl("sim_granularity", rclcpp::ParameterValue(d.sim_granularity)).as_double();
  p.local_plan_length =
    decl("local_plan_length", rclcpp::ParameterValue(d.local_plan_length)).as_double();
  p.carrot_distance =
    decl("carrot_distance", rclcpp::ParameterValue(d.carrot_distance)).as_double();
  p.closest_search_distance =
    decl("closest_search_distance", rclcpp::ParameterValue(d.closest_search_distance)).as_double();
  p.rotate_to_heading_angle =
    decl("rotate_to_heading_angle", rclcpp::ParameterValue(d.rotate_to_heading_angle)).as_double();
  p.rotate_gain = decl("rotate_gain", rclcpp::ParameterValue(d.rotate_gain)).as_double();
  p.xy_goal_tolerance =
    decl("xy_goal_tolerance", rclcpp::ParameterValue(d.xy_goal_tolerance)).as_double();
  p.yaw_goal_tolerance =
    decl("yaw_goal_tolerance", rclcpp::ParameterValue(d.yaw_goal_tolerance)).as_double();
  p.path_distance_scale =
    decl("path_distance_scale", rclcpp::ParameterValue(d.path_distance_scale)).as_double();
  p.carrot_distance_scale =
    decl("carrot_distance_scale", rclcpp::ParameterValue(d.carrot_distance_scale)).as_double();
  p.heading_scale = decl("heading_scale", rclcpp::ParameterValue(d.heading_scale)).as_double();
  p.obstacle_scale = decl("obstacle_scale", rclcpp::ParameterValue(d.obstacle_scale)).as_double();
  p.speed_scale = decl("speed_scale", rclcpp::ParameterValue(d.speed_scale)).as_double();
  p.unknown_is_obstacle =
    decl("unknown_is_obstacle", rclcpp::ParameterValue(d.unknown_is_obstacle)).as_bool();
  p.transform_tolerance =
    decl("transform_tolerance", rclcpp::ParameterValue(d.transform_tolerance)).as_double();
  p.publish_trajectories =
    decl("publish_trajectories", rclcpp::ParameterValue(d.publish_trajectories)).as_bool();
  p.trajectory_marker_stride =
    decl("trajectory_marker_stride", rclcpp::ParameterValue(d.trajectory_marker_stride)).as_int();

  const std::string reason = validate(p);
  if (!reason.empty()) {
    throw std::runtime_error("DwbController [" + name_ + "]: invalid parameters: " + reason);
  }
  std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = p;
}

rcl_interfaces::msg::SetParametersResult DwbController::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  // Callback được gọi cho MỌI tham số của controller_server: chỉ xử lý tham số "<name_>.xxx".
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(params_mutex_);
  DwbParams p = params_;
  // Bảng tra: tên tham số (không có tiền tố) -> trường tương ứng trong bản sao p.
  const std::map<std::string, double *> doubles{
    {"max_vel_x", &p.max_vel_x}, {"min_vel_x", &p.min_vel_x},
    {"max_vel_theta", &p.max_vel_theta}, {"min_in_place_vel_theta", &p.min_in_place_vel_theta},
    {"acc_lim_x", &p.acc_lim_x}, {"decel_lim_x", &p.decel_lim_x},
    {"acc_lim_theta", &p.acc_lim_theta}, {"sim_time", &p.sim_time},
    {"sim_granularity", &p.sim_granularity}, {"local_plan_length", &p.local_plan_length},
    {"carrot_distance", &p.carrot_distance},
    {"closest_search_distance", &p.closest_search_distance},
    {"rotate_to_heading_angle", &p.rotate_to_heading_angle}, {"rotate_gain", &p.rotate_gain},
    {"xy_goal_tolerance", &p.xy_goal_tolerance}, {"yaw_goal_tolerance", &p.yaw_goal_tolerance},
    {"path_distance_scale", &p.path_distance_scale},
    {"carrot_distance_scale", &p.carrot_distance_scale}, {"heading_scale", &p.heading_scale},
    {"obstacle_scale", &p.obstacle_scale}, {"speed_scale", &p.speed_scale},
    {"transform_tolerance", &p.transform_tolerance}};
  const std::map<std::string, int *> ints{
    {"vx_samples", &p.vx_samples}, {"vtheta_samples", &p.vtheta_samples},
    {"trajectory_marker_stride", &p.trajectory_marker_stride}};
  const std::map<std::string, bool *> bools{
    {"unknown_is_obstacle", &p.unknown_is_obstacle},
    {"publish_trajectories", &p.publish_trajectories}};

  const std::string prefix = name_ + ".";
  for (const auto & prm : params) {
    const std::string & full = prm.get_name();
    if (full.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string key = full.substr(prefix.size());
    try {
      if (auto it = doubles.find(key); it != doubles.end()) {
        // Kiểu do ROS 2 kiểm tra trước (double phải ghi dạng 0.5 / 5.0, không ghi 5).
        *it->second = prm.as_double();
      } else if (auto iti = ints.find(key); iti != ints.end()) {
        *iti->second = static_cast<int>(prm.as_int());
      } else if (auto itb = bools.find(key); itb != bools.end()) {
        *itb->second = prm.as_bool();
      }
      // Tên khác (vd. "plugin") không thuộc thuật toán: bỏ qua.
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & ex) {
      result.successful = false;
      result.reason = full + ": wrong type (" + ex.what() + ")";
      return result;
    }
  }
  // Kiểm tra tính hợp lệ trước khi áp dụng (giá trị sai -> từ chối, giữ tham số cũ).
  result.reason = validate(p);
  if (!result.reason.empty()) {
    result.successful = false;
    return result;
  }
  params_ = p;
  return result;
}

std::string DwbController::validate(const DwbParams & p)
{
  // Trả về "" nếu hợp lệ, ngược lại là lý do. Dùng cả lúc configure (YAML) và khi ros2 param set.
  if (p.max_vel_x < 0.0 || p.min_vel_x < 0.0 || p.min_vel_x > p.max_vel_x) {
    return "need 0 <= min_vel_x <= max_vel_x (reversing is not allowed: no rear LiDAR coverage)";
  }
  if (p.max_vel_theta <= 0.0 || p.acc_lim_x <= 0.0 || p.decel_lim_x <= 0.0 ||
    p.acc_lim_theta <= 0.0 || p.min_in_place_vel_theta < 0.0 ||
    p.min_in_place_vel_theta > p.max_vel_theta)
  {
    return "max_vel_theta, acc_lim_x, decel_lim_x, acc_lim_theta must be > 0 and "
           "0 <= min_in_place_vel_theta <= max_vel_theta";
  }
  if (p.vx_samples < 1 || p.vtheta_samples < 1 || p.sim_time <= 0.0 ||
    p.sim_granularity <= 0.0 || p.trajectory_marker_stride < 1)
  {
    return "samples >= 1, sim_time > 0, sim_granularity > 0, trajectory_marker_stride >= 1";
  }
  if (p.carrot_distance <= 0.0 || p.local_plan_length <= 0.0 || p.closest_search_distance <= 0.0 ||
    p.xy_goal_tolerance <= 0.0 || p.yaw_goal_tolerance <= 0.0 || p.transform_tolerance < 0.0)
  {
    return
      "carrot_distance, local_plan_length, closest_search_distance, goal tolerances must be > 0";
  }
  return "";
}

DwbParams DwbController::snapshotParams()
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  DwbParams p = params_;
  if (speed_limit_x_ > 0.0) {
    p.max_vel_x = std::min(p.max_vel_x, speed_limit_x_);
  }
  return p;
}

// =====================================================================================================
// HIỂN THỊ (RViz)
// =====================================================================================================

void DwbController::publishDebug(
  const std::vector<Trajectory> & trajectories, const Trajectory * best,
  const std::vector<Pose2D> & local_path, const Pose2D & carrot, const std::string & frame)
{
  const auto stamp = clock_->now();
  const auto to_pose = [&](const Pose2D & s) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame;
      ps.header.stamp = stamp;
      ps.pose.position.x = s.x;
      ps.pose.position.y = s.y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, s.theta);
      ps.pose.orientation = tf2::toMsg(q);
      return ps;
    };

  // /<plugin>/local_path: đoạn đường A* đang xét.
  if (local_path_pub_->is_activated()) {
    nav_msgs::msg::Path lp;
    lp.header.frame_id = frame;
    lp.header.stamp = stamp;
    for (const auto & s : local_path) {
      lp.poses.push_back(to_pose(s));
    }
    local_path_pub_->publish(lp);
  }

  // /local_plan: quỹ đạo được chọn (rỗng trong các pha xoay tại chỗ).
  if (local_plan_pub_->is_activated()) {
    nav_msgs::msg::Path best_path;
    best_path.header.frame_id = frame;
    best_path.header.stamp = stamp;
    if (best) {
      for (const auto & s : best->poses) {
        best_path.poses.push_back(to_pose(s));
      }
    }
    local_plan_pub_->publish(best_path);
  }

  DwbParams p = snapshotParams();
  if (!p.publish_trajectories || !markers_pub_->is_activated()) {
    return;
  }
  // /<plugin>/trajectories: quỹ đạo ứng viên (xanh lá = hợp lệ, đỏ = va chạm), quỹ đạo tốt nhất (xanh dương),
  // cà rốt (cầu vàng). Xóa marker cũ trước (DELETEALL).
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  int id = 0;
  const auto line = [&](const Trajectory & t, float r, float g, float b, double width) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = stamp;
      m.ns = "candidates";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = width;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.8f;
      for (const auto & s : t.poses) {
        geometry_msgs::msg::Point pt;
        pt.x = s.x;
        pt.y = s.y;
        m.points.push_back(pt);
      }
      return m;
    };
  for (std::size_t i = 0; i < trajectories.size();
    i += static_cast<std::size_t>(p.trajectory_marker_stride))
  {
    const auto & t = trajectories[i];
    arr.markers.push_back(
      t.valid ? line(t, 0.1f, 0.8f, 0.1f, 0.005) : line(
        t, 0.9f, 0.1f, 0.1f,
        0.005));
  }
  if (best) {
    auto m = line(*best, 0.0f, 0.4f, 1.0f, 0.02);
    m.ns = "best";
    arr.markers.push_back(m);
  }
  visualization_msgs::msg::Marker c;
  c.header.frame_id = frame;
  c.header.stamp = stamp;
  c.ns = "carrot";
  c.id = id++;
  c.type = visualization_msgs::msg::Marker::SPHERE;
  c.action = visualization_msgs::msg::Marker::ADD;
  c.pose.position.x = carrot.x;
  c.pose.position.y = carrot.y;
  c.pose.orientation.w = 1.0;
  c.scale.x = c.scale.y = c.scale.z = 0.08;
  c.color.r = 1.0f;
  c.color.g = 0.85f;
  c.color.a = 1.0f;
  arr.markers.push_back(c);
  markers_pub_->publish(arr);
}

}  // namespace stage3_navigation

// Đăng ký lớp với pluginlib: controller_server nạp bằng chuỗi "stage3_navigation::DwbController".
PLUGINLIB_EXPORT_CLASS(stage3_navigation::DwbController, nav2_core::Controller)
