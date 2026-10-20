// =====================================================================================================
// dwb_controller.cpp — Lớp vỏ ROS của CustomDWB (thuật toán ở dwa_core.cpp; tổng quan ở dwb_controller.hpp)
// =====================================================================================================

#include "stage3_navigation/dwb_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace stage3_navigation
{

namespace
{
// Bảng tên tham số -> trường của DwbParams (dùng chung cho declare và callback đổi tham số).
struct ParamRefs
{
  std::vector<std::pair<std::string, double *>> d;
  std::vector<std::pair<std::string, int *>> i;
  std::vector<std::pair<std::string, bool *>> b;
  std::vector<std::pair<std::string, std::string *>> s;
};

ParamRefs refs(DwbParams & p)
{
  ParamRefs r;
  r.d = {
    {"max_vel_x", &p.max_vel_x}, {"min_vel_x", &p.min_vel_x}, {"max_vel_theta", &p.max_vel_theta},
    {"min_in_place_vel_theta", &p.min_in_place_vel_theta}, {"acc_lim_x", &p.acc_lim_x},
    {"decel_lim_x", &p.decel_lim_x}, {"acc_lim_theta", &p.acc_lim_theta},
    {"decel_lim_theta", &p.decel_lim_theta}, {"sim_time", &p.sim_time},
    {"sim_granularity", &p.sim_granularity}, {"control_latency", &p.control_latency},
    {"stopping_margin", &p.stopping_margin}, {"max_feedback_gap_x", &p.max_feedback_gap_x},
    {"max_feedback_gap_theta", &p.max_feedback_gap_theta},
    {"local_plan_length", &p.local_plan_length},
    {"carrot_distance", &p.carrot_distance},
    {"closest_search_distance", &p.closest_search_distance},
    {"max_path_deviation", &p.max_path_deviation},
    {"rotate_to_heading_angle", &p.rotate_to_heading_angle}, {"rotate_gain", &p.rotate_gain},
    {"xy_goal_tolerance", &p.xy_goal_tolerance}, {"yaw_goal_tolerance", &p.yaw_goal_tolerance},
    {"goal_tolerance_fraction", &p.goal_tolerance_fraction},
    {"goal_latch_release_margin", &p.goal_latch_release_margin},
    {"path_distance_scale", &p.path_distance_scale},
    {"carrot_distance_scale", &p.carrot_distance_scale}, {"heading_scale", &p.heading_scale},
    {"obstacle_scale", &p.obstacle_scale}, {"speed_scale", &p.speed_scale},
    {"oscillation_scale", &p.oscillation_scale},
    {"oscillation_reset_dist", &p.oscillation_reset_dist},
    {"oscillation_reset_angle", &p.oscillation_reset_angle},
    {"oscillation_reset_time", &p.oscillation_reset_time},
    {"transform_tolerance", &p.transform_tolerance}, {"max_tf_age", &p.max_tf_age},
    {"max_pose_age", &p.max_pose_age}, {"odom_timeout", &p.odom_timeout},
    {"debug_publish_rate", &p.debug_publish_rate}};
  r.i = {{"vx_samples", &p.vx_samples}, {"vtheta_samples", &p.vtheta_samples},
    {"trajectory_marker_stride", &p.trajectory_marker_stride}};
  r.b = {{"unknown_is_obstacle", &p.unknown_is_obstacle},
    {"publish_trajectories", &p.publish_trajectories}};
  r.s = {{"collision_check", &p.collision_check}, {"velocity_reference", &p.velocity_reference}};
  return r;
}

bool finitePose(const geometry_msgs::msg::Pose & p)
{
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
         std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
}

// Yaw từ quaternion; false nếu quaternion không hữu hạn hoặc chuẩn ~ 0 (không chuẩn hóa được).
bool yawOf(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(n) || n < 1e-6) {
    return false;
  }
  tf2::Quaternion tq(q.x / n, q.y / n, q.z / n, q.w / n);
  yaw = tf2::getYaw(tq);
  return std::isfinite(yaw);
}

std::string fmt(double x, int prec = 3)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(prec) << x;
  return ss.str();
}
}  // namespace

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
    throw std::runtime_error("DwbController: parent node expired");
  }
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declareAndLoadParams();   // YAML sai -> runtime_error -> configure của controller_server thất bại

  // controller_frequency do controller_server khai báo -> dt danh nghĩa của dynamic window.
  if (node->has_parameter("controller_frequency")) {
    controller_frequency_ = node->get_parameter("controller_frequency").as_double();
  }
  if (!(controller_frequency_ > 0.0) || !std::isfinite(controller_frequency_)) {
    throw std::runtime_error("DwbController: controller_frequency must be > 0");
  }
  // controller_server Humble khai báo odom_topic SAU khi configure plugin (OdomSubscriber trong on_configure),
  // nên has_parameter() lúc này là false. Khai báo giống OdomSubscriber (mặc định "odom"; giá trị YAML
  // /odometry/filtered được áp dụng khi khai báo) để watchdog nghe đúng topic mà server dùng.
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_topic",
    rclcpp::ParameterValue(odom_topic_));
  odom_topic_ = node->get_parameter("odom_topic").as_string();

  // Watchdog odometry: chỉ ghi thời điểm nhận (thời gian ROS của node), không dùng nội dung.
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      last_odom_rx_ = clock_->now();
      have_odom_ = true;
    });

  // Publisher hiển thị (tên tương đối -> /local_plan, /<plugin>/local_path, /<plugin>/trajectories,
  // /<plugin>/state). Không publisher nào thuộc đường lệnh motor.
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_path", 1);
  markers_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/trajectories", 1);
  state_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(name_ + "/state", 10);

  param_cb_ = node->add_on_set_parameters_callback(
    std::bind(&DwbController::onParamChange, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "[%s] DwbController (self-written dynamic window) configured: max_vel_x %.2f, "
    "max_vel_theta %.2f, samples %dx%d, sim_time %.2f s, dt %.3f s, latency %.3f s, odom '%s'",
    name_.c_str(), params_.max_vel_x, params_.max_vel_theta, params_.vx_samples,
    params_.vtheta_samples, params_.sim_time, 1.0 / controller_frequency_, params_.control_latency,
    odom_topic_.c_str());
}

void DwbController::cleanup()
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  local_plan_pub_.reset();
  local_path_pub_.reset();
  markers_pub_.reset();
  state_pub_.reset();
  param_cb_.reset();
  odom_sub_.reset();
  tracker_.clear();
  plan_frame_.clear();
  resetState();
}

void DwbController::activate()
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  local_plan_pub_->on_activate();
  local_path_pub_->on_activate();
  markers_pub_->on_activate();
  state_pub_->on_activate();
  resetState();
  compute_ms_max_ = 0.0;
  cycles_ = over_budget_ = missed_deadline_ = 0;

  // Chính sách va chạm + vùng chưa quan sát: in rõ để người vận hành biết giả định đang dùng.
  DwbParams p;
  {
    std::lock_guard<std::mutex> plk(params_mutex_);
    p = params_;
  }
  std::string mode;
  makeCollisionModel(p, mode);
  const auto layered = costmap_ros_->getLayeredCostmap();
  RCLCPP_INFO(
    logger_, "[%s] collision check: %s (inscribed %.3f m, circumscribed %.3f m)", name_.c_str(),
    mode.c_str(), layered->getInscribedRadius(), layered->getCircumscribedRadius());
  if (p.unknown_is_obstacle && !layered->isTrackingUnknown()) {
    RCLCPP_WARN(
      logger_, "[%s] unknown_is_obstacle is true but the local costmap has track_unknown_space: false "
      "-> unobserved cells (e.g. the rear half without LiDAR) are FREE, not unknown. "
      "Forward-only motion is the actual protection.",
      name_.c_str());
  }
}

void DwbController::deactivate()
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  clearDebug();   // xóa marker/đường cũ trên RViz trước khi publisher ngừng
  local_plan_pub_->on_deactivate();
  local_path_pub_->on_deactivate();
  markers_pub_->on_deactivate();
  state_pub_->on_deactivate();
  resetState();
}

void DwbController::resetState()
{
  core_.reset();
  tracker_.resetProgress();
  last_cmd_ = Velocity2D{};
  have_last_cmd_ = false;
  have_last_compute_ = false;
  have_last_debug_ = false;
  last_phase_ = Phase::IDLE;
  last_track_ = PathTracker::Result{};
}

void DwbController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  // Path mới / goal thay thế: bỏ toàn bộ trạng thái theo plan (tiến trình, latch đích, xoay, dao động).
  core_.reset();
  last_phase_ = Phase::IDLE;
  plan_frame_.clear();
  if (path.header.frame_id.empty()) {
    tracker_.clear();
    throw nav2_core::PlannerException("DwbController: plan has an empty frame_id");
  }
  std::vector<Pose2D> pts;
  pts.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    pts.push_back(Pose2D{ps.pose.position.x, ps.pose.position.y, 0.0});
  }
  if (!path.poses.empty()) {
    double yaw = 0.0;
    if (!yawOf(path.poses.back().pose.orientation, yaw)) {
      tracker_.clear();
      throw nav2_core::PlannerException(
              "DwbController: plan goal orientation is not a valid quaternion");
    }
    pts.back().theta = yaw;
  }
  const std::string reason = tracker_.setPlan(pts);
  if (!reason.empty()) {
    throw nav2_core::PlannerException("DwbController: " + reason);
  }
  plan_frame_ = path.header.frame_id;
  RCLCPP_INFO(
    logger_, "[%s] new plan: %zu poses (%zu after merging duplicates), %.2f m, frame '%s'",
    name_.c_str(), path.poses.size(), tracker_.size(), tracker_.length(), plan_frame_.c_str());
}

void DwbController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    RCLCPP_WARN(
      logger_, "[%s] ignoring invalid speed limit %f (keeping the previous limit)", name_.c_str(),
      speed_limit);
    return;
  }
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT || (percentage && speed_limit >= 100.0)) {
    limit_mode_ = LimitMode::NONE;   // Humble: NO_SPEED_LIMIT (0.0) = bỏ giới hạn, về max_vel_x danh nghĩa
  } else if (percentage) {
    limit_mode_ = LimitMode::PERCENT;
    limit_value_ = speed_limit;
  } else {
    limit_mode_ = LimitMode::ABSOLUTE;
    limit_value_ = speed_limit;
  }
}

// =====================================================================================================
// HÀM CHÍNH — gọi mỗi chu kỳ điều khiển
// =====================================================================================================

geometry_msgs::msg::TwistStamped DwbController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto t_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(state_mutex_);

  // Một bản sao tham số cho cả chu kỳ (không đọc lại giữa chừng).
  double max_vel_x_eff = 0.0;
  const DwbParams p = snapshotParams(max_vel_x_eff);
  const double dt = 1.0 / controller_frequency_;   // dt danh nghĩa
  const rclcpp::Time now = clock_->now();

  // Thời gian giữa hai lần gọi: chỉ để thống kê + chọn lệnh trước. KHÔNG dùng để nới dynamic window (khoảng
  // trễ dài không được phép thành bước nhảy vận tốc lớn: cửa sổ dùng dt danh nghĩa; gốc = lệnh trước kẹp quanh
  // vận tốc đo, hoặc vận tốc đo nếu lệnh trước không còn đại diện — xem referenceVelocity).
  if (have_last_compute_) {
    const double gap = std::chrono::duration<double>(t_start - last_compute_).count();
    if (gap > 1.5 * dt) {
      ++missed_deadline_;
    }
    if (gap > 2.5 * dt || gap < 0.0) {
      have_last_cmd_ = false;   // bị ngắt quãng: lệnh trước không còn đại diện cho cái bánh xe đang làm
    }
  }
  last_compute_ = t_start;
  have_last_compute_ = true;

  // ---------- (a) Đầu vào ----------
  if (!tracker_.hasPlan()) {
    fail("no valid plan", p);
  }
  if (!finitePose(pose.pose) || !std::isfinite(velocity.linear.x) ||
    !std::isfinite(velocity.angular.z))
  {
    fail("non-finite robot pose or velocity input", p);
  }
  const std::string frame = costmap_ros_->getGlobalFrameID();
  if (pose.header.frame_id != frame) {
    fail("robot pose frame '" + pose.header.frame_id + "' != costmap frame '" + frame + "'", p);
  }
  const rclcpp::Time pose_stamp(pose.header.stamp, now.get_clock_type());
  if (pose_stamp.nanoseconds() != 0) {   // 0 = TF tĩnh (không có tuổi)
    const double age = (now - pose_stamp).seconds();
    if (age > p.max_pose_age) {
      fail("robot pose (TF " + frame + " -> base) is " + fmt(age, 2) + " s old > max_pose_age", p);
    }
  }
  {
    std::lock_guard<std::mutex> olk(odom_mutex_);
    if (!have_odom_) {
      fail("no odometry received on '" + odom_topic_ + "'", p);
    }
    const double age = (now - last_odom_rx_).seconds();
    if (age > p.odom_timeout) {
      fail("odometry '" + odom_topic_ + "' silent for " + fmt(age, 2) + " s > odom_timeout", p);
    }
  }
  double robot_yaw = 0.0;
  if (!yawOf(pose.pose.orientation, robot_yaw)) {
    fail("robot pose orientation is not a valid quaternion", p);
  }
  const Pose2D robot{pose.pose.position.x, pose.pose.position.y, robot_yaw};

  // ---------- (b) TF plan_frame -> costmap frame (một lần, mới nhất) ----------
  // transform_tolerance = thời gian CHỜ TF; max_tf_age = tuổi chấp nhận được. Hai đại lượng khác nhau.
  // AMCL đóng dấu map->odom vào TƯƠNG LAI (+transform_tolerance của AMCL, 1.0 s) -> tuổi âm là bình thường;
  // nếu AMCL ngừng, TF chỉ bị coi là cũ sau max_tf_age + 1.0 s.
  geometry_msgs::msg::TransformStamped tf_plan;
  try {
    tf_plan = tf_->lookupTransform(
      frame, plan_frame_, tf2::TimePointZero, tf2::durationFromSec(p.transform_tolerance));
  } catch (const tf2::TransformException & ex) {
    fail("cannot transform plan '" + plan_frame_ + "' -> '" + frame + "': " + ex.what(), p);
  }
  const rclcpp::Time tf_stamp(tf_plan.header.stamp, now.get_clock_type());
  if (tf_stamp.nanoseconds() != 0 && plan_frame_ != frame) {
    const double age = (now - tf_stamp).seconds();
    if (age > p.max_tf_age) {
      fail("TF " + plan_frame_ + " -> " + frame + " is " + fmt(age, 2) + " s old > max_tf_age", p);
    }
  }
  double tf_yaw = 0.0;
  if (!yawOf(
      tf_plan.transform.rotation,
      tf_yaw) || !std::isfinite(tf_plan.transform.translation.x) ||
    !std::isfinite(tf_plan.transform.translation.y))
  {
    fail("TF " + plan_frame_ + " -> " + frame + " is not finite", p);
  }
  const Transform2D plan_to_local{tf_plan.transform.translation.x, tf_plan.transform.translation.y,
    tf_yaw};

  // ---------- (c) Tiến trình dọc đường (frame plan) ----------
  const double hysteresis = std::max(2.0 * costmap_->getResolution(), 0.1);
  last_track_ = tracker_.update(
    plan_to_local.inverse().apply(robot), p.closest_search_distance, p.local_plan_length,
    hysteresis);
  // Plan một pose = chỉ có đích, không có "hành lang" để đo độ lệch (đích vẫn phải nằm trong local costmap).
  if (tracker_.size() > 1 && last_track_.deviation > p.max_path_deviation) {
    fail(
      "robot is " + fmt(last_track_.deviation, 2) + " m from the plan > max_path_deviation " +
      fmt(p.max_path_deviation, 2), p);
  }
  std::vector<Pose2D> local_all;
  local_all.reserve(last_track_.local.size());
  for (const auto & q : last_track_.local) {
    local_all.push_back(plan_to_local.apply(q));
  }
  const Pose2D goal = plan_to_local.apply(tracker_.goal());

  // ---------- (d) Dung sai đích = của GoalChecker ----------
  const GoalTolerance tol = goalTolerance(p, goal_checker);

  // ---------- (e) Tính lệnh (giữ khóa costmap ngắn nhất có thể) ----------
  const bool want_debug = debugDue(p, false);
  ControlOutput out;
  std::vector<Pose2D> local_path;
  std::string mode_name;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> clk(*(costmap_->getMutex()));
    for (const auto & q : local_all) {
      unsigned int mx, my;
      if (!costmap_->worldToMap(q.x, q.y, mx, my)) {
        break;   // ra khỏi local costmap
      }
      local_path.push_back(q);
    }
    if (local_path.empty()) {
      out.phase = Phase::BLOCKED;
      out.error = "no part of the plan lies inside the local costmap";
    } else {
      ControlInput in;
      in.robot = robot;
      in.velocity = Velocity2D{velocity.linear.x, velocity.angular.z};
      in.prev_cmd = have_last_cmd_ ? last_cmd_ : in.velocity;
      in.local_path = local_path;
      in.goal = goal;
      in.tol = tol;
      in.dt = dt;
      in.max_vel_x_eff = max_vel_x_eff;
      in.now_s = now.seconds();
      const CollisionModel cm = makeCollisionModel(p, mode_name);
      out = core_.compute(p, in, cm, want_debug && p.publish_trajectories);
    }
  }

  const double compute_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
  ++cycles_;
  compute_ms_max_ = std::max(compute_ms_max_, compute_ms);
  if (compute_ms > 0.8 * dt * 1000.0) {
    ++over_budget_;
  }

  // ---------- (f) Hiển thị + chẩn đoán (ngoài khóa costmap) ----------
  publishPaths(local_path, out.ok ? &out.chosen : nullptr, frame);
  const bool phase_changed = out.phase != last_phase_;
  if (phase_changed && out.ok) {
    RCLCPP_INFO(
      logger_, "[%s] phase %s -> %s (dist to goal %.2f m)", name_.c_str(), phaseName(last_phase_),
      phaseName(out.phase), out.dist_to_goal);
  }
  last_phase_ = out.phase;
  if (want_debug || phase_changed || !out.ok) {
    if (want_debug && p.publish_trajectories) {
      publishMarkers(out, frame, p.trajectory_marker_stride);
    }
    publishState(p, &out, out.ok ? "" : out.error, compute_ms);
  }
  if (!out.ok) {
    have_last_cmd_ = true;
    last_cmd_ = Velocity2D{};   // controller_server sẽ phát lệnh 0
    throw nav2_core::PlannerException("DwbController: " + out.error);
  }

  if (out.phase == Phase::FOLLOW_PATH && out.n_forward_valid == 0) {
    // Không bỏ qua vật cản: đứng chờ (lệnh v = 0); progress checker của server sẽ abort nếu không thông.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "[%s] no admissible forward trajectory (obstacle ahead, %d/%d candidates "
      "rejected): holding position; progress checker aborts if it does not clear",
      name_.c_str(),
      out.n_candidates - out.n_valid, out.n_candidates);
  }
  last_cmd_ = out.cmd;
  have_last_cmd_ = true;
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = now;
  cmd.twist.linear.x = out.cmd.v;
  cmd.twist.angular.z = out.cmd.w;
  RCLCPP_DEBUG(
    logger_,
    "[%s] %s v=%.3f w=%.3f cost=%.3f (obst %.2f path %.2f carrot %.2f head %.2f spd %.2f osc %.0f) "
    "valid %d/%d %.2f ms", name_.c_str(), phaseName(out.phase), out.cmd.v, out.cmd.w,
    out.chosen.total_cost, out.chosen.obstacle_cost, out.chosen.path_cost, out.chosen.carrot_cost,
    out.chosen.heading_cost, out.chosen.speed_cost, out.chosen.oscillation_cost, out.n_valid,
    out.n_candidates, compute_ms);
  return cmd;
}

void DwbController::fail(const std::string & reason, const DwbParams & p)
{
  have_last_cmd_ = true;
  last_cmd_ = Velocity2D{};
  last_phase_ = Phase::BLOCKED;
  publishState(p, nullptr, reason, 0.0);
  throw nav2_core::PlannerException("DwbController: " + reason);
}

CollisionModel DwbController::makeCollisionModel(const DwbParams & p, std::string & mode_name) const
{
  // center: ô dưới tâm >= INSCRIBED. Chỉ đúng khi footprint TRÒN (robot_radius) và tâm xoay = tâm thân, vì khi
  // đó inflation layer đã "bơm" vật cản đúng bằng bán kính nội tiếp (gồm footprint_padding).
  // footprint: thêm kiểm tra viền footprint (đa giác, đã gồm padding). auto: center nếu costmap dùng robot_radius.
  const bool center = p.collision_check == "center" ||
    (p.collision_check == "auto" && costmap_ros_->getUseRadius());
  std::vector<Pose2D> fp;
  if (!center) {
    for (const auto & pt : costmap_ros_->getRobotFootprint()) {
      fp.push_back(Pose2D{pt.x, pt.y, 0.0});
    }
  }
  CollisionModel cm(
    costmap_, center ? CollisionMode::CENTER : CollisionMode::FOOTPRINT, fp, p.unknown_is_obstacle);
  mode_name = cm.mode() == CollisionMode::CENTER ? "center" : "footprint";
  return cm;
}

GoalTolerance DwbController::goalTolerance(
  const DwbParams & p,
  nav2_core::GoalChecker * goal_checker)
{
  GoalTolerance tol{p.xy_goal_tolerance, p.yaw_goal_tolerance};
  if (!goal_checker) {
    return tol;
  }
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (!goal_checker->getTolerances(pose_tol, vel_tol)) {
    return tol;
  }
  double yaw = 0.0;
  const bool xy_ok = std::isfinite(pose_tol.position.x) && pose_tol.position.x > 0.0;
  const bool yaw_ok = yawOf(pose_tol.orientation, yaw) && std::fabs(yaw) > 0.0;
  if (xy_ok) {
    tol.xy = pose_tol.position.x;
  }
  if (yaw_ok) {
    tol.yaw = std::fabs(yaw);
  }
  if (!warned_tolerance_mismatch_ &&
    ((xy_ok && std::fabs(tol.xy - p.xy_goal_tolerance) > 1e-6) ||
    (yaw_ok && std::fabs(tol.yaw - p.yaw_goal_tolerance) > 1e-6)))
  {
    warned_tolerance_mismatch_ = true;
    RCLCPP_WARN(
      logger_, "[%s] goal checker tolerances (xy %.3f m, yaw %.3f rad) differ from %s.xy/yaw_goal_tolerance "
      "(%.3f, %.3f): using the goal checker's values",
      name_.c_str(), tol.xy, tol.yaw, name_.c_str(),
      p.xy_goal_tolerance, p.yaw_goal_tolerance);
  }
  return tol;
}

// =====================================================================================================
// THAM SỐ
// =====================================================================================================

void DwbController::declareAndLoadParams()
{
  auto node = node_.lock();
  DwbParams d;   // mặc định (dwa_core.hpp)
  DwbParams p;
  ParamRefs rd = refs(d);
  ParamRefs rp = refs(p);
  const auto key = [&](const std::string & k) {return name_ + "." + k;};
  for (std::size_t k = 0; k < rd.d.size(); ++k) {
    nav2_util::declare_parameter_if_not_declared(
      node, key(rd.d[k].first), rclcpp::ParameterValue(*rd.d[k].second));
    *rp.d[k].second = node->get_parameter(key(rd.d[k].first)).as_double();
  }
  for (std::size_t k = 0; k < rd.i.size(); ++k) {
    nav2_util::declare_parameter_if_not_declared(
      node, key(rd.i[k].first), rclcpp::ParameterValue(*rd.i[k].second));
    *rp.i[k].second = static_cast<int>(node->get_parameter(key(rd.i[k].first)).as_int());
  }
  for (std::size_t k = 0; k < rd.b.size(); ++k) {
    nav2_util::declare_parameter_if_not_declared(
      node, key(rd.b[k].first), rclcpp::ParameterValue(*rd.b[k].second));
    *rp.b[k].second = node->get_parameter(key(rd.b[k].first)).as_bool();
  }
  for (std::size_t k = 0; k < rd.s.size(); ++k) {
    nav2_util::declare_parameter_if_not_declared(
      node, key(rd.s[k].first), rclcpp::ParameterValue(*rd.s[k].second));
    *rp.s[k].second = node->get_parameter(key(rd.s[k].first)).as_string();
  }

  const std::string reason = validateParams(p);
  if (!reason.empty()) {
    throw std::runtime_error("DwbController [" + name_ + "]: invalid parameters: " + reason);
  }
  std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = p;
}

rcl_interfaces::msg::SetParametersResult DwbController::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  // Gọi cho MỌI tham số của controller_server: chỉ xử lý "<name_>.xxx". Cả lô được kiểm tra trên một BẢN SAO;
  // chỉ khi toàn bộ hợp lệ mới thay params_ (nguyên tử). Lỗi -> giữ nguyên tham số cũ.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(params_mutex_);
  DwbParams p = params_;
  ParamRefs r = refs(p);
  const std::string prefix = name_ + ".";
  bool touched = false;
  for (const auto & prm : params) {
    const std::string & full = prm.get_name();
    if (full.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string k = full.substr(prefix.size());
    try {
      bool found = false;
      for (auto & e : r.d) {
        if (e.first == k) {*e.second = prm.as_double(); found = true;}
      }
      for (auto & e : r.i) {
        if (e.first == k) {*e.second = static_cast<int>(prm.as_int()); found = true;}
      }
      for (auto & e : r.b) {
        if (e.first == k) {*e.second = prm.as_bool(); found = true;}
      }
      for (auto & e : r.s) {
        if (e.first == k) {*e.second = prm.as_string(); found = true;}
      }
      touched = touched || found;   // tên khác (vd. "plugin") không thuộc thuật toán: bỏ qua
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & ex) {
      result.successful = false;
      result.reason = full + ": wrong type (" + ex.what() + ")";
      return result;
    }
  }
  if (!touched) {
    return result;
  }
  result.reason = validateParams(p);
  if (!result.reason.empty()) {
    result.successful = false;
    return result;
  }
  params_ = p;
  return result;
}

DwbParams DwbController::snapshotParams(double & max_vel_x_eff)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  DwbParams p = params_;
  max_vel_x_eff = p.max_vel_x;
  if (limit_mode_ == LimitMode::PERCENT) {
    max_vel_x_eff = p.max_vel_x * limit_value_ / 100.0;
  } else if (limit_mode_ == LimitMode::ABSOLUTE) {
    max_vel_x_eff = std::min(p.max_vel_x, limit_value_);
  }
  return p;
}

// =====================================================================================================
// HIỂN THỊ / CHẨN ĐOÁN
// =====================================================================================================

bool DwbController::debugDue(const DwbParams & p, bool force)
{
  if (p.debug_publish_rate <= 0.0) {
    return false;
  }
  const auto t = std::chrono::steady_clock::now();
  if (!force && have_last_debug_ &&
    std::chrono::duration<double>(t - last_debug_).count() < 1.0 / p.debug_publish_rate - 1e-3)
  {
    return false;
  }
  last_debug_ = t;
  have_last_debug_ = true;
  return true;
}

void DwbController::publishState(
  const DwbParams & p, const ControlOutput * out, const std::string & error, double compute_ms)
{
  if (!state_pub_ || !state_pub_->is_activated() || p.debug_publish_rate <= 0.0) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = name_;
  st.hardware_id = "controller_server";
  const Phase ph = out ? out->phase : Phase::BLOCKED;
  st.level = error.empty() ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  st.message = error.empty() ? phaseName(ph) : std::string(phaseName(ph)) + ": " + error;
  const auto kv = [&st](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    };
  kv("stamp", fmt(clock_->now().seconds(), 3));
  kv("phase", phaseName(ph));
  kv("compute_ms", fmt(compute_ms, 3));
  kv("compute_ms_max", fmt(compute_ms_max_, 3));
  kv("cycles", std::to_string(cycles_));
  kv("over_budget", std::to_string(over_budget_));
  kv("missed_deadline", std::to_string(missed_deadline_));
  kv("progress", fmt(last_track_.progress, 3));
  kv("deviation", fmt(last_track_.deviation, 3));
  kv("osc_flips", std::to_string(core_.oscillation().flips()));
  if (out) {
    kv("v_cmd", fmt(out->cmd.v, 4));
    kv("w_cmd", fmt(out->cmd.w, 4));
    kv("dist_to_goal", fmt(out->dist_to_goal, 3));
    kv("v_meas", fmt(out->measured.v, 4));
    kv("w_meas", fmt(out->measured.w, 4));
    kv("v_ref", fmt(out->reference.v, 4));
    kv("w_ref", fmt(out->reference.w, 4));
    kv("heading_error", fmt(out->heading_error, 4));
    kv("n_candidates", std::to_string(out->n_candidates));
    kv("n_valid", std::to_string(out->n_valid));
    kv("n_forward_valid", std::to_string(out->n_forward_valid));
    kv("rej_collision", std::to_string(out->n_reject[static_cast<int>(Reject::COLLISION)]));
    kv("rej_unknown", std::to_string(out->n_reject[static_cast<int>(Reject::UNKNOWN)]));
    kv("rej_off_costmap", std::to_string(out->n_reject[static_cast<int>(Reject::OFF_MAP)]));
    kv("rej_braking", std::to_string(out->n_reject[static_cast<int>(Reject::BRAKING)]));
    kv("overspeed", (out->window.v_overspeed || out->window.w_overspeed) ? "true" : "false");
    kv("cost_total", fmt(out->chosen.total_cost, 4));
    kv("cost_obstacle", fmt(out->chosen.obstacle_cost, 4));
    kv("cost_path", fmt(out->chosen.path_cost, 4));
    kv("cost_carrot", fmt(out->chosen.carrot_cost, 4));
    kv("cost_heading", fmt(out->chosen.heading_cost, 4));
    kv("cost_speed", fmt(out->chosen.speed_cost, 4));
    kv("cost_oscillation", fmt(out->chosen.oscillation_cost, 4));
  }
  state_pub_->publish(st);
}

void DwbController::publishPaths(
  const std::vector<Pose2D> & local_path, const Trajectory * chosen, const std::string & frame)
{
  const auto stamp = clock_->now();
  const auto to_path = [&](const std::vector<Pose2D> & pts) {
      nav_msgs::msg::Path out;
      out.header.frame_id = frame;
      out.header.stamp = stamp;
      for (const auto & s : pts) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = out.header;
        ps.pose.position.x = s.x;
        ps.pose.position.y = s.y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, s.theta);
        ps.pose.orientation = tf2::toMsg(q);
        out.poses.push_back(ps);
      }
      return out;
    };
  if (local_path_pub_ && local_path_pub_->is_activated()) {
    local_path_pub_->publish(to_path(local_path));
  }
  // /local_plan: quỹ đạo của lệnh được chọn (gồm cả pha xoay: trễ + một chu kỳ); rỗng khi BLOCKED.
  if (local_plan_pub_ && local_plan_pub_->is_activated()) {
    local_plan_pub_->publish(to_path(chosen ? chosen->poses : std::vector<Pose2D>{}));
  }
}

void DwbController::clearDebug()
{
  if (local_plan_pub_ && local_plan_pub_->is_activated()) {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = costmap_ros_->getGlobalFrameID();
    empty.header.stamp = clock_->now();
    local_plan_pub_->publish(empty);
    local_path_pub_->publish(empty);
  }
  if (markers_pub_ && markers_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    markers_pub_->publish(arr);
  }
}

void DwbController::publishMarkers(const ControlOutput & out, const std::string & frame, int stride)
{
  if (!markers_pub_ || !markers_pub_->is_activated()) {
    return;
  }
  // Ứng viên: xanh lá = hợp lệ, đỏ = bị loại; quỹ đạo chọn: xanh dương; đuôi phanh: cam; cà rốt: cầu vàng.
  const auto stamp = clock_->now();
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  int id = 0;
  const auto line = [&](const std::vector<Pose2D> & pts, const char * ns, float r, float g, float b,
      double width) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = width;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.8f;
      for (const auto & s : pts) {
        geometry_msgs::msg::Point pt;
        pt.x = s.x;
        pt.y = s.y;
        m.points.push_back(pt);
      }
      return m;
    };
  for (std::size_t i = 0; i < out.candidates.size(); i += static_cast<std::size_t>(stride)) {
    const auto & t = out.candidates[i];
    arr.markers.push_back(
      t.valid() ? line(t.poses, "candidates", 0.1f, 0.8f, 0.1f, 0.005) :
      line(t.poses, "candidates", 0.9f, 0.1f, 0.1f, 0.005));
  }
  if (out.ok && out.chosen.poses.size() > 1) {
    arr.markers.push_back(line(out.chosen.poses, "best", 0.0f, 0.4f, 1.0f, 0.02));
  }
  if (out.ok && out.chosen.braking.size() > 1) {
    arr.markers.push_back(line(out.chosen.braking, "braking", 1.0f, 0.5f, 0.0f, 0.012));
  }
  visualization_msgs::msg::Marker c;
  c.header.frame_id = frame;
  c.header.stamp = stamp;
  c.ns = "carrot";
  c.id = id++;
  c.type = visualization_msgs::msg::Marker::SPHERE;
  c.action = visualization_msgs::msg::Marker::ADD;
  c.pose.position.x = out.carrot.x;
  c.pose.position.y = out.carrot.y;
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
