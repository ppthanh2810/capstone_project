// =====================================================================================================
// dwa_core.cpp — Lõi Dynamic Window của CustomDWB (xem tổng quan ở dwa_core.hpp và dwb_controller.hpp)
// =====================================================================================================
//
// Giá trị ô costmap (nav2_costmap_2d): 0 = trống, 1..252 = gần vật cản (inflation), 253 = INSCRIBED (tâm robot
// ở đây thì thân robot chạm vật cản), 254 = LETHAL, 255 = NO_INFORMATION.
// =====================================================================================================

#include "stage3_navigation/dwa_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace stage3_navigation
{

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::MAX_NON_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace
{
// Khử trùng mẫu: hai giá trị cách nhau < kSampleEps coi là một.
constexpr double kSampleEps = 1e-9;
// Ngưỡng đổi chiều xoay khi sai số góc gần ±π: trong vùng này giữ chiều đã chọn (chống lật trái/phải).
constexpr double kFlipDeadband = 0.35;   // rad
// Khoảng cách cà rốt nhỏ hơn mức này: không tính hướng tới cà rốt (atan2 không ổn định).
constexpr double kMinCarrotDist = 0.05;  // m

bool finite(double x) {return std::isfinite(x);}

void integrateArc(Pose2D & s, double v, double w, double h)
{
  // Tích phân CHÍNH XÁC chuyển động (v, ω) hằng trong thời gian h (cung tròn / đường thẳng).
  if (std::fabs(w) < 1e-9) {
    s.x += v * h * std::cos(s.theta);
    s.y += v * h * std::sin(s.theta);
  } else {
    const double th1 = s.theta + w * h;
    s.x += v / w * (std::sin(th1) - std::sin(s.theta));
    s.y -= v / w * (std::cos(th1) - std::cos(s.theta));
    s.theta = th1;
  }
  s.theta = normalizeAngle(s.theta);
}
}  // namespace

// =====================================================================================================
// TIỆN ÍCH
// =====================================================================================================

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));   // về (-π, π]
}

Pose2D Transform2D::apply(const Pose2D & p) const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return Pose2D{c * p.x - s * p.y + x, s * p.x + c * p.y + y, normalizeAngle(p.theta + yaw)};
}

Transform2D Transform2D::inverse() const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return Transform2D{-(c * x + s * y), -(-s * x + c * y), -yaw};
}

// =====================================================================================================
// THAM SỐ
// =====================================================================================================

std::size_t estimateRolloutPoses(const DwbParams & p)
{
  if (!(p.sim_granularity > 0.0) || !(p.decel_lim_x > 0.0) || !(p.decel_lim_theta > 0.0)) {
    return std::numeric_limits<std::size_t>::max();
  }
  const double brake = p.max_vel_x / p.decel_lim_x + p.max_vel_theta / p.decel_lim_theta;
  const double steps = std::ceil((p.sim_time + p.control_latency + brake) / p.sim_granularity) +
    2.0;
  const double total = static_cast<double>(std::max(p.vx_samples, 1)) *
    static_cast<double>(std::max(p.vtheta_samples, 1) + 1) * steps;   // +1: mẫu ω = 0 được thêm vào
  if (!finite(total) || total > 1e12) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(total);
}

std::string validateParams(const DwbParams & p)
{
  // Mọi số thực phải hữu hạn: NaN làm mọi phép so sánh "<" / ">" sai và lọt qua kiểm tra miền.
  const std::pair<const char *, double> all[] = {
    {"max_vel_x", p.max_vel_x}, {"min_vel_x", p.min_vel_x}, {"max_vel_theta", p.max_vel_theta},
    {"min_in_place_vel_theta", p.min_in_place_vel_theta}, {"acc_lim_x", p.acc_lim_x},
    {"decel_lim_x", p.decel_lim_x}, {"acc_lim_theta", p.acc_lim_theta},
    {"decel_lim_theta", p.decel_lim_theta}, {"sim_time", p.sim_time},
    {"sim_granularity", p.sim_granularity}, {"control_latency", p.control_latency},
    {"stopping_margin", p.stopping_margin}, {"max_feedback_gap_x", p.max_feedback_gap_x},
    {"max_feedback_gap_theta", p.max_feedback_gap_theta},
    {"local_plan_length", p.local_plan_length},
    {"carrot_distance", p.carrot_distance}, {"closest_search_distance", p.closest_search_distance},
    {"max_path_deviation", p.max_path_deviation},
    {"rotate_to_heading_angle", p.rotate_to_heading_angle}, {"rotate_gain", p.rotate_gain},
    {"xy_goal_tolerance", p.xy_goal_tolerance}, {"yaw_goal_tolerance", p.yaw_goal_tolerance},
    {"goal_tolerance_fraction", p.goal_tolerance_fraction},
    {"goal_latch_release_margin", p.goal_latch_release_margin},
    {"path_distance_scale", p.path_distance_scale},
    {"carrot_distance_scale", p.carrot_distance_scale}, {"heading_scale", p.heading_scale},
    {"obstacle_scale", p.obstacle_scale}, {"speed_scale", p.speed_scale},
    {"oscillation_scale", p.oscillation_scale},
    {"oscillation_reset_dist", p.oscillation_reset_dist},
    {"oscillation_reset_angle", p.oscillation_reset_angle},
    {"oscillation_reset_time", p.oscillation_reset_time},
    {"transform_tolerance", p.transform_tolerance}, {"max_tf_age", p.max_tf_age},
    {"max_pose_age", p.max_pose_age}, {"odom_timeout", p.odom_timeout},
    {"debug_publish_rate", p.debug_publish_rate}};
  for (const auto & kv : all) {
    if (!finite(kv.second)) {
      return std::string(kv.first) + " must be finite";
    }
  }
  // Giới hạn trên là kiểm tra hợp lý (sanity), không phải số đo: teleop tối đa 0.5 m/s, 0.6 rad/s.
  if (!(p.max_vel_x > 0.0 && p.max_vel_x <= 1.0)) {
    return "need 0 < max_vel_x <= 1.0 m/s";
  }
  if (p.min_vel_x < 0.0 || p.min_vel_x > p.max_vel_x) {
    return "need 0 <= min_vel_x <= max_vel_x (reversing is not allowed: no rear LiDAR coverage)";
  }
  if (!(p.max_vel_theta > 0.0 && p.max_vel_theta <= 3.0)) {
    return "need 0 < max_vel_theta <= 3.0 rad/s";
  }
  if (p.min_in_place_vel_theta < 0.0 || p.min_in_place_vel_theta > p.max_vel_theta) {
    return "need 0 <= min_in_place_vel_theta <= max_vel_theta";
  }
  if (!(p.acc_lim_x > 0.0 && p.acc_lim_x <= 5.0) ||
    !(p.decel_lim_x > 0.0 && p.decel_lim_x <= 5.0) ||
    !(p.acc_lim_theta > 0.0 && p.acc_lim_theta <= 10.0) ||
    !(p.decel_lim_theta > 0.0 && p.decel_lim_theta <= 10.0))
  {
    return "need 0 < acc_lim_x, decel_lim_x <= 5.0 and 0 < acc_lim_theta, decel_lim_theta <= 10.0 "
           "(decel limits are positive magnitudes)";
  }
  if (p.vx_samples < 1 || p.vx_samples > kMaxSamplesPerAxis || p.vtheta_samples < 1 ||
    p.vtheta_samples > kMaxSamplesPerAxis)
  {
    return "need 1 <= vx_samples, vtheta_samples <= " + std::to_string(kMaxSamplesPerAxis);
  }
  if (!(p.sim_time > 0.0 && p.sim_time <= 5.0)) {
    return "need 0 < sim_time <= 5.0 s";
  }
  if (!(p.sim_granularity >= 0.005 && p.sim_granularity <= p.sim_time)) {
    return "need 0.005 <= sim_granularity <= sim_time";
  }
  if (p.control_latency < 0.0 || p.control_latency > 1.0) {
    return "need 0 <= control_latency <= 1.0 s";
  }
  if (p.stopping_margin < 0.0 || p.stopping_margin > 0.5) {
    return "need 0 <= stopping_margin <= 0.5 m";
  }
  if (p.velocity_reference != "command" && p.velocity_reference != "measured") {
    return "velocity_reference must be one of: command, measured";
  }
  if (!(p.max_feedback_gap_x > 0.0 && p.max_feedback_gap_x <= 1.0) ||
    !(p.max_feedback_gap_theta > 0.0 && p.max_feedback_gap_theta <= 3.0))
  {
    return "need 0 < max_feedback_gap_x <= 1.0 m/s and 0 < max_feedback_gap_theta <= 3.0 rad/s";
  }
  if (!(p.local_plan_length > 0.0) || !(p.carrot_distance > 0.0) ||
    p.carrot_distance > p.local_plan_length || !(p.closest_search_distance > 0.0) ||
    !(p.max_path_deviation > 0.0))
  {
    return "need local_plan_length, closest_search_distance, max_path_deviation > 0 and "
           "0 < carrot_distance <= local_plan_length";
  }
  if (!(p.rotate_to_heading_angle > 0.0 && p.rotate_to_heading_angle <= M_PI) ||
    !(p.rotate_gain > 0.0 && p.rotate_gain <= 20.0))
  {
    return "need 0 < rotate_to_heading_angle <= pi and 0 < rotate_gain <= 20";
  }
  if (!(p.xy_goal_tolerance > 0.0) ||
    !(p.yaw_goal_tolerance > 0.0 && p.yaw_goal_tolerance <= M_PI) ||
    !(p.goal_tolerance_fraction > 0.0 && p.goal_tolerance_fraction <= 1.0) ||
    p.goal_latch_release_margin < 0.0)
  {
    return "need xy_goal_tolerance > 0, 0 < yaw_goal_tolerance <= pi, "
           "0 < goal_tolerance_fraction <= 1, goal_latch_release_margin >= 0";
  }
  if (p.path_distance_scale < 0.0 || p.carrot_distance_scale < 0.0 || p.heading_scale < 0.0 ||
    p.obstacle_scale < 0.0 || p.speed_scale < 0.0 || p.oscillation_scale < 0.0)
  {
    return "cost weights (*_scale) must be >= 0";
  }
  if (p.path_distance_scale + p.carrot_distance_scale <= 0.0) {
    return
      "path_distance_scale + carrot_distance_scale must be > 0 (otherwise nothing follows the path)";
  }
  if (p.oscillation_reset_dist < 0.0 || p.oscillation_reset_angle < 0.0 ||
    !(p.oscillation_reset_time > 0.0))
  {
    return
      "need oscillation_reset_dist, oscillation_reset_angle >= 0 and oscillation_reset_time > 0";
  }
  if (p.transform_tolerance < 0.0 || p.transform_tolerance > 2.0 || !(p.max_tf_age > 0.0) ||
    !(p.max_pose_age > 0.0) || !(p.odom_timeout > 0.0))
  {
    return "need 0 <= transform_tolerance <= 2.0 and max_tf_age, max_pose_age, odom_timeout > 0";
  }
  if (p.collision_check != "auto" && p.collision_check != "center" &&
    p.collision_check != "footprint")
  {
    return "collision_check must be one of: auto, center, footprint";
  }
  if (p.trajectory_marker_stride < 1 || p.debug_publish_rate < 0.0 || p.debug_publish_rate > 50.0) {
    return "need trajectory_marker_stride >= 1 and 0 <= debug_publish_rate <= 50";
  }
  const std::size_t load = estimateRolloutPoses(p);
  if (load > kMaxRolloutPosesPerCycle) {
    return "computational load too high: ~" + std::to_string(load) +
           " simulated poses per cycle > " +
           std::to_string(kMaxRolloutPosesPerCycle) +
           " (reduce vx_samples, vtheta_samples, sim_time or increase sim_granularity)";
  }
  return "";
}

// =====================================================================================================
// DYNAMIC WINDOW
// =====================================================================================================

Range reachableRange(double x0, double acc, double decel, double dt)
{
  // Biên trên: tăng x. Nếu x0 < 0 thì tăng x = giảm |x| (decel) tới 0, sau đó tăng |x| theo chiều dương (acc).
  const auto upper = [&](double x) {
      if (x >= 0.0) {
        return x + acc * dt;
      }
      const double t0 = -x / decel;
      return t0 >= dt ? x + decel * dt : acc * (dt - t0);
    };
  return Range{-upper(-x0), upper(x0)};
}

double stepToward(double cur, double target, double acc, double decel, double h)
{
  const Range r = reachableRange(cur, acc, decel, h);
  return std::clamp(target, r.lo, r.hi);
}

DynamicWindow computeWindow(
  const DwbParams & p, double max_vel_x_eff, const Velocity2D & current, double dt)
{
  DynamicWindow dw;
  const double vmax = max_vel_x_eff;
  const double vmin = std::min(p.min_vel_x, vmax);
  const Range rv = reachableRange(current.v, p.acc_lim_x, p.decel_lim_x, dt);
  dw.v = Range{std::max(rv.lo, vmin), std::min(rv.hi, vmax)};
  if (dw.v.lo > dw.v.hi) {
    // Tập đạt được nằm ngoài [vmin, vmax] (vd. v0 = 0.50 > vmax = 0.15, decel 0.5, dt 0.1 -> sau một chu kỳ
    // thấp nhất là 0.45). KHÔNG thể vừa giữ hard cap vừa giữ decel_lim trong chu kỳ này. Chính sách: lệnh =
    // biên gần nhất (vmax) -> lệnh luôn trong giới hạn; mô phỏng (rolloutCommand) dùng chuyển động thật
    // (giảm dần theo decel_lim) nên kiểm tra va chạm/phanh không giả định giảm tốc tức thời.
    dw.v_overspeed = true;
    const double v = rv.lo > vmax ? vmax : vmin;
    dw.v = Range{v, v};
  }
  const Range rw = reachableRange(current.w, p.acc_lim_theta, p.decel_lim_theta, dt);
  dw.w = Range{std::max(rw.lo, -p.max_vel_theta), std::min(rw.hi, p.max_vel_theta)};
  if (dw.w.lo > dw.w.hi) {
    dw.w_overspeed = true;
    const double w = rw.lo > p.max_vel_theta ? p.max_vel_theta : -p.max_vel_theta;
    dw.w = Range{w, w};
  }
  return dw;
}

Velocity2D referenceVelocity(const DwbParams & p, const ControlInput & in)
{
  // Với phản hồi trễ τ (driver ramp + lọc odometry), cửa sổ tính từ vận tốc ĐO chỉ cho lệnh nhích acc·dt mỗi τ
  // -> gia tốc/phanh thực ~acc·dt/τ (log robot 2026-10-14: ~0.12 rad/s² thay vì 1.0, xoay lố ~0.95 rad).
  // Tính từ lệnh trước thì lệnh đổi đúng acc/decel_lim. Kẹp quanh vận tốc đo: khi robot không theo lệnh
  // (PS4 chiếm twist_mux, bánh kẹt) gốc không trôi xa thực tế.
  if (p.velocity_reference == "measured") {
    return in.velocity;
  }
  return Velocity2D{
    std::clamp(
      in.prev_cmd.v, in.velocity.v - p.max_feedback_gap_x,
      in.velocity.v + p.max_feedback_gap_x),
    std::clamp(
      in.prev_cmd.w, in.velocity.w - p.max_feedback_gap_theta,
      in.velocity.w + p.max_feedback_gap_theta)};
}

std::vector<double> sampleAxis(double lo, double hi, int n)
{
  std::vector<double> out;
  n = std::max(n, 1);
  out.reserve(static_cast<std::size_t>(n) + 1);
  if (n == 1 || hi - lo < kSampleEps) {
    out.push_back(n == 1 ? 0.5 * (lo + hi) : lo);
  } else {
    for (int k = 0; k < n - 1; ++k) {
      out.push_back(lo + (hi - lo) * static_cast<double>(k) / static_cast<double>(n - 1));
    }
    out.push_back(hi);   // đúng bằng biên (lo + (hi-lo)*1 có thể lớn hơn hi 1 ulp -> vượt max_vel)
  }
  // Số mẫu lẻ chỉ có 0 khi cửa sổ đối xứng; luôn thêm 0 (đứng yên / đi thẳng) nếu nó đạt được.
  if (lo <= 0.0 && 0.0 <= hi) {
    out.push_back(0.0);
  }
  std::sort(out.begin(), out.end());
  out.erase(
    std::unique(
      out.begin(), out.end(), [](double a, double b) {
        return std::fabs(a - b) < kSampleEps;
      }),
    out.end());
  return out;
}

// =====================================================================================================
// KIỂM TRA VA CHẠM
// =====================================================================================================

CollisionModel::CollisionModel(
  const nav2_costmap_2d::Costmap2D * costmap, CollisionMode mode, std::vector<Pose2D> footprint,
  bool unknown_is_obstacle)
: costmap_(costmap), mode_(mode), unknown_is_obstacle_(unknown_is_obstacle)
{
  if (mode_ != CollisionMode::FOOTPRINT || footprint.size() < 3) {
    mode_ = CollisionMode::CENTER;
    return;
  }
  // Làm dày viền footprint: khoảng cách giữa hai điểm kiểm tra <= nửa ô.
  const double step = 0.5 * resolution();
  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const Pose2D & a = footprint[i];
    const Pose2D & b = footprint[(i + 1) % footprint.size()];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int k = 0; k < n; ++k) {
      const double u = static_cast<double>(k) / n;
      Pose2D q{a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), 0.0};
      sweep_radius_ = std::max(sweep_radius_, std::hypot(q.x, q.y));
      outline_.push_back(q);
    }
  }
}

double CollisionModel::resolution() const
{
  return costmap_->getResolution();
}

CellCheck CollisionModel::check(const Pose2D & s, unsigned char & cost) const
{
  unsigned int mx, my;
  if (!finite(s.x) || !finite(s.y) || !costmap_->worldToMap(s.x, s.y, mx, my)) {
    cost = NO_INFORMATION;
    return CellCheck::OFF_MAP;
  }
  cost = costmap_->getCost(mx, my);
  if (cost == NO_INFORMATION) {
    if (unknown_is_obstacle_) {
      return CellCheck::UNKNOWN;
    }
  } else if (cost >= INSCRIBED_INFLATED_OBSTACLE) {
    return CellCheck::COLLISION;
  }
  if (mode_ == CollisionMode::FOOTPRINT) {
    const double c = std::cos(s.theta), sn = std::sin(s.theta);
    for (const auto & q : outline_) {
      const double wx = s.x + c * q.x - sn * q.y;
      const double wy = s.y + sn * q.x + c * q.y;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return CellCheck::OFF_MAP;
      }
      const unsigned char cc = costmap_->getCost(mx, my);
      if (cc == NO_INFORMATION) {
        if (unknown_is_obstacle_) {
          return CellCheck::UNKNOWN;
        }
      } else if (cc >= LETHAL_OBSTACLE) {
        return CellCheck::COLLISION;
      }
    }
  }
  return CellCheck::FREE;
}

// =====================================================================================================
// MÔ PHỎNG
// =====================================================================================================

int advanceState(
  SimState & s, const Velocity2D & target, double duration, const DwbParams & p,
  double half_cell, double sweep_radius, std::vector<Pose2D> * out)
{
  if (!(duration > 0.0)) {
    return 0;
  }
  const double min_h = duration / kMaxStepsPerRollout;
  double t = 0.0;
  int steps = 0;
  while (t < duration - 1e-12 && steps < kMaxStepsPerRollout + 1) {
    double h = std::min(p.sim_granularity, duration - t);
    double v1 = 0.0, w1 = 0.0;
    for (int it = 0; it < 2; ++it) {
      v1 = stepToward(s.vel.v, target.v, p.acc_lim_x, p.decel_lim_x, h);
      w1 = stepToward(s.vel.w, target.w, p.acc_lim_theta, p.decel_lim_theta, h);
      // Vận tốc thay đổi đơn điệu trong bước -> lớn nhất ở một trong hai đầu.
      const double sweep = std::max(std::fabs(s.vel.v), std::fabs(v1)) +
        std::max(std::fabs(s.vel.w), std::fabs(w1)) * sweep_radius;
      if (half_cell > 0.0 && sweep * h > half_cell) {
        h = std::max(half_cell / sweep, min_h);
        h = std::min(h, duration - t);
      } else {
        break;
      }
    }
    v1 = stepToward(s.vel.v, target.v, p.acc_lim_x, p.decel_lim_x, h);
    w1 = stepToward(s.vel.w, target.w, p.acc_lim_theta, p.decel_lim_theta, h);
    integrateArc(s.pose, 0.5 * (s.vel.v + v1), 0.5 * (s.vel.w + w1), h);
    s.vel = Velocity2D{v1, w1};
    t += h;
    ++steps;
    if (out) {
      out->push_back(s.pose);
    }
  }
  return steps;
}

const char * rejectName(Reject r)
{
  switch (r) {
    case Reject::NONE: return "none";
    case Reject::COLLISION: return "collision";
    case Reject::UNKNOWN: return "unknown";
    case Reject::OFF_MAP: return "off_costmap";
    case Reject::BRAKING: return "braking";
  }
  return "?";
}

Trajectory rolloutCommand(
  const DwbParams & p, const Pose2D & start, const Velocity2D & current,
  const Velocity2D & prev_cmd,
  const Velocity2D & cmd, double dt, double horizon, double half_cell, double sweep_radius)
{
  Trajectory t;
  t.v = cmd.v;
  t.w = cmd.w;
  t.poses.reserve(static_cast<std::size_t>(horizon / p.sim_granularity) + 8);
  t.poses.push_back(start);   // pose ban đầu cũng phải hợp lệ
  SimState s{start, current};
  advanceState(s, prev_cmd, p.control_latency, p, half_cell, sweep_radius, &t.poses);
  advanceState(s, cmd, dt, p, half_cell, sweep_radius, &t.poses);
  const SimState commit = s;
  advanceState(s, cmd, horizon - p.control_latency - dt, p, half_cell, sweep_radius, &t.poses);

  // Đuôi phanh: lệnh kế tiếp (có thể là 0) chỉ tác động sau control_latency + dt. Từ đó v, ω giảm về 0 với
  // giới hạn giảm tốc. Quãng phanh thẳng ~ v²/(2·decel); đường cong được tích phân đầy đủ.
  SimState b = commit;
  t.braking.push_back(b.pose);
  const double t_stop =
    std::max(std::fabs(b.vel.v) / p.decel_lim_x, std::fabs(b.vel.w) / p.decel_lim_theta);
  advanceState(b, Velocity2D{0.0, 0.0}, t_stop, p, half_cell, sweep_radius, &t.braking);
  if (p.stopping_margin > 0.0 && commit.vel.v > 1e-6) {
    const int n =
      std::max(1, static_cast<int>(std::ceil(p.stopping_margin / std::max(half_cell, 1e-3))));
    for (int k = 1; k <= n; ++k) {
      const double d = p.stopping_margin * k / n;
      t.braking.push_back(
        Pose2D{b.pose.x + d * std::cos(b.pose.theta), b.pose.y + d * std::sin(b.pose.theta),
          b.pose.theta});
    }
  }
  return t;
}

Reject checkTrajectory(const Trajectory & t, const CollisionModel & cm, unsigned char & max_cost)
{
  max_cost = 0;
  unsigned char c = 0;
  for (const auto & s : t.poses) {
    switch (cm.check(s, c)) {
      case CellCheck::FREE: break;
      case CellCheck::COLLISION: return Reject::COLLISION;
      case CellCheck::UNKNOWN: return Reject::UNKNOWN;
      case CellCheck::OFF_MAP: return Reject::OFF_MAP;
    }
    // unknown được phép (unknown_is_obstacle:=false): coi như rất gần vật cản khi chấm điểm.
    max_cost = std::max(max_cost, c == NO_INFORMATION ? MAX_NON_OBSTACLE : c);
  }
  for (const auto & s : t.braking) {
    if (cm.check(s, c) != CellCheck::FREE) {
      return Reject::BRAKING;
    }
  }
  return Reject::NONE;
}

double obstacleProbe(
  const Trajectory & t, const CollisionModel & cm, double lookahead, double half_cell,
  unsigned char horizon_max_cost)
{
  // Chi phí vật cản (mềm) theo HÌNH DẠNG quỹ đạo, không theo tốc độ: nếu chỉ lấy max trên chân trời thời gian,
  // ứng viên chậm hơn "thấy" ít vùng inflation phía trước hơn -> luôn rẻ hơn -> robot giảm tốc dần về 0 ở mép
  // inflation dù lối đi hợp lệ (đã tái hiện: kịch bản obstacle_near_path). Vì vậy mọi ứng viên có v > 0 được nối
  // dài theo cùng độ cong ω/v tới cùng độ dài cung 'lookahead'. Phần nối dài chỉ để chấm điểm (không phải cam
  // kết): ô va chạm / unknown / ngoài costmap ở phần này = chi phí tối đa (252), KHÔNG loại quỹ đạo.
  unsigned char mc = horizon_max_cost;
  if (t.v <= 1e-3 || t.poses.empty()) {
    return static_cast<double>(mc) / static_cast<double>(MAX_NON_OBSTACLE);
  }
  double s = 0.0;
  for (std::size_t i = 1; i < t.poses.size(); ++i) {
    s += std::hypot(t.poses[i].x - t.poses[i - 1].x, t.poses[i].y - t.poses[i - 1].y);
  }
  const double kappa = t.w / t.v;
  const double step = std::max(half_cell, 1e-3);
  Pose2D q = t.poses.back();
  unsigned char c = 0;
  while (s < lookahead && mc < MAX_NON_OBSTACLE) {
    const double ds = std::min(step, lookahead - s);
    integrateArc(q, 1.0, kappa, ds);
    s += ds;
    if (cm.check(q, c) == CellCheck::FREE) {
      mc = std::max(mc, c == NO_INFORMATION ? MAX_NON_OBSTACLE : c);
    } else {
      mc = MAX_NON_OBSTACLE;
    }
  }
  return static_cast<double>(mc) / static_cast<double>(MAX_NON_OBSTACLE);
}

// =====================================================================================================
// BÁM ĐƯỜNG
// =====================================================================================================

void PathTracker::clear()
{
  pts_.clear();
  s_.clear();
  goal_ = Pose2D{};
  resetProgress();
}

void PathTracker::resetProgress()
{
  progress_s_ = 0.0;
  seg_ = 0;
}

std::string PathTracker::setPlan(const std::vector<Pose2D> & poses)
{
  clear();
  if (poses.empty()) {
    return "plan is empty";
  }
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (!finite(poses[i].x) || !finite(poses[i].y)) {
      return "plan pose " + std::to_string(i) + " has a non-finite position";
    }
  }
  if (!finite(poses.back().theta)) {
    return "plan goal orientation is invalid";
  }
  for (const auto & q : poses) {
    if (pts_.empty() || std::hypot(q.x - pts_.back().x, q.y - pts_.back().y) > 1e-6) {
      pts_.push_back(q);
    }
  }
  s_.assign(pts_.size(), 0.0);
  for (std::size_t i = 1; i < pts_.size(); ++i) {
    s_[i] = s_[i - 1] + std::hypot(pts_[i].x - pts_[i - 1].x, pts_[i].y - pts_[i - 1].y);
  }
  goal_ = Pose2D{pts_.back().x, pts_.back().y, normalizeAngle(poses.back().theta)};
  return "";
}

Pose2D PathTracker::pointAt(double s, std::size_t & seg) const
{
  if (pts_.size() == 1) {
    seg = 0;
    return Pose2D{pts_[0].x, pts_[0].y, goal_.theta};
  }
  s = std::clamp(s, 0.0, s_.back());
  seg = std::min(seg, pts_.size() - 2);
  while (seg + 2 < pts_.size() && s_[seg + 1] < s) {
    ++seg;
  }
  const Pose2D & a = pts_[seg];
  const Pose2D & b = pts_[seg + 1];
  const double len = s_[seg + 1] - s_[seg];
  const double u = len > 1e-12 ? std::clamp((s - s_[seg]) / len, 0.0, 1.0) : 0.0;
  return Pose2D{a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), std::atan2(b.y - a.y, b.x - a.x)};
}

PathTracker::Result PathTracker::update(
  const Pose2D & robot, double search_dist, double local_length, double branch_hysteresis)
{
  Result r;
  if (pts_.empty()) {
    return r;
  }
  if (pts_.size() == 1) {
    r.deviation = std::hypot(pts_[0].x - robot.x, pts_[0].y - robot.y);
    r.local.push_back(Pose2D{pts_[0].x, pts_[0].y, goal_.theta});
    return r;
  }

  // (a) Tiến trình: cực tiểu đầu tiên của khoảng cách dọc [progress, progress + search_dist].
  const double s_max = progress_s_ + search_dist;
  double best_d = std::numeric_limits<double>::infinity();
  double best_s = progress_s_;
  std::size_t best_seg = seg_;
  for (std::size_t k = seg_; k + 1 < pts_.size(); ++k) {
    if (s_[k] > s_max) {
      break;
    }
    const Pose2D & a = pts_[k];
    const Pose2D & b = pts_[k + 1];
    const double len = s_[k + 1] - s_[k];
    double u = 0.0;
    if (len > 1e-12) {
      u = ((robot.x - a.x) * (b.x - a.x) + (robot.y - a.y) * (b.y - a.y)) / (len * len);
    }
    // Không lùi tiến trình, không vượt cửa sổ tìm kiếm.
    const double u_lo = len > 1e-12 ? std::clamp((progress_s_ - s_[k]) / len, 0.0, 1.0) : 0.0;
    const double u_hi = len > 1e-12 ? std::clamp((s_max - s_[k]) / len, 0.0, 1.0) : 1.0;
    u = std::clamp(u, u_lo, std::max(u_lo, u_hi));
    const double qx = a.x + u * (b.x - a.x), qy = a.y + u * (b.y - a.y);
    const double d = std::hypot(qx - robot.x, qy - robot.y);
    if (d < best_d) {
      best_d = d;
      best_s = s_[k] + u * len;
      best_seg = k;
    } else if (d > best_d + branch_hysteresis) {
      break;   // đã rời "lòng chảo" gần nhất: không xét nhánh xa hơn dọc đường
    }
  }
  progress_s_ = std::max(progress_s_, best_s);
  seg_ = best_seg;
  r.progress = progress_s_;
  r.deviation = best_d;

  // (b) Đường cục bộ: [progress, progress + local_length].
  std::size_t hint = seg_;
  r.local.push_back(pointAt(progress_s_, hint));
  const double s_end = std::min(progress_s_ + local_length, s_.back());
  for (std::size_t k = seg_ + 1; k < pts_.size() && s_[k] < s_end; ++k) {
    if (s_[k] > progress_s_ + 1e-9) {
      Pose2D q = pts_[k];
      q.theta = std::atan2(pts_[k].y - pts_[k - 1].y, pts_[k].x - pts_[k - 1].x);
      r.local.push_back(q);
    }
  }
  if (s_end > progress_s_ + 1e-9) {
    r.local.push_back(pointAt(s_end, hint));
  }
  return r;
}

Pose2D pickCarrot(const std::vector<Pose2D> & local_path, double carrot_distance)
{
  double along = 0.0;
  for (std::size_t i = 1; i < local_path.size(); ++i) {
    const Pose2D & a = local_path[i - 1];
    const Pose2D & b = local_path[i];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (along + len >= carrot_distance && len > 1e-12) {
      const double u = (carrot_distance - along) / len;
      return Pose2D{a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), std::atan2(b.y - a.y, b.x - a.x)};
    }
    along += len;
  }
  return local_path.back();
}

double distanceToPolyline(double x, double y, const std::vector<Pose2D> & path)
{
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }
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

// =====================================================================================================
// CHỐNG DAO ĐỘNG
// =====================================================================================================

void OscillationGuard::reset()
{
  sign_ = 0;
  recently_flipped_ = false;
  ref_pose_ = Pose2D{};
  ref_time_ = 0.0;
  flips_ = 0;
}

void OscillationGuard::update(const Pose2D & robot, double w_cmd, double now_s, const DwbParams & p)
{
  if (recently_flipped_) {
    const double moved = std::hypot(robot.x - ref_pose_.x, robot.y - ref_pose_.y);
    const double turned = std::fabs(normalizeAngle(robot.theta - ref_pose_.theta));
    if (moved > p.oscillation_reset_dist || turned > p.oscillation_reset_angle ||
      now_s - ref_time_ > p.oscillation_reset_time || now_s < ref_time_)
    {
      recently_flipped_ = false;
    }
  }
  const int s = std::fabs(w_cmd) > kEps ? (w_cmd > 0.0 ? 1 : -1) : 0;
  if (s != 0 && sign_ != 0 && s != sign_) {
    ++flips_;
    recently_flipped_ = true;
    ref_pose_ = robot;
    ref_time_ = now_s;
  }
  if (s != 0) {
    sign_ = s;
  }
}

double OscillationGuard::penalty(double w) const
{
  if (!recently_flipped_ || sign_ == 0 || std::fabs(w) <= kEps) {
    return 0.0;
  }
  return ((w > 0.0 ? 1 : -1) != sign_) ? 1.0 : 0.0;
}

// =====================================================================================================
// MỘT CHU KỲ ĐIỀU KHIỂN
// =====================================================================================================

const char * phaseName(Phase ph)
{
  switch (ph) {
    case Phase::IDLE: return "IDLE";
    case Phase::FOLLOW_PATH: return "FOLLOW_PATH";
    case Phase::ROTATE_TO_PATH: return "ROTATE_TO_PATH";
    case Phase::ROTATE_TO_GOAL: return "ROTATE_TO_GOAL";
    case Phase::BLOCKED: return "BLOCKED";
  }
  return "?";
}

void DwaCore::reset()
{
  goal_latched_ = false;
  rotating_to_path_ = false;
  rotate_dir_ = 0;
  osc_.reset();
}

ControlOutput DwaCore::rotate(
  const DwbParams & p, const ControlInput & in, const CollisionModel & cm, double error,
  Phase phase, ControlOutput out)
{
  // Chiều xoay: theo dấu sai số, nhưng khi |sai số| gần π (hai chiều gần như bằng nhau) giữ chiều đã chọn
  // để nhiễu không làm robot lật trái/phải liên tục.
  int dir = error >= 0.0 ? 1 : -1;
  if (rotate_dir_ != 0 && std::fabs(error) > M_PI - kFlipDeadband) {
    dir = rotate_dir_;
  }
  rotate_dir_ = dir;
  out.heading_error = error;
  // Góc còn lại sau phần quay đã "trong đường ống" (lệnh trước còn hiệu lực thêm control_latency), rồi giới
  // hạn ω để phanh kịp với decel_lim_theta: ω <= sqrt(2·decel·còn lại). Không có giới hạn này robot thoát pha
  // xoay ở max_vel_theta và quay lố qua đích.
  const double ref_w = out.reference.w;
  const double remaining =
    std::fabs(error) - std::max(0.0, dir * ref_w) * (p.control_latency + 0.5 * in.dt);
  double mag = 0.0;
  if (remaining > 0.0) {
    mag = std::min(
      {p.rotate_gain * remaining, std::sqrt(2.0 * p.decel_lim_theta * remaining), p.max_vel_theta});
    mag = std::max(mag, p.min_in_place_vel_theta);
  }
  const double w = stepToward(ref_w, dir * mag, p.acc_lim_theta, p.decel_lim_theta, in.dt);
  return checkedCommand(p, in, cm, w, phase, std::move(out));
}

ControlOutput DwaCore::checkedCommand(
  const DwbParams & p, const ControlInput & in, const CollisionModel & cm, double w, Phase phase,
  ControlOutput out)
{
  // Tịnh tiến giảm dần về 0 theo decel_lim_x; không vượt max (speed limit) theo chính sách overspeed.
  const double v = std::clamp(
    stepToward(out.reference.v, 0.0, p.acc_lim_x, p.decel_lim_x, in.dt), 0.0, in.max_vel_x_eff);

  // Lệnh xoay / giữ ở đích vẫn có thể còn v > 0 (đang giảm tốc) -> kiểm tra như một chuyển động cong thật:
  // trễ + một chu kỳ + đuôi phanh. Chỉ kiểm tra ô tâm hiện tại là KHÔNG đủ.
  const double half_cell = 0.5 * cm.resolution();
  out.chosen = rolloutCommand(
    p, in.robot, in.velocity, in.prev_cmd, Velocity2D{v, w}, in.dt, p.control_latency + in.dt,
    half_cell, cm.sweepRadius());
  unsigned char max_cost = 0;
  const Reject rj = checkTrajectory(out.chosen, cm, max_cost);
  out.chosen.reject = rj;
  out.n_candidates = 1;
  if (rj != Reject::NONE) {
    ++out.n_reject[static_cast<int>(rj)];
    out.ok = false;
    out.phase = Phase::BLOCKED;
    std::ostringstream ss;
    ss << phaseName(phase) << " command (v=" << v << ", w=" << w << ") rejected: " <<
      rejectName(rj);
    out.error = ss.str();
    return out;
  }
  out.n_valid = 1;
  out.ok = true;
  out.phase = phase;
  out.cmd = Velocity2D{v, w};
  osc_.update(in.robot, w, in.now_s, p);
  return out;
}

ControlOutput DwaCore::compute(
  const DwbParams & p, const ControlInput & in, const CollisionModel & cm, bool keep_candidates)
{
  ControlOutput out;
  out.dist_to_goal = std::hypot(in.goal.x - in.robot.x, in.goal.y - in.robot.y);
  out.carrot = in.goal;
  out.measured = in.velocity;
  out.reference = referenceVelocity(p, in);

  // (0) Pose hiện tại phải hợp lệ trên costmap: nếu không, không có chuyển động nào được coi là an toàn.
  unsigned char c0 = 0;
  const CellCheck start = cm.check(in.robot, c0);
  if (start != CellCheck::FREE) {
    out.phase = Phase::BLOCKED;
    out.error = std::string("robot pose is ") +
      (start == CellCheck::COLLISION ? "in collision" :
      start == CellCheck::UNKNOWN ? "on an unknown cell" : "outside the local costmap") +
      " (cell cost " + std::to_string(static_cast<int>(c0)) + ")";
    return out;
  }
  if (in.local_path.empty()) {
    out.phase = Phase::BLOCKED;
    out.error = "local path is empty";
    return out;
  }

  // (1) ROTATE_TO_GOAL. Ngưỡng vào/ra lấy từ dung sai THỰC của GoalChecker (in.tol):
  //   vào khi d < xy*fraction (để GoalChecker chắc chắn cũng thấy đạt), ra khi d > xy + release_margin, hoặc khi
  //   đã đúng hướng mà vẫn ngoài ngưỡng vào (GoalChecker không stateful sẽ không bao giờ báo thành công ->
  //   phải tiếp cận lại thay vì đứng yên tới khi progress checker abort).
  const double xy_enter = in.tol.xy * p.goal_tolerance_fraction;
  const double xy_release = in.tol.xy + p.goal_latch_release_margin;
  const double yaw_target = in.tol.yaw * p.goal_tolerance_fraction;
  if (goal_latched_ && out.dist_to_goal > xy_release) {
    goal_latched_ = false;
  }
  if (!goal_latched_ && out.dist_to_goal < xy_enter) {
    goal_latched_ = true;
    rotating_to_path_ = false;
  }
  if (goal_latched_) {
    const double err = normalizeAngle(in.goal.theta - in.robot.theta);
    if (std::fabs(err) > yaw_target) {
      return rotate(p, in, cm, err, Phase::ROTATE_TO_GOAL, out);
    }
    if (out.dist_to_goal <= xy_enter) {
      // Đúng vị trí + hướng: giảm (v, ω) về 0 theo giới hạn (vẫn kiểm tra va chạm vì v có thể còn > 0);
      // GoalChecker của controller_server kết thúc action.
      const double w = stepToward(out.reference.w, 0.0, p.acc_lim_theta, p.decel_lim_theta, in.dt);
      return checkedCommand(p, in, cm, w, Phase::ROTATE_TO_GOAL, out);
    }
    goal_latched_ = false;   // đúng hướng nhưng trôi ra khỏi ngưỡng vào: tiếp cận lại
  }

  // (2) ROTATE_TO_PATH (có trễ: vào khi > ngưỡng, ra khi < ngưỡng/2).
  out.carrot = pickCarrot(in.local_path, p.carrot_distance);
  const double carrot_dist = std::hypot(out.carrot.x - in.robot.x, out.carrot.y - in.robot.y);
  if (carrot_dist > kMinCarrotDist) {
    const double herr =
      normalizeAngle(
      std::atan2(
        out.carrot.y - in.robot.y,
        out.carrot.x - in.robot.x) - in.robot.theta);
    if (std::fabs(herr) > p.rotate_to_heading_angle ||
      (rotating_to_path_ && std::fabs(herr) > 0.5 * p.rotate_to_heading_angle))
    {
      rotating_to_path_ = true;
      return rotate(p, in, cm, herr, Phase::ROTATE_TO_PATH, out);
    }
  }
  rotating_to_path_ = false;
  rotate_dir_ = 0;

  // (3) FOLLOW_PATH: dynamic window -> lấy mẫu -> mô phỏng -> loại cứng -> chấm điểm mềm.
  out.window = computeWindow(p, in.max_vel_x_eff, out.reference, in.dt);
  const std::vector<double> vs = sampleAxis(out.window.v.lo, out.window.v.hi, p.vx_samples);
  const std::vector<double> ws = sampleAxis(out.window.w.lo, out.window.w.hi, p.vtheta_samples);
  const double half_cell = 0.5 * cm.resolution();
  const double horizon = std::max(p.sim_time, p.control_latency + in.dt);
  const double vnorm = in.max_vel_x_eff > 1e-6 ? in.max_vel_x_eff : 1.0;
  // Độ dài cung chung cho chi phí vật cản: quãng đi được ở tốc độ tối đa trong sim_time.
  const double obstacle_lookahead = in.max_vel_x_eff * p.sim_time;

  if (keep_candidates) {
    out.candidates.reserve(vs.size() * ws.size());
  }
  Trajectory best;
  bool have_best = false;
  for (const double v : vs) {
    for (const double w : ws) {
      Trajectory t = rolloutCommand(
        p, in.robot, in.velocity, in.prev_cmd, Velocity2D{v, w}, in.dt, horizon, half_cell,
        cm.sweepRadius());
      unsigned char max_cost = 0;
      t.reject = checkTrajectory(t, cm, max_cost);
      ++out.n_candidates;
      if (t.valid()) {
        ++out.n_valid;
        if (v > 1e-3) {
          ++out.n_forward_valid;
        }
        const Pose2D & end = t.poses.back();
        t.obstacle_cost = obstacleProbe(t, cm, obstacle_lookahead, half_cell, max_cost);
        t.path_cost = distanceToPolyline(end.x, end.y, in.local_path);
        t.carrot_cost = std::hypot(out.carrot.x - end.x, out.carrot.y - end.y);
        t.heading_cost = t.carrot_cost > kMinCarrotDist ?
          std::fabs(
          normalizeAngle(
            std::atan2(
              out.carrot.y - end.y,
              out.carrot.x - end.x) - end.theta)) /
          M_PI : 0.0;
        t.speed_cost = std::clamp((in.max_vel_x_eff - v) / vnorm, 0.0, 1.0);
        t.oscillation_cost = osc_.penalty(w);
        t.total_cost = p.obstacle_scale * t.obstacle_cost + p.path_distance_scale * t.path_cost +
          p.carrot_distance_scale * t.carrot_cost + p.heading_scale * t.heading_cost +
          p.speed_scale * t.speed_cost + p.oscillation_scale * t.oscillation_cost;
        if (!have_best || t.total_cost < best.total_cost) {
          best = t;
          have_best = true;
          if (keep_candidates) {
            out.best = static_cast<int>(out.candidates.size());
          }
        }
      } else {
        ++out.n_reject[static_cast<int>(t.reject)];
      }
      if (keep_candidates) {
        out.candidates.push_back(std::move(t));
      }
    }
  }

  if (!have_best) {
    out.phase = Phase::BLOCKED;
    std::ostringstream ss;
    ss << "no admissible trajectory in the dynamic window (" << out.n_candidates << " candidates: "
       << out.n_reject[static_cast<int>(Reject::COLLISION)] << " collision, "
       << out.n_reject[static_cast<int>(Reject::UNKNOWN)] << " unknown, "
       << out.n_reject[static_cast<int>(Reject::OFF_MAP)] << " off-costmap, "
       << out.n_reject[static_cast<int>(Reject::BRAKING)] << " cannot brake in time)";
    out.error = ss.str();
    return out;
  }
  out.ok = true;
  out.phase = Phase::FOLLOW_PATH;
  out.cmd = Velocity2D{best.v, best.w};
  out.chosen = std::move(best);
  osc_.update(in.robot, out.cmd.w, in.now_s, p);
  return out;
}

}  // namespace stage3_navigation
