// replan_supervisor.cpp — xem replan_supervisor.hpp. Chuyển nguyên logic từ goal_path_bridge.cpp (2026-11-25).

#include "stage3_navigation/replan_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace stage3_navigation
{

namespace
{
std::string fmt1(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", v);
  return buf;
}
std::string fmt2(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", v);
  return buf;
}
std::string fmtxy(double x, double y)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.2f, %.2f", x, y);
  return buf;
}
}  // namespace

const char * replanStateName(ReplanState s)
{
  switch (s) {
    case ReplanState::NORMAL: return "NORMAL";
    case ReplanState::PATH_BLOCKED: return "PATH_BLOCKED";
    case ReplanState::LOCAL_BLOCKED: return "LOCAL_BLOCKED";
    case ReplanState::GLOBAL_BLOCKED: return "GLOBAL_BLOCKED";
    case ReplanState::SAFE_STOP: return "SAFE_STOP";
  }
  return "?";
}

void ReplanSupervisor::begin(double now)
{
  *this = ReplanSupervisor{};
  ref_time_ = now;
  best_remaining_t_ = now;
}

void ReplanSupervisor::setPath(const std::vector<PathPoint> & path, double now)
{
  path_ = path;
  progress_segment_ = 0;
  path_s_.assign(path_.size(), 0.0);
  for (std::size_t i = 1; i < path_.size(); ++i) {
    path_s_[i] = path_s_[i - 1] + std::hypot(
      path_[i].x - path_[i - 1].x,
      path_[i].y - path_[i - 1].y);
  }
  ref_progress_ = 0.0;
  ref_time_ = now;
  last_progress_ = 0.0;
  path_length_ = path_s_.empty() ? 0.0 : path_s_.back();
  clear_since_.reset();
  blocked_since_.reset();
  waiting_since_.reset();
  costmap_cleared_ = false;
  low_valid_since_.reset();
  controller_t_.reset();
  last_valid_ratio_ = 1.0;
  local_bad_since_.reset();
  local_reason_.clear();
  ref_xy_set_ = false;
  last_block_distance_ = -1.0;
  state_ = ReplanState::NORMAL;
  approach_latched_ = false;
}

void ReplanSupervisor::reportController(double now, const ControllerReport & r, const ReplanParams & p)
{
  // Giữ đường V4 (tỷ lệ quỹ đạo tiến hợp lệ) để so sánh.
  reportController(now, r.ok ? r.n_forward_valid : 0, r.ok ? r.n_candidates : 0, p);
  // R2: phân loại chu kỳ thành XẤU (DWA không tiến an toàn được) / TỐT / TRUNG TÍNH (đang xoay, đủ khoảng hở).
  std::string bad;
  if (!r.ok) {
    bad = "dwa_blocked";
  } else if (r.follow && r.n_forward_valid == 0) {
    bad = "no_forward";      // chỉ còn đứng yên / xoay tại chỗ: không phải bằng chứng tiếp tục được
  } else if (r.fs_valid && p.local_clearance_min > 0.0 && r.fs_clearance < p.local_clearance_min) {
    bad = "clearance";
  } else if (r.fs_valid && p.local_free_ratio_min > 0.0 && r.fs_arcs > 0 &&
    static_cast<double>(r.fs_forward_free) / r.fs_arcs < p.local_free_ratio_min)
  {
    bad = "free_space";
  }
  if (!bad.empty()) {
    if (!local_bad_since_) {
      local_bad_since_ = now;
    }
    local_reason_ = bad;
  } else if (r.follow) {
    local_bad_since_.reset();   // FOLLOW_PATH có quỹ đạo tiến + đủ không gian: DWA đang tự xử lý
    local_reason_.clear();
  }
  // r.rotating và không xấu: trung tính (không bắt đầu, không xóa chuỗi)
}

void ReplanSupervisor::reportController(
  double now, int n_forward_valid, int n_candidates, const ReplanParams & p)
{
  const double ratio = n_candidates > 0 ?
    static_cast<double>(n_forward_valid) / static_cast<double>(n_candidates) : 0.0;
  // Khoảng trống giữa hai báo cáo quá dài -> chuỗi "thấp" cũ không còn liên tục.
  if (controller_t_ && now - *controller_t_ > p.controller_timeout) {
    low_valid_since_.reset();
  }
  controller_t_ = now;
  last_valid_ratio_ = ratio;
  if (ratio < p.early_valid_ratio) {
    if (!low_valid_since_) {
      low_valid_since_ = now;
    }
  } else {
    low_valid_since_.reset();
  }
}

ReplanDecision ReplanSupervisor::checkBlockage(
  double now, double robot_x, double robot_y, const GridView & grid, const ReplanParams & p,
  BlockageEvent * event, std::string * where_out)
{
  if (event) {
    *event = BlockageEvent::NONE;
  }
  if (path_.empty()) {
    return ReplanDecision::none();
  }
  double along = 0.0;
  progress_segment_ = projectOnPath(path_, robot_x, robot_y, progress_segment_, 2.0, along);
  // Tiến trình dọc đường (chỉ tăng khi robot thật sự tiến theo đường, kể cả khi đang vòng né sát đường).
  const double progress =
    (progress_segment_ < path_s_.size() ? path_s_[progress_segment_] : 0.0) + along;
  {
    // độ lệch ngang robot <-> điểm chiếu trên đường (DWA đang vòng né -> lệch lớn)
    const std::size_t i = std::min(progress_segment_, path_.size() - 1);
    const std::size_t j = std::min(i + 1, path_.size() - 1);
    const double L = std::hypot(path_[j].x - path_[i].x, path_[j].y - path_[i].y);
    const double u = L > 1e-9 ? along / L : 0.0;
    const double px = path_[i].x + u * (path_[j].x - path_[i].x);
    const double py = path_[i].y + u * (path_[j].y - path_[i].y);
    last_deviation_ = std::hypot(robot_x - px, robot_y - py);
  }
  last_progress_ = progress;
  if (!ref_xy_set_) {
    ref_x_ = robot_x;
    ref_y_ = robot_y;
    ref_xy_set_ = true;
  }
  const bool moved = p.useful_displacement > 0.0 &&
    std::hypot(robot_x - ref_x_, robot_y - ref_y_) >= p.useful_displacement;
  if (progress >= ref_progress_ + p.stall_progress || moved) {
    ref_progress_ = std::max(ref_progress_, progress);
    ref_time_ = now;
    ref_x_ = robot_x;
    ref_y_ = robot_y;
  }

  const auto b = findBlockage(
    path_, grid, progress_segment_, along, p.start_offset, p.lookahead_distance, p.blocked_cost);
  last_block_distance_ = b.blocked ? b.distance_ahead : -1.0;
  if (!b.blocked) {
    if (!blocked_since_) {
      state_ = ReplanState::NORMAL;
    }
    if (blocked_since_) {
      // Hysteresis: một vài chu kỳ "thông" (vật cản ở mép tầm LiDAR, nhiễu) không xóa trạng thái chặn.
      if (!clear_since_) {
        clear_since_ = now;
      }
      if (now - *clear_since_ >= p.clear_time) {
        if (event) {
          *event = BlockageEvent::CLEARED;
        }
        blocked_since_.reset();
        clear_since_.reset();
        state_ = ReplanState::NORMAL;
        approach_latched_ = false;
      }
    }
    return ReplanDecision::none();
  }
  clear_since_.reset();
  const std::string where = "obstacle on the path " + fmt2(b.distance_ahead) + " m ahead at (" +
    fmtxy(b.x, b.y) + "), cost " + std::to_string(b.cost);
  if (where_out) {
    *where_out = where;
  }
  if (!blocked_since_) {
    blocked_since_ = now;
    if (event) {
      *event = BlockageEvent::CONFIRMING;
    }
  }
  if (now - *blocked_since_ < p.confirm_time) {
    return ReplanDecision::none();
  }
  state_ = ReplanState::PATH_BLOCKED;
  if (p.approach_speed > 0.0 && b.distance_ahead <= p.approach_distance) {
    approach_latched_ = true;
  }
  // ---- R2 Local Avoidance First: đường bị chặn KHÔNG đủ để lập lại ----
  if (p.trigger == "conditional") {
    const bool fresh = controller_t_ && now - *controller_t_ <= p.controller_timeout;
    // (a) LOCAL_BLOCKED theo DWA: bằng chứng xấu liên tục local_hold_time kể từ khi đường bị chặn
    if (fresh && local_bad_since_ &&
      now - std::max(*local_bad_since_, *blocked_since_) >= p.local_hold_time)
    {
      state_ = ReplanState::LOCAL_BLOCKED;
      auto d = requestObstacleReplan(
        now, where + ", LOCAL_BLOCKED (" + local_reason_ + ") for " +
        fmt1(now - std::max(*local_bad_since_, *blocked_since_)) + " s", p);
      if (d.type == ReplanDecision::Type::REPLAN) {state_ = ReplanState::GLOBAL_BLOCKED;}
      if (d.type == ReplanDecision::Type::END) {state_ = ReplanState::SAFE_STOP;}
      return d;
    }
    // (b) LOCAL_BLOCKED theo tiến triển hữu ích (kể cả đi vòng): không tiến dọc đường và không dịch chuyển
    const double since = std::max(ref_time_, *blocked_since_);
    if (now - since >= p.stall_time) {
      state_ = ReplanState::LOCAL_BLOCKED;
      auto d = requestObstacleReplan(
        now, where + ", LOCAL_BLOCKED (no useful progress) for " + fmt1(now - since) + " s", p);
      if (d.type == ReplanDecision::Type::REPLAN) {state_ = ReplanState::GLOBAL_BLOCKED;}
      if (d.type == ReplanDecision::Type::END) {state_ = ReplanState::SAFE_STOP;}
      return d;
    }
    return ReplanDecision::none();
  }
  // Trigger sớm V4: đường bị chặn + DWA gần hết lựa chọn tiến (robot đang bị ép sát vật cản).
  if (p.early_valid_ratio > 0.0 && low_valid_since_ && controller_t_ &&
    now - *controller_t_ <= p.controller_timeout &&
    now - std::max(*low_valid_since_, *blocked_since_) >= p.early_valid_time)
  {
    return requestObstacleReplan(
      now, where + ", forward-valid trajectories " + fmt2(last_valid_ratio_) + " < " +
      fmt2(p.early_valid_ratio) + " for " + fmt1(now - *low_valid_since_) + " s", p);
  }
  if (p.early_distance > 0.0 && b.distance_ahead <= p.early_distance &&
    (p.early_max_deviation <= 0.0 || last_deviation_ <= p.early_max_deviation))
  {
    return requestObstacleReplan(
      now, where + ", within early_distance " + fmt2(p.early_distance) + " m (deviation " +
      fmt2(last_deviation_) + " m)", p);
  }
  if (p.trigger != "immediate") {
    // Event-triggered: CustomDWB đang tự vòng qua (tiến trình dọc đường còn tăng) -> không lập lại.
    const double since = std::max(ref_time_, *blocked_since_);
    const double stalled = now - since;
    if (stalled < p.stall_time) {
      return ReplanDecision::none();
    }
    return requestObstacleReplan(now, where + ", no progress for " + fmt1(stalled) + " s", p);
  }
  return requestObstacleReplan(now, where, p);
}

// Lập lại do vật cản: cooldown (min_interval) + giới hạn số lần (max_obstacle_replans) -> FAILED (Safe Stop).
ReplanDecision ReplanSupervisor::requestObstacleReplan(
  double now, const std::string & where, const ReplanParams & p)
{
  if (!p.obstacle_replan) {
    return ReplanDecision::none();     // R3 (chỉ DWA, mô phỏng): chỉ ghi trạng thái, không lập lại
  }
  if (last_attempt_ && now - *last_attempt_ < p.min_interval) {
    return ReplanDecision::none();
  }
  if (p.max_obstacle_replans > 0 && obstacle_replans_ >= p.max_obstacle_replans) {
    state_ = ReplanState::SAFE_STOP;
    return ReplanDecision::end("FAILED: path blocked, replan.max_obstacle_replans reached");
  }
  ++obstacle_replans_;
  state_ = ReplanState::GLOBAL_BLOCKED;
  return ReplanDecision::replan(where);
}

double ReplanSupervisor::speedLimit(const ReplanParams & p) const
{
  if (p.approach_speed <= 0.0 || !blocked_since_ || state_ == ReplanState::NORMAL) {
    return 0.0;
  }
  // đã từng tới gần trong chuỗi chặn này -> giữ tới khi thông (tránh tăng tốc lại khi đang vòng sát vật cản)
  if (approach_latched_ || (last_block_distance_ >= 0.0 && last_block_distance_ <= p.approach_distance)) {
    return p.approach_speed;
  }
  return 0.0;
}

ReplanDecision ReplanSupervisor::checkGoalProgress(double now, const ReplanParams & p)
{
  // Đang bám đường: quãng còn lại theo đường hiện tại; đang chờ (không đường): giữ giá trị cũ, đồng hồ vẫn chạy.
  if (!path_.empty() && !waiting_since_) {
    const double remaining = std::max(0.0, path_length_ - last_progress_);
    if (remaining <= best_remaining_ - p.goal_progress_min) {
      best_remaining_ = remaining;
      best_remaining_t_ = now;
    }
  }
  const double stagnant = now - best_remaining_t_;
  max_stagnation_ = std::max(max_stagnation_, stagnant);
  if (p.goal_progress_timeout > 0.0 && stagnant > p.goal_progress_timeout) {
    return ReplanDecision::end(
      "FAILED: no progress toward the goal for " + fmt1(stagnant) + " s (best remaining " +
      fmt2(best_remaining_) + " m, replan.goal_progress_timeout)");
  }
  return ReplanDecision::none();
}

ReplanDecision ReplanSupervisor::retryWhileWaiting(double now, const ReplanParams & p)
{
  if (!waiting_since_) {
    return ReplanDecision::none();
  }
  const double waited = now - *waiting_since_;
  if (waited > p.wait_timeout) {
    return ReplanDecision::end("FAILED: no free path to the goal for " + fmt1(waited) + " s");
  }
  if (last_attempt_ && now - *last_attempt_ < p.min_interval) {
    return ReplanDecision::none();
  }
  return ReplanDecision::replan("retry, waiting " + fmt1(waited) + " s");
}

ReplanDecision ReplanSupervisor::onFollowAborted(double now, const ReplanParams & p)
{
  if (!p.enabled) {
    return ReplanDecision::end("ABORTED");
  }
  if (abort_replans_ >= p.max_abort_replans) {
    return ReplanDecision::end(
      "ABORTED: FollowPath aborted " + std::to_string(abort_replans_ + 1) +
      " times (replan.max_abort_replans " + std::to_string(p.max_abort_replans) + ")");
  }
  ++abort_replans_;
  if (!waiting_since_) {
    waiting_since_ = now;
  }
  if (in_flight_) {
    return ReplanDecision::none();
  }
  return ReplanDecision::replan(
    "FollowPath aborted (" + std::to_string(abort_replans_) + "/" +
    std::to_string(p.max_abort_replans) + ")");
}

void ReplanSupervisor::onReplanFailed(double now)
{
  if (!waiting_since_) {
    waiting_since_ = now;
  }
}

bool ReplanSupervisor::takeCostmapClear()
{
  if (costmap_cleared_) {
    return false;
  }
  costmap_cleared_ = true;
  return true;
}

}  // namespace stage3_navigation
