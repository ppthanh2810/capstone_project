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

void ReplanSupervisor::begin(double now)
{
  *this = ReplanSupervisor{};
  ref_time_ = now;
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
  clear_since_.reset();
  blocked_since_.reset();
  waiting_since_.reset();
  costmap_cleared_ = false;
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
  last_progress_ = progress;
  if (progress >= ref_progress_ + p.stall_progress) {
    ref_progress_ = progress;
    ref_time_ = now;
  }

  const auto b = findBlockage(
    path_, grid, progress_segment_, along, p.start_offset, p.lookahead_distance, p.blocked_cost);
  if (!b.blocked) {
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
  if (last_attempt_ && now - *last_attempt_ < p.min_interval) {
    return ReplanDecision::none();
  }
  if (p.max_obstacle_replans > 0 && obstacle_replans_ >= p.max_obstacle_replans) {
    return ReplanDecision::end("FAILED: path blocked, replan.max_obstacle_replans reached");
  }
  ++obstacle_replans_;
  return ReplanDecision::replan(where);
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
