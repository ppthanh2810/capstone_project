// replan_supervisor: logic replanning event-triggered của goal_path_bridge, TÁCH KHỎI ROS (2026-11-26).
//
// Dùng chung bởi:
//   - goal_path_bridge (ROS node: TF, action ComputePathToPose/FollowPath, localization, log)
//   - stage5_benchmark Fast 2D simulator (không ROS) -> cùng quy tắc replan trong benchmark và robot thật.
// Nội dung chuyển nguyên từ goal_path_bridge.cpp (2026-11-25): Session, checkBlockage, requestObstacleReplan,
// retryWhileWaiting, onFollowAborted, onReplanFailed (phần trạng thái). Thời gian là giây (double) của một đồng hồ
// đơn điệu bất kỳ (bridge: now() = sim/ROS time; Fast 2D: thời gian mô phỏng).
//
// Quy tắc (xem nav2_params.yaml goal_path_bridge.replan.*):
//   bị chặn = ô costmap >= blocked_cost dọc đường trong lookahead phía trước, liên tục confirm_time;
//   thông lại = liên tục clear_time (hysteresis);
//   trigger "stall": lập lại chỉ khi bị chặn VÀ không tiến >= stall_progress trong stall_time;
//   cooldown min_interval, max_obstacle_replans -> FAILED; FollowPath abort -> lập lại (max_abort_replans);
//   không có đường -> chờ (robot dừng), thử lại mỗi min_interval tới wait_timeout -> FAILED.

#ifndef STAGE3_NAVIGATION__REPLAN_SUPERVISOR_HPP_
#define STAGE3_NAVIGATION__REPLAN_SUPERVISOR_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "stage3_navigation/path_blockage.hpp"

namespace stage3_navigation
{

struct ReplanParams
{
  bool enabled{true};
  std::string trigger{"stall"};      // stall | immediate
  double lookahead_distance{2.5};
  double start_offset{0.10};
  int blocked_cost{99};
  double confirm_time{0.5};
  double clear_time{1.0};
  double stall_time{4.0};
  double stall_progress{0.15};
  double min_interval{2.0};
  int max_obstacle_replans{10};
  int max_abort_replans{3};
  double wait_timeout{15.0};
};

struct ReplanDecision
{
  enum class Type {NONE, REPLAN, END};
  Type type{Type::NONE};
  std::string text;    // REPLAN: lý do (cho log / trạng thái "REPLANNING: ..."); END: trạng thái cuối
  static ReplanDecision none() {return {};}
  static ReplanDecision replan(std::string why) {return {Type::REPLAN, std::move(why)};}
  static ReplanDecision end(std::string state) {return {Type::END, std::move(state)};}
};

// Sự kiện để lớp vỏ ghi log (giữ nguyên thông điệp cũ của bridge).
enum class BlockageEvent {NONE, CONFIRMING, CLEARED};

class ReplanSupervisor
{
public:
  // Phiên mới (kích hoạt FollowPath).
  void begin(double now);
  // Đường mới được gửi tới FollowPath (đầu phiên hoặc sau replan): đặt lại tiến trình / chặn / chờ.
  void setPath(const std::vector<PathPoint> & path, double now);

  // Đang bám đường, costmap còn mới: kiểm tra chặn + quyết định lập lại.
  ReplanDecision checkBlockage(
    double now, double robot_x, double robot_y, const GridView & grid, const ReplanParams & p,
    BlockageEvent * event = nullptr, std::string * where = nullptr);
  // Costmap cũ: không kiểm tra, bỏ chuỗi "bị chặn" hiện tại.
  void costmapStale() {blocked_since_.reset();}

  // Không có FollowPath (đang chờ đường): thử lại / bỏ cuộc.
  ReplanDecision retryWhileWaiting(double now, const ReplanParams & p);
  // FollowPath bị abort trong phiên.
  ReplanDecision onFollowAborted(double now, const ReplanParams & p);

  // Lớp vỏ bắt đầu gửi ComputePathToPose (requestReplan).
  void markAttempt(double now) {last_attempt_ = now;}
  void setInFlight(bool v) {in_flight_ = v;}
  // Lập lại thất bại: bắt đầu (hoặc tiếp tục) chờ (lớp vỏ hủy FollowPath -> robot dừng).
  void onReplanFailed(double now);
  // true đúng một lần trong đợt chờ hiện tại (replan.clear_costmap_on_failure: xóa obstacle layer một lần).
  bool takeCostmapClear();

  bool inFlight() const {return in_flight_;}
  bool waiting() const {return waiting_since_.has_value();}
  int obstacleReplans() const {return obstacle_replans_;}
  int abortReplans() const {return abort_replans_;}
  int totalReplans() const {return obstacle_replans_ + abort_replans_;}
  bool blocked() const {return blocked_since_.has_value();}
  double progress() const {return last_progress_;}

private:
  ReplanDecision requestObstacleReplan(
    double now, const std::string & where,
    const ReplanParams & p);

  int obstacle_replans_{0};
  int abort_replans_{0};
  bool in_flight_{false};
  bool costmap_cleared_{false};
  std::optional<double> waiting_since_;
  std::optional<double> blocked_since_;
  std::optional<double> clear_since_;
  double ref_progress_{0.0};
  double ref_time_{0.0};
  double last_progress_{0.0};
  std::optional<double> last_attempt_;
  std::vector<PathPoint> path_;
  std::vector<double> path_s_;
  std::size_t progress_segment_{0};
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__REPLAN_SUPERVISOR_HPP_
