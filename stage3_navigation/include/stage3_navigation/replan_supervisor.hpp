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
//   Trigger sớm (2026-11-26, V4, tùy chọn; early_valid_ratio <= 0 = tắt = hành vi V3): đường bị chặn (đã xác nhận)
//   VÀ CustomDWB báo tỷ lệ quỹ đạo tiến hợp lệ n_forward_valid / n_candidates < early_valid_ratio liên tục
//   early_valid_time (pha BLOCKED = 0) -> lập lại ngay, trước khi robot bị ép vào ô INSCRIBED (Q6). Không lập lại
//   khi DWA còn nhiều quỹ đạo tiến hợp lệ (đang tự né được). early_distance > 0 (chỉ để so sánh): lập lại khi ô bị
//   chặn còn cách <= early_distance m dọc đường, bất kể DWA.
//   Watchdog tiến triển về đích (V4, tùy chọn; goal_progress_timeout <= 0 = tắt): quãng còn lại = độ dài đường hiện
//   tại − tiến trình dọc đường; nếu giá trị tốt nhất trong phiên không giảm thêm goal_progress_min m trong
//   goal_progress_timeout s (kể cả lúc chờ đường) -> FAILED (Safe Stop). Chặn vòng dò lối vô hạn (S1 House).

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
  double early_valid_ratio{0.0};     // <= 0: tắt trigger sớm theo DWA (V3)
  double early_valid_time{0.5};      // s liên tục dưới ngưỡng
  double early_distance{0.0};        // m, <= 0: tắt
  double early_max_deviation{0.0};   // m, > 0: trigger khoảng cách chỉ khi robot lệch đường <= giá trị này
                                     // (DWA chưa tự vòng); <= 0: không điều kiện
  double controller_timeout{1.0};    // s, báo cáo DWA cũ hơn -> coi như không có
  double goal_progress_timeout{0.0}; // s, <= 0: tắt
  double goal_progress_min{0.5};     // m
  // ---- R2 "Local Avoidance First" (trigger: conditional, 2026-11-27) ----
  // Bằng chứng LOCAL_BLOCKED (mỗi báo cáo CustomDWB): DWA lỗi / BLOCKED, hoặc FOLLOW_PATH không còn quỹ đạo TIẾN hợp
  // lệ (đứng yên / chỉ xoay không phải bằng chứng tiếp tục được), hoặc khoảng hở fs_clearance < local_clearance_min,
  // hoặc tỷ lệ cung tiến tự do < local_free_ratio_min. Pha xoay (ROTATE_TO_*) đủ khoảng hở = trung tính.
  double local_clearance_min{0.0};   // m, <= 0: không dùng
  double local_free_ratio_min{0.0};  // 0..1, <= 0: không dùng
  double local_hold_time{1.0};       // s bằng chứng liên tục (persistence)
  double useful_displacement{0.0};   // m, > 0: dịch chuyển khỏi điểm tham chiếu cũng tính là tiến triển (đi vòng)
  // false: KHÔNG lập lại do vật cản (vẫn tính trạng thái để chẩn đoán). Thí nghiệm R3 "chỉ DWA" (CHỈ mô phỏng):
  // obstacle_replan false + max_abort_replans 0 -> không replan nào sau đường ban đầu.
  bool obstacle_replan{true};
  // Tiếp cận chậm (R2): đường bị chặn (đã xác nhận) và ô bị chặn còn cách <= approach_distance m dọc đường ->
  // giới hạn tốc độ approach_speed m/s (nav2_msgs/SpeedLimit tuyệt đối -> controller_server -> CustomDWB
  // setSpeedLimit). Để quyết định LOCAL_BLOCKED muộn (khi bằng chứng rõ) vẫn dừng kịp trước ô INSCRIBED.
  // Gỡ khi đường thông lại (clear_time) hoặc có đường mới. approach_speed <= 0: tắt.
  double approach_speed{0.0};
  double approach_distance{1.0};
};

// Báo cáo một chu kỳ CustomDWB (/CustomDWB/state hoặc Fast 2D). Không phụ thuộc tham số DWA trừ n_forward_valid.
struct ControllerReport
{
  bool ok{false};             // DWA phát lệnh (pha != BLOCKED, không lỗi)
  bool follow{false};         // pha FOLLOW_PATH (lấy mẫu dynamic window)
  bool rotating{false};       // pha ROTATE_TO_PATH / ROTATE_TO_GOAL
  int n_candidates{0};
  int n_forward_valid{0};
  bool fs_valid{false};       // có chẩn đoán không gian trống
  double fs_clearance{0.0};   // m
  int fs_forward_free{0};
  int fs_arcs{0};
};

// Trạng thái quyết định (R2). NORMAL: đường thông. PATH_BLOCKED: đường bị chặn nhưng DWA còn tự né được.
// LOCAL_BLOCKED: DWA không tiến an toàn được (bằng chứng liên tục) -> GLOBAL_BLOCKED: lập lại A*.
// SAFE_STOP: hết lượt / không có đường (ReplanDecision END).
enum class ReplanState {NORMAL, PATH_BLOCKED, LOCAL_BLOCKED, GLOBAL_BLOCKED, SAFE_STOP};
const char * replanStateName(ReplanState s);

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
  // Báo cáo một chu kỳ CustomDWB (n_candidates = 0: không tính được / pha BLOCKED -> tỷ lệ 0).
  // Lớp vỏ không có CustomDWB (Nav2 DWB) thì không gọi -> trigger sớm không bao giờ bật.
  void reportController(double now, int n_forward_valid, int n_candidates, const ReplanParams & p);
  // Báo cáo đầy đủ (R2). Gọi bản này thì không cần gọi bản trên.
  void reportController(double now, const ControllerReport & r, const ReplanParams & p);
  // Costmap cũ: không kiểm tra, bỏ chuỗi "bị chặn" hiện tại.
  void costmapStale() {blocked_since_.reset();}

  // Watchdog tiến triển về đích, gọi định kỳ trong phiên (lớp vỏ, 5 Hz). END khi quá goal_progress_timeout.
  ReplanDecision checkGoalProgress(double now, const ReplanParams & p);
  // Thống kê: thời gian dài nhất (s) quãng còn lại tốt nhất không cải thiện >= goal_progress_min trong phiên.
  double maxGoalStagnation() const {return max_stagnation_;}
  double bestRemaining() const {return best_remaining_;}

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
  double validRatio() const {return last_valid_ratio_;}
  double deviation() const {return last_deviation_;}
  ReplanState state() const {return state_;}
  // Giới hạn tốc độ tiến (m/s) khuyến nghị cho controller; 0 = không giới hạn.
  double speedLimit(const ReplanParams & p) const;
  double blockDistance() const {return last_block_distance_;}   // m dọc đường tới ô bị chặn (-1 = thông)
  bool localBad() const {return local_bad_since_.has_value();}
  // Lý do bằng chứng LOCAL_BLOCKED gần nhất ("dwa_blocked" | "no_forward" | "clearance" | "free_space" | "")
  const std::string & localReason() const {return local_reason_;}

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
  std::optional<double> low_valid_since_;
  std::optional<double> controller_t_;
  double last_valid_ratio_{1.0};
  double path_length_{0.0};
  double last_deviation_{0.0};
  double best_remaining_{1e18};
  double best_remaining_t_{0.0};
  double max_stagnation_{0.0};
  ReplanState state_{ReplanState::NORMAL};
  std::optional<double> local_bad_since_;
  std::string local_reason_;
  double last_block_distance_{-1.0};
  bool approach_latched_{false};
  double ref_x_{0.0}, ref_y_{0.0};
  bool ref_xy_set_{false};
  std::vector<PathPoint> path_;
  std::vector<double> path_s_;
  std::size_t progress_segment_{0};
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__REPLAN_SUPERVISOR_HPP_
