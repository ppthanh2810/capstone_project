// =====================================================================================================
// dwa_core.hpp — Lõi thuật toán Dynamic Window của CustomDWB, TÁCH KHỎI ROS node
// =====================================================================================================
//
// File này chứa toàn bộ phần toán của bộ điều khiển (dynamic window, mô phỏng quỹ đạo, kiểm tra va chạm +
// khả năng phanh, bám đường theo tiến trình, chống dao động, chọn pha). Nó chỉ phụ thuộc
// nav2_costmap_2d::Costmap2D (một lớp lưới thuần C++, không cần node) để:
//   - unit test (test/test_dwa_core.cpp) và mô phỏng kín vòng (test/dwa_closed_loop_sim.cpp) chạy KHÔNG cần
//     ROS graph;
//   - plugin (dwb_controller.cpp) chỉ còn là lớp vỏ: TF, odometry watchdog, tham số, debug.
//
// Quy ước: SI (m, rad, s); robot vi sai: chỉ (v, ω), vy = 0; v >= min_vel_x >= 0 (KHÔNG lùi).
// Nguồn tham khảo đã dùng: xem stage3_navigation/doc/sources.md.
// =====================================================================================================

#ifndef STAGE3_NAVIGATION__DWA_CORE_HPP_
#define STAGE3_NAVIGATION__DWA_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace stage3_navigation
{

// Một pose 2D (x, y, θ).
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// Vận tốc của robot vi sai.
struct Velocity2D
{
  double v{0.0};   // m/s
  double w{0.0};   // rad/s
};

// Phép biến đổi cứng 2D (frame A -> frame B): p_B = R(yaw) p_A + (x, y).
struct Transform2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  Pose2D apply(const Pose2D & p) const;
  Transform2D inverse() const;
};

// Giới hạn tải tính toán (bảo vệ chu kỳ 10 Hz khỏi cấu hình quá lớn). Xem validateParams().
constexpr int kMaxSamplesPerAxis = 101;
constexpr std::size_t kMaxRolloutPosesPerCycle = 100000;   // ước lượng vx*vθ*(sim_time/granularity)
constexpr int kMaxStepsPerRollout = 2000;                   // chặn cứng số bước một lần mô phỏng

// Toàn bộ tham số (đọc từ YAML dưới CustomDWB.*, đổi được bằng ros2 param set; xem nav2_params.yaml).
struct DwbParams
{
  // ---- Giới hạn động học (giá trị khởi đầu, NOT_VERIFIED trên robot) ----
  double max_vel_x{0.15};               // m/s
  double min_vel_x{0.0};                // m/s, 0 = KHÔNG lùi (phía sau không có LiDAR); < 0 bị từ chối
  double max_vel_theta{0.5};            // rad/s
  double min_in_place_vel_theta{0.1};   // rad/s, |ω| tối thiểu khi xoay tại chỗ (thắng ma sát tĩnh)
  double acc_lim_x{0.5};                // m/s², tăng |v|
  double decel_lim_x{0.5};              // m/s², giảm |v| (giá trị dương)
  double acc_lim_theta{1.0};            // rad/s², tăng |ω|
  double decel_lim_theta{1.0};          // rad/s², giảm |ω| (giá trị dương)

  // ---- Lấy mẫu + mô phỏng ----
  int vx_samples{20};
  int vtheta_samples{21};
  double sim_time{1.7};                 // s, chân trời chấm điểm (tính từ "bây giờ", gồm cả trễ)
  double sim_granularity{0.05};         // s, bước tích phân tối đa (còn bị giới hạn theo độ phân giải ô)

  // ---- Mô hình an toàn (Fox et al. 1997: vận tốc chấp nhận được = phanh kịp trước vật cản) ----
  double control_latency{0.1};          // s, trễ lệnh -> bánh (SYNTHETIC, chưa đo trên robot)
  double stopping_margin{0.05};         // m, khoảng trống thêm phía trước sau khi phanh dừng hẳn

  // ---- Vận tốc gốc của dynamic window (xem referenceVelocity) ----
  // "command": tính từ lệnh trước, kẹp trong ± max_feedback_gap_* quanh vận tốc đo (mặc định).
  // "measured": tính từ vận tốc đo (hành vi cũ; với phản hồi trễ τ, gia tốc thực chỉ còn ~acc·dt/τ -> xoay lố).
  std::string velocity_reference{"command"};
  double max_feedback_gap_x{0.15};      // m/s
  double max_feedback_gap_theta{0.5};   // rad/s

  // ---- Đường cục bộ + cà rốt ----
  double local_plan_length{1.5};        // m
  double carrot_distance{0.6};          // m
  double closest_search_distance{1.0};  // m, cửa sổ tìm tiến trình phía trước (theo độ dài cung)
  double max_path_deviation{0.8};       // m, robot lệch đường A* hơn mức này -> dừng (PlannerException)

  // ---- Các pha xoay tại chỗ ----
  double rotate_to_heading_angle{0.8};  // rad: vào ROTATE_TO_PATH khi lệch > ngưỡng, ra khi < ngưỡng/2
  double rotate_gain{1.5};              // ω = gain * sai số góc (rồi giới hạn)

  // ---- Đích (dự phòng khi GoalChecker không cung cấp dung sai) ----
  double xy_goal_tolerance{0.15};       // m
  double yaw_goal_tolerance{0.25};      // rad
  double goal_tolerance_fraction{0.9};  // dùng tol*fraction bên trong để chắc chắn GoalChecker cũng đạt
  double goal_latch_release_margin{0.05};   // m, trôi ra ngoài xy_tol + margin -> bỏ ROTATE_TO_GOAL

  // ---- Trọng số hàm chi phí mềm (ràng buộc cứng KHÔNG nằm ở đây) ----
  double path_distance_scale{4.0};      // x khoảng cách cuối quỹ đạo -> đường cục bộ (m)
  double carrot_distance_scale{2.0};    // x khoảng cách cuối quỹ đạo -> cà rốt (m)
  double heading_scale{1.0};            // x lệch hướng cuối quỹ đạo so với hướng tới cà rốt / π (0..1)
  double obstacle_scale{0.5};           // x chi phí costmap lớn nhất dọc cung cố định max_vel_x*sim_time / 252 (0..1)
  double speed_scale{0.3};              // x (max_vel_x - v) / max_vel_x (0..1)
  double oscillation_scale{1.0};        // phạt ω đổi chiều ngay sau một lần đổi chiều (0 hoặc 1)

  // ---- Chống dao động (đặt lại khi đi được / xoay được / hết thời gian) ----
  double oscillation_reset_dist{0.05};  // m
  double oscillation_reset_angle{0.2};  // rad
  double oscillation_reset_time{2.0};   // s (> 0: không bao giờ khóa chiều xoay vô hạn)

  // ---- Dữ liệu / TF ----
  bool unknown_is_obstacle{true};       // ô NO_INFORMATION = vật cản (chỉ có tác dụng khi costmap theo dõi unknown)
  double transform_tolerance{0.3};      // s, thời gian CHỜ TF (không phải tuổi)
  double max_tf_age{1.0};               // s, tuổi tối đa của TF plan_frame -> costmap_frame (map -> odom)
  double max_pose_age{0.5};             // s, tuổi tối đa của pose robot (TF odom -> base_footprint)
  double odom_timeout{0.5};             // s, không nhận odometry quá lâu -> dừng
  std::string collision_check{"auto"};  // auto | center | footprint

  // ---- Hiển thị / chẩn đoán ----
  bool publish_trajectories{true};
  int trajectory_marker_stride{4};
  double debug_publish_rate{5.0};       // Hz, 0 = tắt marker/state (local_plan vẫn publish)
};

// "" = hợp lệ; ngược lại là lý do. Kiểm tra hữu hạn, miền, quan hệ chéo và tải tính toán.
std::string validateParams(const DwbParams & p);
// Ước lượng số pose mô phỏng mỗi chu kỳ (để chặn cấu hình quá tải).
std::size_t estimateRolloutPoses(const DwbParams & p);

double normalizeAngle(double a);

// =====================================================================================================
// Dynamic window
// =====================================================================================================
struct Range
{
  double lo{0.0};
  double hi{0.0};
};

// Tập giá trị đạt được sau thời gian dt từ x0, với gia tốc tăng |x| <= acc và giảm |x| <= decel
// (xử lý đúng trường hợp đi qua 0: giảm về 0 bằng decel rồi tăng theo chiều ngược bằng acc).
Range reachableRange(double x0, double acc, double decel, double dt);
// Một bước tiến từ cur về target trong thời gian h với cùng quy tắc acc/decel.
double stepToward(double cur, double target, double acc, double decel, double h);

struct DynamicWindow
{
  Range v;
  Range w;
  // true: tập đạt được KHÔNG giao giới hạn tuyệt đối (vd. v0 = 0.5 > max 0.15 sau khi PS4 lái nhanh).
  // Chính sách: lệnh = biên giới hạn gần nhất (hard cap luôn đúng cho LỆNH); mô phỏng vẫn dùng chuyển động
  // thật (giảm tốc theo decel_lim) nên KHÔNG giả định lệnh đạt được trong một chu kỳ.
  bool v_overspeed{false};
  bool w_overspeed{false};
};
DynamicWindow computeWindow(
  const DwbParams & p, double max_vel_x_eff, const Velocity2D & current, double dt);
// n giá trị chia đều [lo, hi] (gồm hai đầu), cộng thêm 0 nếu 0 nằm trong khoảng; sắp xếp + khử trùng.
std::vector<double> sampleAxis(double lo, double hi, int n);

// =====================================================================================================
// Kiểm tra va chạm trên costmap
// =====================================================================================================
enum class CollisionMode { CENTER, FOOTPRINT };
enum class CellCheck : std::uint8_t {FREE, COLLISION, UNKNOWN, OFF_MAP};

class CollisionModel
{
public:
  // CENTER: chỉ ô dưới tâm (đúng khi footprint tròn và local costmap đã bơm bán kính nội tiếp).
  // FOOTPRINT: ô tâm >= INSCRIBED, hoặc một điểm trên viền footprint >= LETHAL / unknown / ngoài map.
  CollisionModel(
    const nav2_costmap_2d::Costmap2D * costmap, CollisionMode mode,
    std::vector<Pose2D> footprint, bool unknown_is_obstacle);

  // Kiểm tra một pose; cost = chi phí ô tâm (để chấm điểm). Không phải FREE = không được đi.
  CellCheck check(const Pose2D & s, unsigned char & cost) const;
  // Khoảng cách xa nhất từ tâm tới một điểm footprint (0 với CENTER): giới hạn bước theo ω.
  double sweepRadius() const {return sweep_radius_;}
  double resolution() const;
  CollisionMode mode() const {return mode_;}

private:
  const nav2_costmap_2d::Costmap2D * costmap_;
  CollisionMode mode_;
  std::vector<Pose2D> outline_;   // điểm dày đặc dọc viền footprint (frame robot)
  bool unknown_is_obstacle_;
  double sweep_radius_{0.0};
};

// =====================================================================================================
// Mô phỏng quỹ đạo
// =====================================================================================================
struct SimState
{
  Pose2D pose;
  Velocity2D vel;
};

// Tiến trạng thái thêm duration giây, vận tốc tiến về target theo giới hạn acc/decel; mỗi bước <= max_step_time
// và quãng quét <= half_cell (= 0.5 * độ phân giải costmap) để không bỏ lọt ô giữa hai pose. Tích phân cung tròn
// chính xác với vận tốc trung điểm của bước; bước cuối dừng ĐÚNG tại duration (không vượt do làm tròn).
// Ghi các pose (không gồm pose đầu) vào out nếu out != nullptr. Trả về số bước.
int advanceState(
  SimState & s, const Velocity2D & target, double duration, const DwbParams & p,
  double half_cell, double sweep_radius, std::vector<Pose2D> * out);

// Lý do loại một quỹ đạo (ràng buộc CỨNG).
enum class Reject : std::uint8_t {NONE, COLLISION, UNKNOWN, OFF_MAP, BRAKING};
const char * rejectName(Reject r);

// Một ứng viên lệnh (v, ω) và kết quả đánh giá.
struct Trajectory
{
  double v{0.0};
  double w{0.0};
  std::vector<Pose2D> poses;     // [pose hiện tại, ..., pose tại sim_time]: trễ -> chuyển tiếp -> giữ lệnh
  std::vector<Pose2D> braking;   // từ thời điểm cam kết (trễ + 1 chu kỳ) phanh tới 0, + stopping_margin
  Reject reject{Reject::NONE};
  bool valid() const {return reject == Reject::NONE;}
  double total_cost{0.0};
  double obstacle_cost{0.0};
  double path_cost{0.0};
  double carrot_cost{0.0};
  double heading_cost{0.0};
  double speed_cost{0.0};
  double oscillation_cost{0.0};
};

// Mô phỏng lệnh cmd từ trạng thái hiện tại:
//   [0, latency)            : vận tốc tiến về prev_cmd (lệnh trước vẫn đang có hiệu lực)
//   [latency, latency + dt) : tiến về cmd (cam kết ít nhất một chu kỳ)
//   [latency + dt, horizon) : tiếp tục tiến về / giữ cmd  (horizon = max(sim_time, latency + dt))
// Đuôi phanh: từ trạng thái tại latency + dt, giảm v và ω về 0 với decel_lim_x / decel_lim_theta.
Trajectory rolloutCommand(
  const DwbParams & p, const Pose2D & start, const Velocity2D & current,
  const Velocity2D & prev_cmd,
  const Velocity2D & cmd, double dt, double horizon, double half_cell, double sweep_radius);
// Kiểm tra cứng: mọi pose của quỹ đạo + đuôi phanh. Trả về NONE nếu được phép; max_cost = chi phí lớn nhất.
Reject checkTrajectory(const Trajectory & t, const CollisionModel & cm, unsigned char & max_cost);
// Chi phí vật cản MỀM (0..1) của một quỹ đạo hợp lệ: max chi phí ô trên chân trời VÀ trên phần nối dài theo cùng
// độ cong tới độ dài cung 'lookahead' (không phụ thuộc tốc độ ứng viên). Xem giải thích trong dwa_core.cpp.
double obstacleProbe(
  const Trajectory & t, const CollisionModel & cm, double lookahead, double half_cell,
  unsigned char horizon_max_cost);

// =====================================================================================================
// Bám đường A* theo tiến trình (độ dài cung), không theo khoảng cách Euclid toàn cục
// =====================================================================================================
class PathTracker
{
public:
  // poses: frame của plan; theta của pose cuối = hướng đích (theta các pose khác bị bỏ qua). Điểm trùng liên
  // tiếp bị gộp. Trả về "" nếu hợp lệ; plan cũ bị xóa nếu không hợp lệ.
  std::string setPlan(const std::vector<Pose2D> & poses);
  void clear();
  void resetProgress();
  bool hasPlan() const {return !pts_.empty();}
  const Pose2D & goal() const {return goal_;}
  double length() const {return s_.empty() ? 0.0 : s_.back();}
  double progress() const {return progress_s_;}
  std::size_t size() const {return pts_.size();}

  struct Result
  {
    double progress{0.0};          // m dọc đường
    double deviation{0.0};         // m, khoảng cách robot -> điểm chiếu
    std::vector<Pose2D> local;     // [điểm chiếu, các đỉnh tiếp theo..., điểm tại progress + length]
  };
  // Cập nhật tiến trình: chỉ tìm trong [progress, progress + search_dist], lấy CỰC TIỂU ĐẦU TIÊN dọc đường
  // (dừng khi khoảng cách tăng quá best + branch_hysteresis) -> không nhảy sang nhánh khác của đường tự cắt /
  // chữ U chỉ vì gần hơn. Tiến trình không bao giờ giảm.
  Result update(
    const Pose2D & robot, double search_dist, double local_length,
    double branch_hysteresis);

private:
  std::vector<Pose2D> pts_;
  std::vector<double> s_;          // độ dài cung tích lũy
  Pose2D goal_;
  double progress_s_{0.0};
  std::size_t seg_{0};             // đoạn [seg_, seg_ + 1] chứa progress_s_
  Pose2D pointAt(double s, std::size_t & seg_hint) const;
};

// Điểm cách đầu đường cục bộ carrot_distance (nội suy); đường ngắn hơn -> điểm cuối.
Pose2D pickCarrot(const std::vector<Pose2D> & local_path, double carrot_distance);
// Khoảng cách từ (x, y) tới polyline.
double distanceToPolyline(double x, double y, const std::vector<Pose2D> & path);

// =====================================================================================================
// Chống dao động ω (giống ý tưởng critic Oscillation của DWB, nhưng là chi phí mềm có đặt lại theo thời gian)
// =====================================================================================================
class OscillationGuard
{
public:
  void reset();
  // Gọi sau mỗi lệnh được chọn.
  void update(const Pose2D & robot, double w_cmd, double now_s, const DwbParams & p);
  // 1.0 nếu ω ngược chiều đã cam kết trong lúc còn "vừa đổi chiều", ngược lại 0.0.
  double penalty(double w) const;
  int flips() const {return flips_;}
  int committedSign() const {return sign_;}

private:
  static constexpr double kEps = 0.05;   // rad/s, |ω| nhỏ hơn coi như không xoay
  int sign_{0};
  bool recently_flipped_{false};
  Pose2D ref_pose_;
  double ref_time_{0.0};
  int flips_{0};
};

// =====================================================================================================
// Một chu kỳ điều khiển
// =====================================================================================================
enum class Phase : std::uint8_t {IDLE, FOLLOW_PATH, ROTATE_TO_PATH, ROTATE_TO_GOAL, BLOCKED};
const char * phaseName(Phase ph);

struct GoalTolerance
{
  double xy{0.15};
  double yaw{0.25};
};

struct ControlInput
{
  Pose2D robot;                      // frame costmap
  Velocity2D velocity;               // đo được (odometry, đã qua ngưỡng của controller_server)
  Velocity2D prev_cmd;               // lệnh đã gửi ở chu kỳ trước
  std::vector<Pose2D> local_path;    // frame costmap, đã cắt ở mép costmap (không rỗng)
  Pose2D goal;                       // frame costmap
  GoalTolerance tol;                 // dung sai THỰC của GoalChecker (hoặc tham số dự phòng)
  double dt{0.1};                    // chu kỳ danh nghĩa
  double max_vel_x_eff{0.15};        // max_vel_x sau speed limit
  double now_s{0.0};
};

// Vận tốc gốc cho dynamic window và các bước acc/decel của pha xoay/dừng (xem velocity_reference).
// Mô phỏng quỹ đạo vẫn bắt đầu từ vận tốc ĐO (an toàn không phụ thuộc lựa chọn này).
Velocity2D referenceVelocity(const DwbParams & p, const ControlInput & in);

struct ControlOutput
{
  bool ok{false};
  std::string error;                 // khi !ok
  Velocity2D cmd;
  Phase phase{Phase::IDLE};
  Pose2D carrot;
  DynamicWindow window;
  std::vector<Trajectory> candidates;   // chỉ khi keep_candidates
  int best{-1};                         // chỉ số trong candidates (khi giữ), -1 nếu không có
  Trajectory chosen;                    // quỹ đạo của lệnh được chọn (cả pha xoay)
  int n_candidates{0};
  int n_valid{0};
  int n_forward_valid{0};               // ứng viên hợp lệ có v > 0 (0 = chỉ còn đứng yên/xoay: chờ, không tiến)
  int n_reject[5]{0, 0, 0, 0, 0};       // theo Reject
  double dist_to_goal{0.0};
  Velocity2D measured;                  // in.velocity (chẩn đoán)
  Velocity2D reference;                 // referenceVelocity() (chẩn đoán)
  double heading_error{0.0};            // rad, sai số của pha xoay (0 khi FOLLOW_PATH)
};

class DwaCore
{
public:
  void reset();             // lifecycle / plan mới
  ControlOutput compute(
    const DwbParams & p, const ControlInput & in, const CollisionModel & cm, bool keep_candidates);
  const OscillationGuard & oscillation() const {return osc_;}
  bool goalLatched() const {return goal_latched_;}

private:
  ControlOutput rotate(
    const DwbParams & p, const ControlInput & in, const CollisionModel & cm, double error,
    Phase phase, ControlOutput out);
  // Lệnh (v giảm theo decel, ω cho trước) được mô phỏng trễ + một chu kỳ + đuôi phanh và kiểm tra cứng.
  ControlOutput checkedCommand(
    const DwbParams & p, const ControlInput & in, const CollisionModel & cm, double w, Phase phase,
    ControlOutput out);
  bool goal_latched_{false};
  bool rotating_to_path_{false};
  int rotate_dir_{0};       // chiều xoay đã chọn (giữ khi sai số gần ±π)
  OscillationGuard osc_;
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__DWA_CORE_HPP_
