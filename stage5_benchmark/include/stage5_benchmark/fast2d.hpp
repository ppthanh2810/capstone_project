// =====================================================================================================
// fast2d.hpp — Fast 2D simulator cho benchmark SAC-DWA (Master Plan v3, bước 5–9). KHÔNG chạm phần cứng.
// =====================================================================================================
//
// Mục tiêu: chạy rất nhiều episode nhanh hơn thời gian thực với CHÍNH code điều hướng của robot:
//   - stage3_navigation::DwaCore / PathTracker / validateParams / pickValidCarrot  (CustomDWB, libdwa_core)
//   - stage3_navigation::ReplanSupervisor + path_blockage                         (goal_path_bridge replan)
//   - nav2_navfn_planner::NavFn (A* của planner_server, cùng thư viện)
//   - nav2_costmap_2d::Costmap2D + raytraceLine + footprint (inscribed radius như Nav2)
//   - stage4_gazebo::ZlacDriverModel (trễ lệnh + ramp driver, như zlac_sim trong Gazebo)
// Phần viết lại (mô phỏng, cố ý đơn giản, ghi rõ giới hạn):
//   - World 2D = Static Map M (lưới 0/100/-1) + vật cản mới Δ (hộp xoay / tròn). LiDAR chỉ thấy W = M ⊕ Δ qua
//     raycast (FOV ±90° trước, 720 tia/360°, 10 Hz, nhiễu Gauss, tầm 0.05–16 m); costmap chỉ nhận dữ liệu LiDAR.
//   - ObstacleLayer (raytrace clear + mark, rolling window, footprint clearing, track_unknown, inf_is_valid) và
//     InflationLayer (công thức + luật unknown của Nav2 Humble) tái hiện trên Costmap2D.
//   - controller_server Humble (10 Hz, progress checker, goal checker stateful, failure_tolerance, chờ costmap
//     current), twist_mux timeout 0.5 s + watchdog driver 0.5 s, odometry v lọc thông thấp như zlac8015d_odom.
//   - Định vị HOÀN HẢO (pose = ground truth): Planner Benchmark tách khỏi localization (Master Plan v3 A8).
//   - Robot unicycle động học, không trượt, không mô hình điện/mô-men/ma sát (A2).
// Ground truth W chỉ dùng cho LiDAR, va chạm và chấm điểm; không bao giờ đưa vào DWA/A*/SAC.
// =====================================================================================================

#ifndef STAGE5_BENCHMARK__FAST2D_HPP_
#define STAGE5_BENCHMARK__FAST2D_HPP_

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "stage3_navigation/dwa_core.hpp"
#include "stage3_navigation/replan_supervisor.hpp"
#include "stage4_gazebo/zlac_driver_model.hpp"

namespace stage5_benchmark
{

using stage3_navigation::Pose2D;
using stage3_navigation::Velocity2D;

// ---------------------------------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------------------------------
struct Shape
{
  enum class Type {BOX, CIRCLE};
  Type type{Type::BOX};
  double x{0.0}, y{0.0};
  double sx{0.0}, sy{0.0}, yaw{0.0};   // BOX: kích thước toàn phần, góc xoay
  double r{0.0};                        // CIRCLE
  // Khoảng cách có dấu từ điểm tới biên (âm = bên trong).
  double distance(double px, double py) const;
  // Giao tia (ox, oy) + t·(dx, dy), |d| = 1 -> t nhỏ nhất >= 0, +inf nếu không cắt.
  double ray(double ox, double oy, double dx, double dy) const;
};

// Lưới chiếm chỗ kiểu nav_msgs/OccupancyGrid (0 free, 100 occupied, -1 unknown), hàng y tăng dần.
struct Grid
{
  double origin_x{0.0}, origin_y{0.0}, resolution{0.05};
  int width{0}, height{0};
  std::vector<std::int8_t> data;
  bool inside(int i, int j) const {return i >= 0 && j >= 0 && i < width && j < height;}
  std::int8_t at(int i, int j) const {return data[static_cast<std::size_t>(j) * width + i];}
  bool occupied(int i, int j) const {return inside(i, j) && at(i, j) >= 50;}
};

class World
{
public:
  World(Grid map, std::vector<Shape> extra);
  // Khoảng cách tâm robot -> vật cản gần nhất (biên ô M hoặc Δ), tìm trong bán kính search (cắt ở search).
  double clearance(double x, double y, double search = 1.0) const;
  // Tia LiDAR: khoảng cách tới điểm chạm đầu tiên trong [0, max_range], +inf nếu không chạm.
  double raycast(double ox, double oy, double angle, double max_range) const;
  const Grid & map() const {return map_;}
  const std::vector<Shape> & extra() const {return extra_;}

private:
  Grid map_;
  std::vector<Shape> extra_;
};

// ---------------------------------------------------------------------------------------------------
// Cấu hình (một nguồn: stage5_benchmark/config/benchmark.yaml -> Python -> JSON)
// ---------------------------------------------------------------------------------------------------
struct SimConfig
{
  // robot + driver (zlac_sim / zlac8015d_move)
  double wheel_radius{0.0535};
  double wheel_distance{0.34};
  double accel_time{0.5};
  double decel_time{0.5};
  double command_delay{0.15};
  double command_timeout{0.5};          // watchdog driver
  double twist_mux_timeout{0.5};
  double robot_radius{0.20};            // thân thật (va chạm GT)
  // LiDAR
  double lidar_x{0.15};                 // laser_frame trước base_footprint
  int lidar_beams{720};
  double lidar_half_fov{1.57};          // laser_filters giữ |góc| < 1.57 rad
  double lidar_noise{0.01};
  double lidar_range_min{0.05};
  double lidar_range_max{16.0};
  // costmap
  double local_size{2.0};
  double resolution{0.05};             // global costmap = map
  double local_resolution{0.05};       // local costmap (A1: thử 0.025 để giảm sai số rời rạc)
  double footprint_radius{0.20};
  double local_padding{0.05};
  double global_padding{0.05};
  double local_inflation{0.30};
  double global_inflation{0.40};
  double cost_scaling{3.0};
  bool local_track_unknown{false};
  bool inf_is_valid{false};
  double obstacle_max_range{2.5};
  double raytrace_max_range{3.0};
  double local_update_rate{5.0};
  double global_update_rate{2.0};
  double expected_update_rate{0.5};     // local: scan cũ hơn -> costmap không current
  bool global_obstacle_layer{true};
  bool local_footprint_clearing{true};  // ObstacleLayer footprint_clearing_enabled (đa giác đã padding)
  bool global_footprint_clearing{true};
  bool allow_unknown{false};
  // controller_server
  double controller_rate{10.0};
  double failure_tolerance{0.3};
  double progress_radius{0.25};
  double progress_time{10.0};
  double xy_goal_tolerance{0.15};
  double yaw_goal_tolerance{0.25};
  // goal_path_bridge
  double follow_timeout{120.0};
  double max_start_distance{0.5};
  double watch_rate{5.0};
  // episode
  double physics_dt{0.005};
  double warmup{1.0};                   // s cảm biến chạy trước khi gửi goal (robot đứng yên, đã định vị)
  double timeout{120.0};                // T_max (s)
  double post_terminal{2.5};            // s mô phỏng thêm sau trạng thái cuối (đo đứng yên / Safe Stop)
  double stuck_window{30.0};            // s: dịch < stuck_radius -> dừng sớm (evaluator vẫn chấm từ trace)
  double stuck_radius{0.25};
  double odom_alpha{0.2};               // zlac8015d_odom: lọc thông thấp v 20 Hz
  std::uint32_t seed{0};
  // seed != 0: nhiễu Gauss lên pose xuất phát (đặt robot / định vị lúc bắt đầu), để các lần lặp khác nhau. 0 = danh nghĩa.
  double start_noise_xy{0.03};
  double start_noise_yaw{0.05};
  // tiêm lỗi
  double stale_scan_start{-1.0};        // s; < 0 = tắt
  double stale_scan_duration{0.0};
  bool keep_trace{true};
};

struct TraceRow
{
  double t, x, y, yaw, v, w, v_cmd, w_cmd, clearance;
  int phase;              // stage3_navigation::Phase, -1 = không tính (controller không chạy), -2 = lỗi
  int n_valid, n_forward, n_offmap, n_collision, n_unknown;
  double compute_ms;
  double carrot_shift;
  // R2 (2026-11-27): chẩn đoán không gian trống (DwaCore, không phụ thuộc tham số) + trạng thái ReplanSupervisor
  double fs_clearance{-1.0};
  double fs_free_ratio{-1.0};
  int sup_state{0};          // stage3_navigation::ReplanState
  double block_dist{-1.0};   // m dọc đường tới ô bị chặn, -1 = thông
  int local_bad{0};
  int n_candidates{0};
  double deviation{0.0};   // m lệch ngang khỏi Global Path hiện tại (ReplanSupervisor)
};

struct Event
{
  double t;
  std::string text;
};

struct EpisodeResult
{
  std::string terminal_state{"none"};     // trạng thái cuối của bridge (SUCCEEDED / FAILED: ... / ABORTED...)
  double t_terminal{-1.0};
  double t_end{0.0};
  bool collision{false};
  double t_collision{-1.0};
  double min_clearance{std::numeric_limits<double>::infinity()};
  bool stuck_early_stop{false};
  bool timeout{false};
  std::string error;                      // lỗi mô phỏng (SYSTEM_ERROR)
  int replans{0};
  int obstacle_replans{0};
  int abort_replans{0};
  double max_goal_stagnation{0.0};      // s, ReplanSupervisor::maxGoalStagnation (hiệu chỉnh watchdog V4)
  double path_length{0.0};                // m robot đi (GT)
  double initial_path_length{0.0};        // m A* ban đầu
  std::vector<double> plan_lengths;
  // Mọi Global Path đã gửi FollowPath (t, đường): [0] = ban đầu, sau đó mỗi lần replan (để vẽ trước/sau replan)
  std::vector<std::pair<double, std::vector<Pose2D>>> plans;
  int controller_cycles{0};
  int cycles_no_valid{0};
  int cycles_holding{0};
  long long rejects[5]{0, 0, 0, 0, 0};    // Reject NONE, COLLISION, UNKNOWN, OFF_MAP, BRAKING (tổng)
  long long candidates{0};
  double compute_ms_mean{0.0};
  double compute_ms_p99{0.0};
  double compute_ms_max{0.0};
  int missed_deadlines{0};
  int stale_cycles{0};
  double final_x{0.0}, final_y{0.0}, final_yaw{0.0};
  std::vector<TraceRow> trace;
  std::vector<Event> events;
};

// ---------------------------------------------------------------------------------------------------
// Episode
// ---------------------------------------------------------------------------------------------------
struct Scenario
{
  Grid map;                       // M (planner, costmap static layer)
  std::vector<Shape> new_obstacles;   // Δ (chỉ trong world)
  Pose2D start;
  Pose2D goal;
};

class Episode
{
public:
  Episode(
    const Scenario & sc, const SimConfig & cfg, const stage3_navigation::DwbParams & dwb,
    const stage3_navigation::ReplanParams & replan);
  ~Episode();

  // Tiến mô phỏng tối đa `duration` giây (dừng sớm nếu xong). Trả về done().
  bool step(double duration);
  bool run() {return step(1e9);}
  bool done() const {return done_;}
  double time() const {return t_;}

  // Đổi tham số CustomDWB giữa episode (M1/SAC). "" = OK; ngược lại lý do (giữ tham số cũ, như ros2 param set).
  std::string setDwbParams(const stage3_navigation::DwbParams & p);
  const stage3_navigation::DwbParams & dwbParams() const {return dwb_;}

  // ---- quan sát (chỉ dữ liệu robot thật có) ----
  Pose2D pose() const {return pose_;}                       // = ước lượng định vị (hoàn hảo trong Planner Benchmark)
  Velocity2D measuredVelocity() const {return meas_;}
  Velocity2D lastCommand() const {return last_cmd_;}
  const std::vector<float> & scan() const {return scan_ranges_;}   // 720 tia, góc 0 = trước, nửa sau NaN
  const nav2_costmap_2d::Costmap2D & localCostmap() const;
  const nav2_costmap_2d::Costmap2D & globalCostmap() const;
  const std::vector<Pose2D> & currentPlan() const {return plan_;}
  const stage3_navigation::ControlOutput & lastControl() const {return last_out_;}
  const std::string & bridgeState() const {return bridge_state_;}
  bool following() const {return following_;}
  bool blocked() const;
  int replans() const;      // số lần lập lại trong phiên (đang chạy)
  double progress() const;
  double groundTruthClearance() const;     // CHỈ cho reward/evaluator, không đưa vào policy

  const EpisodeResult & result() const {return res_;}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  // trạng thái hay dùng (đọc nhanh)
  SimConfig cfg_;
  stage3_navigation::DwbParams dwb_;
  stage3_navigation::ReplanParams replan_;
  double t_{0.0};
  bool done_{false};
  Pose2D pose_;
  Velocity2D meas_;
  Velocity2D last_cmd_;
  std::vector<float> scan_ranges_;
  std::vector<Pose2D> plan_;
  stage3_navigation::ControlOutput last_out_;
  std::string bridge_state_{"IDLE"};
  bool following_{false};
  EpisodeResult res_;

  void tick();
  void publishState(const std::string & s);
};

// Lập kế hoạch NavFn A* trên một costmap (như NavfnPlanner::makePlan, Humble). Rỗng = không có đường.
std::vector<Pose2D> navfnPlan(
  const nav2_costmap_2d::Costmap2D & costmap, const Pose2D & start, const Pose2D & goal,
  bool allow_unknown);

// Bán kính nội tiếp của footprint tròn (robot_radius) + padding, tính đúng như Nav2 (đa giác 16 đỉnh, padFootprint).
double nav2InscribedRadius(double robot_radius, double padding);

// Static costmap của M + inflation (như global_costmap lúc chưa có quan sát): dùng cho generator / feasibility
// (cùng A* với hệ thống). Trả về costmap đã inflate.
std::unique_ptr<nav2_costmap_2d::Costmap2D> staticCostmap(
  const Grid & map, double robot_radius, double padding, double inflation_radius, double cost_scaling);

// Áp dụng tham số theo tên (dùng chung Python bindings + CLI). Trả về tên không hợp lệ (rỗng = OK).
std::string setDwbField(stage3_navigation::DwbParams & p, const std::string & name, double value,
  const std::string & text = "");
std::string setReplanField(stage3_navigation::ReplanParams & p, const std::string & name, double value,
  const std::string & text = "");
std::string setSimField(SimConfig & c, const std::string & name, double value);
std::vector<std::string> dwbFieldNames();

}  // namespace stage5_benchmark

#endif  // STAGE5_BENCHMARK__FAST2D_HPP_
