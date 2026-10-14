// =====================================================================================================
// sim_world.hpp — Fixture kín vòng KHÔNG cần ROS graph cho lõi DWA (test + benchmark)
// =====================================================================================================
//
// Mô hình (cố ý đơn giản, KHÔNG phải mô phỏng vật lý của robot thật):
//   - Thế giới 2D: hình tròn + hình chữ nhật (vật cản), khoảng cách hình học chính xác.
//   - Local costmap: Costmap2D rolling W x W m quanh robot mỗi chu kỳ, cost theo công thức InflationLayer của
//     Nav2: d <= 0 -> LETHAL, d <= inscribed -> INSCRIBED, d <= inflation_radius ->
//     252 * exp(-factor * (d - inscribed)), xa hơn -> 0 (d = khoảng cách tâm ô -> vật cản). Toàn bộ vật cản đều
//     "được thấy" (không mô phỏng nửa sau bị che: phần đó do fixture ROS test/offline/ kiểm tra).
//   - Robot: unicycle, lệnh tới bánh sau latency giây, vận tốc bám lệnh với giới hạn gia tốc/giảm tốc RIÊNG
//     của robot (có thể khác tham số controller), tích phân 2 ms. Odometry vận tốc có nhiễu Gauss (seed).
//   - Vòng điều khiển 10 Hz giống controller_server Humble: PathTracker + DwaCore; lỗi -> lệnh 0, quá
//     failure_tolerance -> abort; GoalChecker stateful (xy rồi yaw); progress checker 0.25 m / 10 s.
// =====================================================================================================

#ifndef SIM_WORLD_HPP_
#define SIM_WORLD_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "stage3_navigation/dwa_core.hpp"

namespace stage3_navigation
{
namespace sim
{

struct Circle
{
  double x, y, r;
};
struct Box
{
  double xmin, ymin, xmax, ymax;
};

struct World
{
  std::vector<Circle> circles;
  std::vector<Box> boxes;
  // Khoảng cách có dấu từ (x, y) tới vật cản gần nhất (âm = bên trong). +inf nếu không có vật cản.
  double distance(double x, double y) const;
};

struct CostmapSpec
{
  double size{2.0};               // m (rolling window size x size)
  double resolution{0.05};
  double inscribed{0.30};         // robot_radius 0.20 + footprint_padding 0.10 (như local_costmap)
  double inflation_radius{0.45};
  double cost_scaling_factor{3.0};
};

// Ghi cost vào costmap (đã có kích thước/origin) theo thế giới.
void rasterize(const World & w, const CostmapSpec & spec, nav2_costmap_2d::Costmap2D & cm);

struct RobotModel
{
  double acc_x{0.5};
  double decel_x{0.5};
  double acc_theta{1.0};
  double decel_theta{1.0};
  double latency{0.1};            // s, lệnh -> bánh
  double odom_vel_noise{0.0};     // m/s và rad/s (độ lệch chuẩn)
  double radius{0.20};            // bán kính thân THẬT để đo va chạm / khoảng hở
};

struct Scenario
{
  std::string name;
  World world;
  std::vector<Pose2D> path;       // frame odom (= plan frame trong fixture này)
  Pose2D start;
  Velocity2D start_vel;
  Velocity2D start_cmd;           // lệnh đang có hiệu lực lúc bắt đầu (vd. PS4 vừa thả)
  double timeout{90.0};
  bool expect_success{true};      // false: dừng/abort có lý do là kết quả đúng
  double start_pose_noise{0.0};   // m, rad (độ lệch chuẩn) theo seed
};

struct RunMetrics
{
  std::string scenario;
  std::string profile;
  std::uint32_t seed{0};
  bool success{false};
  std::string result;             // "succeeded" | "aborted: ..." | "timeout"
  double time_s{0.0};
  int collisions{0};              // số bước tích phân thân robot chồng vật cản
  // m, khoảng cách thân robot (bán kính thật) -> vật cản; +inf khi kịch bản không có vật cản
  double min_clearance{std::numeric_limits<double>::infinity()};
  double cross_track_rms{0.0};
  double cross_track_max{0.0};
  int w_sign_flips{0};
  int limit_violations{0};        // lệnh vượt [min,max] của v / |ω|
  int nonfinite_cmds{0};
  int cmd_rate_exceed{0};         // |Δ lệnh| giữa hai chu kỳ > acc*dt (thông tin; DWA neo cửa sổ vào v ĐO)
  int blocked_cycles{0};
  double final_xy_error{0.0};
  double final_yaw_error{0.0};
  double stop_latency{-1.0};      // s, từ lần BLOCKED đầu tiên tới khi |v| < 0.01 (nếu có)
  double stop_distance{-1.0};     // m, quãng đi được trong khoảng đó
  double path_length{0.0};        // m robot đã đi
  std::vector<double> compute_ms;
  int n_cycles{0};
  int missed_deadlines{0};        // compute > 100 ms
};

struct TraceRow
{
  double t, x, y, theta, v, w, v_cmd, w_cmd;
  std::string phase;
};

// Chạy một kịch bản. trace != nullptr -> ghi từng chu kỳ.
RunMetrics runScenario(
  const DwbParams & p, const Scenario & sc, const RobotModel & robot, std::uint32_t seed,
  const CostmapSpec & spec = CostmapSpec{}, std::vector<TraceRow> * trace = nullptr);

// Các kịch bản chuẩn (dùng chung cho gtest và benchmark).
std::vector<Scenario> standardScenarios();
Scenario findScenario(const std::string & name);

// Tiện ích: đường gấp khúc qua các điểm, lấy mẫu 0.05 m (giống NavFn), hướng pose cuối = goal_yaw.
std::vector<Pose2D> polyline(
  const std::vector<Pose2D> & waypoints, double goal_yaw,
  double step = 0.05);

double percentile(std::vector<double> v, double q);

}  // namespace sim
}  // namespace stage3_navigation

#endif  // SIM_WORLD_HPP_
