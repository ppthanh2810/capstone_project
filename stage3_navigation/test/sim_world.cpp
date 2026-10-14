// sim_world.cpp — fixture kín vòng cho lõi DWA (xem sim_world.hpp: mô hình và giới hạn).

#include "sim_world.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace stage3_navigation
{
namespace sim
{

double World::distance(double x, double y) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & c : circles) {
    best = std::min(best, std::hypot(x - c.x, y - c.y) - c.r);
  }
  for (const auto & b : boxes) {
    const double dx = std::max({b.xmin - x, 0.0, x - b.xmax});
    const double dy = std::max({b.ymin - y, 0.0, y - b.ymax});
    double d = std::hypot(dx, dy);
    if (dx == 0.0 && dy == 0.0) {
      d = -std::min({x - b.xmin, b.xmax - x, y - b.ymin, b.ymax - y});
    }
    best = std::min(best, d);
  }
  return best;
}

void rasterize(const World & w, const CostmapSpec & spec, nav2_costmap_2d::Costmap2D & cm)
{
  const double res = cm.getResolution();
  for (unsigned int j = 0; j < cm.getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < cm.getSizeInCellsX(); ++i) {
      double wx, wy;
      cm.mapToWorld(i, j, wx, wy);
      const double d = w.distance(wx, wy);
      unsigned char c = nav2_costmap_2d::FREE_SPACE;
      if (d <= 0.5 * res) {
        c = nav2_costmap_2d::LETHAL_OBSTACLE;   // vật cản chiếm ô
      } else if (d <= spec.inscribed) {
        c = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      } else if (d <= spec.inflation_radius) {
        c = static_cast<unsigned char>(
          (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) *
          std::exp(-spec.cost_scaling_factor * (d - spec.inscribed)));
      }
      cm.setCost(i, j, c);
    }
  }
}

std::vector<Pose2D> polyline(const std::vector<Pose2D> & wp, double goal_yaw, double step)
{
  std::vector<Pose2D> out;
  if (wp.empty()) {
    return out;
  }
  out.push_back(wp.front());
  for (std::size_t i = 1; i < wp.size(); ++i) {
    const double len = std::hypot(wp[i].x - wp[i - 1].x, wp[i].y - wp[i - 1].y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    const double yaw = std::atan2(wp[i].y - wp[i - 1].y, wp[i].x - wp[i - 1].x);
    for (int k = 1; k <= n; ++k) {
      const double u = static_cast<double>(k) / n;
      out.push_back(
        Pose2D{wp[i - 1].x + u * (wp[i].x - wp[i - 1].x), wp[i - 1].y + u * (wp[i].y - wp[i - 1].y),
          yaw});
    }
  }
  out.back().theta = goal_yaw;
  return out;
}

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const double idx = q * static_cast<double>(v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (idx - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

RunMetrics runScenario(
  const DwbParams & p, const Scenario & sc, const RobotModel & robot, std::uint32_t seed,
  const CostmapSpec & spec, std::vector<TraceRow> * trace)
{
  RunMetrics m;
  m.scenario = sc.name;
  m.seed = seed;
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);

  const double dt = 0.1;              // controller_frequency 10 Hz
  const double h = 0.002;             // bước vật lý
  const double failure_tolerance = 0.3;
  const GoalTolerance tol{0.15, 0.25};
  const double search_hyst = std::max(2.0 * spec.resolution, 0.1);

  PathTracker tracker;
  const std::string err = tracker.setPlan(sc.path);
  if (!err.empty()) {
    m.result = "aborted: " + err;
    return m;
  }
  DwaCore core;

  Pose2D pose = sc.start;
  if (sc.start_pose_noise > 0.0) {
    pose.x += sc.start_pose_noise * gauss(rng);
    pose.y += sc.start_pose_noise * gauss(rng);
    pose.theta = normalizeAngle(pose.theta + sc.start_pose_noise * gauss(rng));
  }
  Velocity2D vel = sc.start_vel;
  Velocity2D active = sc.start_cmd;
  std::deque<std::pair<double, Velocity2D>> queue;
  Velocity2D last_cmd = sc.start_cmd;
  bool have_prev = true;

  nav2_costmap_2d::Costmap2D cm(
    static_cast<unsigned int>(std::lround(spec.size / spec.resolution)),
    static_cast<unsigned int>(std::lround(spec.size / spec.resolution)), spec.resolution, 0.0, 0.0);

  bool xy_latched = false;
  Pose2D progress_base = pose;
  double progress_time = 0.0;
  double last_valid = 0.0;
  double first_block = -1.0;
  Pose2D block_pose;
  double t = 0.0;
  double ct_sq = 0.0;
  int ct_n = 0;
  int last_sign = 0;
  Velocity2D prev_sent;
  bool have_sent = false;
  bool finished = false;

  const auto physics = [&](double duration) {
      const int n = static_cast<int>(std::lround(duration / h));
      for (int k = 0; k < n; ++k) {
        t += h;
        while (!queue.empty() && queue.front().first <= t + 1e-12) {
          active = queue.front().second;
          queue.pop_front();
        }
        const double v1 = stepToward(vel.v, active.v, robot.acc_x, robot.decel_x, h);
        const double w1 = stepToward(vel.w, active.w, robot.acc_theta, robot.decel_theta, h);
        SimState s{pose, vel};
        // tích phân cung với vận tốc trung điểm (bước 2 ms)
        const double vm = 0.5 * (vel.v + v1), wm = 0.5 * (vel.w + w1);
        if (std::fabs(wm) < 1e-9) {
          s.pose.x += vm * h * std::cos(pose.theta);
          s.pose.y += vm * h * std::sin(pose.theta);
        } else {
          s.pose.x += vm / wm * (std::sin(pose.theta + wm * h) - std::sin(pose.theta));
          s.pose.y -= vm / wm * (std::cos(pose.theta + wm * h) - std::cos(pose.theta));
          s.pose.theta = normalizeAngle(pose.theta + wm * h);
        }
        m.path_length += std::hypot(s.pose.x - pose.x, s.pose.y - pose.y);
        pose = s.pose;
        vel = Velocity2D{v1, w1};
        const double clearance = sc.world.distance(pose.x, pose.y) - robot.radius;
        m.min_clearance = std::min(m.min_clearance, clearance);
        if (clearance < 0.0) {
          ++m.collisions;
        }
      }
    };

  while (t < sc.timeout && !finished) {
    // ---- progress checker (SimpleProgressChecker: 0.25 m / 10 s) ----
    if (std::hypot(pose.x - progress_base.x, pose.y - progress_base.y) > 0.25) {
      progress_base = pose;
      progress_time = t;
    } else if (t - progress_time > 10.0) {
      m.result = "aborted: Failed to make progress";
      break;
    }

    // ---- một chu kỳ điều khiển ----
    Velocity2D meas{vel.v + robot.odom_vel_noise * gauss(rng),
      vel.w + robot.odom_vel_noise * gauss(rng)};
    if (std::fabs(meas.v) < 0.001) {meas.v = 0.0;}
    if (std::fabs(meas.w) < 0.001) {meas.w = 0.0;}

    const auto c0 = std::chrono::steady_clock::now();
    ControlOutput out;
    const auto upd = tracker.update(
      pose, p.closest_search_distance, p.local_plan_length,
      search_hyst);
    const double ox = std::floor((pose.x - 0.5 * spec.size) / spec.resolution) * spec.resolution;
    const double oy = std::floor((pose.y - 0.5 * spec.size) / spec.resolution) * spec.resolution;
    cm.updateOrigin(ox, oy);
    rasterize(sc.world, spec, cm);
    std::vector<Pose2D> local;
    for (const auto & q : upd.local) {
      unsigned int mx, my;
      if (!cm.worldToMap(q.x, q.y, mx, my)) {
        break;
      }
      local.push_back(q);
    }
    if (tracker.size() > 1 && upd.deviation > p.max_path_deviation) {
      out.error = "robot too far from the plan";
    } else if (local.empty()) {
      out.error = "no part of the plan inside the local costmap";
    } else {
      ControlInput in;
      in.robot = pose;
      in.velocity = meas;
      in.prev_cmd = have_prev ? last_cmd : meas;
      in.local_path = local;
      in.goal = tracker.goal();
      in.tol = tol;
      in.dt = dt;
      in.max_vel_x_eff = p.max_vel_x;
      in.now_s = t;
      const CollisionModel model(&cm, CollisionMode::CENTER, {}, p.unknown_is_obstacle);
      out = core.compute(p, in, model, false);
    }
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
    m.compute_ms.push_back(ms);
    ++m.n_cycles;
    if (ms > 100.0) {
      ++m.missed_deadlines;
    }

    Velocity2D cmd;
    if (out.ok) {
      cmd = out.cmd;
      last_valid = t;
    } else {
      ++m.blocked_cycles;
      if (first_block < 0.0) {
        first_block = t;
        block_pose = pose;
      }
      if (t - last_valid > failure_tolerance) {
        m.result = "aborted: " + out.error;
      }
    }
    if (!std::isfinite(cmd.v) || !std::isfinite(cmd.w)) {
      ++m.nonfinite_cmds;
      cmd = Velocity2D{};
    }
    if (cmd.v > p.max_vel_x + 1e-9 || cmd.v < p.min_vel_x - 1e-9 ||
      std::fabs(cmd.w) > p.max_vel_theta + 1e-9)
    {
      ++m.limit_violations;
    }
    if (have_sent &&
      (std::fabs(cmd.v - prev_sent.v) > std::max(p.acc_lim_x, p.decel_lim_x) * dt + 1e-6 ||
      std::fabs(cmd.w - prev_sent.w) > std::max(p.acc_lim_theta, p.decel_lim_theta) * dt + 1e-6))
    {
      ++m.cmd_rate_exceed;
    }
    prev_sent = cmd;
    have_sent = true;
    const int sgn = std::fabs(cmd.w) > 0.05 ? (cmd.w > 0 ? 1 : -1) : 0;
    if (sgn != 0 && last_sign != 0 && sgn != last_sign) {
      ++m.w_sign_flips;
    }
    if (sgn != 0) {
      last_sign = sgn;
    }
    last_cmd = cmd;
    have_prev = true;
    queue.emplace_back(t + robot.latency, cmd);
    if (trace) {
      trace->push_back(
        TraceRow{t, pose.x, pose.y, pose.theta, vel.v, vel.w, cmd.v, cmd.w,
          out.ok ? phaseName(out.phase) : "BLOCKED"});
    }
    {
      const double d = distanceToPolyline(pose.x, pose.y, sc.path);
      ct_sq += d * d;
      ++ct_n;
      m.cross_track_max = std::max(m.cross_track_max, d);
    }
    if (!m.result.empty()) {
      break;   // abort: server phát lệnh 0 (xử lý bên dưới)
    }

    // ---- GoalChecker stateful (sau khi publish lệnh, như controller_server) ----
    const Pose2D g = tracker.goal();
    const double dxy = std::hypot(pose.x - g.x, pose.y - g.y);
    if (!xy_latched && dxy <= tol.xy) {
      xy_latched = true;
    }
    if (xy_latched && std::fabs(normalizeAngle(g.theta - pose.theta)) <= tol.yaw) {
      m.success = true;
      m.result = "succeeded";
      finished = true;
      break;
    }
    physics(dt);
  }
  if (m.result.empty()) {
    m.result = "timeout";
  }
  m.time_s = t;

  // Kết thúc (thành công / abort / timeout): controller_server phát lệnh 0 -> mô phỏng tiếp 3 s để đo quãng dừng.
  queue.emplace_back(t + robot.latency, Velocity2D{});
  const Pose2D end_cmd_pose = pose;
  const double end_t = t;
  double stopped_at = -1.0;
  for (int k = 0; k < 150; ++k) {
    physics(0.02);
    if (stopped_at < 0.0 && std::fabs(vel.v) < 0.01 && std::fabs(vel.w) < 0.01) {
      stopped_at = t;
    }
  }
  if (first_block >= 0.0) {
    // dừng tính từ lần BLOCKED đầu tiên (lệnh 0 đầu tiên của chuỗi lỗi)
    const Pose2D ref = m.success ? end_cmd_pose : block_pose;
    const double ref_t = m.success ? end_t : first_block;
    if (stopped_at >= 0.0) {
      m.stop_latency = stopped_at - ref_t;
    }
    m.stop_distance = std::hypot(pose.x - ref.x, pose.y - ref.y);
  }
  const Pose2D g = tracker.goal();
  m.final_xy_error = std::hypot(pose.x - g.x, pose.y - g.y);
  m.final_yaw_error = std::fabs(normalizeAngle(g.theta - pose.theta));
  m.cross_track_rms = ct_n > 0 ? std::sqrt(ct_sq / ct_n) : 0.0;
  return m;
}

std::vector<Scenario> standardScenarios()
{
  std::vector<Scenario> v;
  const auto add = [&v](Scenario s) {v.push_back(std::move(s));};

  {
    Scenario s;
    s.name = "straight";
    s.path = polyline({{0, 0, 0}, {3, 0, 0}}, 0.0);
    s.start_pose_noise = 0.02;
    add(s);
  }
  {
    Scenario s;
    s.name = "l_turn";
    s.path = polyline({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}}, M_PI / 2);
    s.start_pose_noise = 0.02;
    add(s);
  }
  {
    Scenario s;
    s.name = "s_curve";
    std::vector<Pose2D> wp;
    for (int i = 0; i <= 45; ++i) {
      const double x = 0.1 * i;
      wp.push_back(Pose2D{x, 0.5 * std::sin(x * M_PI / 1.5), 0});
    }
    s.path = polyline(wp, 0.0);
    s.start.theta = std::atan2(0.5 * M_PI / 1.5, 1.0);
    s.start_pose_noise = 0.02;
    add(s);
  }
  {
    Scenario s;   // đường tự cắt tại (1, 0): đi qua hai lần, tiến trình không được nhảy
    s.name = "self_intersect";
    s.path = polyline(
      {{0, 0, 0}, {2, 0, 0}, {2, 1.2, 0}, {1, 1.2, 0}, {1, -0.8, 0}, {3.0, -0.8,
          0}}, 0.0);
    s.timeout = 150.0;
    s.start_pose_noise = 0.02;
    add(s);
  }
  {
    Scenario s;   // điểm trùng liên tiếp (planner lặp điểm)
    s.name = "duplicates";
    auto base = polyline({{0, 0, 0}, {2, 0, 0}}, 0.0);
    for (const auto & q : base) {
      s.path.push_back(q);
      s.path.push_back(q);
    }
    s.path.push_back(base.back());
    add(s);
  }
  {
    Scenario s;
    s.name = "single_pose";
    s.path = {Pose2D{0.6, 0.3, 0.5}};   // đích phải nằm trong local costmap (±1 m)
    add(s);
  }
  {
    Scenario s;   // đích rất gần (trong dung sai xy): chỉ xoay
    s.name = "goal_near";
    s.path = polyline({{0, 0, 0}, {0.08, 0, 0}}, M_PI / 2);
    add(s);
  }
  {
    Scenario s;   // hành lang 1.0 m (tâm còn 0.4 m tự do với bán kính nội tiếp 0.30)
    s.name = "corridor_ok";
    s.path = polyline({{0, 0, 0}, {4, 0, 0}}, 0.0);
    s.world.boxes = {{0.6, 0.5, 3.4, 0.7}, {0.6, -0.7, 3.4, -0.5}};
    s.start_pose_noise = 0.02;
    add(s);
  }
  {
    Scenario s;   // khe 0.54 m: thân 0.40 m lọt (A* cho qua) nhưng < 2 x 0.30 -> controller phải dừng
    s.name = "corridor_narrow";
    s.path = polyline({{0, 0, 0}, {3.5, 0, 0}}, 0.0);
    s.world.boxes = {{1.2, 0.27, 2.2, 0.8}, {1.2, -0.8, 2.2, -0.27}};
    s.expect_success = false;
    add(s);
  }
  {
    Scenario s;   // vật cản nhỏ trên đường, còn chỗ vòng
    s.name = "obstacle_bypass";
    s.path = polyline({{0, 0, 0}, {4, 0, 0}}, 0.0);
    s.world.circles = {{2.0, 0.0, 0.10}};
    s.expect_success = false;   // dừng có lý do cũng chấp nhận (không hứa né mọi vật cản)
    s.start_pose_noise = 0.02;
    add(s);
  }
  {
    Scenario s;   // tường chắn ngang toàn bộ
    s.name = "blocked_wall";
    s.path = polyline({{0, 0, 0}, {4, 0, 0}}, 0.0);
    s.world.boxes = {{2.0, -2.0, 2.1, 2.0}};
    s.expect_success = false;
    add(s);
  }
  {
    // Vật cản sát đường: mép cách đường 0.36 m = bán kính nội tiếp 0.30 + 1 ô (0.05) + 0.01. Với 0.32 m robot
    // dừng trước vật cản: ô chứa tâm robot (tâm ô lệch tới 0.025 m) đã thành INSCRIBED do lượng tử hóa.
    Scenario s;
    s.name = "obstacle_near_path";
    s.path = polyline({{0, 0, 0}, {4, 0, 0}}, 0.0);
    s.world.boxes = {{1.5, 0.36, 2.5, 1.0}};
    add(s);
  }
  {
    Scenario s;   // vật cản đối xứng chắn giữa đường (thử dao động trái/phải)
    s.name = "symmetric_obstacle";
    s.path = polyline({{0, 0, 0}, {4, 0, 0}}, 0.0);
    s.world.boxes = {{1.8, -0.3, 2.2, 0.3}};
    s.expect_success = false;
    add(s);
  }
  {
    Scenario s;   // xuất phát quay lưng về đường (gần ±π)
    s.name = "start_backwards";
    s.path = polyline({{0, 0, 0}, {2.5, 0, 0}}, 0.0);
    s.start.theta = M_PI - 0.01;
    add(s);
  }
  {
    Scenario s;   // vừa thả PS4 khi đang chạy 0.5 m/s (> max_vel_x 0.15)
    s.name = "overspeed_start";
    s.path = polyline({{0, 0, 0}, {5, 0, 0}}, 0.0);
    s.start_vel = Velocity2D{0.5, 0.0};
    s.start_cmd = Velocity2D{0.5, 0.0};
    add(s);
  }
  {
    Scenario s;   // overspeed khi có vật cản phía trước: không được chạm dù phải giảm từ 0.5 m/s
    s.name = "overspeed_wall";
    s.path = polyline({{0, 0, 0}, {5, 0, 0}}, 0.0);
    s.world.boxes = {{1.6, -2.0, 1.7, 2.0}};
    s.start_vel = Velocity2D{0.5, 0.0};
    s.start_cmd = Velocity2D{0.5, 0.0};
    s.expect_success = false;
    add(s);
  }
  {
    Scenario s;   // hướng đích = π (xoay 180° ở đích, qua ±π)
    s.name = "goal_yaw_pi";
    s.path = polyline({{0, 0, 0}, {2, 0, 0}}, M_PI);
    add(s);
  }
  return v;
}

Scenario findScenario(const std::string & name)
{
  for (auto & s : standardScenarios()) {
    if (s.name == name) {
      return s;
    }
  }
  throw std::runtime_error("unknown scenario " + name);
}

}  // namespace sim
}  // namespace stage3_navigation
