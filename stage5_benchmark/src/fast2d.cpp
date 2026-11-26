// fast2d.cpp — xem fast2d.hpp (mô hình, phần dùng lại nguyên code robot, giới hạn).

#include "stage5_benchmark/fast2d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "stage3_navigation/path_blockage.hpp"

namespace stage5_benchmark
{

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using stage3_navigation::normalizeAngle;
namespace s3 = stage3_navigation;

// =====================================================================================================
// World
// =====================================================================================================
double Shape::distance(double px, double py) const
{
  if (type == Type::CIRCLE) {
    return std::hypot(px - x, py - y) - r;
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double lx = c * (px - x) + s * (py - y);
  const double ly = -s * (px - x) + c * (py - y);
  const double hx = 0.5 * sx, hy = 0.5 * sy;
  const double dx = std::fabs(lx) - hx, dy = std::fabs(ly) - hy;
  if (dx <= 0.0 && dy <= 0.0) {
    return std::max(dx, dy);
  }
  return std::hypot(std::max(dx, 0.0), std::max(dy, 0.0));
}

double Shape::ray(double ox, double oy, double dx, double dy) const
{
  const double inf = std::numeric_limits<double>::infinity();
  if (type == Type::CIRCLE) {
    const double fx = ox - x, fy = oy - y;
    const double b = fx * dx + fy * dy;
    const double cc = fx * fx + fy * fy - r * r;
    const double disc = b * b - cc;
    if (disc < 0.0) {
      return inf;
    }
    const double sq = std::sqrt(disc);
    const double t0 = -b - sq, t1 = -b + sq;
    if (t0 >= 0.0) {return t0;}
    if (t1 >= 0.0) {return 0.0;}   // gốc tia trong hình
    return inf;
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double lox = c * (ox - x) + s * (oy - y), loy = -s * (ox - x) + c * (oy - y);
  const double ldx = c * dx + s * dy, ldy = -s * dx + c * dy;
  const double hx = 0.5 * sx, hy = 0.5 * sy;
  double tmin = -inf, tmax = inf;
  const double o[2] = {lox, loy}, d[2] = {ldx, ldy}, h[2] = {hx, hy};
  for (int k = 0; k < 2; ++k) {
    if (std::fabs(d[k]) < 1e-12) {
      if (o[k] < -h[k] || o[k] > h[k]) {return inf;}
    } else {
      double t1 = (-h[k] - o[k]) / d[k], t2 = (h[k] - o[k]) / d[k];
      if (t1 > t2) {std::swap(t1, t2);}
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) {return inf;}
    }
  }
  if (tmax < 0.0) {return inf;}
  return tmin >= 0.0 ? tmin : 0.0;
}

World::World(Grid map, std::vector<Shape> extra)
: map_(std::move(map)), extra_(std::move(extra))
{
  if (map_.width <= 0 || map_.height <= 0 || map_.resolution <= 0.0 ||
    map_.data.size() != static_cast<std::size_t>(map_.width) * map_.height)
  {
    throw std::invalid_argument("World: invalid map grid");
  }
}

double World::clearance(double x, double y, double search) const
{
  double best = search;
  for (const auto & sh : extra_) {
    best = std::min(best, sh.distance(x, y));
  }
  const double res = map_.resolution;
  const int ci = static_cast<int>(std::floor((x - map_.origin_x) / res));
  const int cj = static_cast<int>(std::floor((y - map_.origin_y) / res));
  const int rad = static_cast<int>(std::ceil(std::min(best, search) / res)) + 1;
  for (int j = cj - rad; j <= cj + rad; ++j) {
    for (int i = ci - rad; i <= ci + rad; ++i) {
      if (!map_.occupied(i, j)) {
        continue;
      }
      const double x0 = map_.origin_x + i * res, y0 = map_.origin_y + j * res;
      const double dx = std::max({x0 - x, 0.0, x - (x0 + res)});
      const double dy = std::max({y0 - y, 0.0, y - (y0 + res)});
      double d = std::hypot(dx, dy);
      if (dx == 0.0 && dy == 0.0) {
        d = -std::min({x - x0, x0 + res - x, y - y0, y0 + res - y});
      }
      best = std::min(best, d);
    }
  }
  return best;
}

double World::raycast(double ox, double oy, double angle, double max_range) const
{
  const double dx = std::cos(angle), dy = std::sin(angle);
  double best = std::numeric_limits<double>::infinity();
  for (const auto & sh : extra_) {
    best = std::min(best, sh.ray(ox, oy, dx, dy));
  }
  // DDA trên lưới M (Amanatides & Woo): biên ô chính xác.
  const double res = map_.resolution;
  double gx = (ox - map_.origin_x) / res, gy = (oy - map_.origin_y) / res;
  int i = static_cast<int>(std::floor(gx)), j = static_cast<int>(std::floor(gy));
  const int step_i = dx > 0 ? 1 : -1, step_j = dy > 0 ? 1 : -1;
  const double inf = std::numeric_limits<double>::infinity();
  const double tdx = std::fabs(dx) < 1e-12 ? inf : res / std::fabs(dx);
  const double tdy = std::fabs(dy) < 1e-12 ? inf : res / std::fabs(dy);
  double tx = std::fabs(dx) < 1e-12 ? inf :
    ((dx > 0 ? (i + 1 - gx) : (gx - i)) * res) / std::fabs(dx);
  double ty = std::fabs(dy) < 1e-12 ? inf :
    ((dy > 0 ? (j + 1 - gy) : (gy - j)) * res) / std::fabs(dy);
  double t = 0.0;
  const double limit = std::min(max_range, best);
  while (t <= limit) {
    if (map_.occupied(i, j)) {
      best = std::min(best, t);
      break;
    }
    if (!map_.inside(i, j) && t > 0.0) {
      // ra ngoài bản đồ: không còn tường M nào (Δ đã xét ở trên)
      const bool leaving = (i < 0 && step_i < 0) || (i >= map_.width && step_i > 0) ||
        (j < 0 && step_j < 0) || (j >= map_.height && step_j > 0);
      if (leaving) {break;}
    }
    if (tx < ty) {
      t = tx; tx += tdx; i += step_i;
    } else {
      t = ty; ty += tdy; j += step_j;
    }
  }
  return best <= max_range ? best : inf;
}

// =====================================================================================================
// Costmap helpers (Nav2 Humble semantics)
// =====================================================================================================
namespace
{

class Layer2D : public Costmap2D
{
public:
  Layer2D(unsigned int nx, unsigned int ny, double res, double ox, double oy, unsigned char def)
  : Costmap2D(nx, ny, res, ox, oy, def) {}
  void clearLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int maxlen)
  {
    MarkCell m(costmap_, FREE_SPACE);
    raytraceLine(m, x0, y0, x1, y1, maxlen, 0);
  }
  unsigned char * raw() {return costmap_;}
};

std::vector<geometry_msgs::msg::Point> paddedFootprint(double radius, double padding)
{
  auto fp = nav2_costmap_2d::makeFootprintFromRadius(radius);
  nav2_costmap_2d::padFootprint(fp, padding);
  return fp;
}

// Nav2 InflationLayer::computeCost (cell distance d).
unsigned char inflationCost(double d_cells, double res, double inscribed, double factor)
{
  if (d_cells == 0.0) {
    return LETHAL_OBSTACLE;
  }
  if (d_cells * res <= inscribed) {
    return INSCRIBED_INFLATED_OBSTACLE;
  }
  const double f = std::exp(-1.0 * factor * (d_cells * res - inscribed));
  return static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * f);
}

// master: đã có lethal (254) từ các layer; thêm inflation như Nav2 (đĩa bán kính cellDistance(r), luật unknown).
void inflate(Costmap2D & master, double inflation_radius, double inscribed, double factor)
{
  const unsigned nx = master.getSizeInCellsX(), ny = master.getSizeInCellsY();
  const double res = master.getResolution();
  const int R = static_cast<int>(master.cellDistance(inflation_radius));
  // bảng chi phí theo (di, dj)
  std::vector<unsigned char> table((2 * R + 1) * (2 * R + 1), 0);
  std::vector<char> within((2 * R + 1) * (2 * R + 1), 0);
  for (int dj = -R; dj <= R; ++dj) {
    for (int di = -R; di <= R; ++di) {
      const double d = std::hypot(di, dj);
      const std::size_t k = (dj + R) * (2 * R + 1) + (di + R);
      within[k] = d <= R ? 1 : 0;
      table[k] = inflationCost(d, res, inscribed, factor);
    }
  }
  unsigned char * a = master.getCharMap();
  std::vector<unsigned char> src(a, a + static_cast<std::size_t>(nx) * ny);
  std::vector<unsigned char> infl(src.size(), 0);
  for (unsigned j = 0; j < ny; ++j) {
    for (unsigned i = 0; i < nx; ++i) {
      if (src[j * nx + i] != LETHAL_OBSTACLE) {
        continue;
      }
      for (int dj = -R; dj <= R; ++dj) {
        const int jj = static_cast<int>(j) + dj;
        if (jj < 0 || jj >= static_cast<int>(ny)) {continue;}
        for (int di = -R; di <= R; ++di) {
          const int ii = static_cast<int>(i) + di;
          if (ii < 0 || ii >= static_cast<int>(nx)) {continue;}
          const std::size_t k = (dj + R) * (2 * R + 1) + (di + R);
          if (!within[k]) {continue;}
          unsigned char & c = infl[static_cast<std::size_t>(jj) * nx + ii];
          c = std::max(c, table[k]);
        }
      }
    }
  }
  for (std::size_t n = 0; n < src.size(); ++n) {
    const unsigned char old_cost = src[n], cost = infl[n];
    if (old_cost == NO_INFORMATION) {
      // inflate_unknown false: unknown chỉ bị ghi đè khi cost >= INSCRIBED
      a[n] = cost >= INSCRIBED_INFLATED_OBSTACLE ? cost : old_cost;
    } else {
      a[n] = std::max(old_cost, cost);
    }
  }
}

// Costmap2DPublisher cost_translation_table_ (Humble).
std::int8_t translateCost(unsigned char c)
{
  if (c == 0) {return 0;}
  if (c == INSCRIBED_INFLATED_OBSTACLE) {return 99;}
  if (c == LETHAL_OBSTACLE) {return 100;}
  if (c == NO_INFORMATION) {return -1;}
  return static_cast<std::int8_t>(1 + (97 * (c - 1)) / 251);
}

double yawOfSegment(const Pose2D & a, const Pose2D & b) {return std::atan2(b.y - a.y, b.x - a.x);}

}  // namespace

double nav2InscribedRadius(double robot_radius, double padding)
{
  double ins = 0.0, circ = 0.0;
  nav2_costmap_2d::calculateMinAndMaxDistances(paddedFootprint(robot_radius, padding), ins, circ);
  return ins;
}

std::unique_ptr<Costmap2D> staticCostmap(
  const Grid & map, double robot_radius, double padding, double inflation_radius, double cost_scaling)
{
  auto cm = std::make_unique<Costmap2D>(
    map.width, map.height, map.resolution, map.origin_x, map.origin_y, NO_INFORMATION);
  unsigned char * a = cm->getCharMap();
  for (std::size_t n = 0; n < map.data.size(); ++n) {
    const int v = map.data[n];
    a[n] = v < 0 ? NO_INFORMATION : (v >= 100 ? LETHAL_OBSTACLE : FREE_SPACE);
  }
  inflate(*cm, inflation_radius, nav2InscribedRadius(robot_radius, padding), cost_scaling);
  return cm;
}

// =====================================================================================================
// NavFn A* (NavfnPlanner::makePlan, Humble, tolerance 0, use_final_approach_orientation false)
// =====================================================================================================
std::vector<Pose2D> navfnPlan(
  const Costmap2D & costmap_in, const Pose2D & start, const Pose2D & goal, bool allow_unknown)
{
  std::vector<Pose2D> plan;
  Costmap2D costmap(costmap_in);
  unsigned int mx, my;
  if (!costmap.worldToMap(start.x, start.y, mx, my)) {
    return plan;
  }
  costmap.setCost(mx, my, FREE_SPACE);   // clearRobotCell
  const int nx = static_cast<int>(costmap.getSizeInCellsX()), ny = static_cast<int>(costmap.getSizeInCellsY());
  nav2_navfn_planner::NavFn nav(nx, ny);
  nav.setNavArr(nx, ny);
  nav.setCostmap(costmap.getCharMap(), true, allow_unknown);
  int map_start[2] = {static_cast<int>(mx), static_cast<int>(my)};
  unsigned int gx, gy;
  if (!costmap.worldToMap(goal.x, goal.y, gx, gy)) {
    return plan;
  }
  int map_goal[2] = {static_cast<int>(gx), static_cast<int>(gy)};
  nav.setStart(map_goal);
  nav.setGoal(map_start);
  nav.calcNavFnAstar();
  const double potential = nav.potarr[gy * nx + gx];
  if (!(potential < POT_HIGH)) {
    return plan;
  }
  nav.setStart(map_goal);
  const int max_cycles = (nx >= ny ? nx : ny) * 4;
  const int len = nav.calcPath(max_cycles);
  if (len == 0) {
    return plan;
  }
  const float * px = nav.getPathX();
  const float * py = nav.getPathY();
  const int n = nav.getPathLen();
  const double res = costmap.getResolution();
  for (int i = n - 1; i >= 0; --i) {
    plan.push_back(Pose2D{costmap.getOriginX() + px[i] * res, costmap.getOriginY() + py[i] * res, 0.0});
  }
  // smoothApproachToGoal
  if (plan.size() >= 2) {
    const Pose2D & s2l = plan[plan.size() - 2];
    const Pose2D & last = plan.back();
    const double d_last = std::hypot(last.x - s2l.x, last.y - s2l.y);
    const double d_goal = std::hypot(goal.x - s2l.x, goal.y - s2l.y);
    if (d_last > d_goal) {
      plan.back() = goal;
      return plan;
    }
  }
  plan.push_back(goal);
  return plan;
}

// =====================================================================================================
// Episode
// =====================================================================================================
struct Episode::Impl
{
  World world;
  std::mt19937 rng;
  std::normal_distribution<double> gauss{0.0, 1.0};
  stage4_gazebo::ZlacDriverModel driver;

  // costmap
  double ins_local{0.25}, ins_global{0.25};
  std::vector<geometry_msgs::msg::Point> fp_local, fp_global;
  std::unique_ptr<Layer2D> local_obs;
  std::unique_ptr<Costmap2D> local_master;
  std::unique_ptr<Costmap2D> static_layer;     // M: 254 / 255 / 0 (không inflate)
  std::unique_ptr<Layer2D> global_obs;
  std::unique_ptr<Costmap2D> global_master;
  std::vector<std::int8_t> global_grid;        // OccupancyGrid đã dịch (path_blockage)
  bool local_ready{false};

  // LiDAR
  struct ScanObs
  {
    double t{-1.0};
    double ox{0.0}, oy{0.0};
    std::vector<std::pair<double, double>> points;       // điểm hữu hạn (marking + clearing)
    std::vector<std::pair<double, double>> clear_only;   // inf -> max range (chỉ khi inf_is_valid)
  };
  ScanObs scan;

  // robot GT
  Pose2D pose;
  double v_true{0.0}, w_true{0.0};
  double v_filt{0.0};

  // controller_server
  bool ctrl_active{false};
  s3::PathTracker tracker;
  s3::DwaCore core;
  bool have_last_cmd{false};
  Velocity2D last_cmd_ctrl;
  double last_compute_t{-1.0};
  double last_valid_t{0.0};
  Pose2D progress_base;
  double progress_t{0.0};
  bool xy_latched{false};
  std::vector<double> compute_ms;

  // twist_mux + driver watchdog
  double last_nav_cmd_t{-1e9};
  Velocity2D nav_cmd;
  double last_driver_cmd_t{-1e9};
  bool watchdog_zeroed{true};

  // goal_path_bridge
  bool session{false};
  double session_start{0.0};
  s3::ReplanSupervisor sup;
  Pose2D goal;
  bool goal_sent{false};
  double terminal_t{-1.0};

  // stuck window
  std::deque<std::pair<double, Pose2D>> hist;

  Impl(const Scenario & sc, const SimConfig & cfg)
  : world(sc.map, sc.new_obstacles), rng(cfg.seed), driver([&] {
        stage4_gazebo::ZlacModelParams zp;
        zp.accel_time_s = cfg.accel_time;
        zp.decel_time_s = cfg.decel_time;
        zp.command_delay_s = cfg.command_delay;
        return zp;
      }())
  {}
};

const Costmap2D & Episode::localCostmap() const {return *impl_->local_master;}
const Costmap2D & Episode::globalCostmap() const {return *impl_->global_master;}
bool Episode::blocked() const {return impl_->sup.blocked();}
double Episode::progress() const {return impl_->tracker.progress();}
double Episode::groundTruthClearance() const
{
  return impl_->world.clearance(pose_.x, pose_.y) - cfg_.robot_radius;
}

Episode::Episode(
  const Scenario & sc, const SimConfig & cfg, const s3::DwbParams & dwb, const s3::ReplanParams & replan)
: cfg_(cfg), dwb_(dwb), replan_(replan)
{
  const std::string why = s3::validateParams(dwb_);
  if (!why.empty()) {
    throw std::invalid_argument("invalid DwbParams: " + why);
  }
  impl_ = std::make_unique<Impl>(sc, cfg);
  auto & I = *impl_;
  I.pose = sc.start;
  if (cfg.seed != 0) {
    I.pose.x += cfg.start_noise_xy * I.gauss(I.rng);
    I.pose.y += cfg.start_noise_xy * I.gauss(I.rng);
    I.pose.theta = normalizeAngle(I.pose.theta + cfg.start_noise_yaw * I.gauss(I.rng));
  }
  I.goal = sc.goal;
  pose_ = I.pose;
  I.progress_base = I.pose;

  I.fp_local = paddedFootprint(cfg.footprint_radius, cfg.local_padding);
  I.fp_global = paddedFootprint(cfg.footprint_radius, cfg.global_padding);
  I.ins_local = nav2InscribedRadius(cfg.footprint_radius, cfg.local_padding);
  I.ins_global = nav2InscribedRadius(cfg.footprint_radius, cfg.global_padding);

  const unsigned n_local = static_cast<unsigned>(std::lround(cfg.local_size / cfg.local_resolution));
  const unsigned char local_def = cfg.local_track_unknown ? NO_INFORMATION : FREE_SPACE;
  const double ox = pose_.x - 0.5 * cfg.local_size, oy = pose_.y - 0.5 * cfg.local_size;
  I.local_obs = std::make_unique<Layer2D>(n_local, n_local, cfg.local_resolution, ox, oy, local_def);
  I.local_master = std::make_unique<Costmap2D>(n_local, n_local, cfg.local_resolution, ox, oy, local_def);

  const auto & M = sc.map;
  if (std::fabs(M.resolution - cfg.resolution) > 1e-9) {
    throw std::invalid_argument("map resolution must equal costmap resolution (0.05)");
  }
  I.static_layer = std::make_unique<Costmap2D>(
    M.width, M.height, M.resolution, M.origin_x, M.origin_y, NO_INFORMATION);
  unsigned char * st = I.static_layer->getCharMap();
  for (std::size_t n = 0; n < M.data.size(); ++n) {
    const int v = M.data[n];
    st[n] = v < 0 ? NO_INFORMATION : (v >= 100 ? LETHAL_OBSTACLE : FREE_SPACE);
  }
  I.global_obs = std::make_unique<Layer2D>(
    M.width, M.height, M.resolution, M.origin_x, M.origin_y, NO_INFORMATION);
  I.global_master = std::make_unique<Costmap2D>(*I.static_layer);
  I.global_grid.assign(M.data.size(), -1);
  scan_ranges_.assign(cfg.lidar_beams, std::numeric_limits<float>::quiet_NaN());
  res_.events.reserve(64);
}

Episode::~Episode() = default;

std::string Episode::setDwbParams(const s3::DwbParams & p)
{
  const std::string why = s3::validateParams(p);
  if (why.empty()) {
    dwb_ = p;
  }
  return why;
}

void Episode::publishState(const std::string & s)
{
  bridge_state_ = s;
  res_.events.push_back(Event{t_, s});
}

// Một bước vật lý; các tác vụ định kỳ chạy theo bộ đếm tick (không trôi số thực).
bool Episode::step(double duration)
{
  const double t_stop = t_ + duration;
  while (!done_ && t_ < t_stop - 1e-12) {
    try {
      tick();
    } catch (const std::exception & e) {
      res_.error = e.what();
      done_ = true;
    }
  }
  return done_;
}

void Episode::tick()
{
  auto & I = *impl_;
  const double h = cfg_.physics_dt;
  const long long k = std::llround(t_ / h);
  const auto every = [&](double rate, int phase) {
      const long long period = std::max<long long>(1, std::llround(1.0 / (rate * h)));
      return (k % period) == (phase % period);
    };
  const bool stale = cfg_.stale_scan_start >= 0.0 && t_ >= cfg_.stale_scan_start &&
    t_ < cfg_.stale_scan_start + cfg_.stale_scan_duration;

  // ------------------------------------------------------------------ LiDAR 10 Hz
  if (every(10.0, 0) && !stale) {
    const double lx = I.pose.x + cfg_.lidar_x * std::cos(I.pose.theta);
    const double ly = I.pose.y + cfg_.lidar_x * std::sin(I.pose.theta);
    I.scan.t = t_;
    I.scan.ox = lx;
    I.scan.oy = ly;
    I.scan.points.clear();
    I.scan.clear_only.clear();
    const int nb = cfg_.lidar_beams;
    for (int b = 0; b < nb; ++b) {
      // góc 0..2π (sau scan_rotate), 0 = trước; laser_filters bỏ nửa sau
      double a = 2.0 * M_PI * b / nb;
      if (a > M_PI) {a -= 2.0 * M_PI;}
      if (std::fabs(a) >= cfg_.lidar_half_fov) {
        scan_ranges_[b] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      double r = I.world.raycast(lx, ly, I.pose.theta + a, cfg_.lidar_range_max);
      if (std::isfinite(r)) {
        r += cfg_.lidar_noise * I.gauss(I.rng);
      }
      if (!std::isfinite(r) || r < cfg_.lidar_range_min || r > cfg_.lidar_range_max) {
        scan_ranges_[b] = std::numeric_limits<float>::infinity();
        if (cfg_.inf_is_valid) {
          const double rr = cfg_.lidar_range_max - 1e-3;
          I.scan.clear_only.emplace_back(
            lx + rr * std::cos(I.pose.theta + a), ly + rr * std::sin(I.pose.theta + a));
        }
        continue;
      }
      scan_ranges_[b] = static_cast<float>(r);
      I.scan.points.emplace_back(lx + r * std::cos(I.pose.theta + a), ly + r * std::sin(I.pose.theta + a));
    }
  }

  // ObstacleLayer: raytrace clear rồi mark (scan mới nhất; observation_persistence 0)
  const auto updateObstacleLayer = [&](Layer2D & L, const std::vector<geometry_msgs::msg::Point> & fp,
      bool footprint_clearing) {
      if (I.scan.t < 0.0) {return;}
      unsigned int x0, y0;
      if (L.worldToMap(I.scan.ox, I.scan.oy, x0, y0)) {
        const double map_ox = L.getOriginX(), map_oy = L.getOriginY();
        const double map_ex = map_ox + L.getSizeInCellsX() * L.getResolution();
        const double map_ey = map_oy + L.getSizeInCellsY() * L.getResolution();
        const unsigned maxc = L.cellDistance(cfg_.raytrace_max_range);
        const auto trace = [&](double wx, double wy) {
            const double ox = I.scan.ox, oy = I.scan.oy;
            const double a = wx - ox, b = wy - oy;
            if (wx < map_ox) {const double t = (map_ox - ox) / a; wx = map_ox; wy = oy + b * t;}
            if (wy < map_oy) {const double t = (map_oy - oy) / b; wx = ox + a * t; wy = map_oy;}
            if (wx > map_ex) {const double t = (map_ex - ox) / a; wx = map_ex - .001; wy = oy + b * t;}
            if (wy > map_ey) {const double t = (map_ey - oy) / b; wx = ox + a * t; wy = map_ey - .001;}
            unsigned int x1, y1;
            if (!L.worldToMap(wx, wy, x1, y1)) {return;}
            L.clearLine(x0, y0, x1, y1, maxc);
          };
        for (const auto & p : I.scan.points) {trace(p.first, p.second);}
        for (const auto & p : I.scan.clear_only) {trace(p.first, p.second);}
      }
      const double r2 = cfg_.obstacle_max_range * cfg_.obstacle_max_range;
      for (const auto & p : I.scan.points) {
        const double dx = p.first - I.scan.ox, dy = p.second - I.scan.oy;
        if (dx * dx + dy * dy >= r2) {continue;}
        unsigned int mx, my;
        if (L.worldToMap(p.first, p.second, mx, my)) {
          L.setCost(mx, my, LETHAL_OBSTACLE);
        }
      }
      // footprint clearing (đa giác đã padding, như ObstacleLayer::updateCosts)
      if (!footprint_clearing) {return;}
      std::vector<geometry_msgs::msg::Point> oriented;
      nav2_costmap_2d::transformFootprint(I.pose.x, I.pose.y, I.pose.theta, fp, oriented);
      L.setConvexPolygonCost(oriented, FREE_SPACE);
    };

  // ------------------------------------------------------------------ local costmap 5 Hz
  if (every(cfg_.local_update_rate, 3)) {
    const double ox = I.pose.x - 0.5 * cfg_.local_size, oy = I.pose.y - 0.5 * cfg_.local_size;
    I.local_obs->updateOrigin(ox, oy);
    I.local_master->updateOrigin(ox, oy);
    updateObstacleLayer(*I.local_obs, I.fp_local, cfg_.local_footprint_clearing);
    // master = obstacle (updateWithMax trên master đã reset về default)
    const unsigned n = I.local_obs->getSizeInCellsX() * I.local_obs->getSizeInCellsY();
    const unsigned char def = cfg_.local_track_unknown ? NO_INFORMATION : FREE_SPACE;
    unsigned char * m = I.local_master->getCharMap();
    const unsigned char * l = I.local_obs->raw();
    for (unsigned q = 0; q < n; ++q) {
      m[q] = def;
      if (l[q] == NO_INFORMATION) {continue;}
      if (m[q] == NO_INFORMATION || m[q] < l[q]) {m[q] = l[q];}
    }
    inflate(*I.local_master, cfg_.local_inflation, I.ins_local, cfg_.cost_scaling);
    I.local_ready = true;
  }

  // ------------------------------------------------------------------ global costmap 2 Hz
  if (every(cfg_.global_update_rate, 7)) {
    if (cfg_.global_obstacle_layer) {
      updateObstacleLayer(*I.global_obs, I.fp_global, cfg_.global_footprint_clearing);
    }
    const std::size_t n = I.global_grid.size();
    unsigned char * m = I.global_master->getCharMap();
    const unsigned char * s = I.static_layer->getCharMap();
    const unsigned char * l = I.global_obs->raw();
    for (std::size_t q = 0; q < n; ++q) {
      m[q] = s[q];
      if (!cfg_.global_obstacle_layer || l[q] == NO_INFORMATION) {continue;}
      if (m[q] == NO_INFORMATION || m[q] < l[q]) {m[q] = l[q];}
    }
    inflate(*I.global_master, cfg_.global_inflation, I.ins_global, cfg_.cost_scaling);
    for (std::size_t q = 0; q < n; ++q) {I.global_grid[q] = translateCost(m[q]);}
  }

  // controller_server publish cmd_vel -> twist_mux (chuyển ngay) -> zlac_sim (động học, lround RPM, setRpm).
  const auto publishNav = [&](const Velocity2D & cmd) {
      I.nav_cmd = cmd;
      I.last_nav_cmd_t = t_;
      const double vl = cmd.v - cmd.w * cfg_.wheel_distance / 2.0;
      const double vr = cmd.v + cmd.w * cfg_.wheel_distance / 2.0;
      const int rl = static_cast<int>(std::lround(vl / cfg_.wheel_radius * 30.0 / M_PI));
      const int rr = static_cast<int>(std::lround(vr / cfg_.wheel_radius * 30.0 / M_PI));
      I.driver.setRpm(t_, rl, rr);
      I.last_driver_cmd_t = t_;
      I.watchdog_zeroed = false;
    };
  const auto planNow = [&]() {
      return navfnPlan(*I.global_master, I.pose, I.goal, cfg_.allow_unknown);
    };
  const auto planLength = [](const std::vector<Pose2D> & p) {
      double L = 0.0;
      for (std::size_t i = 1; i < p.size(); ++i) {L += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);}
      return L;
    };
  const auto nearestOnPath = [&](const std::vector<Pose2D> & p) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto & q : p) {best = std::min(best, std::hypot(q.x - I.pose.x, q.y - I.pose.y));}
      return best;
    };
  // controller_server: nhận path mới (FollowPath goal / preempt)
  const auto sendFollow = [&](std::vector<Pose2D> path) {
      path.back().theta = I.goal.theta;
      plan_ = path;
      std::vector<s3::PathPoint> pts;
      for (const auto & q : path) {pts.push_back(s3::PathPoint{q.x, q.y});}
      I.sup.setPath(pts, t_);
      I.core.reset();
      const std::string why = I.tracker.setPlan(path);
      if (!why.empty()) {
        throw std::runtime_error("setPlan: " + why);
      }
      I.xy_latched = false;           // goal checker reset (setPlannerPath)
      if (!I.ctrl_active) {           // FollowPath mới (không phải preempt)
        I.ctrl_active = true;
        I.progress_base = I.pose;
        I.progress_t = t_;
        I.last_valid_t = t_;
      }
      following_ = true;
      publishState(I.sup.totalReplans() > 0 ?
        "FOLLOWING (replanned " + std::to_string(I.sup.totalReplans()) + "x)" : "FOLLOWING");
    };
  const auto stopFollow = [&]() {
      if (!I.ctrl_active) {return false;}
      I.ctrl_active = false;
      following_ = false;
      publishNav(Velocity2D{});      // controller_server: hủy / xong / abort -> publishZeroVelocity
      return true;
    };
  const auto endSession = [&](const std::string & state) {
      stopFollow();
      I.session = false;
      publishState(state);
      if (res_.terminal_state == "none") {
        res_.terminal_state = state;
        res_.t_terminal = t_;
        I.terminal_t = t_;
      }
    };
  const auto requestReplan = [&](const std::string & reason) {
      I.sup.markAttempt(t_);
      publishState("REPLANNING: " + reason);
      auto path = planNow();
      if (path.size() >= 2) {
        res_.plan_lengths.push_back(planLength(path));
        if (nearestOnPath(path) <= cfg_.max_start_distance) {
          sendFollow(path);
          return;
        }
      }
      // onReplanFailed
      stopFollow();
      I.sup.onReplanFailed(t_);
      plan_.clear();
      publishState(std::string("WAITING: ") + (path.size() >= 2 ? "new path not followable" : "no path"));
    };
  const auto apply = [&](const s3::ReplanDecision & d) {
      if (d.type == s3::ReplanDecision::Type::REPLAN) {
        requestReplan(d.text);
      } else if (d.type == s3::ReplanDecision::Type::END) {
        endSession(d.text);
      }
    };

  // ------------------------------------------------------------------ goal (sau warm-up)
  if (!I.goal_sent && t_ >= cfg_.warmup) {
    I.goal_sent = true;
    publishState("GOAL");
    auto path = planNow();
    if (path.size() < 2) {
      res_.initial_path_length = 0.0;
      endSession("PLANNING_FAILED: no path on the static map");
    } else {
      res_.initial_path_length = planLength(path);
      res_.plan_lengths.push_back(res_.initial_path_length);
      if (nearestOnPath(path) > cfg_.max_start_distance) {
        endSession("REFUSED: robot too far from the path");
      } else {
        I.session = true;
        I.session_start = t_;
        I.sup.begin(t_);
        publishState("FOLLOW_REQUESTED");
        sendFollow(path);
      }
    }
  }

  // ------------------------------------------------------------------ controller_server 10 Hz
  TraceRow cycle_row{};
  bool have_cycle_row = false;
  if (every(cfg_.controller_rate, 11) && I.ctrl_active) {
    const double dt = 1.0 / cfg_.controller_rate;
    const bool current = I.local_ready && I.scan.t >= 0.0 && (t_ - I.scan.t) <= cfg_.expected_update_rate;
    if (!current) {
      ++res_.stale_cycles;    // controller_server chờ costmap current: không tính, không phát lệnh
    } else {
      // progress checker (trước khi tính)
      if (std::hypot(I.pose.x - I.progress_base.x, I.pose.y - I.progress_base.y) > cfg_.progress_radius) {
        I.progress_base = I.pose;
        I.progress_t = t_;
      }
      std::string abort_reason;
      if (t_ - I.progress_t > cfg_.progress_time) {
        abort_reason = "Failed to make progress";
      }
      s3::ControlOutput out;
      double ms = 0.0;
      if (abort_reason.empty()) {
        if (I.last_compute_t >= 0.0 && (t_ - I.last_compute_t > 2.5 * dt)) {
          I.have_last_cmd = false;
        }
        I.last_compute_t = t_;
        const auto c0 = std::chrono::steady_clock::now();
        const double hyst = std::max(2.0 * cfg_.local_resolution, 0.1);
        const auto upd = I.tracker.update(I.pose, dwb_.closest_search_distance, dwb_.local_plan_length, hyst);
        std::vector<Pose2D> local;
        for (const auto & q : upd.local) {
          unsigned int mx, my;
          if (!I.local_master->worldToMap(q.x, q.y, mx, my)) {break;}
          local.push_back(q);
        }
        if (I.tracker.size() > 1 && upd.deviation > dwb_.max_path_deviation) {
          out.error = "robot is too far from the plan";
        } else if (local.empty()) {
          out.error = "no part of the plan lies inside the local costmap";
        } else {
          s3::ControlInput in;
          in.robot = I.pose;
          in.velocity = meas_;
          in.prev_cmd = I.have_last_cmd ? I.last_cmd_ctrl : meas_;
          in.local_path = local;
          in.goal = I.tracker.goal();
          in.tol = s3::GoalTolerance{cfg_.xy_goal_tolerance, cfg_.yaw_goal_tolerance};
          in.dt = dt;
          in.max_vel_x_eff = dwb_.max_vel_x;
          in.now_s = t_;
          const s3::CollisionModel cm(
            I.local_master.get(), s3::CollisionMode::CENTER, {}, dwb_.unknown_is_obstacle);
          out = I.core.compute(dwb_, in, cm, false);
        }
        ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
        I.compute_ms.push_back(ms);
        ++res_.controller_cycles;
        if (ms > 1000.0 * dt) {++res_.missed_deadlines;}
        res_.candidates += out.n_candidates;
        for (int r = 0; r < 5; ++r) {res_.rejects[r] += out.n_reject[r];}
        if (out.ok && out.n_valid == 0) {++res_.cycles_no_valid;}
        if (out.ok && out.phase == s3::Phase::FOLLOW_PATH && out.n_forward_valid == 0) {++res_.cycles_holding;}
        last_out_ = out;
        Velocity2D cmd;
        if (out.ok) {
          cmd = out.cmd;
          I.last_valid_t = t_;
          I.last_cmd_ctrl = out.cmd;
          I.have_last_cmd = true;
        } else {
          ++res_.cycles_no_valid;
          I.last_cmd_ctrl = Velocity2D{};
          I.have_last_cmd = true;
          if (t_ - I.last_valid_t > cfg_.failure_tolerance) {
            abort_reason = "Controller patience exceeded (" + out.error + ")";
          }
        }
        last_cmd_ = cmd;
        publishNav(cmd);
        cycle_row = TraceRow{t_, 0, 0, 0, 0, 0, cmd.v, cmd.w, 0, out.ok ? static_cast<int>(out.phase) : -2,
          out.n_valid, out.n_forward_valid, out.n_reject[3], out.n_reject[1], out.n_reject[2], ms,
          out.carrot_shift};
        have_cycle_row = true;
      }
      if (!abort_reason.empty()) {
        // controller_server abort -> lệnh 0, FollowPath ABORTED -> bridge
        res_.events.push_back(Event{t_, "controller: " + abort_reason});
        stopFollow();
        if (I.session) {
          apply(I.sup.onFollowAborted(t_, replan_));
        }
      } else {
        // goal checker (stateful) sau khi phát lệnh
        const Pose2D g = I.tracker.goal();
        const double dxy = std::hypot(I.pose.x - g.x, I.pose.y - g.y);
        if (!I.xy_latched && dxy <= cfg_.xy_goal_tolerance) {I.xy_latched = true;}
        if (I.xy_latched && std::fabs(normalizeAngle(g.theta - I.pose.theta)) <= cfg_.yaw_goal_tolerance) {
          stopFollow();
          if (I.session) {endSession("SUCCEEDED");}
        }
      }
    }
  }

  // trace 10 Hz từ lúc gửi goal (cả khi controller không chạy: đo đứng yên sau Safe Stop)
  if (cfg_.keep_trace && I.goal_sent && every(10.0, 11)) {
    TraceRow row = have_cycle_row ? cycle_row : TraceRow{t_, 0, 0, 0, 0, 0, I.nav_cmd.v, I.nav_cmd.w, 0, -1, 0, 0,
      0, 0, 0, 0.0, 0.0};
    row.t = t_;
    row.x = I.pose.x;
    row.y = I.pose.y;
    row.yaw = I.pose.theta;
    row.v = I.v_true;
    row.w = I.w_true;
    row.clearance = groundTruthClearance();
    res_.trace.push_back(row);
  }

  // ------------------------------------------------------------------ bridge watchFollow 5 Hz
  if (every(cfg_.watch_rate, 17) && I.session) {
    if (cfg_.follow_timeout > 0.0 && t_ - I.session_start > cfg_.follow_timeout) {
      endSession("CANCELED: follow_timeout exceeded");
    } else if (!replan_.enabled) {
      if (!following_) {endSession("FAILED: replan disabled while waiting for a path");}
    } else if (following_) {
      s3::GridView grid;
      grid.origin_x = I.global_master->getOriginX();
      grid.origin_y = I.global_master->getOriginY();
      grid.resolution = I.global_master->getResolution();
      grid.width = I.global_master->getSizeInCellsX();
      grid.height = I.global_master->getSizeInCellsY();
      grid.data = I.global_grid.data();
      s3::BlockageEvent ev = s3::BlockageEvent::NONE;
      std::string where;
      const auto d = I.sup.checkBlockage(t_, I.pose.x, I.pose.y, grid, replan_, &ev, &where);
      if (ev == s3::BlockageEvent::CONFIRMING) {
        res_.events.push_back(Event{t_, "replan: " + where + ", confirming"});
      } else if (ev == s3::BlockageEvent::CLEARED) {
        res_.events.push_back(Event{t_, "replan: path is clear again"});
      }
      apply(d);
    } else if (I.sup.waiting()) {
      apply(I.sup.retryWhileWaiting(t_, replan_));
    }
  }

  // ------------------------------------------------------------------ watchdog zlac_sim (100 ms, 0.5 s)
  if (every(10.0, 5) && !I.watchdog_zeroed && t_ - I.last_driver_cmd_t > cfg_.command_timeout) {
    I.driver.setRpm(t_, 0, 0);
    I.watchdog_zeroed = true;
  }

  // ------------------------------------------------------------------ vật lý
  const auto rpm = I.driver.update(t_);
  const double wl = rpm.first * M_PI / 30.0, wr = rpm.second * M_PI / 30.0;
  const double v1 = cfg_.wheel_radius * (wl + wr) / 2.0;
  const double w1 = cfg_.wheel_radius * (wr - wl) / cfg_.wheel_distance;
  const Pose2D before = I.pose;
  if (std::fabs(w1) < 1e-9) {
    I.pose.x += v1 * h * std::cos(I.pose.theta);
    I.pose.y += v1 * h * std::sin(I.pose.theta);
  } else {
    I.pose.x += v1 / w1 * (std::sin(I.pose.theta + w1 * h) - std::sin(I.pose.theta));
    I.pose.y -= v1 / w1 * (std::cos(I.pose.theta + w1 * h) - std::cos(I.pose.theta));
    I.pose.theta = normalizeAngle(I.pose.theta + w1 * h);
  }
  I.v_true = v1;
  I.w_true = w1;
  res_.path_length += std::hypot(I.pose.x - before.x, I.pose.y - before.y);
  t_ += h;
  pose_ = I.pose;

  // odometry: v lọc thông thấp 20 Hz (zlac8015d_odom), ω từ IMU (đúng)
  if (every(20.0, 1)) {
    I.v_filt = cfg_.odom_alpha * v1 + (1.0 - cfg_.odom_alpha) * I.v_filt;
    double v = std::fabs(I.v_filt) < 0.004 ? 0.0 : I.v_filt;
    double w = w1;
    if (std::fabs(v) < 0.001) {v = 0.0;}      // min_x_velocity_threshold
    if (std::fabs(w) < 0.001) {w = 0.0;}
    meas_ = Velocity2D{v, w};
  }

  // va chạm GT 50 Hz
  if (every(50.0, 2)) {
    const double c = I.world.clearance(I.pose.x, I.pose.y) - cfg_.robot_radius;
    res_.min_clearance = std::min(res_.min_clearance, c);
    if (c < 0.0 && !res_.collision) {
      res_.collision = true;
      res_.t_collision = t_;
      res_.events.push_back(Event{t_, "COLLISION (GT)"});
    }
  }

  // ------------------------------------------------------------------ kết thúc
  if (every(1.0, 0)) {
    I.hist.emplace_back(t_, I.pose);
    // giữ thêm 1 s để evaluator (trace 10 Hz) thấy trọn cửa sổ stuck_window
    while (!I.hist.empty() && t_ - I.hist.front().first > cfg_.stuck_window + 1.0 + 1e-6) {I.hist.pop_front();}
  }
  bool end = false;
  if (res_.collision && t_ - res_.t_collision > 0.5) {end = true;}
  if (I.terminal_t >= 0.0 && t_ - I.terminal_t >= cfg_.post_terminal) {end = true;}
  if (t_ >= cfg_.timeout + cfg_.warmup) {
    end = true;
    res_.timeout = true;
  }
  if (I.goal_sent && I.terminal_t < 0.0 && !I.hist.empty() &&
    t_ - I.hist.front().first >= cfg_.stuck_window - 1e-6 && t_ - cfg_.warmup > cfg_.stuck_window + 1.0)
  {
    double maxd = 0.0;
    for (const auto & hp : I.hist) {
      maxd = std::max(maxd, std::hypot(hp.second.x - I.pose.x, hp.second.y - I.pose.y));
    }
    if (maxd < cfg_.stuck_radius) {
      end = true;
      res_.stuck_early_stop = true;
    }
  }
  if (end) {
    done_ = true;
    res_.t_end = t_;
    res_.final_x = I.pose.x;
    res_.final_y = I.pose.y;
    res_.final_yaw = I.pose.theta;
    res_.replans = I.sup.totalReplans();
    res_.obstacle_replans = I.sup.obstacleReplans();
    res_.abort_replans = I.sup.abortReplans();
    if (!I.compute_ms.empty()) {
      double sum = 0.0;
      for (double m : I.compute_ms) {sum += m;}
      res_.compute_ms_mean = sum / I.compute_ms.size();
      auto v = I.compute_ms;
      std::sort(v.begin(), v.end());
      res_.compute_ms_p99 = v[static_cast<std::size_t>(0.99 * (v.size() - 1))];
      res_.compute_ms_max = v.back();
    }
  }
  (void)yawOfSegment;
}

// =====================================================================================================
// Tham số theo tên
// =====================================================================================================
#define DWB_FIELDS(X) \
  X(max_vel_x) X(min_vel_x) X(max_vel_theta) X(min_in_place_vel_theta) X(acc_lim_x) X(decel_lim_x) \
  X(acc_lim_theta) X(decel_lim_theta) X(sim_time) X(sim_granularity) X(control_latency) X(stopping_margin) \
  X(max_feedback_gap_x) X(max_feedback_gap_theta) X(local_plan_length) X(carrot_distance) \
  X(closest_search_distance) X(max_path_deviation) X(rotate_to_heading_angle) X(rotate_gain) \
  X(xy_goal_tolerance) X(yaw_goal_tolerance) X(goal_tolerance_fraction) X(goal_latch_release_margin) \
  X(path_distance_scale) X(carrot_distance_scale) X(heading_scale) X(obstacle_scale) X(speed_scale) \
  X(oscillation_scale) X(oscillation_reset_dist) X(oscillation_reset_angle) X(oscillation_reset_time) \
  X(transform_tolerance) X(max_tf_age) X(max_pose_age) X(odom_timeout)

std::string setDwbField(s3::DwbParams & p, const std::string & name, double value, const std::string & text)
{
#define X(f) if (name == #f) {p.f = value; return "";}
  DWB_FIELDS(X)
#undef X
  if (name == "vx_samples") {p.vx_samples = static_cast<int>(value); return "";}
  if (name == "vtheta_samples") {p.vtheta_samples = static_cast<int>(value); return "";}
  if (name == "carrot_validation") {p.carrot_validation = value != 0.0; return "";}
  if (name == "unknown_is_obstacle") {p.unknown_is_obstacle = value != 0.0; return "";}
  if (name == "velocity_reference") {p.velocity_reference = text; return "";}
  if (name == "collision_check") {p.collision_check = text; return "";}
  if (name == "publish_trajectories" || name == "trajectory_marker_stride" || name == "debug_publish_rate" ||
    name == "plugin")
  {
    return "";   // chỉ hiển thị
  }
  return name;
}

std::vector<std::string> dwbFieldNames()
{
  std::vector<std::string> v;
#define X(f) v.push_back(#f);
  DWB_FIELDS(X)
#undef X
  v.insert(v.end(), {"vx_samples", "vtheta_samples", "carrot_validation", "unknown_is_obstacle",
      "velocity_reference", "collision_check"});
  return v;
}

std::string setReplanField(s3::ReplanParams & p, const std::string & name, double value, const std::string & text)
{
  if (name == "enabled") {p.enabled = value != 0.0; return "";}
  if (name == "trigger") {p.trigger = text; return "";}
  if (name == "lookahead_distance") {p.lookahead_distance = value; return "";}
  if (name == "start_offset") {p.start_offset = value; return "";}
  if (name == "blocked_cost") {p.blocked_cost = static_cast<int>(value); return "";}
  if (name == "confirm_time") {p.confirm_time = value; return "";}
  if (name == "clear_time") {p.clear_time = value; return "";}
  if (name == "stall_time") {p.stall_time = value; return "";}
  if (name == "stall_progress") {p.stall_progress = value; return "";}
  if (name == "min_interval") {p.min_interval = value; return "";}
  if (name == "max_obstacle_replans") {p.max_obstacle_replans = static_cast<int>(value); return "";}
  if (name == "max_abort_replans") {p.max_abort_replans = static_cast<int>(value); return "";}
  if (name == "wait_timeout") {p.wait_timeout = value; return "";}
  if (name == "costmap_timeout" || name == "clear_costmap_on_failure" || name == "clear_keep_distance" ||
    name == "clear_costmap_service")
  {
    return "";   // không mô phỏng (mặc định tắt / dịch vụ ROS)
  }
  return name;
}

#define SIM_FIELDS(X) \
  X(wheel_radius) X(wheel_distance) X(accel_time) X(decel_time) X(command_delay) X(command_timeout) \
  X(twist_mux_timeout) X(robot_radius) X(lidar_x) X(lidar_half_fov) X(lidar_noise) X(lidar_range_min) \
  X(lidar_range_max) X(local_size) X(resolution) X(local_resolution) X(footprint_radius) X(local_padding) X(global_padding) \
  X(local_inflation) X(global_inflation) X(cost_scaling) X(obstacle_max_range) X(raytrace_max_range) \
  X(local_update_rate) X(global_update_rate) X(expected_update_rate) X(controller_rate) X(failure_tolerance) \
  X(progress_radius) X(progress_time) X(xy_goal_tolerance) X(yaw_goal_tolerance) X(follow_timeout) \
  X(max_start_distance) X(watch_rate) X(physics_dt) X(warmup) X(timeout) X(post_terminal) X(stuck_window) \
  X(stuck_radius) X(odom_alpha) X(stale_scan_start) X(stale_scan_duration) X(start_noise_xy) X(start_noise_yaw)

std::string setSimField(SimConfig & c, const std::string & name, double value)
{
#define X(f) if (name == #f) {c.f = value; return "";}
  SIM_FIELDS(X)
#undef X
  if (name == "lidar_beams") {c.lidar_beams = static_cast<int>(value); return "";}
  if (name == "local_track_unknown") {c.local_track_unknown = value != 0.0; return "";}
  if (name == "inf_is_valid") {c.inf_is_valid = value != 0.0; return "";}
  if (name == "global_obstacle_layer") {c.global_obstacle_layer = value != 0.0; return "";}
  if (name == "local_footprint_clearing") {c.local_footprint_clearing = value != 0.0; return "";}
  if (name == "global_footprint_clearing") {c.global_footprint_clearing = value != 0.0; return "";}
  if (name == "allow_unknown") {c.allow_unknown = value != 0.0; return "";}
  if (name == "keep_trace") {c.keep_trace = value != 0.0; return "";}
  if (name == "seed") {c.seed = static_cast<std::uint32_t>(value); return "";}
  return name;
}

}  // namespace stage5_benchmark
