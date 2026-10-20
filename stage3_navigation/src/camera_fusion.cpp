// camera_fusion.cpp — Lõi hợp nhất camera + LiDAR (M2). Giải thích luật: include/stage3_navigation/camera_fusion.hpp

#include "stage3_navigation/camera_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace stage3_navigation
{

namespace
{
double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

std::string validateFusionParams(const FusionParams & p)
{
  std::ostringstream e;
  auto pos = [&](double v, const char * name) {
      if (!(std::isfinite(v) && v > 0.0)) {e << name << " phải > 0; ";}
    };
  pos(p.resolution, "resolution");
  pos(p.max_range, "max_range");
  pos(p.horizontal_fov, "horizontal_fov");
  pos(p.vertical_fov, "vertical_fov");
  pos(p.azimuth_bin, "azimuth_bin");
  pos(p.elevation_bin, "elevation_bin");
  pos(p.decay_time, "decay_time");
  pos(p.lidar_max_age, "lidar_max_age");
  pos(p.max_cell_distance, "max_cell_distance");
  if (!(std::isfinite(p.min_obstacle_height) && std::isfinite(p.max_obstacle_height) &&
    p.min_obstacle_height < p.max_obstacle_height))
  {
    e << "cần min_obstacle_height < max_obstacle_height; ";
  }
  if (!(std::isfinite(p.min_range) && p.min_range >= 0.0 && p.min_range < p.max_range)) {
    e << "cần 0 <= min_range < max_range; ";
  }
  if (p.horizontal_fov >= 2.0 * M_PI || p.vertical_fov >= M_PI) {
    e << "horizontal_fov < 2π, vertical_fov < π; ";
  }
  if (p.min_points_per_cell < 1) {e << "min_points_per_cell >= 1; ";}
  if (p.confirm_frames < 1) {e << "confirm_frames >= 1; ";}
  if (p.clear_frames < 1) {e << "clear_frames >= 1; ";}
  if (!(std::isfinite(p.lidar_association_radius) && p.lidar_association_radius >= 0.0)) {
    e << "lidar_association_radius >= 0; ";
  }
  if (!(std::isfinite(p.self_filter_radius) && p.self_filter_radius >= 0.0)) {
    e << "self_filter_radius >= 0; ";
  }
  if (std::isfinite(p.azimuth_bin) && std::isfinite(p.elevation_bin) && p.azimuth_bin > 0.0 &&
    p.elevation_bin > 0.0 &&
    (p.horizontal_fov / p.azimuth_bin) * (p.vertical_fov / p.elevation_bin) > 1e6)
  {
    e << "quá nhiều bin (azimuth_bin/elevation_bin quá nhỏ); ";
  }
  return e.str();
}

CameraFusionGrid::CameraFusionGrid(const FusionParams & p)
: p_(p) {}

void CameraFusionGrid::setParams(const FusionParams & p)
{
  const bool res_changed = p.resolution != p_.resolution;
  p_ = p;
  if (res_changed) {
    reset();
  }
}

void CameraFusionGrid::reset()
{
  cells_.clear();
  lidar_cells_.clear();
  lidar_stamp_ = -1e18;
}

CellKey CameraFusionGrid::keyOf(double x, double y) const
{
  return CellKey{static_cast<int32_t>(std::floor(x / p_.resolution)),
    static_cast<int32_t>(std::floor(y / p_.resolution))};
}

std::pair<double, double> CameraFusionGrid::centerOf(const CellKey & k) const
{
  return {(k.i + 0.5) * p_.resolution, (k.j + 0.5) * p_.resolution};
}

void CameraFusionGrid::setLidarHits(
  double stamp, const std::vector<std::pair<double, double>> & hits)
{
  lidar_cells_.clear();
  lidar_stamp_ = stamp;
  const double r = p_.lidar_association_radius;
  const int n = static_cast<int>(std::ceil(r / p_.resolution));
  for (const auto & h : hits) {
    if (!std::isfinite(h.first) || !std::isfinite(h.second)) {
      continue;
    }
    const CellKey c = keyOf(h.first, h.second);
    for (int di = -n; di <= n; ++di) {
      for (int dj = -n; dj <= n; ++dj) {
        const CellKey k{c.i + di, c.j + dj};
        const auto ctr = centerOf(k);
        if (std::hypot(ctr.first - h.first, ctr.second - h.second) <= r + 0.5 * p_.resolution) {
          lidar_cells_.insert(k);
        }
      }
    }
  }
}

bool CameraFusionGrid::lidarNear(const CellKey & k, double t) const
{
  return std::abs(t - lidar_stamp_) <= p_.lidar_max_age && lidar_cells_.count(k) > 0;
}

FusionFrameStats CameraFusionGrid::addCameraFrame(const CameraObservation & obs)
{
  FusionFrameStats st;
  ++frame_id_;
  const double half_h = 0.5 * p_.horizontal_fov;
  const double half_v = 0.5 * p_.vertical_fov;
  const int n_az = static_cast<int>(std::ceil(p_.horizontal_fov / p_.azimuth_bin));
  const int n_el = static_cast<int>(std::ceil(p_.vertical_fov / p_.elevation_bin));
  const float inf = std::numeric_limits<float>::infinity();
  // "Ảnh độ sâu" thô: khoảng cách ngang nhỏ nhất theo (phương vị, góc ngẩng) quanh trục camera.
  std::vector<float> bin_min(static_cast<std::size_t>(n_az) * n_el, inf);
  auto azIndex = [&](double az) {
      return std::min(n_az - 1, static_cast<int>(std::floor((az + half_h) / p_.azimuth_bin)));
    };
  auto elIndex = [&](double el) {
      return std::min(n_el - 1, static_cast<int>(std::floor((el + half_v) / p_.elevation_bin)));
    };

  struct Acc
  {
    int n{0};
    float zmin{std::numeric_limits<float>::infinity()};
    float zmax{-std::numeric_limits<float>::infinity()};
  };
  std::unordered_map<CellKey, Acc, CellKeyHash> acc;

  for (const auto & q : obs.points) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
      continue;
    }
    ++st.points_in;
    const double dx = q.x - obs.sensor.x, dy = q.y - obs.sensor.y, dz = q.z - obs.sensor.z;
    const double d = std::hypot(dx, dy);
    if (d < 1e-3) {
      continue;
    }
    const double az = wrapAngle(std::atan2(dy, dx) - obs.sensor_yaw);
    const double el = std::atan2(dz, d);
    if (std::abs(az) <= half_h && std::abs(el) <= half_v) {
      float & b = bin_min[static_cast<std::size_t>(azIndex(az)) * n_el + elIndex(el)];
      b = std::min(b, static_cast<float>(d));
    }
    if (d < p_.min_range || d > p_.max_range || q.z < p_.min_obstacle_height ||
      q.z > p_.max_obstacle_height ||
      std::hypot(q.x - obs.robot_x, q.y - obs.robot_y) < p_.self_filter_radius)
    {
      continue;
    }
    ++st.points_marking;
    Acc & a = acc[keyOf(q.x, q.y)];
    ++a.n;
    a.zmin = std::min(a.zmin, static_cast<float>(q.z));
    a.zmax = std::max(a.zmax, static_cast<float>(q.z));
  }

  // 1) Ô camera thấy trong frame này.
  for (const auto & kv : acc) {
    if (kv.second.n < p_.min_points_per_cell) {
      continue;
    }
    ++st.hit_cells;
    auto it = cells_.find(kv.first);
    if (it == cells_.end()) {
      it = cells_.emplace(kv.first, Cell()).first;
      it->second.zmin = kv.second.zmin;
      it->second.zmax = kv.second.zmax;
    } else {
      it->second.zmin = std::min(it->second.zmin, kv.second.zmin);
      it->second.zmax = std::max(it->second.zmax, kv.second.zmax);
    }
    Cell & c = it->second;
    c.hit_streak = (c.last_hit_frame + 1 == frame_id_) ? c.hit_streak + 1 : 1;
    c.last_hit_frame = frame_id_;
    c.last_hit_time = obs.stamp;
    c.free_streak = 0;
    if (c.state == CellState::PENDING &&
      (c.hit_streak >= p_.confirm_frames || lidarNear(kv.first, obs.stamp)))
    {
      c.state = CellState::CONFIRMED;
      ++st.new_confirmed;
    }
  }

  // 2) Ô không được thấy trong frame này.
  for (auto it = cells_.begin(); it != cells_.end(); ) {
    Cell & c = it->second;
    if (c.last_hit_frame == frame_id_) {
      ++it;
      continue;
    }
    if (c.state == CellState::PENDING) {   // chuỗi frame liên tiếp bị đứt
      ++st.dropped_pending;
      it = cells_.erase(it);
      continue;
    }
    // Ô đã xác nhận: chỉ xóa khi có bằng chứng camera nhìn xuyên qua toàn bộ dải độ cao của nó.
    bool seen_through = false;
    const auto ctr = centerOf(it->first);
    const double dx = ctr.first - obs.sensor.x, dy = ctr.second - obs.sensor.y;
    const double d = std::hypot(dx, dy);
    const double az = wrapAngle(std::atan2(dy, dx) - obs.sensor_yaw);
    if (d >= p_.min_range && d <= p_.max_range && std::abs(az) <= half_h - 0.5 * p_.azimuth_bin) {
      const double hr = 0.5 * p_.resolution;
      const double dn = std::max(d - hr, 1e-3), df = d + hr;
      const double el_hi = std::max(
        std::atan2(c.zmax - obs.sensor.z, dn),
        std::atan2(c.zmax - obs.sensor.z, df));
      const double el_lo = std::min(
        std::atan2(c.zmin - obs.sensor.z, dn),
        std::atan2(c.zmin - obs.sensor.z, df));
      if (el_lo >= -half_v && el_hi <= half_v) {
        const std::size_t ia = static_cast<std::size_t>(azIndex(az));
        seen_through = true;
        for (int ie = elIndex(el_lo); ie <= elIndex(el_hi); ++ie) {
          const float b = bin_min[ia * n_el + ie];
          if (!(std::isfinite(b) && b > d + p_.resolution)) {
            seen_through = false;
            break;
          }
        }
      }
    }
    if (seen_through) {
      if (++c.free_streak >= p_.clear_frames) {
        ++st.cleared;
        it = cells_.erase(it);
        continue;
      }
    } else {
      c.free_streak = 0;
    }
    ++it;
  }
  return st;
}

std::size_t CameraFusionGrid::decay(double now, double robot_x, double robot_y)
{
  std::size_t n = 0;
  for (auto it = cells_.begin(); it != cells_.end(); ) {
    const auto ctr = centerOf(it->first);
    const bool too_old = it->second.state == CellState::CONFIRMED &&
      now - it->second.last_hit_time > p_.decay_time;
    const bool too_far =
      std::hypot(ctr.first - robot_x, ctr.second - robot_y) > p_.max_cell_distance;
    if (too_old || too_far) {
      it = cells_.erase(it);
      ++n;
    } else {
      ++it;
    }
  }
  return n;
}

std::vector<std::pair<double, double>> CameraFusionGrid::obstacleCells(
  double now, std::size_t * suppressed) const
{
  std::vector<std::pair<double, double>> out;
  std::size_t sup = 0;
  for (const auto & kv : cells_) {
    if (kv.second.state != CellState::CONFIRMED) {
      continue;
    }
    if (lidarNear(kv.first, now)) {
      ++sup;
      continue;
    }
    out.push_back(centerOf(kv.first));
  }
  if (suppressed) {
    *suppressed = sup;
  }
  return out;
}

CellState CameraFusionGrid::state(double x, double y) const
{
  const auto it = cells_.find(keyOf(x, y));
  return it == cells_.end() ? CellState::NONE : it->second.state;
}

std::size_t CameraFusionGrid::pendingCount() const
{
  return static_cast<std::size_t>(std::count_if(
           cells_.begin(), cells_.end(),
           [](const auto & kv) {return kv.second.state == CellState::PENDING;}));
}

std::size_t CameraFusionGrid::confirmedCount() const
{
  return cells_.size() - pendingCount();
}

}  // namespace stage3_navigation
