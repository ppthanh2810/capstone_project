#include "stage2_slam/scan_map_consistency.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stage2_slam
{

MapRaycaster::MapRaycaster(const GridMap & map, int occupied_threshold)
: width_(map.width), height_(map.height), resolution_(map.resolution), origin_(map.origin),
  cos_o_(std::cos(map.origin.yaw)), sin_o_(std::sin(map.origin.yaw)),
  occupied_threshold_(occupied_threshold), data_(map.data)
{
}

std::optional<double> MapRaycaster::cast(double x, double y, double angle, double max_range) const
{
  if (width_ <= 0 || height_ <= 0 || !(resolution_ > 0.0) ||
    data_.size() != static_cast<std::size_t>(width_) * height_)
  {
    return 0.0;
  }
  // Đổi sang toạ độ ô liên tục (đơn vị ô) và hướng trong frame lưới.
  const double dx = x - origin_.x;
  const double dy = y - origin_.y;
  const double gx = (cos_o_ * dx + sin_o_ * dy) / resolution_;
  const double gy = (-sin_o_ * dx + cos_o_ * dy) / resolution_;
  const double a = angle - origin_.yaw;
  const double dir_x = std::cos(a);
  const double dir_y = std::sin(a);

  int ix = static_cast<int>(std::floor(gx));
  int iy = static_cast<int>(std::floor(gy));
  auto stops = [this](int cx, int cy) {
      if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {return true;}
      const int8_t v = data_[static_cast<std::size_t>(cy) * width_ + cx];
      return v < 0 || v >= occupied_threshold_;
    };
  if (stops(ix, iy)) {return 0.0;}

  // Amanatides & Woo: duyệt đúng từng ô tia đi qua; t tính theo đơn vị ô.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const int step_x = dir_x > 0.0 ? 1 : -1;
  const int step_y = dir_y > 0.0 ? 1 : -1;
  const double t_delta_x = dir_x != 0.0 ? 1.0 / std::abs(dir_x) : kInf;
  const double t_delta_y = dir_y != 0.0 ? 1.0 / std::abs(dir_y) : kInf;
  double t_max_x = dir_x > 0.0 ? (ix + 1 - gx) * t_delta_x :
    dir_x < 0.0 ? (gx - ix) * t_delta_x : kInf;
  double t_max_y = dir_y > 0.0 ? (iy + 1 - gy) * t_delta_y :
    dir_y < 0.0 ? (gy - iy) * t_delta_y : kInf;

  const double max_t = max_range / resolution_;
  while (true) {
    double t;
    if (t_max_x < t_max_y) {
      t = t_max_x;
      t_max_x += t_delta_x;
      ix += step_x;
    } else {
      t = t_max_y;
      t_max_y += t_delta_y;
      iy += step_y;
    }
    if (t > max_t) {return std::nullopt;}
    if (stops(ix, iy)) {return t * resolution_;}
  }
}

ConsistencyResult evaluateScanConsistency(
  const ScoreField & field, const MapRaycaster & raycaster, const std::vector<Point2D> & points,
  const Pose2D & sensor_in_map, const DistanceFilterConfig & config)
{
  ConsistencyResult r;
  r.total = points.size();
  if (points.empty()) {return r;}

  const double c = std::cos(sensor_in_map.yaw);
  const double s = std::sin(sensor_in_map.yaw);
  double sum_all = 0.0;
  double sum_counted = 0.0;
  for (const auto & p : points) {
    const double mx = sensor_in_map.x + c * p.x - s * p.y;
    const double my = sensor_in_map.y + s * p.x + c * p.y;
    const double score = field.scoreAtWorld(mx, my);
    sum_all += score;

    bool counted = true;
    if (config.enabled) {
      const double range = std::hypot(p.x, p.y);
      const double angle = sensor_in_map.yaw + std::atan2(p.y, p.x);
      // Chỉ cần biết r_exp lớn hơn r + margin (SHORT) hay nhỏ hơn r - margin (LONG).
      const double horizon = range + std::max(config.short_margin, config.long_margin) + 0.1;
      const auto expected = raycaster.cast(sensor_in_map.x, sensor_in_map.y, angle, horizon);
      const double r_exp = expected ? *expected : std::numeric_limits<double>::infinity();
      if (r_exp - range > config.short_margin) {
        if (score < config.match_score) {
          ++r.short_beams;
          counted = false;
        }
      } else if (range - r_exp > config.long_margin) {
        ++r.long_beams;
      }
    }
    if (counted) {
      ++r.counted;
      sum_counted += score;
    }
  }

  r.consistency_all = sum_all / points.size();
  r.short_ratio = static_cast<double>(r.short_beams) / points.size();
  r.long_ratio = static_cast<double>(r.long_beams) / points.size();
  if (r.counted > 0 && r.counted >= config.min_counted_points) {
    r.consistency = sum_counted / r.counted;
  }
  return r;
}

}  // namespace stage2_slam
