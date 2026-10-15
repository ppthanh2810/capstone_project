#include "stage3_navigation/path_blockage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stage3_navigation
{

bool GridView::cost(double x, double y, int & out) const
{
  if (!valid()) {
    return false;
  }
  const double fx = std::floor((x - origin_x) / resolution);
  const double fy = std::floor((y - origin_y) / resolution);
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0.0 || fy < 0.0 ||
    fx >= static_cast<double>(width) || fy >= static_cast<double>(height))
  {
    return false;
  }
  out = data[static_cast<std::size_t>(fy) * width + static_cast<std::size_t>(fx)];
  return true;
}

namespace
{
double segLength(const std::vector<PathPoint> & p, std::size_t i)
{
  return std::hypot(p[i + 1].x - p[i].x, p[i + 1].y - p[i].y);
}
}  // namespace

std::size_t projectOnPath(
  const std::vector<PathPoint> & path, double x, double y, std::size_t from_segment,
  double window, double & along)
{
  along = 0.0;
  if (path.size() < 2) {
    return 0;
  }
  const std::size_t last_seg = path.size() - 2;
  std::size_t best = std::min(from_segment, last_seg);
  double best_d = std::numeric_limits<double>::infinity();
  double arc = 0.0;
  for (std::size_t i = std::min(from_segment, last_seg); i <= last_seg; ++i) {
    const double len = segLength(path, i);
    double t = 0.0;
    if (len > 1e-9) {
      t = ((x - path[i].x) * (path[i + 1].x - path[i].x) +
        (y - path[i].y) * (path[i + 1].y - path[i].y)) / (len * len);
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = path[i].x + t * (path[i + 1].x - path[i].x);
    const double py = path[i].y + t * (path[i + 1].y - path[i].y);
    const double d = std::hypot(x - px, y - py);
    if (d < best_d) {
      best_d = d;
      best = i;
      along = t * len;
    }
    arc += len;
    if (arc > window) {
      break;
    }
  }
  return best;
}

Blockage findBlockage(
  const std::vector<PathPoint> & path, const GridView & grid, std::size_t segment, double along,
  double start_offset, double lookahead, int blocked_cost)
{
  Blockage out;
  if (path.empty() || !grid.valid() || lookahead <= 0.0) {
    return out;
  }
  const double step = 0.5 * grid.resolution;
  const auto check = [&](double px, double py, double dist) {
      int c = 0;
      if (dist >= start_offset && grid.cost(px, py, c) && c >= blocked_cost) {
        out.blocked = true;
        out.distance_ahead = dist;
        out.x = px;
        out.y = py;
        out.cost = c;
        return true;
      }
      return false;
    };

  if (path.size() == 1) {
    check(path[0].x, path[0].y, start_offset);
    return out;
  }
  const double end = start_offset + lookahead;
  double dist = -along;   // độ dài cung (so với điểm chiếu) tại đầu đoạn hiện tại
  for (std::size_t i = std::min(segment, path.size() - 2); i + 1 < path.size(); ++i) {
    const double len = segLength(path, i);
    // Mẫu trên đoạn i ở các khoảng cách cung s (so với điểm chiếu) trong [max(dist, 0), dist + len].
    double s = std::max(dist, 0.0);
    while (s <= dist + len + 1e-9 && s <= end) {
      const double t = len > 1e-9 ? (s - dist) / len : 0.0;
      if (check(
          path[i].x + t * (path[i + 1].x - path[i].x),
          path[i].y + t * (path[i + 1].y - path[i].y), s))
      {
        return out;
      }
      s += step;
    }
    dist += len;
    if (dist > end) {
      break;
    }
  }
  // Điểm cuối (đích) luôn được xét nếu nằm trong tầm nhìn.
  if (dist <= end) {
    check(path.back().x, path.back().y, std::max(dist, start_offset));
  }
  return out;
}

}  // namespace stage3_navigation
