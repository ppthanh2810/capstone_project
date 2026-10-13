#include "stage2_slam/scan_match_initializer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace stage2_slam
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kFar = 1e20;

// Felzenszwalb & Huttenlocher, 1D squared Euclidean distance transform (đơn vị ô).
void distanceTransform1D(
  const double * f, int n, double * d, int * v, double * z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const double dq = double(q) - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

std::string formatPose(const Pose2D & p)
{
  std::ostringstream ss;
  ss.precision(3);
  ss << std::fixed << "(" << p.x << ", " << p.y << ", " << p.yaw << ")";
  return ss.str();
}

}  // namespace

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalizeAngle(a.yaw + b.yaw)};
}

Pose2D between(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, normalizeAngle(b.yaw - a.yaw)};
}

const char * toString(MatchStatus status)
{
  switch (status) {
    case MatchStatus::ACCEPTED: return "ACCEPTED";
    case MatchStatus::REJECTED: return "REJECTED";
    case MatchStatus::AMBIGUOUS: return "AMBIGUOUS";
    case MatchStatus::NO_SCAN: return "NO_SCAN";
    case MatchStatus::NO_MAP: return "NO_MAP";
    case MatchStatus::TF_ERROR: return "TF_ERROR";
    case MatchStatus::TIMEOUT: return "TIMEOUT";
    case MatchStatus::AMCL_INACTIVE: return "AMCL_INACTIVE";
    case MatchStatus::INVALID_INPUT: return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// ScoreField
// ---------------------------------------------------------------------------
std::optional<ScoreField> ScoreField::build(
  const GridMap & map, const ScoreFieldConfig & config, std::string * error)
{
  auto fail = [error](const std::string & msg) -> std::optional<ScoreField> {
      if (error) {*error = msg;}
      return std::nullopt;
    };
  if (map.width <= 0 || map.height <= 0) {return fail("map has zero size");}
  if (!(map.resolution > 0.0) || !std::isfinite(map.resolution)) {
    return fail("map resolution must be > 0");
  }
  if (map.data.size() != static_cast<std::size_t>(map.width) * map.height) {
    return fail("map data size != width*height");
  }
  if (!std::isfinite(map.origin.x) || !std::isfinite(map.origin.y) ||
    !std::isfinite(map.origin.yaw))
  {
    return fail("map origin is not finite");
  }
  if (!(config.sigma > 0.0) || !(config.max_distance > 0.0)) {
    return fail("score field sigma/max_distance must be > 0");
  }

  ScoreField field;
  field.width_ = map.width;
  field.height_ = map.height;
  field.resolution_ = map.resolution;
  field.origin_ = map.origin;
  field.cos_o_ = std::cos(map.origin.yaw);
  field.sin_o_ = std::sin(map.origin.yaw);

  const int w = map.width;
  const int h = map.height;
  std::vector<double> grid(static_cast<std::size_t>(w) * h, kFar);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    if (map.data[i] >= config.occupied_threshold) {
      grid[i] = 0.0;
      ++field.occupied_cells_;
    }
  }
  if (field.occupied_cells_ == 0) {
    return fail("map has no occupied cells");
  }

  // 2D EDT: theo cột rồi theo hàng.
  const int n = std::max(w, h);
  std::vector<double> f(n), d(n), z(n + 1);
  std::vector<int> v(n);
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {f[y] = grid[static_cast<std::size_t>(y) * w + x];}
    distanceTransform1D(f.data(), h, d.data(), v.data(), z.data());
    for (int y = 0; y < h; ++y) {grid[static_cast<std::size_t>(y) * w + x] = d[y];}
  }
  for (int y = 0; y < h; ++y) {
    double * row = &grid[static_cast<std::size_t>(y) * w];
    std::copy(row, row + w, f.begin());
    distanceTransform1D(f.data(), w, d.data(), v.data(), z.data());
    std::copy(d.begin(), d.begin() + w, row);
  }

  field.score_.resize(grid.size());
  const double inv_two_sigma2 = 1.0 / (2.0 * config.sigma * config.sigma);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    if (map.data[i] < 0) {
      field.score_[i] = static_cast<float>(config.unknown_score);
      continue;
    }
    const double dist = std::sqrt(grid[i]) * map.resolution;
    field.score_[i] = dist > config.max_distance ?
      0.0f : static_cast<float>(std::exp(-dist * dist * inv_two_sigma2));
  }
  return field;
}

void ScoreField::worldToGrid(double x, double y, double & gx, double & gy) const
{
  const double dx = x - origin_.x;
  const double dy = y - origin_.y;
  gx = (cos_o_ * dx + sin_o_ * dy) / resolution_;
  gy = (-sin_o_ * dx + cos_o_ * dy) / resolution_;
}

float ScoreField::cellScore(int cx, int cy) const
{
  if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {return 0.0f;}
  return score_[static_cast<std::size_t>(cy) * width_ + cx];
}

double ScoreField::scoreAtGrid(double gx, double gy) const
{
  return cellScore(static_cast<int>(std::floor(gx)), static_cast<int>(std::floor(gy)));
}

double ScoreField::scoreAtGridBilinear(double gx, double gy) const
{
  // Giá trị ô đặt tại tâm ô (i + 0.5).
  const double fx = gx - 0.5;
  const double fy = gy - 0.5;
  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  const double tx = fx - x0;
  const double ty = fy - y0;
  const double s00 = cellScore(x0, y0);
  const double s10 = cellScore(x0 + 1, y0);
  const double s01 = cellScore(x0, y0 + 1);
  const double s11 = cellScore(x0 + 1, y0 + 1);
  return (1 - ty) * ((1 - tx) * s00 + tx * s10) + ty * ((1 - tx) * s01 + tx * s11);
}

double ScoreField::scoreAtWorld(double x, double y) const
{
  double gx, gy;
  worldToGrid(x, y, gx, gy);
  return scoreAtGrid(gx, gy);
}

// ---------------------------------------------------------------------------
// Scan
// ---------------------------------------------------------------------------
PreprocessResult scanToPoints(
  const ScanData & scan, const Pose2D & sensor_in_base, const PreprocessConfig & config)
{
  PreprocessResult out;
  out.total_beams = scan.ranges.size();
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
    !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0)
  {
    return out;
  }
  const double rmin = std::max(config.min_range, std::isfinite(scan.range_min) ? scan.range_min : 0.0);
  double rmax = config.max_range;
  if (std::isfinite(scan.range_max) && scan.range_max > 0.0) {rmax = std::min(rmax, scan.range_max);}

  std::vector<Point2D> sensor_points;
  sensor_points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < rmin || r > rmax) {continue;}
    // Góc dùng nguyên trạng: scan đã được scan_rotate chuẩn hóa về laser_frame.
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    sensor_points.push_back({r * std::cos(a), r * std::sin(a)});
    out.max_point_range = std::max(out.max_point_range, r);
  }
  out.valid_beams = sensor_points.size();

  const std::size_t stride = (config.max_points > 0 && sensor_points.size() > config.max_points) ?
    (sensor_points.size() + config.max_points - 1) / config.max_points : 1;
  const double c = std::cos(sensor_in_base.yaw);
  const double s = std::sin(sensor_in_base.yaw);
  for (std::size_t i = 0; i < sensor_points.size(); i += stride) {
    const auto & p = sensor_points[i];
    out.points.push_back(
      {sensor_in_base.x + c * p.x - s * p.y, sensor_in_base.y + s * p.x + c * p.y});
  }
  return out;
}

// ---------------------------------------------------------------------------
// Matcher
// ---------------------------------------------------------------------------
double scorePose(const ScoreField & field, const std::vector<Point2D> & points, const Pose2D & pose)
{
  if (points.empty()) {return 0.0;}
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double sum = 0.0;
  for (const auto & p : points) {
    double gx, gy;
    field.worldToGrid(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y, gx, gy);
    sum += field.scoreAtGrid(gx, gy);
  }
  return sum / points.size();
}

double scorePoseBilinear(
  const ScoreField & field, const std::vector<Point2D> & points, const Pose2D & pose)
{
  if (points.empty()) {return 0.0;}
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double sum = 0.0;
  for (const auto & p : points) {
    double gx, gy;
    field.worldToGrid(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y, gx, gy);
    sum += field.scoreAtGridBilinear(gx, gy);
  }
  return sum / points.size();
}

namespace
{

double inlierFraction(
  const ScoreField & field, const std::vector<Point2D> & points, const Pose2D & pose,
  double threshold)
{
  if (points.empty()) {return 0.0;}
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  std::size_t n = 0;
  for (const auto & p : points) {
    double gx, gy;
    field.worldToGrid(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y, gx, gy);
    if (field.scoreAtGridBilinear(gx, gy) >= threshold) {++n;}
  }
  return static_cast<double>(n) / points.size();
}

// Hooke-Jeeves pattern search trên score nội suy, giới hạn quanh pose thô.
Pose2D refinePose(
  const ScoreField & field, const std::vector<Point2D> & points, const Pose2D & start,
  double lin_bound, double ang_bound, const RefineConfig & cfg)
{
  Pose2D best = start;
  double best_score = scorePoseBilinear(field, points, best);
  double lin = lin_bound / 2.0;
  double ang = ang_bound / 2.0;
  for (int it = 0; it < cfg.max_iterations; ++it) {
    if (lin < cfg.min_linear_step && ang < cfg.min_angular_step) {break;}
    Pose2D cand_best = best;
    double cand_score = best_score;
    const Pose2D moves[6] = {
      {lin, 0, 0}, {-lin, 0, 0}, {0, lin, 0}, {0, -lin, 0}, {0, 0, ang}, {0, 0, -ang}};
    for (const auto & m : moves) {
      Pose2D p{best.x + m.x, best.y + m.y, best.yaw + m.yaw};
      if (std::abs(p.x - start.x) > lin_bound || std::abs(p.y - start.y) > lin_bound ||
        std::abs(normalizeAngle(p.yaw - start.yaw)) > ang_bound)
      {
        continue;
      }
      const double sc = scorePoseBilinear(field, points, p);
      if (sc > cand_score + 1e-12) {
        cand_score = sc;
        cand_best = p;
      }
    }
    if (cand_score > best_score) {
      best = cand_best;
      best_score = cand_score;
    } else {
      lin *= 0.5;
      ang *= 0.5;
    }
  }
  best.yaw = normalizeAngle(best.yaw);
  return best;
}

}  // namespace

MatchResult matchScan(
  const ScoreField & field, const std::vector<Point2D> & points, std::size_t total_beams,
  const Pose2D & guess, const MatcherConfig & config)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&t0]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
  const SearchConfig & sc = config.search;
  const ValidationConfig & vc = config.validation;

  MatchResult r;
  r.guess = guess;
  r.pose = guess;
  r.coarse_pose = guess;
  r.valid_points = points.size();
  (void)total_beams;

  auto done = [&](MatchStatus st, const std::string & why) {
      r.status = st;
      r.reason = why;
      r.elapsed = elapsed();
      return r;
    };

  if (!std::isfinite(guess.x) || !std::isfinite(guess.y) || !std::isfinite(guess.yaw)) {
    return done(MatchStatus::INVALID_INPUT, "initial guess is not finite");
  }
  if (!(sc.linear_window >= 0.0) || !(sc.angular_window >= 0.0) || !(sc.linear_step > 0.0)) {
    return done(MatchStatus::INVALID_INPUT, "invalid search window/step");
  }
  if (points.size() < vc.min_valid_points) {
    return done(
      MatchStatus::NO_SCAN, "insufficient valid scan points (" + std::to_string(points.size()) +
      " < " + std::to_string(vc.min_valid_points) + ")");
  }

  // Bước góc: tự tính từ tầm xa của điểm sao cho điểm xa nhất dịch ~ 1 linear_step.
  double max_r = 0.0;
  for (const auto & p : points) {max_r = std::max(max_r, std::hypot(p.x, p.y));}
  double dtheta = sc.angular_step;
  if (!(dtheta > 0.0)) {
    const double rr = sc.linear_step;
    const double arg = 1.0 - (rr * rr) / (2.0 * std::max(max_r, rr) * std::max(max_r, rr));
    dtheta = std::acos(std::clamp(arg, -1.0, 1.0));
    dtheta = std::clamp(dtheta, sc.min_auto_angular_step, sc.max_auto_angular_step);
  }
  r.angular_step = dtheta;

  const int na = static_cast<int>(std::ceil(sc.angular_window / dtheta - 1e-9));
  const int nl = static_cast<int>(std::ceil(sc.linear_window / sc.linear_step - 1e-9));
  const int ny_a = 2 * na + 1;
  const int n_l = 2 * nl + 1;
  const std::size_t total = static_cast<std::size_t>(ny_a) * n_l * n_l;
  r.candidates = total;
  if (total > sc.max_candidates) {
    return done(
      MatchStatus::INVALID_INPUT, "search space too large (" + std::to_string(total) +
      " > max_candidates " + std::to_string(sc.max_candidates) + ")");
  }

  // Lưới ứng viên: volume[a][ix][iy].
  std::vector<float> volume(total, 0.0f);
  const double res = field.resolution();
  const double co = field.cosOrigin();
  const double so = field.sinOrigin();
  // Độ dời một bước ix / iy (frame map) -> độ dời ô.
  const double ux_x = co * sc.linear_step / res, ux_y = -so * sc.linear_step / res;
  const double uy_x = so * sc.linear_step / res, uy_y = co * sc.linear_step / res;

  std::vector<double> bx(points.size()), by(points.size());
  float best = -1.0f;
  int best_a = 0, best_ix = 0, best_iy = 0;
  for (int ai = 0; ai < ny_a; ++ai) {
    const double yaw = guess.yaw + (ai - na) * dtheta;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (std::size_t i = 0; i < points.size(); ++i) {
      // Ô liên tục của điểm tại translation (guess.x - nl*step, guess.y - nl*step).
      const double wx = guess.x - nl * sc.linear_step + c * points[i].x - s * points[i].y;
      const double wy = guess.y - nl * sc.linear_step + s * points[i].x + c * points[i].y;
      field.worldToGrid(wx, wy, bx[i], by[i]);
    }
    for (int ix = 0; ix < n_l; ++ix) {
      for (int iy = 0; iy < n_l; ++iy) {
        const double ox = ix * ux_x + iy * uy_x;
        const double oy = ix * ux_y + iy * uy_y;
        double sum = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
          sum += field.scoreAtGrid(bx[i] + ox, by[i] + oy);
        }
        const float sc_val = static_cast<float>(sum / points.size());
        volume[(static_cast<std::size_t>(ai) * n_l + ix) * n_l + iy] = sc_val;
        if (sc_val > best) {
          best = sc_val;
          best_a = ai;
          best_ix = ix;
          best_iy = iy;
        }
      }
    }
    if (elapsed() > sc.time_budget) {
      return done(
        MatchStatus::TIMEOUT, "search exceeded time budget (" + std::to_string(sc.time_budget) +
        " s)");
    }
  }

  auto poseOf = [&](int ai, int ix, int iy) {
      return Pose2D{guess.x + (ix - nl) * sc.linear_step, guess.y + (iy - nl) * sc.linear_step,
        normalizeAngle(guess.yaw + (ai - na) * dtheta)};
    };
  auto at = [&](int ai, int ix, int iy) {
      return volume[(static_cast<std::size_t>(ai) * n_l + ix) * n_l + iy];
    };

  r.coarse_pose = poseOf(best_a, best_ix, best_iy);
  r.score = best;
  r.guess_score = scorePose(field, points, guess);

  // Score nearest-cell bão hòa thành một vùng đỉnh phẳng rộng ~1 ô bản đồ. Lấy ứng viên đầu tiên
  // đạt max sẽ lệch theo thứ tự duyệt -> dùng trọng tâm các ứng viên bằng max nằm sát đỉnh.
  {
    const float tie = best - 1e-4f;
    const int ra = std::max(1, static_cast<int>(std::ceil(vc.peak_separation_angular / dtheta)));
    const int rl = std::max(1, static_cast<int>(std::ceil(vc.peak_separation_linear / sc.linear_step)));
    double sx = 0.0, sy = 0.0, sa = 0.0;
    int cnt = 0;
    for (int ai = std::max(0, best_a - ra); ai <= std::min(ny_a - 1, best_a + ra); ++ai) {
      for (int ix = std::max(0, best_ix - rl); ix <= std::min(n_l - 1, best_ix + rl); ++ix) {
        for (int iy = std::max(0, best_iy - rl); iy <= std::min(n_l - 1, best_iy + rl); ++iy) {
          if (at(ai, ix, iy) < tie) {continue;}
          sx += ix;
          sy += iy;
          sa += ai;
          ++cnt;
        }
      }
    }
    if (cnt > 1) {
      r.coarse_pose = Pose2D{
        guess.x + (sx / cnt - nl) * sc.linear_step, guess.y + (sy / cnt - nl) * sc.linear_step,
        normalizeAngle(guess.yaw + (sa / cnt - na) * dtheta)};
    }
  }

  // Refinement cục bộ, giới hạn trong ~1 ô bản đồ / 1 bước góc quanh pose thô (không nhảy đỉnh).
  r.pose = r.coarse_pose;
  if (config.refine.enabled) {
    r.pose = refinePose(
      field, points, r.coarse_pose, std::max(sc.linear_step, field.resolution()), dtheta,
      config.refine);
  }
  r.inlier_fraction = inlierFraction(field, points, r.pose, vc.inlier_score);

  // Cực đại cục bộ độc lập lớn nhất: cách đỉnh đủ xa, không nằm trên biên cửa sổ và bị ngăn cách
  // với đỉnh bởi một thung lũng (xem ValidationConfig::valley_ratio).
  const bool full_circle = sc.angular_window >= kPi;
  auto onBoundary = [&](int ai, int ix, int iy) {
      return (nl > 0 && (ix == 0 || ix == n_l - 1 || iy == 0 || iy == n_l - 1)) ||
             (!full_circle && na > 0 && (ai == 0 || ai == ny_a - 1));
    };
  struct Peak
  {
    float val;
    int ai, ix, iy;
  };
  std::vector<Peak> peaks;
  for (int ai = 0; ai < ny_a; ++ai) {
    for (int ix = 0; ix < n_l; ++ix) {
      for (int iy = 0; iy < n_l; ++iy) {
        const float val = at(ai, ix, iy);
        if (val < vc.min_score * vc.ambiguity_ratio) {continue;}  // không thể gây AMBIGUOUS
        if (onBoundary(ai, ix, iy)) {continue;}
        const Pose2D p = poseOf(ai, ix, iy);
        const double dl = std::hypot(p.x - r.coarse_pose.x, p.y - r.coarse_pose.y);
        const double da = std::abs(normalizeAngle(p.yaw - r.coarse_pose.yaw));
        if (dl <= vc.peak_separation_linear && da <= vc.peak_separation_angular) {continue;}
        bool is_max = true;
        for (int da_ = -1; da_ <= 1 && is_max; ++da_) {
          for (int dx = -1; dx <= 1 && is_max; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              const int a2 = ai + da_, x2 = ix + dx, y2 = iy + dy;
              if ((da_ | dx | dy) == 0 || a2 < 0 || x2 < 0 || y2 < 0 || a2 >= ny_a || x2 >= n_l ||
                y2 >= n_l)
              {
                continue;
              }
              if (at(a2, x2, y2) > val) {is_max = false; break;}
            }
          }
        }
        if (is_max) {peaks.push_back({val, ai, ix, iy});}
      }
    }
  }
  std::sort(
    peaks.begin(), peaks.end(), [](const Peak & a, const Peak & b) {return a.val > b.val;});
  // Đỉnh tham chiếu trên lưới: ô gần coarse_pose (trọng tâm vùng đỉnh) nhất.
  const int b_a = std::clamp(
    static_cast<int>(std::lround(normalizeAngle(r.coarse_pose.yaw - guess.yaw) / dtheta)) + na,
    0, ny_a - 1);
  const int b_ix = std::clamp(
    static_cast<int>(std::lround((r.coarse_pose.x - guess.x) / sc.linear_step)) + nl, 0, n_l - 1);
  const int b_iy = std::clamp(
    static_cast<int>(std::lround((r.coarse_pose.y - guess.y) / sc.linear_step)) + nl, 0, n_l - 1);
  float second = 0.0f;
  int s_a = -1, s_ix = 0, s_iy = 0;
  constexpr std::size_t kMaxPeaksChecked = 64;
  for (std::size_t k = 0; k < peaks.size() && k < kMaxPeaksChecked; ++k) {
    const Peak & pk = peaks[k];
    const int steps = std::max(
      {std::abs(pk.ai - b_a), std::abs(pk.ix - b_ix), std::abs(pk.iy - b_iy), 1});
    float valley = pk.val;
    for (int t = 1; t < steps; ++t) {
      const double f = static_cast<double>(t) / steps;
      const int a2 = static_cast<int>(std::lround(b_a + f * (pk.ai - b_a)));
      const int x2 = static_cast<int>(std::lround(b_ix + f * (pk.ix - b_ix)));
      const int y2 = static_cast<int>(std::lround(b_iy + f * (pk.iy - b_iy)));
      valley = std::min(valley, at(a2, x2, y2));
    }
    if (valley < static_cast<float>(vc.valley_ratio) * pk.val) {
      second = pk.val;
      s_a = pk.ai;
      s_ix = pk.ix;
      s_iy = pk.iy;
      break;
    }
  }
  if (s_a >= 0) {
    r.second_score = second;
    r.second_pose = poseOf(s_a, s_ix, s_iy);
  }
  r.ambiguity = best > 0.0f ? r.second_score / best : 0.0;

  // Hiệp phương sai heuristic từ các ứng viên gần đỉnh.
  {
    const CovarianceConfig & cc = config.covariance;
    double sw = 0.0;
    double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    const float thr = static_cast<float>(cc.score_fraction) * best;
    for (int ai = 0; ai < ny_a; ++ai) {
      for (int ix = 0; ix < n_l; ++ix) {
        for (int iy = 0; iy < n_l; ++iy) {
          const float val = at(ai, ix, iy);
          if (val < thr) {continue;}
          const Pose2D p = poseOf(ai, ix, iy);
          const double e[3] = {p.x - r.pose.x, p.y - r.pose.y, normalizeAngle(p.yaw - r.pose.yaw)};
          sw += val;
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {m[i][j] += val * e[i] * e[j];}
          }
        }
      }
    }
    double sd[3];
    const double lo[3] = {cc.min_std_linear, cc.min_std_linear, cc.min_std_angular};
    const double hi[3] = {cc.max_std_linear, cc.max_std_linear, cc.max_std_angular};
    for (int i = 0; i < 3; ++i) {
      const double var = sw > 0.0 ? m[i][i] / sw : 0.0;
      sd[i] = std::clamp(std::sqrt(std::max(var, 0.0)), lo[i], hi[i]);
    }
    for (int i = 0; i < 3; ++i) {
      r.covariance[i * 3 + i] = sd[i] * sd[i];
      for (int j = i + 1; j < 3; ++j) {
        const double vi = sw > 0.0 ? m[i][i] / sw : 0.0;
        const double vj = sw > 0.0 ? m[j][j] / sw : 0.0;
        double rho = (vi > 0.0 && vj > 0.0) ? (m[i][j] / sw) / std::sqrt(vi * vj) : 0.0;
        rho = std::clamp(rho, -0.9, 0.9);
        r.covariance[i * 3 + j] = r.covariance[j * 3 + i] = rho * sd[i] * sd[j];
      }
    }
  }

  r.at_window_boundary = best_ix == 0 || best_ix == n_l - 1 || best_iy == 0 || best_iy == n_l - 1 ||
    (!full_circle && (best_a == 0 || best_a == ny_a - 1));

  if (best < vc.min_score) {
    std::ostringstream ss;
    ss << "best score " << best << " < min_score " << vc.min_score;
    return done(MatchStatus::REJECTED, ss.str());
  }
  if (r.ambiguity > vc.ambiguity_ratio) {
    std::ostringstream ss;
    ss << "competing peak at " << formatPose(r.second_pose) << " score " << r.second_score
       << " / best " << best << " = " << r.ambiguity << " > " << vc.ambiguity_ratio;
    return done(MatchStatus::AMBIGUOUS, ss.str());
  }
  if (vc.reject_at_window_boundary && r.at_window_boundary && nl > 0) {
    return done(
      MatchStatus::REJECTED,
      "best pose " + formatPose(r.coarse_pose) + " lies on search window boundary; true pose may "
      "be outside the window");
  }
  return done(MatchStatus::ACCEPTED, "ok");
}

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------
InitializationSession::InitializationSession(
  const Pose2D & guess, double start_time, const SessionConfig & session,
  const MatcherConfig & matcher)
: guess_(guess), start_time_(start_time), session_(session), matcher_(matcher)
{
  if (session_.num_scans == 0) {session_.num_scans = 1;}
}

SessionResult InitializationSession::finish(MatchStatus status, const std::string & reason) const
{
  SessionResult out;
  out.status = status;
  out.reason = reason;
  out.matches = matches_;
  out.stale_scans = stale_scans_;
  if (!matches_.empty()) {
    out.final_match = matches_.back();
  }
  out.stamp = last_stamp_;
  out.publish = status == MatchStatus::ACCEPTED;
  return out;
}

std::optional<SessionResult> InitializationSession::addScan(
  const ScanSample & sample, double now, const ScoreField * field)
{
  if (field == nullptr) {
    return finish(MatchStatus::NO_MAP, "no valid map received");
  }
  if (!std::isfinite(sample.stamp) || now - sample.stamp > session_.scan_max_age ||
    sample.stamp - now > session_.scan_max_age)
  {
    ++stale_scans_;
    return std::nullopt;
  }
  if (!matches_.empty() && sample.stamp <= last_stamp_) {
    return std::nullopt;  // scan trùng hoặc lùi thời gian
  }
  if (!sample.sensor_in_base || !sample.odom_pose) {
    last_tf_error_ =
      sample.tf_error.empty() ? std::string("TF unavailable at scan stamp") : sample.tf_error;
    if (++tf_failures_ >= session_.max_tf_failures) {
      return finish(
        MatchStatus::TF_ERROR, last_tf_error_ + " (" + std::to_string(tf_failures_) +
        " consecutive failures)");
    }
    return std::nullopt;
  }
  tf_failures_ = 0;

  const PreprocessResult pre = scanToPoints(sample.scan, *sample.sensor_in_base, matcher_.preprocess);
  MatchResult m = matchScan(*field, pre.points, pre.total_beams, guess_, matcher_);
  last_stamp_ = sample.stamp;
  matches_.push_back(m);
  odom_poses_.push_back(*sample.odom_pose);

  if (m.status != MatchStatus::ACCEPTED) {
    return finish(m.status, m.reason);
  }

  const Pose2D motion = between(odom_poses_.front(), odom_poses_.back());
  if (std::hypot(motion.x, motion.y) > session_.max_motion_linear ||
    std::abs(motion.yaw) > session_.max_motion_angular)
  {
    return finish(MatchStatus::REJECTED, "robot moved during initialization (odom)");
  }

  if (matches_.size() < session_.num_scans) {
    return std::nullopt;
  }

  const Pose2D & ref = matches_.back().pose;
  for (const auto & mm : matches_) {
    const double dl = std::hypot(mm.pose.x - ref.x, mm.pose.y - ref.y);
    const double da = std::abs(normalizeAngle(mm.pose.yaw - ref.yaw));
    if (dl > session_.max_spread_linear || da > session_.max_spread_angular) {
      std::ostringstream ss;
      ss << "inconsistent matches across scans (spread " << dl << " m, " << da << " rad)";
      return finish(MatchStatus::REJECTED, ss.str());
    }
  }
  return finish(MatchStatus::ACCEPTED, "ok");
}

std::optional<SessionResult> InitializationSession::checkTimeout(double now) const
{
  if (now - start_time_ <= session_.timeout) {
    return std::nullopt;
  }
  if (matches_.empty() && !last_tf_error_.empty()) {
    return finish(MatchStatus::TF_ERROR, "timeout; last TF error: " + last_tf_error_);
  }
  if (matches_.empty()) {
    return finish(
      MatchStatus::NO_SCAN, "no fresh scan within timeout (stale scans: " +
      std::to_string(stale_scans_) + ")");
  }
  return finish(
    MatchStatus::TIMEOUT, "only " + std::to_string(matches_.size()) + "/" +
    std::to_string(session_.num_scans) + " scans matched before timeout");
}

}  // namespace stage2_slam
