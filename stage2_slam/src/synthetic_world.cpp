#include "stage2_slam/synthetic_world.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace stage2_slam::synthetic
{

namespace
{

void cellCenterWorld(const GridMap & map, int cx, int cy, double & x, double & y)
{
  const double lx = (cx + 0.5) * map.resolution;
  const double ly = (cy + 0.5) * map.resolution;
  const double c = std::cos(map.origin.yaw);
  const double s = std::sin(map.origin.yaw);
  x = map.origin.x + c * lx - s * ly;
  y = map.origin.y + s * lx + c * ly;
}

void worldToGridF(const GridMap & map, double x, double y, double & gx, double & gy)
{
  const double dx = x - map.origin.x;
  const double dy = y - map.origin.y;
  const double c = std::cos(map.origin.yaw);
  const double s = std::sin(map.origin.yaw);
  gx = (c * dx + s * dy) / map.resolution;
  gy = (-s * dx + c * dy) / map.resolution;
}

bool worldToCell(const GridMap & map, double x, double y, int & cx, int & cy)
{
  double gx, gy;
  worldToGridF(map, x, y, gx, gy);
  cx = static_cast<int>(std::floor(gx));
  cy = static_cast<int>(std::floor(gy));
  return cx >= 0 && cy >= 0 && cx < map.width && cy < map.height;
}

// Ô (hình vuông có thể xoay) và hình chữ nhật trục thẳng có chồng lấn diện tích > 0? (SAT)
bool cellOverlapsRect(const GridMap & map, int cx, int cy, const Rect & r)
{
  const double c = std::cos(map.origin.yaw);
  const double s = std::sin(map.origin.yaw);
  double px[4], py[4];
  const double lx[4] = {0, 1, 1, 0}, ly[4] = {0, 0, 1, 1};
  for (int i = 0; i < 4; ++i) {
    const double ax = (cx + lx[i]) * map.resolution;
    const double ay = (cy + ly[i]) * map.resolution;
    px[i] = map.origin.x + c * ax - s * ay;
    py[i] = map.origin.y + s * ax + c * ay;
  }
  constexpr double eps = 1e-9;
  // Trục thế giới
  if (*std::max_element(px, px + 4) <= r.x0 + eps || *std::min_element(px, px + 4) >= r.x1 - eps) {
    return false;
  }
  if (*std::max_element(py, py + 4) <= r.y0 + eps || *std::min_element(py, py + 4) >= r.y1 - eps) {
    return false;
  }
  // Trục của ô
  const double rx[4] = {r.x0, r.x1, r.x1, r.x0}, ry[4] = {r.y0, r.y0, r.y1, r.y1};
  const double axes[2][2] = {{c, s}, {-s, c}};
  for (const auto & a : axes) {
    double cmin = 1e18, cmax = -1e18, rmin = 1e18, rmax = -1e18;
    for (int i = 0; i < 4; ++i) {
      const double pc = px[i] * a[0] + py[i] * a[1];
      const double pr = rx[i] * a[0] + ry[i] * a[1];
      cmin = std::min(cmin, pc);
      cmax = std::max(cmax, pc);
      rmin = std::min(rmin, pr);
      rmax = std::max(rmax, pr);
    }
    if (cmax <= rmin + eps || cmin >= rmax - eps) {return false;}
  }
  return true;
}

void addBox(World & w, double x0, double y0, double x1, double y1, double t)
{
  w.obstacles.push_back({x0, y0, x1, y0 + t});
  w.obstacles.push_back({x0, y1 - t, x1, y1});
  w.obstacles.push_back({x0, y0, x0 + t, y1});
  w.obstacles.push_back({x1 - t, y0, x1, y1});
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\"'");
  const auto e = s.find_last_not_of(" \t\r\"'");
  return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

}  // namespace

World makeOfficeWorld(unsigned seed)
{
  World w;
  w.known_area = {0.0, 0.0, 16.0, 10.0};
  addBox(w, 0.0, 0.0, 16.0, 10.0, 0.10);
  // Vách ngăn với cửa.
  w.obstacles.push_back({6.0, 0.0, 6.1, 4.0});
  w.obstacles.push_back({6.0, 5.0, 6.1, 10.0});
  w.obstacles.push_back({6.1, 6.5, 9.5, 6.6});
  w.obstacles.push_back({10.5, 6.5, 16.0, 6.6});
  w.obstacles.push_back({11.0, 0.0, 11.1, 2.5});
  // Vật cản ngẫu nhiên (bàn, tủ).
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(0.6, 15.0), uy(0.6, 9.0), us(0.2, 0.8);
  for (int i = 0; i < 14; ++i) {
    const double x = ux(rng), y = uy(rng), sx = us(rng), sy = us(rng);
    w.obstacles.push_back({x, y, x + sx, y + sy});
  }
  return w;
}

World makeCorridorWorld()
{
  World w;
  w.known_area = {0.0, 0.5, 60.0, 3.5};
  w.obstacles.push_back({0.0, 0.5, 60.0, 0.6});
  w.obstacles.push_back({0.0, 3.4, 60.0, 3.5});
  w.obstacles.push_back({0.0, 0.5, 0.1, 3.5});
  w.obstacles.push_back({59.9, 0.5, 60.0, 3.5});
  return w;
}

World makePeriodicCorridorWorld(double period)
{
  World w = makeCorridorWorld();
  for (double x = 0.5; x < 59.5; x += period) {
    w.obstacles.push_back({x, 0.6, x + 0.15, 0.9});
    w.obstacles.push_back({x, 3.1, x + 0.15, 3.4});
  }
  return w;
}

GridMap makeMap(double size_x, double size_y, double resolution, const Pose2D & origin)
{
  GridMap map;
  map.resolution = resolution;
  map.width = static_cast<int>(std::round(size_x / resolution));
  map.height = static_cast<int>(std::round(size_y / resolution));
  map.origin = origin;
  map.data.assign(static_cast<std::size_t>(map.width) * map.height, 0);
  return map;
}

GridMap rasterize(
  const World & world, double resolution, const Pose2D & origin, double size_x, double size_y)
{
  GridMap map = makeMap(size_x, size_y, resolution, origin);
  const Rect & k = world.known_area;
  for (int cy = 0; cy < map.height; ++cy) {
    for (int cx = 0; cx < map.width; ++cx) {
      double x, y;
      cellCenterWorld(map, cx, cy, x, y);
      const bool known = x >= k.x0 && x <= k.x1 && y >= k.y0 && y <= k.y1;
      map.data[static_cast<std::size_t>(cy) * map.width + cx] = known ? 0 : -1;
    }
  }
  for (const auto & r : world.obstacles) {
    double gmin_x = 1e18, gmin_y = 1e18, gmax_x = -1e18, gmax_y = -1e18;
    const double rx[4] = {r.x0, r.x1, r.x1, r.x0}, ry[4] = {r.y0, r.y0, r.y1, r.y1};
    for (int i = 0; i < 4; ++i) {
      double gx, gy;
      worldToGridF(map, rx[i], ry[i], gx, gy);
      gmin_x = std::min(gmin_x, gx);
      gmin_y = std::min(gmin_y, gy);
      gmax_x = std::max(gmax_x, gx);
      gmax_y = std::max(gmax_y, gy);
    }
    const int x0 = std::max(0, static_cast<int>(std::floor(gmin_x)) - 1);
    const int y0 = std::max(0, static_cast<int>(std::floor(gmin_y)) - 1);
    const int x1 = std::min(map.width - 1, static_cast<int>(std::ceil(gmax_x)) + 1);
    const int y1 = std::min(map.height - 1, static_cast<int>(std::ceil(gmax_y)) + 1);
    for (int cy = y0; cy <= y1; ++cy) {
      for (int cx = x0; cx <= x1; ++cx) {
        if (cellOverlapsRect(map, cx, cy, r)) {
          map.data[static_cast<std::size_t>(cy) * map.width + cx] = 100;
        }
      }
    }
  }
  return map;
}

bool loadMapYaml(const std::string & yaml_path, GridMap & map, std::string * error)
{
  auto fail = [error](const std::string & m) {
      if (error) {*error = m;}
      return false;
    };
  std::ifstream yf(yaml_path);
  if (!yf) {return fail("cannot open " + yaml_path);}
  std::string image;
  double res = 0.0, occ = 0.65, fr = 0.25;
  int negate = 0;
  std::vector<double> org;
  std::string line;
  while (std::getline(yf, line)) {
    const auto pos = line.find(':');
    if (pos == std::string::npos || line[0] == '#') {continue;}
    const std::string key = trim(line.substr(0, pos));
    std::string val = trim(line.substr(pos + 1));
    if (key == "image") {
      image = val;
    } else if (key == "resolution") {
      res = std::stod(val);
    } else if (key == "occupied_thresh") {
      occ = std::stod(val);
    } else if (key == "free_thresh") {
      fr = std::stod(val);
    } else if (key == "negate") {
      negate = std::stoi(val);
    } else if (key == "mode" && val != "trinary") {
      return fail("only mode: trinary supported");
    } else if (key == "origin") {
      std::replace(val.begin(), val.end(), '[', ' ');
      std::replace(val.begin(), val.end(), ']', ' ');
      std::replace(val.begin(), val.end(), ',', ' ');
      std::istringstream ss(val);
      double d;
      while (ss >> d) {org.push_back(d);}
    }
  }
  if (image.empty() || res <= 0.0 || org.size() != 3) {return fail("incomplete map yaml");}
  if (image[0] != '/') {
    const auto slash = yaml_path.find_last_of('/');
    image = (slash == std::string::npos ? std::string() : yaml_path.substr(0, slash + 1)) + image;
  }
  std::ifstream pf(image, std::ios::binary);
  if (!pf) {return fail("cannot open " + image);}
  std::string magic;
  pf >> magic;
  if (magic != "P5") {return fail("only binary PGM (P5) supported");}
  int vals[3];
  for (int i = 0; i < 3; ) {
    pf >> std::ws;
    if (pf.peek() == '#') {
      std::string c;
      std::getline(pf, c);
      continue;
    }
    pf >> vals[i++];
  }
  pf.get();
  const int w = vals[0], h = vals[1];
  if (vals[2] > 255) {return fail("16-bit PGM not supported");}
  std::vector<unsigned char> px(static_cast<std::size_t>(w) * h);
  pf.read(reinterpret_cast<char *>(px.data()), static_cast<std::streamsize>(px.size()));
  if (!pf) {return fail("truncated PGM");}

  map.width = w;
  map.height = h;
  map.resolution = res;
  map.origin = {org[0], org[1], org[2]};
  map.data.assign(px.size(), -1);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const double v = px[static_cast<std::size_t>(y) * w + x];
      const double p = negate ? v / 255.0 : (255.0 - v) / 255.0;
      int8_t out = -1;
      if (p > occ) {
        out = 100;
      } else if (p < fr) {
        out = 0;
      }
      // Ảnh: hàng 0 ở trên; OccupancyGrid: hàng 0 ở dưới (như nav2_map_server).
      map.data[static_cast<std::size_t>(h - 1 - y) * w + x] = out;
    }
  }
  return true;
}

namespace
{

template<typename CastFn>
ScanData simulate(
  const Pose2D & robot_pose, const Pose2D & sensor_in_base, const ScanSimConfig & config,
  std::mt19937 & rng, CastFn cast)
{
  ScanData scan;
  scan.angle_min = config.angle_min;
  scan.angle_increment = config.angle_increment;
  scan.range_min = config.range_min;
  scan.range_max = config.range_max;
  scan.ranges.assign(config.beams, std::numeric_limits<float>::infinity());
  std::normal_distribution<double> noise(0.0, config.noise_std);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  const Pose2D sensor = compose(robot_pose, sensor_in_base);
  for (int i = 0; i < config.beams; ++i) {
    const double a = config.angle_min + i * config.angle_increment;
    if (config.blank_rear) {
      const double an = std::fmod(std::fmod(a, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
      if (an > config.blank_min && an < config.blank_max) {
        scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
    }
    if (config.dropout > 0.0 && u01(rng) < config.dropout) {
      scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    const double wa = sensor.yaw + a;
    const double r = cast(sensor.x, sensor.y, std::cos(wa), std::sin(wa));
    if (std::isfinite(r) && r >= config.range_min && r <= config.range_max) {
      scan.ranges[i] = static_cast<float>(std::max(config.range_min, r + noise(rng)));
    }
  }
  return scan;
}

}  // namespace

ScanData simulateScan(
  const World & world, const Pose2D & robot_pose, const Pose2D & sensor_in_base,
  const ScanSimConfig & config, std::mt19937 & rng)
{
  return simulate(
    robot_pose, sensor_in_base, config, rng,
    [&world](double ox, double oy, double dx, double dy) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto & r : world.obstacles) {
        // Slab method.
        double tmin = -std::numeric_limits<double>::infinity();
        double tmax = std::numeric_limits<double>::infinity();
        const double o[2] = {ox, oy}, d[2] = {dx, dy};
        const double lo[2] = {r.x0, r.y0}, hi[2] = {r.x1, r.y1};
        bool miss = false;
        for (int k = 0; k < 2; ++k) {
          if (std::abs(d[k]) < 1e-12) {
            if (o[k] < lo[k] || o[k] > hi[k]) {miss = true; break;}
            continue;
          }
          double t1 = (lo[k] - o[k]) / d[k];
          double t2 = (hi[k] - o[k]) / d[k];
          if (t1 > t2) {std::swap(t1, t2);}
          tmin = std::max(tmin, t1);
          tmax = std::min(tmax, t2);
        }
        if (miss || tmax < tmin || tmax <= 0.0) {continue;}
        const double t = tmin > 0.0 ? tmin : 0.0;
        best = std::min(best, t);
      }
      return best;
    });
}

ScanData simulateScan(
  const GridMap & map, const Pose2D & robot_pose, const Pose2D & sensor_in_base,
  const ScanSimConfig & config, std::mt19937 & rng)
{
  const double step = map.resolution * 0.1;
  return simulate(
    robot_pose, sensor_in_base, config, rng,
    [&map, &config, step](double ox, double oy, double dx, double dy) {
      for (double r = config.range_min; r <= config.range_max; r += step) {
        int cx, cy;
        if (!worldToCell(map, ox + r * dx, oy + r * dy, cx, cy)) {break;}
        if (map.data[static_cast<std::size_t>(cy) * map.width + cx] >= 65) {
          // Điểm chạm nằm bên trong ô chiếm chỗ (như điểm chạm mà SLAM đã ghi vào ô đó).
          return r + 0.5 * map.resolution;
        }
      }
      return std::numeric_limits<double>::infinity();
    });
}

bool sampleFreePose(
  const GridMap & map, double clearance, std::mt19937 & rng, Pose2D & pose, int max_tries)
{
  std::uniform_int_distribution<int> ux(0, map.width - 1), uy(0, map.height - 1);
  std::uniform_real_distribution<double> ua(-M_PI, M_PI);
  const int rc = static_cast<int>(std::ceil(clearance / map.resolution));
  for (int t = 0; t < max_tries; ++t) {
    const int cx = ux(rng), cy = uy(rng);
    bool ok = true;
    for (int dy = -rc; dy <= rc && ok; ++dy) {
      for (int dx = -rc; dx <= rc; ++dx) {
        const int x = cx + dx, y = cy + dy;
        if (x < 0 || y < 0 || x >= map.width || y >= map.height ||
          map.data[static_cast<std::size_t>(y) * map.width + x] != 0)
        {
          ok = false;
          break;
        }
      }
    }
    if (!ok) {continue;}
    cellCenterWorld(map, cx, cy, pose.x, pose.y);
    pose.yaw = ua(rng);
    return true;
  }
  return false;
}

}  // namespace stage2_slam::synthetic
