// csm_benchmark: benchmark offline cho ScanMatchInitializer trên dữ liệu TỔNG HỢP có ground truth.
//
// Không dùng ROS, không chạm phần cứng. Mỗi trial: chọn pose thật, mô phỏng N scan (nhiễu mới mỗi
// scan, nửa sau bị lọc như angular_filter.yaml), tạo guess lệch khỏi pose thật, chạy
// InitializationSession như node thật, so sánh với ground truth.
//
//   scenario=office    : thế giới giải tích (phòng + vách + vật cản, seed khác nhau mỗi trial)
//   scenario=map       : bản đồ map_server (vd. maps/map.yaml); scan dò tia trên chính bản đồ đó
//                        -> lạc quan (map = thế giới), chỉ dùng để thấy độ nhập nhằng của hình học thật.
//
// "Baseline (AMCL không có CSM)" ở đây = sai số của guess mà AMCL sẽ nhận trực tiếp qua /initialpose.
// Hội tụ của AMCL KHÔNG được mô phỏng trong tool này (xem tools/amcl_sim_benchmark.py).
//
// Ví dụ:
//   ros2 run stage2_slam csm_benchmark scenario=office trials=200 seed=1 guess=in csv=/tmp/a.csv
//   ros2 run stage2_slam csm_benchmark scenario=map map=$HOME/capstone_ws/capstone_project/maps/map.yaml

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "stage2_slam/scan_match_initializer.hpp"
#include "stage2_slam/synthetic_world.hpp"

using namespace stage2_slam;  // NOLINT
namespace syn = stage2_slam::synthetic;

namespace
{

struct Args
{
  std::map<std::string, std::string> kv;
  std::string get(const std::string & k, const std::string & d) const
  {
    auto it = kv.find(k);
    return it == kv.end() ? d : it->second;
  }
  double num(const std::string & k, double d) const {return std::stod(get(k, std::to_string(d)));}
};

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {return NAN;}
  std::sort(v.begin(), v.end());
  const double idx = p * (v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(idx));
  const auto hi = static_cast<std::size_t>(std::ceil(idx));
  return v[lo] + (v[hi] - v[lo]) * (idx - lo);
}

double mean(const std::vector<double> & v)
{
  if (v.empty()) {return NAN;}
  double s = 0;
  for (double x : v) {s += x;}
  return s / v.size();
}

double rms(const std::vector<double> & v)
{
  if (v.empty()) {return NAN;}
  double s = 0;
  for (double x : v) {s += x * x;}
  return std::sqrt(s / v.size());
}

void applyOverrides(const Args & a, MatcherConfig & m, SessionConfig & s)
{
  m.validation.min_score = a.num("min_score", m.validation.min_score);
  m.validation.ambiguity_ratio = a.num("ambiguity_ratio", m.validation.ambiguity_ratio);
  m.validation.min_valid_points =
    static_cast<std::size_t>(a.num("min_valid_points", m.validation.min_valid_points));
  m.search.linear_window = a.num("linear_window", m.search.linear_window);
  m.search.angular_window = a.num("angular_window", m.search.angular_window);
  m.search.linear_step = a.num("linear_step", m.search.linear_step);
  m.search.angular_step = a.num("angular_step", m.search.angular_step);
  m.score_field.sigma = a.num("sigma", m.score_field.sigma);
  m.refine.enabled = a.get("refine", m.refine.enabled ? "true" : "false") == "true";
  s.num_scans = static_cast<std::size_t>(a.num("num_scans", s.num_scans));
}

}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string t = argv[i];
    const auto eq = t.find('=');
    if (eq == std::string::npos) {
      std::fprintf(stderr, "argument must be key=value: %s\n", t.c_str());
      return 2;
    }
    a.kv[t.substr(0, eq)] = t.substr(eq + 1);
  }
  const std::string scenario = a.get("scenario", "office");
  const int trials = static_cast<int>(a.num("trials", 100));
  const unsigned seed = static_cast<unsigned>(a.num("seed", 1));
  const std::string guess_mode = a.get("guess", "in");  // in | out | mixed
  const double noise = a.num("noise", 0.01);
  const double dropout = a.num("dropout", 0.02);
  const double range_max = a.num("range_max", 12.0);
  const int clutter = static_cast<int>(a.num("clutter", 0));  // vật cản có trong thế giới nhưng không có trên map
  const double tol_pos = a.num("tol_pos", 0.10);
  const double tol_yaw = a.num("tol_yaw", 2.0 * M_PI / 180.0);
  const std::string csv = a.get("csv", "");

  MatcherConfig mcfg;
  SessionConfig scfg;
  applyOverrides(a, mcfg, scfg);
  const Pose2D sensor{0.15, 0.0, 0.0};  // URDF lidar_joint (x 0.15), laser_joint identity

  std::mt19937 rng(seed);
  GridMap real_map;
  std::optional<ScoreField> real_field;
  if (scenario == "map") {
    std::string err;
    if (!syn::loadMapYaml(a.get("map", ""), real_map, &err)) {
      std::fprintf(stderr, "cannot load map: %s\n", err.c_str());
      return 1;
    }
    real_field = ScoreField::build(real_map, mcfg.score_field, &err);
    if (!real_field) {
      std::fprintf(stderr, "invalid map: %s\n", err.c_str());
      return 1;
    }
  } else if (scenario != "office") {
    std::fprintf(stderr, "unknown scenario %s\n", scenario.c_str());
    return 2;
  }

  std::ofstream out;
  if (!csv.empty()) {
    out.open(csv);
    out << "trial,guess_kind,status,truth_x,truth_y,truth_yaw,guess_err_pos,guess_err_yaw,"
      "est_err_pos,est_err_yaw,score,guess_score,ambiguity,elapsed_s,reason\n";
  }

  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::map<std::string, int> status_count;
  int n_in = 0, n_out = 0, acc_in = 0, ok_in = 0, false_acc = 0, acc_out = 0, n_acc = 0;
  std::vector<double> est_pos, est_yaw, guess_pos, guess_yaw, latency;

  for (int t = 0; t < trials; ++t) {
    // --- Thế giới và map ---
    syn::World world;
    GridMap map_local;
    const GridMap * map = nullptr;
    std::optional<ScoreField> field_local;
    const ScoreField * field = nullptr;
    GridMap truth_grid;  // chỉ dùng cho scenario=map
    if (scenario == "office") {
      world = syn::makeOfficeWorld(seed * 7919u + static_cast<unsigned>(t));
      std::uniform_real_distribution<double> uo(-0.05, 0.0);
      map_local = syn::rasterize(world, 0.05, Pose2D{-3.0 + uo(rng), -3.0 + uo(rng), 0.0}, 26, 26);
      map = &map_local;
      field_local = ScoreField::build(map_local, mcfg.score_field);
      field = &*field_local;
      // Vật cản không có trên map (người, đồ đạc dời chỗ).
      std::uniform_real_distribution<double> ux(0.5, 15.5), uy(0.5, 9.5), us(0.15, 0.5);
      for (int c = 0; c < clutter; ++c) {
        const double x = ux(rng), y = uy(rng), s = us(rng);
        world.obstacles.push_back({x, y, x + s, y + s});
      }
    } else {
      map = &real_map;
      field = &*real_field;
      truth_grid = real_map;
      std::uniform_int_distribution<int> cx(0, real_map.width - 1), cy(0, real_map.height - 1);
      for (int c = 0; c < clutter * 20; ++c) {  // khối 2x2 ô
        const int x = cx(rng), y = cy(rng);
        for (int dy = 0; dy < 2; ++dy) {
          for (int dx = 0; dx < 2; ++dx) {
            const int xx = std::min(x + dx, real_map.width - 1), yy = std::min(y + dy, real_map.height - 1);
            auto & v = truth_grid.data[static_cast<std::size_t>(yy) * real_map.width + xx];
            if (v == 0) {v = 100;}
          }
        }
      }
    }

    Pose2D truth;
    if (!syn::sampleFreePose(*map, 0.4, rng, truth)) {
      std::fprintf(stderr, "cannot sample free pose\n");
      return 1;
    }

    // --- Guess ---
    bool inside = guess_mode == "in" || (guess_mode == "mixed" && u01(rng) < 0.5);
    Pose2D guess = truth;
    if (inside) {
      // Trong cửa sổ (chừa biên 0.1 m / 5 deg)
      const double lw = mcfg.search.linear_window - 0.1;
      const double aw = mcfg.search.angular_window - 5.0 * M_PI / 180.0;
      guess.x += (2 * u01(rng) - 1) * lw;
      guess.y += (2 * u01(rng) - 1) * lw;
      guess.yaw = normalizeAngle(guess.yaw + (2 * u01(rng) - 1) * aw);
    } else {
      // Pose thật nằm ngoài cửa sổ: lệch vị trí 1.5-3x cửa sổ hoặc lệch góc 1.5-3x cửa sổ
      const double k = 1.5 + 1.5 * u01(rng);
      if (u01(rng) < 0.5) {
        const double dir = 2 * M_PI * u01(rng);
        guess.x += k * mcfg.search.linear_window * std::cos(dir);
        guess.y += k * mcfg.search.linear_window * std::sin(dir);
      } else {
        guess.yaw = normalizeAngle(guess.yaw + (u01(rng) < 0.5 ? -1 : 1) * k * mcfg.search.angular_window);
      }
    }
    const double g_pos = std::hypot(guess.x - truth.x, guess.y - truth.y);
    const double g_yaw = std::abs(normalizeAngle(guess.yaw - truth.yaw));

    // --- Session như node ---
    syn::ScanSimConfig sim;
    sim.noise_std = noise;
    sim.dropout = dropout;
    sim.range_max = range_max;
    InitializationSession session(guess, 0.0, scfg, mcfg);
    std::optional<SessionResult> res;
    const auto t0 = std::chrono::steady_clock::now();
    for (std::size_t k = 0; k < scfg.num_scans + 2 && !res; ++k) {
      ScanSample s;
      s.stamp = 0.1 * (k + 1);
      s.scan = scenario == "office" ? syn::simulateScan(world, truth, sensor, sim, rng) :
        syn::simulateScan(truth_grid, truth, sensor, sim, rng);
      s.sensor_in_base = sensor;
      s.odom_pose = Pose2D{};
      res = session.addScan(s, s.stamp + 0.02, field);
    }
    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (!res) {res = session.checkTimeout(1e9);}

    const bool accepted = res->status == MatchStatus::ACCEPTED;
    const Pose2D est = res->final_match.pose;
    const double e_pos = std::hypot(est.x - truth.x, est.y - truth.y);
    const double e_yaw = std::abs(normalizeAngle(est.yaw - truth.yaw));
    const bool correct = e_pos <= tol_pos && e_yaw <= tol_yaw;
    status_count[toString(res->status)]++;
    latency.push_back(dt);
    if (inside) {
      ++n_in;
      guess_pos.push_back(g_pos);
      guess_yaw.push_back(g_yaw);
    } else {
      ++n_out;
    }
    if (accepted) {
      ++n_acc;
      if (!correct) {++false_acc;}
      if (inside) {
        ++acc_in;
        if (correct) {
          ++ok_in;
        }
        est_pos.push_back(e_pos);
        est_yaw.push_back(e_yaw);
      } else {
        ++acc_out;
      }
    }
    if (out) {
      std::string reason = res->reason;
      std::replace(reason.begin(), reason.end(), ',', ';');
      out << t << ',' << (inside ? "in" : "out") << ',' << toString(res->status) << ',' << truth.x
          << ',' << truth.y << ',' << truth.yaw << ',' << g_pos << ',' << g_yaw << ','
          << (accepted ? e_pos : NAN) << ',' << (accepted ? e_yaw : NAN) << ','
          << res->final_match.score << ',' << res->final_match.guess_score << ','
          << res->final_match.ambiguity << ',' << dt << ',' << reason << '\n';
    }
  }

  rusage ru{};
  getrusage(RUSAGE_SELF, &ru);
  std::printf("scenario=%s trials=%d seed=%u guess=%s noise=%.3f dropout=%.2f clutter=%d "
    "min_score=%.2f ambiguity_ratio=%.2f window=+-%.2fm/+-%.1fdeg step=%.3fm num_scans=%zu "
    "refine=%s\n",
    scenario.c_str(), trials, seed, guess_mode.c_str(), noise, dropout, clutter,
    mcfg.validation.min_score, mcfg.validation.ambiguity_ratio, mcfg.search.linear_window,
    mcfg.search.angular_window * 180 / M_PI, mcfg.search.linear_step, scfg.num_scans,
    mcfg.refine.enabled ? "true" : "false");
  std::printf("status:");
  for (const auto & kv : status_count) {std::printf(" %s=%d", kv.first.c_str(), kv.second);}
  std::printf("\n");
  if (n_in > 0) {
    std::printf(
      "in-window : n=%d accepted=%d (%.1f%%) correct(<=%.2fm,<=%.1fdeg)=%d init_success=%.1f%%\n",
      n_in, acc_in, 100.0 * acc_in / n_in, tol_pos, tol_yaw * 180 / M_PI, ok_in,
      100.0 * ok_in / n_in);
    std::printf(
      "  baseline (guess -> AMCL) : pos RMSE %.3f m, yaw RMSE %.2f deg\n", rms(guess_pos),
      rms(guess_yaw) * 180 / M_PI);
    std::printf(
      "  CSM accepted             : pos RMSE %.3f m (p95 %.3f, max %.3f), yaw RMSE %.2f deg "
      "(p95 %.2f)\n", rms(est_pos), percentile(est_pos, 0.95),
      est_pos.empty() ? NAN : *std::max_element(est_pos.begin(), est_pos.end()),
      rms(est_yaw) * 180 / M_PI, percentile(est_yaw, 0.95) * 180 / M_PI);
  }
  if (n_out > 0) {
    std::printf("out-of-window: n=%d accepted=%d (%.1f%%)\n", n_out, acc_out, 100.0 * acc_out / n_out);
  }
  std::printf(
    "false acceptance (accepted but error > tol): %d / %d accepted = %.1f%%\n", false_acc, n_acc,
    n_acc ? 100.0 * false_acc / n_acc : 0.0);
  std::printf(
    "latency per request (%zu scans, CPU of this process): mean %.3f s, p95 %.3f s, max %.3f s; "
    "peak RSS %.1f MB\n", scfg.num_scans, latency.empty() ? NAN :
    mean(latency), percentile(latency, 0.95),
    latency.empty() ? NAN : *std::max_element(latency.begin(), latency.end()),
    ru.ru_maxrss / 1024.0);
  return 0;
}
