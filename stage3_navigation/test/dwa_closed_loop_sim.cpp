// =====================================================================================================
// dwa_closed_loop_sim.cpp — Benchmark kín vòng lõi CustomDWB, KHÔNG cần ROS graph (không node, không topic)
// =====================================================================================================
//
//   ros2 run stage3_navigation dwa_closed_loop_sim --out <dir> [--runs 5] [--scenarios a,b]
//        [--robot nominal|sluggish|laggy] [--set key=value ...] [--trace]
//
// Ghi: <dir>/runs.csv (mỗi lần chạy một dòng), <dir>/summary.json, <dir>/params.yaml (tham số đã dùng),
// <dir>/traces/<scenario>_<seed>.csv (khi --trace). Mô hình + giới hạn: test/sim_world.hpp.
// =====================================================================================================

#include <sys/stat.h>
#include <sys/utsname.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "sim_world.hpp"

using stage3_navigation::DwbParams;
using stage3_navigation::validateParams;
namespace sim = stage3_navigation::sim;

namespace
{
bool setParam(DwbParams & p, const std::string & kv)
{
  const auto eq = kv.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  const std::string k = kv.substr(0, eq);
  const std::string v = kv.substr(eq + 1);
  std::map<std::string, double *> d{
    {"max_vel_x", &p.max_vel_x}, {"max_vel_theta", &p.max_vel_theta},
    {"acc_lim_x", &p.acc_lim_x}, {"decel_lim_x", &p.decel_lim_x},
    {"acc_lim_theta", &p.acc_lim_theta}, {"decel_lim_theta", &p.decel_lim_theta},
    {"sim_time", &p.sim_time}, {"sim_granularity", &p.sim_granularity},
    {"control_latency", &p.control_latency}, {"stopping_margin", &p.stopping_margin},
    {"path_distance_scale", &p.path_distance_scale},
    {"carrot_distance_scale", &p.carrot_distance_scale}, {"heading_scale", &p.heading_scale},
    {"obstacle_scale", &p.obstacle_scale}, {"speed_scale", &p.speed_scale},
    {"oscillation_scale", &p.oscillation_scale}, {"carrot_distance", &p.carrot_distance},
    {"rotate_to_heading_angle", &p.rotate_to_heading_angle},
    {"max_feedback_gap_x", &p.max_feedback_gap_x},
    {"max_feedback_gap_theta", &p.max_feedback_gap_theta}};
  std::map<std::string, int *> i{{"vx_samples", &p.vx_samples},
    {"vtheta_samples", &p.vtheta_samples}};
  if (d.count(k)) {
    *d[k] = std::stod(v);
    return true;
  }
  if (k == "velocity_reference") {
    p.velocity_reference = v;
    return true;
  }
  if (i.count(k)) {
    *i[k] = std::stoi(v);
    return true;
  }
  return false;
}

std::string esc(const std::string & s)
{
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
    }
    o += (c == ',' ? ';' : c);
  }
  return o;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string out_dir = "dwa_sim_out";
  int runs = 5;
  std::string robot_name = "nominal";
  std::vector<std::string> only;
  bool trace = false;
  DwbParams p;
  std::vector<std::string> overrides;
  for (int a = 1; a < argc; ++a) {
    const std::string s = argv[a];
    if (s == "--out" && a + 1 < argc) {
      out_dir = argv[++a];
    } else if (s == "--runs" && a + 1 < argc) {
      runs = std::atoi(argv[++a]);
    } else if (s == "--robot" && a + 1 < argc) {
      robot_name = argv[++a];
    } else if (s == "--scenarios" && a + 1 < argc) {
      std::stringstream ss(argv[++a]);
      std::string item;
      while (std::getline(ss, item, ',')) {
        only.push_back(item);
      }
    } else if (s == "--set" && a + 1 < argc) {
      overrides.push_back(argv[++a]);
      if (!setParam(p, overrides.back())) {
        std::cerr << "unknown --set " << overrides.back() << "\n";
        return 2;
      }
    } else if (s == "--trace") {
      trace = true;
    } else {
      std::cerr <<
        "usage: dwa_closed_loop_sim --out DIR [--runs N] [--robot nominal|sluggish|laggy] "
        "[--scenarios a,b] [--set key=value] [--trace]\n";
      return 2;
    }
  }
  const std::string err = validateParams(p);
  if (!err.empty()) {
    std::cerr << "invalid parameters: " << err << "\n";
    return 2;
  }
  sim::RobotModel robot;
  if (robot_name == "sluggish") {
    // Robot chậm hơn giả định của controller: gia tốc 0.3, trễ 0.2 s (controller vẫn giả định 0.5 / 0.1 s).
    robot.acc_x = robot.decel_x = 0.3;
    robot.acc_theta = robot.decel_theta = 0.6;
    robot.latency = 0.2;
    robot.odom_vel_noise = 0.005;
  } else if (robot_name == "laggy") {
    // Trễ lệnh -> bánh ~0.7 s như ước lượng từ log robot 2026-10-14 (ω tăng ~0.12 rad/s² khi cửa sổ tính từ
    // vận tốc đo). Gia tốc giữ như giả định.
    robot.latency = 0.7;
    robot.odom_vel_noise = 0.005;
  } else if (robot_name == "nominal") {
    robot.odom_vel_noise = 0.002;
  } else {
    std::cerr << "unknown robot model " << robot_name << "\n";
    return 2;
  }

  ::mkdir(out_dir.c_str(), 0755);
  if (trace) {
    ::mkdir((out_dir + "/traces").c_str(), 0755);
  }
  {
    std::ofstream y(out_dir + "/params.yaml");
    y <<
      "# CustomDWB parameters used by dwa_closed_loop_sim (defaults from dwa_core.hpp + overrides)\n";
    y << "robot_model: " << robot_name << "\n";
    y << "robot: {acc_x: " << robot.acc_x << ", decel_x: " << robot.decel_x << ", acc_theta: " <<
      robot.acc_theta << ", decel_theta: " << robot.decel_theta << ", latency: " << robot.latency <<
      ", odom_vel_noise: " << robot.odom_vel_noise << ", radius: " << robot.radius << "}\n";
    y << "overrides: [";
    for (std::size_t k = 0; k < overrides.size(); ++k) {
      y << (k ? ", " : "") << "\"" << overrides[k] << "\"";
    }
    y << "]\n";
    y << "max_vel_x: " << p.max_vel_x << "\nmax_vel_theta: " << p.max_vel_theta <<
      "\nacc_lim_x: " <<
      p.acc_lim_x << "\ndecel_lim_x: " << p.decel_lim_x << "\nacc_lim_theta: " << p.acc_lim_theta <<
      "\ndecel_lim_theta: " << p.decel_lim_theta << "\nvx_samples: " << p.vx_samples <<
      "\nvtheta_samples: " << p.vtheta_samples << "\nsim_time: " << p.sim_time <<
      "\nsim_granularity: " << p.sim_granularity << "\ncontrol_latency: " << p.control_latency <<
      "\nstopping_margin: " << p.stopping_margin << "\npath_distance_scale: " <<
      p.path_distance_scale << "\ncarrot_distance_scale: " << p.carrot_distance_scale <<
      "\nheading_scale: " << p.heading_scale << "\nobstacle_scale: " << p.obstacle_scale <<
      "\nspeed_scale: " << p.speed_scale << "\noscillation_scale: " << p.oscillation_scale << "\n";
  }

  std::ofstream csv(out_dir + "/runs.csv");
  csv << "scenario,robot,seed,expect_success,success,result,time_s,collisions,min_clearance_m,"
    "cross_track_rms_m,cross_track_max_m,w_sign_flips,limit_violations,nonfinite_cmds,cmd_rate_exceed,"
    "blocked_cycles,final_xy_error_m,final_yaw_error_rad,stop_latency_s,stop_distance_m,path_length_m,"
    "n_cycles,compute_p50_ms,compute_p95_ms,compute_p99_ms,compute_max_ms,missed_deadlines\n";
  std::ostringstream js;
  struct utsname un {};
  ::uname(&un);
  js << "{\n  \"tool\": \"dwa_closed_loop_sim\",\n  \"robot_model\": \"" << robot_name <<
    "\",\n  \"machine\": \"" << un.nodename << " " << un.machine << " " << un.release <<
    "\",\n  \"runs_per_scenario\": " << runs << ",\n  \"scenarios\": [\n";
  bool first = true;
  int bad = 0;
  for (const auto & sc : sim::standardScenarios()) {
    if (!only.empty() && std::find(only.begin(), only.end(), sc.name) == only.end()) {
      continue;
    }
    int ok = 0, coll = 0;
    double min_clear = std::numeric_limits<double>::infinity(), t_sum = 0.0;
    std::vector<double> all_ms;
    for (int r = 0; r < runs; ++r) {
      const std::uint32_t seed = 1000u + static_cast<std::uint32_t>(r);
      std::vector<sim::TraceRow> tr;
      const auto m =
        sim::runScenario(p, sc, robot, seed, sim::CostmapSpec{}, trace ? &tr : nullptr);
      ok += m.success;
      coll += m.collisions > 0;
      min_clear = std::min(min_clear, m.min_clearance);
      t_sum += m.time_s;
      all_ms.insert(all_ms.end(), m.compute_ms.begin(), m.compute_ms.end());
      if (m.collisions > 0 || m.limit_violations > 0 || m.nonfinite_cmds > 0 ||
        (sc.expect_success && !m.success))
      {
        ++bad;
      }
      char buf[1024];
      std::snprintf(
        buf, sizeof(buf),
        "%s,%s,%u,%d,%d,%s,%.2f,%d,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%.4f,%.4f,%.3f,%.4f,%.3f,%d,%.3f,%.3f,"
        "%.3f,%.3f,%d\n",
        sc.name.c_str(), robot_name.c_str(), seed, sc.expect_success, m.success,
        esc(m.result).c_str(), m.time_s, m.collisions, m.min_clearance, m.cross_track_rms,
        m.cross_track_max, m.w_sign_flips, m.limit_violations, m.nonfinite_cmds, m.cmd_rate_exceed,
        m.blocked_cycles, m.final_xy_error, m.final_yaw_error, m.stop_latency, m.stop_distance,
        m.path_length, m.n_cycles, sim::percentile(m.compute_ms, 0.5),
        sim::percentile(m.compute_ms, 0.95), sim::percentile(m.compute_ms, 0.99),
        sim::percentile(m.compute_ms, 1.0), m.missed_deadlines);
      csv << buf;
      if (trace) {
        std::ofstream tf(out_dir + "/traces/" + sc.name + "_" + std::to_string(seed) + ".csv");
        tf << "t,x,y,theta,v,w,v_cmd,w_cmd,phase\n";
        for (const auto & row : tr) {
          tf << row.t << "," << row.x << "," << row.y << "," << row.theta << "," << row.v << "," <<
            row.w << "," << row.v_cmd << "," << row.w_cmd << "," << row.phase << "\n";
        }
      }
    }
    js << (first ? "" : ",\n") << "    {\"name\": \"" << sc.name << "\", \"expect_success\": " <<
      (sc.expect_success ? "true" : "false") << ", \"success\": " << ok << ", \"runs\": " << runs <<
      ", \"runs_with_collision\": " << coll << ", \"min_clearance_m\": " <<
      (std::isfinite(min_clear) ? std::to_string(min_clear) : std::string("null")) <<
      ", \"mean_time_s\": " << t_sum / runs << ", \"compute_p50_ms\": " <<
      sim::percentile(all_ms, 0.5) << ", \"compute_p99_ms\": " << sim::percentile(all_ms, 0.99) <<
      ", \"compute_max_ms\": " << sim::percentile(all_ms, 1.0) << "}";
    first = false;
    std::printf(
      "%-20s success %d/%d  collision-runs %d  min clearance %.3f m  p99 %.2f ms\n",
      sc.name.c_str(),
      ok, runs, coll, min_clear, sim::percentile(all_ms, 0.99));
  }
  js << "\n  ],\n  \"runs_violating_expectations\": " << bad << "\n}\n";
  std::ofstream(out_dir + "/summary.json") << js.str();
  std::printf(
    "runs violating expectations (collision / limit / non-finite / expected success): %d\n", bad);
  return bad == 0 ? 0 : 1;
}
