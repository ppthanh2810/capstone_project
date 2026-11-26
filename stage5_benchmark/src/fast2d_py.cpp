// fast2d_py.cpp — Python bindings (pybind11) cho Fast 2D simulator: generator / evaluator / benchmark / SAC env.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "stage5_benchmark/fast2d.hpp"

namespace py = pybind11;
namespace s3 = stage3_navigation;
using stage5_benchmark::Episode;
using stage5_benchmark::Grid;
using stage5_benchmark::Shape;

namespace
{

Grid toGrid(const py::dict & m)
{
  Grid g;
  auto origin = m["origin"].cast<std::vector<double>>();
  g.origin_x = origin.at(0);
  g.origin_y = origin.at(1);
  g.resolution = m["resolution"].cast<double>();
  auto arr = py::array_t<std::int8_t, py::array::c_style | py::array::forcecast>(m["data"]);
  if (arr.ndim() != 2) {
    throw std::invalid_argument("map data must be 2D (height, width), row 0 = y min");
  }
  g.height = static_cast<int>(arr.shape(0));
  g.width = static_cast<int>(arr.shape(1));
  g.data.assign(arr.data(), arr.data() + arr.size());
  return g;
}

Shape toShape(const py::dict & d)
{
  Shape s;
  const auto type = d["type"].cast<std::string>();
  s.x = d["x"].cast<double>();
  s.y = d["y"].cast<double>();
  if (type == "circle") {
    s.type = Shape::Type::CIRCLE;
    s.r = d["r"].cast<double>();
  } else if (type == "box") {
    s.type = Shape::Type::BOX;
    s.sx = d["sx"].cast<double>();
    s.sy = d["sy"].cast<double>();
    s.yaw = d.contains("yaw") ? d["yaw"].cast<double>() : 0.0;
  } else {
    throw std::invalid_argument("obstacle type must be box | circle");
  }
  return s;
}

s3::Pose2D toPose(const py::handle & h)
{
  auto v = h.cast<std::vector<double>>();
  return s3::Pose2D{v.at(0), v.at(1), v.size() > 2 ? v[2] : 0.0};
}

void applyDwb(s3::DwbParams & p, const py::dict & d)
{
  for (const auto & kv : d) {
    const auto name = kv.first.cast<std::string>();
    std::string bad;
    if (py::isinstance<py::str>(kv.second)) {
      bad = stage5_benchmark::setDwbField(p, name, 0.0, kv.second.cast<std::string>());
    } else {
      bad = stage5_benchmark::setDwbField(p, name, kv.second.cast<double>());
    }
    if (!bad.empty()) {
      throw std::invalid_argument("unknown CustomDWB parameter: " + bad);
    }
  }
}

void applyReplan(s3::ReplanParams & p, const py::dict & d)
{
  for (const auto & kv : d) {
    const auto name = kv.first.cast<std::string>();
    std::string bad;
    if (py::isinstance<py::str>(kv.second)) {
      bad = stage5_benchmark::setReplanField(p, name, 0.0, kv.second.cast<std::string>());
    } else {
      bad = stage5_benchmark::setReplanField(p, name, kv.second.cast<double>());
    }
    if (!bad.empty()) {
      throw std::invalid_argument("unknown replan parameter: " + bad);
    }
  }
}

stage5_benchmark::SimConfig toSim(const py::dict & d)
{
  stage5_benchmark::SimConfig c;
  for (const auto & kv : d) {
    const auto bad = stage5_benchmark::setSimField(c, kv.first.cast<std::string>(), kv.second.cast<double>());
    if (!bad.empty()) {
      throw std::invalid_argument("unknown sim parameter: " + bad);
    }
  }
  return c;
}

py::array_t<std::uint8_t> costmapArray(const nav2_costmap_2d::Costmap2D & cm)
{
  const auto nx = cm.getSizeInCellsX(), ny = cm.getSizeInCellsY();
  py::array_t<std::uint8_t> a({static_cast<py::ssize_t>(ny), static_cast<py::ssize_t>(nx)});
  std::memcpy(a.mutable_data(), cm.getCharMap(), static_cast<std::size_t>(nx) * ny);
  return a;
}

py::dict controlDict(const s3::ControlOutput & o)
{
  py::dict d;
  d["ok"] = o.ok;
  d["error"] = o.error;
  d["phase"] = std::string(s3::phaseName(o.phase));
  d["v"] = o.cmd.v;
  d["w"] = o.cmd.w;
  d["carrot"] = std::vector<double>{o.carrot.x, o.carrot.y, o.carrot.theta};
  d["carrot_shift"] = o.carrot_shift;
  d["fs_clearance"] = o.free_space.clearance;
  d["fs_forward_free"] = o.free_space.forward_free;
  d["fs_arcs"] = o.free_space.arcs;
  d["carrot_blocked"] = o.carrot_blocked;
  d["n_candidates"] = o.n_candidates;
  d["n_valid"] = o.n_valid;
  d["n_forward_valid"] = o.n_forward_valid;
  d["n_reject"] = std::vector<int>(o.n_reject, o.n_reject + 5);
  d["dist_to_goal"] = o.dist_to_goal;
  d["heading_error"] = o.heading_error;
  d["cost_total"] = o.chosen.total_cost;
  d["cost_obstacle"] = o.chosen.obstacle_cost;
  d["cost_path"] = o.chosen.path_cost;
  d["cost_carrot"] = o.chosen.carrot_cost;
  d["cost_heading"] = o.chosen.heading_cost;
  d["cost_speed"] = o.chosen.speed_cost;
  return d;
}

py::dict resultDict(const stage5_benchmark::EpisodeResult & r)
{
  py::dict d;
  d["terminal_state"] = r.terminal_state;
  d["t_terminal"] = r.t_terminal;
  d["t_end"] = r.t_end;
  d["collision"] = r.collision;
  d["t_collision"] = r.t_collision;
  d["min_clearance"] = r.min_clearance;
  d["stuck_early_stop"] = r.stuck_early_stop;
  d["timeout"] = r.timeout;
  d["error"] = r.error;
  d["replans"] = r.replans;
  d["obstacle_replans"] = r.obstacle_replans;
  d["abort_replans"] = r.abort_replans;
  d["max_goal_stagnation"] = r.max_goal_stagnation;
  d["path_length"] = r.path_length;
  d["initial_path_length"] = r.initial_path_length;
  d["plan_lengths"] = r.plan_lengths;
  d["controller_cycles"] = r.controller_cycles;
  d["cycles_no_valid"] = r.cycles_no_valid;
  d["cycles_holding"] = r.cycles_holding;
  d["rejects"] = py::dict(
    py::arg("none") = r.rejects[0], py::arg("collision") = r.rejects[1], py::arg("unknown") = r.rejects[2],
    py::arg("off_map") = r.rejects[3], py::arg("braking") = r.rejects[4]);
  d["candidates"] = r.candidates;
  d["compute_ms_mean"] = r.compute_ms_mean;
  d["compute_ms_p99"] = r.compute_ms_p99;
  d["compute_ms_max"] = r.compute_ms_max;
  d["missed_deadlines"] = r.missed_deadlines;
  d["stale_cycles"] = r.stale_cycles;
  d["final_pose"] = std::vector<double>{r.final_x, r.final_y, r.final_yaw};
  const auto n = static_cast<py::ssize_t>(r.trace.size());
  constexpr int NC = 24;
  py::array_t<double> tr({n, static_cast<py::ssize_t>(NC)});
  auto m = tr.mutable_unchecked<2>();
  for (py::ssize_t i = 0; i < n; ++i) {
    const auto & q = r.trace[static_cast<std::size_t>(i)];
    const double row[NC] = {q.t, q.x, q.y, q.yaw, q.v, q.w, q.v_cmd, q.w_cmd, q.clearance,
      static_cast<double>(q.phase), static_cast<double>(q.n_valid), static_cast<double>(q.n_forward),
      static_cast<double>(q.n_offmap), static_cast<double>(q.n_collision), static_cast<double>(q.n_unknown),
      q.compute_ms, q.carrot_shift, q.fs_clearance, q.fs_free_ratio, static_cast<double>(q.sup_state),
      q.block_dist, static_cast<double>(q.local_bad), static_cast<double>(q.n_candidates), q.deviation};
    for (int c = 0; c < NC; ++c) {m(i, c) = row[c];}
  }
  d["trace"] = tr;
  d["trace_columns"] = std::vector<std::string>{"t", "x", "y", "yaw", "v", "w", "v_cmd", "w_cmd", "clearance",
    "phase", "n_valid", "n_forward", "n_offmap", "n_collision", "n_unknown", "compute_ms", "carrot_shift",
    "fs_clearance", "fs_free_ratio", "sup_state", "block_dist", "local_bad", "n_candidates", "deviation"};
  py::list plans;
  for (const auto & pl : r.plans) {
    py::array_t<double> a({static_cast<py::ssize_t>(pl.second.size()), static_cast<py::ssize_t>(2)});
    auto am = a.mutable_unchecked<2>();
    for (std::size_t k = 0; k < pl.second.size(); ++k) {
      am(static_cast<py::ssize_t>(k), 0) = pl.second[k].x;
      am(static_cast<py::ssize_t>(k), 1) = pl.second[k].y;
    }
    plans.append(py::make_tuple(pl.first, a));
  }
  d["plans"] = plans;
  py::list ev;
  for (const auto & e : r.events) {
    ev.append(py::make_tuple(e.t, e.text));
  }
  d["events"] = ev;
  return d;
}

}  // namespace

PYBIND11_MODULE(fast2d, m)
{
  m.doc() = "Fast 2D simulator (stage5_benchmark) dùng chính DwaCore + ReplanSupervisor + NavFn + ZlacDriverModel";

  m.def("validate_dwb", [](const py::dict & d) {
      s3::DwbParams p;
      applyDwb(p, d);
      return s3::validateParams(p);
    }, "'' = hợp lệ (cùng validateParams của plugin), ngược lại lý do");
  m.def("dwb_defaults", []() {
      // giá trị mặc định của DwbParams (code); YAML robot ghi đè qua benchmark.yaml
      s3::DwbParams p;
      py::dict d;
      d["sim_time"] = p.sim_time;
      return d;
    });
  m.def("dwb_field_names", &stage5_benchmark::dwbFieldNames);
  m.def("nav2_inscribed_radius", &stage5_benchmark::nav2InscribedRadius);
  m.def("static_costmap", [](const py::dict & map, double radius, double padding, double inflation, double factor) {
      auto cm = stage5_benchmark::staticCostmap(toGrid(map), radius, padding, inflation, factor);
      return costmapArray(*cm);
    }, "costmap static + inflation (uint8 0..255, như global_costmap lúc chưa quan sát)");
  m.def("navfn_plan", [](const py::dict & map, py::object start, py::object goal, double radius, double padding,
    double inflation, double factor, bool allow_unknown) {
      auto cm = stage5_benchmark::staticCostmap(toGrid(map), radius, padding, inflation, factor);
      const auto plan = stage5_benchmark::navfnPlan(*cm, toPose(start), toPose(goal), allow_unknown);
      py::array_t<double> a({static_cast<py::ssize_t>(plan.size()), static_cast<py::ssize_t>(2)});
      auto mm = a.mutable_unchecked<2>();
      for (std::size_t i = 0; i < plan.size(); ++i) {
        mm(i, 0) = plan[i].x;
        mm(i, 1) = plan[i].y;
      }
      return a;
    }, "A* NavFn (như planner_server) trên static costmap của map; rỗng = không có đường");
  m.def("navfn_plan_costmap", [](py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast> cost,
    std::vector<double> origin, double res, py::object start, py::object goal, bool allow_unknown) {
      const auto ny = static_cast<unsigned>(cost.shape(0)), nx = static_cast<unsigned>(cost.shape(1));
      nav2_costmap_2d::Costmap2D cm(nx, ny, res, origin.at(0), origin.at(1), 0);
      std::memcpy(cm.getCharMap(), cost.data(), static_cast<std::size_t>(nx) * ny);
      const auto plan = stage5_benchmark::navfnPlan(cm, toPose(start), toPose(goal), allow_unknown);
      py::array_t<double> a({static_cast<py::ssize_t>(plan.size()), static_cast<py::ssize_t>(2)});
      auto mm = a.mutable_unchecked<2>();
      for (std::size_t i = 0; i < plan.size(); ++i) {
        mm(i, 0) = plan[i].x;
        mm(i, 1) = plan[i].y;
      }
      return a;
    }, "A* NavFn trên một costmap uint8 cho trước (vd. W đã inflate để kiểm tra khả thi)");

  // World GT dùng chung cho evaluator Gazebo (V4: cùng hàm khoảng hở với Fast 2D, không dùng EDT lượng tử ô).
  py::class_<stage5_benchmark::World>(m, "World")
  .def(py::init([](const py::dict & scenario) {
      std::vector<Shape> extra;
      for (const auto & o : scenario["obstacles"].cast<py::list>()) {
        extra.push_back(toShape(o.cast<py::dict>()));
      }
      return std::make_unique<stage5_benchmark::World>(toGrid(scenario["map"].cast<py::dict>()), extra);
    }), py::arg("scenario"))
  .def("clearance", &stage5_benchmark::World::clearance, py::arg("x"), py::arg("y"), py::arg("search") = 1.0,
    "Khoảng cách tâm -> biên vật cản gần nhất (m), < 0 trong vật cản; trừ robot_radius để ra hở thân");

  py::class_<Episode>(m, "Episode")
  .def(py::init([](const py::dict & scenario, const py::dict & sim, const py::dict & dwb, const py::dict & replan) {
      stage5_benchmark::Scenario sc;
      sc.map = toGrid(scenario["map"].cast<py::dict>());
      for (const auto & o : scenario["obstacles"].cast<py::list>()) {
        sc.new_obstacles.push_back(toShape(o.cast<py::dict>()));
      }
      sc.start = toPose(scenario["start"]);
      sc.goal = toPose(scenario["goal"]);
      s3::DwbParams p;
      applyDwb(p, dwb);
      s3::ReplanParams rp;
      applyReplan(rp, replan);
      return std::make_unique<Episode>(sc, toSim(sim), p, rp);
    }), py::arg("scenario"), py::arg("sim"), py::arg("dwb"), py::arg("replan"))
  .def("step", &Episode::step, py::call_guard<py::gil_scoped_release>())
  .def("run", &Episode::run, py::call_guard<py::gil_scoped_release>())
  .def_property_readonly("done", &Episode::done)
  .def_property_readonly("time", &Episode::time)
  .def("set_dwb", [](Episode & e, const py::dict & d) {
      s3::DwbParams p = e.dwbParams();
      applyDwb(p, d);
      return e.setDwbParams(p);
    }, "đổi một phần tham số CustomDWB (validateParams; lỗi -> giữ cũ, trả lý do)")
  .def("pose", [](const Episode & e) {auto p = e.pose(); return std::vector<double>{p.x, p.y, p.theta};})
  .def("measured_velocity", [](const Episode & e) {
      auto v = e.measuredVelocity(); return std::vector<double>{v.v, v.w};
    })
  .def("last_command", [](const Episode & e) {auto v = e.lastCommand(); return std::vector<double>{v.v, v.w};})
  .def("scan", [](const Episode & e) {
      const auto & s = e.scan();
      py::array_t<float> a(static_cast<py::ssize_t>(s.size()));
      std::copy(s.begin(), s.end(), a.mutable_data());
      return a;
    }, "720 tia, góc i*2π/720 (0 = trước), nửa sau NaN, inf = không phản hồi")
  .def("local_costmap", [](const Episode & e) {
      const auto & cm = e.localCostmap();
      return py::make_tuple(costmapArray(cm), cm.getOriginX(), cm.getOriginY(), cm.getResolution());
    })
  .def("global_costmap", [](const Episode & e) {
      const auto & cm = e.globalCostmap();
      return py::make_tuple(costmapArray(cm), cm.getOriginX(), cm.getOriginY(), cm.getResolution());
    })
  .def("plan", [](const Episode & e) {
      const auto & p = e.currentPlan();
      py::array_t<double> a({static_cast<py::ssize_t>(p.size()), static_cast<py::ssize_t>(2)});
      auto mm = a.mutable_unchecked<2>();
      for (std::size_t i = 0; i < p.size(); ++i) {mm(i, 0) = p[i].x; mm(i, 1) = p[i].y;}
      return a;
    })
  .def("last_control", [](const Episode & e) {return controlDict(e.lastControl());})
  .def_property_readonly("bridge_state", &Episode::bridgeState)
  .def_property_readonly("following", &Episode::following)
  .def_property_readonly("blocked", &Episode::blocked)
  .def_property_readonly("replans", &Episode::replans)
  .def_property_readonly("progress", &Episode::progress)
  .def("gt_clearance", &Episode::groundTruthClearance, "CHỈ cho reward/evaluator (ground truth)")
  .def("result", [](const Episode & e) {return resultDict(e.result());});
}
