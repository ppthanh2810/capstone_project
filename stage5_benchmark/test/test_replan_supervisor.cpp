// test_replan_supervisor.cpp — kiểm tra cơ chế Replanning R2 "Local Avoidance First" (ReplanSupervisor) và chẩn
// đoán không gian trống (DwaCore::probeFreeSpace), không cần ROS graph. 2026-11-27.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "stage3_navigation/dwa_core.hpp"
#include "stage3_navigation/replan_supervisor.hpp"

namespace s3 = stage3_navigation;

namespace
{

// Đường thẳng y = 0 từ x = 0 tới x = 10, lưới 0.05 m; ô (x in [3, 3.3]) bị chặn (cost 100) nếu blocked.
struct World
{
  std::vector<std::int8_t> data;
  s3::GridView grid;
  std::vector<s3::PathPoint> path;
  explicit World(bool blocked)
  {
    grid.origin_x = -1.0;
    grid.origin_y = -2.0;
    grid.resolution = 0.05;
    grid.width = 240;
    grid.height = 80;
    data.assign(grid.width * grid.height, 0);
    if (blocked) {
      for (unsigned j = 0; j < grid.height; ++j) {
        for (unsigned i = 0; i < grid.width; ++i) {
          const double x = grid.origin_x + (i + 0.5) * grid.resolution;
          const double y = grid.origin_y + (j + 0.5) * grid.resolution;
          if (x >= 3.0 && x <= 3.3 && y > -0.3 && y < 0.3) {data[j * grid.width + i] = 100;}
        }
      }
    }
    grid.data = data.data();
    for (int k = 0; k <= 100; ++k) {path.push_back({0.1 * k, 0.0});}
  }
};

s3::ReplanParams r2()
{
  s3::ReplanParams p;
  p.trigger = "conditional";
  p.confirm_time = 0.5;
  p.stall_time = 4.0;
  p.local_free_ratio_min = 0.10;
  p.local_hold_time = 0.4;
  p.useful_displacement = 0.3;
  p.min_interval = 2.0;
  p.max_obstacle_replans = 2;
  return p;
}

s3::ControllerReport good()
{
  s3::ControllerReport r;
  r.ok = r.follow = true;
  r.n_candidates = 420;
  r.n_forward_valid = 300;
  r.fs_valid = true;
  r.fs_clearance = 0.3;
  r.fs_arcs = 21;
  r.fs_forward_free = 15;
  return r;
}

}  // namespace

// Đường bị chặn trong 1 m NHƯNG DWA còn quỹ đạo tiến + không gian trống: KHÔNG lập lại (khác R1).
TEST(ReplanR2, PathBlockedAloneDoesNotReplan)
{
  World w(true);
  s3::ReplanSupervisor sup;
  const auto p = r2();
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  double x = 2.0;
  for (double t = 0.0; t < 3.0; t += 0.2) {
    sup.reportController(t, good(), p);
    x += 0.04;   // vẫn tiến dọc đường (đang vòng né)
    const auto d = sup.checkBlockage(t, x, 0.4, w.grid, p);
    EXPECT_EQ(d.type, s3::ReplanDecision::Type::NONE) << "t=" << t;
  }
  EXPECT_EQ(sup.state(), s3::ReplanState::PATH_BLOCKED);
  EXPECT_EQ(sup.totalReplans(), 0);
}

// Hết cung tiến tự do liên tục local_hold_time khi đường bị chặn -> LOCAL_BLOCKED -> GLOBAL_BLOCKED (replan).
TEST(ReplanR2, NoFreeSpaceTriggersReplanAfterHold)
{
  World w(true);
  s3::ReplanSupervisor sup;
  const auto p = r2();
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto bad = good();
  bad.fs_forward_free = 0;
  int fired_at = -1;
  for (int k = 0; k < 20; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, bad, p);
    const auto d = sup.checkBlockage(t, 2.6, 0.0, w.grid, p);
    if (d.type == s3::ReplanDecision::Type::REPLAN) {
      fired_at = k;
      EXPECT_NE(d.text.find("LOCAL_BLOCKED (free_space)"), std::string::npos) << d.text;
      break;
    }
  }
  // confirm 0.5 s (chặn từ t=0 -> xác nhận ở t=0.6) + hold 0.4 tính từ max(bắt đầu xấu, bắt đầu chặn) -> t=0.6
  ASSERT_GE(fired_at, 0);
  EXPECT_LE(0.2 * fired_at, 1.0);
  EXPECT_EQ(sup.state(), s3::ReplanState::GLOBAL_BLOCKED);
}

// Bằng chứng xấu ngắt quãng (< hold) không lập lại; pha xoay đủ không gian là trung tính.
TEST(ReplanR2, TransientEvidenceAndRotationAreNotEnough)
{
  World w(true);
  s3::ReplanSupervisor sup;
  auto p = r2();
  p.local_hold_time = 1.0;
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto bad = good();
  bad.fs_forward_free = 0;
  for (int k = 0; k < 15; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, (k % 4 == 3) ? good() : bad, p);   // cứ 0.8 s có một chu kỳ tốt
    const double x = 2.0 + 0.05 * k;
    EXPECT_EQ(sup.checkBlockage(t, x, 0.4, w.grid, p).type, s3::ReplanDecision::Type::NONE) << "t=" << t;
  }
  // xoay tại chỗ (không xấu) không xóa và không bắt đầu chuỗi
  s3::ControllerReport rot = good();
  rot.follow = false;
  rot.rotating = true;
  s3::ReplanSupervisor sup2;
  sup2.begin(0.0);
  sup2.setPath(w.path, 0.0);
  sup2.reportController(0.0, rot, p);
  EXPECT_FALSE(sup2.localBad());
}

// Đứng yên / chỉ xoay trong FOLLOW_PATH (không còn quỹ đạo TIẾN) là bằng chứng LOCAL_BLOCKED.
TEST(ReplanR2, NoForwardTrajectoryIsLocalBlocked)
{
  s3::ReplanSupervisor sup;
  const auto p = r2();
  World w(true);
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto r = good();
  r.n_forward_valid = 0;
  sup.reportController(0.0, r, p);
  EXPECT_TRUE(sup.localBad());
  EXPECT_EQ(sup.localReason(), "no_forward");
  r = good();
  r.ok = false;
  sup.reportController(0.2, r, p);
  EXPECT_EQ(sup.localReason(), "dwa_blocked");
}

// Không tiến hữu ích (không tiến dọc đường, không dịch chuyển) stall_time khi bị chặn -> replan dù DWA "ổn".
TEST(ReplanR2, NoUsefulProgressBackstop)
{
  World w(true);
  s3::ReplanSupervisor sup;
  const auto p = r2();
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  int fired = -1;
  for (int k = 0; k < 40; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, good(), p);
    const auto d = sup.checkBlockage(t, 2.5, 0.0, w.grid, p);
    if (d.type == s3::ReplanDecision::Type::REPLAN) {
      fired = k;
      EXPECT_NE(d.text.find("no useful progress"), std::string::npos);
      break;
    }
  }
  ASSERT_GE(fired, 0);
  EXPECT_NEAR(0.2 * fired, 4.0, 0.21);
}

// Đi vòng (dịch chuyển >= useful_displacement) là tiến triển hữu ích dù tiến trình dọc đường không tăng.
TEST(ReplanR2, DetourDisplacementCountsAsProgress)
{
  World w(true);
  s3::ReplanSupervisor sup;
  const auto p = r2();
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  for (int k = 0; k < 40; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, good(), p);
    const double y = 0.05 * k;   // đi ngang ra khỏi đường, tiến trình dọc đường không đổi
    EXPECT_EQ(sup.checkBlockage(t, 2.5, y, w.grid, p).type, s3::ReplanDecision::Type::NONE) << "t=" << t;
  }
}

// Cooldown + giới hạn số lần -> SAFE_STOP (FAILED).
TEST(ReplanR2, CooldownAndLimitEndInSafeStop)
{
  World w(true);
  s3::ReplanSupervisor sup;
  const auto p = r2();
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto bad = good();
  bad.fs_forward_free = 0;
  std::vector<double> replans;
  bool ended = false;
  for (int k = 0; k < 100 && !ended; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, bad, p);
    const auto d = sup.checkBlockage(t, 2.6, 0.0, w.grid, p);
    if (d.type == s3::ReplanDecision::Type::REPLAN) {
      sup.markAttempt(t);
      replans.push_back(t);
      sup.setPath(w.path, t);    // A* trả lại cùng đường (vật cản chưa có trong map)
    } else if (d.type == s3::ReplanDecision::Type::END) {
      ended = true;
      EXPECT_NE(d.text.find("max_obstacle_replans"), std::string::npos);
    }
  }
  ASSERT_EQ(replans.size(), 2u);
  EXPECT_GE(replans[1] - replans[0], p.min_interval - 1e-9);
  EXPECT_TRUE(ended);
  EXPECT_EQ(sup.state(), s3::ReplanState::SAFE_STOP);
}

// R3 (obstacle_replan false): trạng thái vẫn tính nhưng không bao giờ lập lại.
TEST(ReplanR3, ObservesButNeverReplans)
{
  World w(true);
  s3::ReplanSupervisor sup;
  auto p = r2();
  p.obstacle_replan = false;
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto bad = good();
  bad.fs_forward_free = 0;
  for (int k = 0; k < 60; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, bad, p);
    EXPECT_EQ(sup.checkBlockage(t, 2.6, 0.0, w.grid, p).type, s3::ReplanDecision::Type::NONE);
  }
  EXPECT_EQ(sup.state(), s3::ReplanState::LOCAL_BLOCKED);
}

// R0 (stall, V3) không đổi: bằng chứng DWA không có tác dụng, chỉ stall_time.
TEST(ReplanR0, StallUnchanged)
{
  World w(true);
  s3::ReplanSupervisor sup;
  s3::ReplanParams p;          // mặc định: stall
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto bad = good();
  bad.fs_forward_free = 0;
  bad.n_forward_valid = 0;
  int fired = -1;
  for (int k = 0; k < 40; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, bad, p);
    if (sup.checkBlockage(t, 2.5, 0.0, w.grid, p).type == s3::ReplanDecision::Type::REPLAN) {
      fired = k;
      break;
    }
  }
  ASSERT_GE(fired, 0);
  EXPECT_NEAR(0.2 * fired, 4.0, 0.21);
}

// Đường thông: NORMAL, không replan dù DWA báo xấu (LOCAL_BLOCKED cần PATH_BLOCKED).
TEST(ReplanR2, ClearPathNeverReplansOnLocalEvidence)
{
  World w(false);
  s3::ReplanSupervisor sup;
  const auto p = r2();
  sup.begin(0.0);
  sup.setPath(w.path, 0.0);
  auto bad = good();
  bad.ok = false;
  for (int k = 0; k < 50; ++k) {
    const double t = 0.2 * k;
    sup.reportController(t, bad, p);
    EXPECT_EQ(sup.checkBlockage(t, 2.5, 0.0, w.grid, p).type, s3::ReplanDecision::Type::NONE);
  }
  EXPECT_EQ(sup.state(), s3::ReplanState::NORMAL);
}

// ---- chẩn đoán không gian trống: chỉ phụ thuộc costmap + pose, KHÔNG phụ thuộc tham số DWA ----
TEST(FreeSpace, IndependentOfDwaParamsAndDetectsWall)
{
  nav2_costmap_2d::Costmap2D cm(160, 160, 0.025, -2.0, -2.0, 0);
  // tường ngang trước robot: x in [0.4, 0.5], y in [-1, 1] -> LETHAL; vùng inflation tới 0.25 m -> INSCRIBED
  for (unsigned j = 0; j < 160; ++j) {
    for (unsigned i = 0; i < 160; ++i) {
      double x, y;
      cm.mapToWorld(i, j, x, y);
      if (y > -1.0 && y < 1.0 && x > 0.15 && x < 0.75) {
        cm.setCost(i, j, (x >= 0.4 && x <= 0.5) ? nav2_costmap_2d::LETHAL_OBSTACLE :
          nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
    }
  }
  const s3::CollisionModel col(&cm, s3::CollisionMode::CENTER, {}, false);
  const auto fs = s3::probeFreeSpace(s3::Pose2D{0.0, 0.0, 0.0}, col);
  ASSERT_TRUE(fs.valid);
  EXPECT_EQ(fs.arcs, s3::kFreeSpaceArcs);
  EXPECT_NEAR(fs.clearance, 0.15, 0.03);
  EXPECT_LT(fs.forward_free_ratio(), 0.5);            // các cung thẳng / cong nhẹ chạm tường
  const auto back = s3::probeFreeSpace(s3::Pose2D{0.0, 0.0, M_PI}, col);
  EXPECT_DOUBLE_EQ(back.forward_free_ratio(), 1.0);   // quay lưng về tường: mọi cung tự do

  // compute() với hai bộ tham số khác nhau (trọng số, sim_time) -> cùng free_space
  s3::ControlInput in;
  in.robot = s3::Pose2D{0.0, 0.0, 0.0};
  in.local_path = {s3::Pose2D{0.0, 0.0, 0.0}, s3::Pose2D{1.0, 0.0, 0.0}};
  in.goal = s3::Pose2D{1.0, 0.0, 0.0};
  in.tol = s3::GoalTolerance{0.15, 0.25};
  in.max_vel_x_eff = 0.3;
  s3::DwbParams a, b;
  b.sim_time = 3.0;
  b.obstacle_scale = 0.0;
  b.speed_scale = 3.0;
  s3::DwaCore ca, cb;
  const auto oa = ca.compute(a, in, col, false);
  const auto ob = cb.compute(b, in, col, false);
  EXPECT_DOUBLE_EQ(oa.free_space.clearance, ob.free_space.clearance);
  EXPECT_EQ(oa.free_space.forward_free, ob.free_space.forward_free);
}

// An toàn độc lập với replan: không còn quỹ đạo hợp lệ -> DwaCore không phát lệnh (controller dừng ngay).
TEST(Safety, NoAdmissibleTrajectoryMeansNoCommand)
{
  nav2_costmap_2d::Costmap2D cm(80, 80, 0.025, -1.0, -1.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const s3::CollisionModel col(&cm, s3::CollisionMode::CENTER, {}, false);
  s3::ControlInput in;
  in.robot = s3::Pose2D{0.0, 0.0, 0.0};
  in.velocity = s3::Velocity2D{0.3, 0.0};
  in.prev_cmd = in.velocity;
  in.local_path = {s3::Pose2D{0.0, 0.0, 0.0}, s3::Pose2D{0.5, 0.0, 0.0}};
  in.goal = s3::Pose2D{0.5, 0.0, 0.0};
  in.tol = s3::GoalTolerance{0.15, 0.25};
  in.max_vel_x_eff = 0.3;
  s3::DwbParams p;
  s3::DwaCore core;
  const auto out = core.compute(p, in, col, false);
  EXPECT_FALSE(out.ok);
  EXPECT_EQ(out.phase, s3::Phase::BLOCKED);
  EXPECT_DOUBLE_EQ(out.free_space.forward_free_ratio(), 0.0);
}
