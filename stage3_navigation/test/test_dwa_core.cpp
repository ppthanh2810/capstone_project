// =====================================================================================================
// test_dwa_core.cpp — Unit test + kịch bản kín vòng cho lõi CustomDWB (không cần ROS graph)
// =====================================================================================================
// Mỗi test kiểm một HÀNH VI / BẤT BIẾN (vd. "lệnh luôn trong giới hạn", "không bỏ lọt vật cản giữa hai bước"),
// không kiểm lại công thức vừa viết. Mã yêu cầu R1..R10: doc/requirements_validation.md.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "sim_world.hpp"
#include "stage3_navigation/dwa_core.hpp"

using namespace stage3_navigation;  // NOLINT

namespace
{
// Costmap 4 x 4 m, 0.05 m, origin (-2, -2): robot ở giữa.
nav2_costmap_2d::Costmap2D makeCostmap(unsigned char fill = nav2_costmap_2d::FREE_SPACE)
{
  return nav2_costmap_2d::Costmap2D(80, 80, 0.05, -2.0, -2.0, fill);
}

void setWorldCost(nav2_costmap_2d::Costmap2D & cm, double x, double y, unsigned char c)
{
  unsigned int mx, my;
  ASSERT_TRUE(cm.worldToMap(x, y, mx, my));
  cm.setCost(mx, my, c);
}

// Dải chướng ngại (cost c) cho mọi ô có x >= x0 (toàn chiều cao).
void wallFrom(nav2_costmap_2d::Costmap2D & cm, double x0, unsigned char c)
{
  for (unsigned int j = 0; j < cm.getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < cm.getSizeInCellsX(); ++i) {
      double wx, wy;
      cm.mapToWorld(i, j, wx, wy);
      if (wx >= x0) {
        cm.setCost(i, j, c);
      }
    }
  }
}

ControlInput straightInput(double goal_x = 2.0)
{
  ControlInput in;
  for (int k = 0; k <= 20; ++k) {
    in.local_path.push_back(Pose2D{goal_x * k / 20.0, 0.0, 0.0});
  }
  in.goal = Pose2D{goal_x, 0.0, 0.0};
  in.dt = 0.1;
  in.max_vel_x_eff = 0.15;
  return in;
}
}  // namespace

// ------------------------------------------------------------------------------------------------
// R2 — Dynamic window, overspeed, sampler
// ------------------------------------------------------------------------------------------------

TEST(DynamicWindow, ReproducesOldOverspeedBugAndNewPolicy)
{
  // Phát hiện P0: v0 = 0.50, max = 0.15, decel = 0.50, dt = 0.10. Mức đạt được thấp nhất là 0.45.
  DwbParams p;
  const Range r = reachableRange(0.50, p.acc_lim_x, p.decel_lim_x, 0.1);
  EXPECT_NEAR(r.lo, 0.45, 1e-12);
  // Code cũ: v_lo = 0.45 > v_hi = 0.15 -> ép về 0.15 và coi 0.15 là "đạt được" khi mô phỏng (v hằng 0.15).
  const double old_cmd = std::max(p.min_vel_x, std::min(r.lo, p.max_vel_x));
  EXPECT_DOUBLE_EQ(old_cmd, 0.15);

  // Mới: lệnh vẫn = 0.15 (hard cap của LỆNH) nhưng được gắn cờ overspeed, và quỹ đạo mô phỏng dùng vận tốc
  // thật giảm dần theo decel_lim (không giả định 0.15 ngay lập tức).
  const DynamicWindow dw = computeWindow(p, 0.15, Velocity2D{0.50, 0.0}, 0.1);
  EXPECT_TRUE(dw.v_overspeed);
  EXPECT_DOUBLE_EQ(dw.v.lo, 0.15);
  EXPECT_DOUBLE_EQ(dw.v.hi, 0.15);
  const Trajectory t = rolloutCommand(
    p, Pose2D{}, Velocity2D{0.5, 0.0}, Velocity2D{0.5, 0.0}, Velocity2D{0.15, 0.0}, 0.1, 1.0, 0.025,
    0.0);
  // trễ 0.1 s ở 0.5 m/s (lệnh trước) = 0.05 m, rồi giảm 0.5 -> 0.15 với 0.5 m/s² trong 0.7 s, rồi 0.15 m/s.
  // Quãng đường thật trong 1.0 s: 0.05 + (0.5*0.7 - 0.25*0.49) + 0.15*0.2 = 0.3075 m (nếu v = 0.15 hằng: 0.15 m).
  EXPECT_NEAR(t.poses.back().x, 0.3075, 1e-6);
  // Đuôi phanh bắt đầu tại trễ + dt = 0.2 s với v = 0.45 -> 0.45² / (2*0.5) = 0.2025 m + margin.
  const double commit_x = t.braking.front().x;
  EXPECT_NEAR(commit_x, 0.05 + 0.5 * 0.1 - 0.25 * 0.01, 1e-9);
  EXPECT_NEAR(t.braking.back().x - commit_x, 0.2025 + p.stopping_margin, 1e-6);
}

TEST(DynamicWindow, NormalWindowAndLoweredLimit)
{
  DwbParams p;
  DynamicWindow dw = computeWindow(p, 0.15, Velocity2D{0.10, 0.0}, 0.1);
  EXPECT_FALSE(dw.v_overspeed);
  EXPECT_NEAR(dw.v.lo, 0.05, 1e-12);
  EXPECT_NEAR(dw.v.hi, 0.15, 1e-12);
  EXPECT_NEAR(dw.w.lo, -0.10, 1e-12);
  EXPECT_NEAR(dw.w.hi, 0.10, 1e-12);
  // Speed limit hạ xuống 0.05 giữa lúc chạy: [0.05, 0.15] ∩ [0, 0.05] = {0.05}, không overspeed.
  dw = computeWindow(p, 0.05, Velocity2D{0.10, 0.0}, 0.1);
  EXPECT_FALSE(dw.v_overspeed);
  EXPECT_NEAR(dw.v.lo, 0.05, 1e-12);
  EXPECT_NEAR(dw.v.hi, 0.05, 1e-12);
}

TEST(DynamicWindow, ZeroCrossingAndReverseNeverCommanded)
{
  // Giảm về 0 trong 0.04 s rồi tăng theo chiều âm với acc 1.0 trong 0.06 s -> -0.06.
  const Range r = reachableRange(0.02, 1.0, 0.5, 0.1);
  EXPECT_NEAR(r.lo, -0.06, 1e-12);
  EXPECT_NEAR(r.hi, 0.12, 1e-12);
  // v0 âm (PS4 lùi, hoặc nhiễu odom) với min_vel_x = 0: lệnh không bao giờ âm.
  DwbParams p;
  const DynamicWindow dw = computeWindow(p, 0.15, Velocity2D{-0.2, 0.0}, 0.1);
  EXPECT_TRUE(dw.v_overspeed);
  EXPECT_GE(dw.v.lo, 0.0);
  EXPECT_DOUBLE_EQ(dw.v.hi, 0.0);
}

TEST(DynamicWindow, AngularOverspeedUsesSeparateDecel)
{
  DwbParams p;
  p.decel_lim_theta = 2.0;
  const DynamicWindow dw = computeWindow(p, 0.15, Velocity2D{0.0, 1.0}, 0.1);
  EXPECT_TRUE(dw.w_overspeed);
  EXPECT_DOUBLE_EQ(dw.w.lo, p.max_vel_theta);
  const Range r = reachableRange(1.0, p.acc_lim_theta, p.decel_lim_theta, 0.1);
  EXPECT_NEAR(r.lo, 0.8, 1e-12);   // giảm |ω| dùng decel_lim_theta
  EXPECT_NEAR(r.hi, 1.1, 1e-12);   // tăng |ω| dùng acc_lim_theta
}

TEST(Sampler, AlwaysContainsZeroWhenReachableAndIsUnique)
{
  for (int n : {1, 2, 3, 4, 20, 21}) {
    const auto s = sampleAxis(-0.1, 0.3, n);
    EXPECT_TRUE(std::find(s.begin(), s.end(), 0.0) != s.end()) << "n=" << n;
    for (std::size_t i = 1; i < s.size(); ++i) {
      EXPECT_LT(s[i - 1], s[i]);
    }
    EXPECT_GE(s.front(), -0.1);
    EXPECT_LE(s.back(), 0.3);
  }
  EXPECT_EQ(sampleAxis(0.0, 0.0, 5).size(), 1u);
  const auto pos = sampleAxis(0.05, 0.15, 5);
  EXPECT_TRUE(std::find(pos.begin(), pos.end(), 0.0) == pos.end());
  EXPECT_EQ(pos.size(), 5u);
}

// ------------------------------------------------------------------------------------------------
// R3 — Mô phỏng, va chạm, phanh
// ------------------------------------------------------------------------------------------------

TEST(Rollout, EndsExactlyAtHorizonAndMatchesClosedForm)
{
  DwbParams p;
  for (double gran : {0.05, 0.07, 0.3}) {
    p.sim_granularity = gran;
    SimState s{Pose2D{}, Velocity2D{0.15, 0.5}};
    std::vector<Pose2D> out;
    advanceState(s, Velocity2D{0.15, 0.5}, 1.7, p, 0.0, 0.0, &out);
    // Cung tròn chính xác: x = v/ω sin(ωT), y = v/ω (1 - cos(ωT)).
    EXPECT_NEAR(s.pose.x, 0.3 * std::sin(0.85), 1e-9) << gran;
    EXPECT_NEAR(s.pose.y, 0.3 * (1.0 - std::cos(0.85)), 1e-9) << gran;
    EXPECT_NEAR(s.pose.theta, 0.85, 1e-9) << gran;
  }
}

TEST(Rollout, StepNeverSkipsMoreThanHalfACell)
{
  DwbParams p;
  p.max_vel_x = 1.0;
  p.sim_granularity = 0.2;   // 1.0 m/s * 0.2 s = 0.2 m >> ô 0.05 m nếu không giới hạn bước
  SimState s{Pose2D{}, Velocity2D{1.0, 0.0}};
  std::vector<Pose2D> out{s.pose};
  advanceState(s, Velocity2D{1.0, 0.0}, 1.0, p, 0.025, 0.0, &out);
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_LE(std::hypot(out[i].x - out[i - 1].x, out[i].y - out[i - 1].y), 0.025 + 1e-9);
  }
  EXPECT_NEAR(s.pose.x, 1.0, 1e-9);
}

TEST(Rollout, ThinObstacleBetweenCoarseStepsIsDetected)
{
  auto cm = makeCostmap();
  setWorldCost(cm, 0.52, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);   // một ô duy nhất, không inflation
  DwbParams p;
  p.max_vel_x = 1.0;
  p.sim_granularity = 0.2;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  const Trajectory t = rolloutCommand(
    p, Pose2D{}, Velocity2D{1.0, 0.0}, Velocity2D{1.0, 0.0}, Velocity2D{1.0, 0.0}, 0.1, 1.0,
    0.5 * model.resolution(), model.sweepRadius());
  unsigned char mc = 0;
  EXPECT_EQ(checkTrajectory(t, model, mc), Reject::COLLISION);
}

TEST(Rollout, StartPoseIsChecked)
{
  auto cm = makeCostmap();
  setWorldCost(cm, 0.0, 0.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in = straightInput();
  const ControlOutput out = core.compute(p, in, model, false);
  EXPECT_FALSE(out.ok);
  EXPECT_EQ(out.phase, Phase::BLOCKED);
  EXPECT_NE(out.error.find("collision"), std::string::npos);
}

TEST(Braking, RejectsVelocityThatCannotStopEvenIfHorizonIsFree)
{
  // Chân trời ngắn (0.2 s) không chạm tường ở x = 0.2 m, nhưng từ v ~ 0.45 m/s robot cần ~0.2 m để dừng.
  auto cm = makeCostmap();
  wallFrom(cm, 0.2, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  DwbParams p;
  p.sim_time = 0.2;
  p.sim_granularity = 0.05;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in = straightInput();
  in.velocity = Velocity2D{0.5, 0.0};
  in.prev_cmd = Velocity2D{0.5, 0.0};
  const ControlOutput out = core.compute(p, in, model, true);
  EXPECT_FALSE(out.ok);
  EXPECT_GT(out.n_reject[static_cast<int>(Reject::BRAKING)], 0);
  EXPECT_EQ(out.n_reject[static_cast<int>(Reject::COLLISION)], 0);
  EXPECT_NE(out.error.find("cannot brake"), std::string::npos);
}

TEST(Braking, StoppingDistanceGrowsWithLatency)
{
  DwbParams p;
  const auto tail = [&p](double latency) {
      p.control_latency = latency;
      const Trajectory t = rolloutCommand(
        p, Pose2D{}, Velocity2D{0.15, 0.0}, Velocity2D{0.15, 0.0}, Velocity2D{0.15, 0.0}, 0.1, 1.7,
        0.025, 0.0);
      return t.braking.back().x;
    };
  // d = v·(τ + dt) + v²/(2a) + margin, v giữ 0.15 trong trễ.
  EXPECT_NEAR(tail(0.0), 0.15 * 0.1 + 0.0225 + 0.05, 1e-6);
  EXPECT_NEAR(tail(0.3), 0.15 * 0.4 + 0.0225 + 0.05, 1e-6);
}

TEST(Rotation, ResidualForwardSpeedIsCollisionChecked)
{
  // Robot đang chạy 0.15 m/s, ô INSCRIBED bắt đầu từ x = 0.05 m phía trước (ô tâm robot vẫn trống), đường A* ở
  // phía sau (phải xoay). Code cũ chỉ kiểm ô tâm hiện tại rồi trả v = 0.10 -> robot còn trôi ~0.09 m (trễ +
  // chu kỳ + phanh + margin) vào vùng nội tiếp. Mới: mô phỏng chuyển động đó và từ chối.
  auto cm = makeCostmap();
  wallFrom(cm, 0.06, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in;
  in.local_path = {Pose2D{0.0, 0.0, M_PI}, Pose2D{-1.0, 0.0, M_PI}};
  in.goal = Pose2D{-1.0, 0.0, M_PI};
  in.velocity = Velocity2D{0.15, 0.0};
  in.prev_cmd = Velocity2D{0.15, 0.0};
  in.max_vel_x_eff = 0.15;
  const ControlOutput out = core.compute(p, in, model, false);
  EXPECT_FALSE(out.ok);
  EXPECT_NE(out.error.find("ROTATE_TO_PATH"), std::string::npos);
  // Cùng tình huống nhưng tường xa (1.0 m): lệnh xoay được chấp nhận, v giảm theo decel.
  auto cm2 = makeCostmap();
  wallFrom(cm2, 1.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const CollisionModel model2(&cm2, CollisionMode::CENTER, {}, true);
  DwaCore core2;
  const ControlOutput ok = core2.compute(p, in, model2, false);
  ASSERT_TRUE(ok.ok) << ok.error;
  EXPECT_EQ(ok.phase, Phase::ROTATE_TO_PATH);
  EXPECT_NEAR(ok.cmd.v, 0.10, 1e-9);
}

TEST(Goal, HoldCommandAtGoalIsCollisionChecked)
{
  // Đã đúng vị trí và hướng nhưng còn trôi 0.15 m/s, vùng INSCRIBED ngay phía trước: lệnh giảm tốc (v > 0)
  // cũng phải được kiểm tra như mọi chuyển động, không được trả thẳng.
  auto cm = makeCostmap();
  wallFrom(cm, 0.06, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in = straightInput(0.02);
  in.tol = GoalTolerance{0.15, 0.25};
  in.velocity = Velocity2D{0.15, 0.0};
  in.prev_cmd = Velocity2D{0.15, 0.0};
  const ControlOutput out = core.compute(p, in, model, false);
  EXPECT_FALSE(out.ok);
  EXPECT_NE(out.error.find("ROTATE_TO_GOAL"), std::string::npos);
  // Đứng yên ở đích: lệnh (0, 0) hợp lệ.
  in.velocity = Velocity2D{};
  in.prev_cmd = Velocity2D{};
  DwaCore core2;
  const ControlOutput ok = core2.compute(p, in, model, false);
  ASSERT_TRUE(ok.ok) << ok.error;
  EXPECT_DOUBLE_EQ(ok.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(ok.cmd.w, 0.0);
}

TEST(Rotation, DirectionHeldNearPi)
{
  // Đường ngay sau lưng: sai số ~±π; nhiễu yaw ±0.01 không được làm đổi chiều xoay mỗi chu kỳ.
  auto cm = makeCostmap();
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in;
  in.local_path = {Pose2D{0.0, 0.0, M_PI}, Pose2D{-1.0, 0.0, M_PI}};
  in.goal = Pose2D{-1.0, 0.0, M_PI};
  in.max_vel_x_eff = 0.15;
  int sign = 0;
  for (int k = 0; k < 10; ++k) {
    in.robot.theta = (k % 2 == 0) ? 0.01 : -0.01;
    const ControlOutput out = core.compute(p, in, model, false);
    ASSERT_TRUE(out.ok);
    const int s = out.cmd.w > 0 ? 1 : -1;
    if (sign != 0) {
      EXPECT_EQ(s, sign) << "flip at cycle " << k;
    }
    sign = s;
    in.velocity = Velocity2D{0.0, out.cmd.w};
  }
}

TEST(CollisionModel, FootprintCornerVsCenterAndUnknown)
{
  auto cm = makeCostmap();
  setWorldCost(cm, 0.28, 0.28, nav2_costmap_2d::LETHAL_OBSTACLE);   // chỉ chạm góc hình vuông 0.6 x 0.6
  const std::vector<Pose2D> square{{0.3, 0.3, 0}, {0.3, -0.3, 0}, {-0.3, -0.3, 0}, {-0.3, 0.3, 0}};
  const CollisionModel center(&cm, CollisionMode::CENTER, {}, true);
  const CollisionModel fp(&cm, CollisionMode::FOOTPRINT, square, true);
  unsigned char c = 0;
  EXPECT_EQ(center.check(Pose2D{}, c), CellCheck::FREE);
  EXPECT_EQ(fp.check(Pose2D{}, c), CellCheck::COLLISION);
  EXPECT_GT(fp.sweepRadius(), 0.42);
  // Xoay 45°: góc không còn chạm ô đó.
  EXPECT_EQ(fp.check(Pose2D{0.0, 0.0, M_PI / 4}, c), CellCheck::FREE);

  setWorldCost(cm, -1.0, -1.0, nav2_costmap_2d::NO_INFORMATION);
  const CollisionModel unk_obst(&cm, CollisionMode::CENTER, {}, true);
  const CollisionModel unk_free(&cm, CollisionMode::CENTER, {}, false);
  EXPECT_EQ(unk_obst.check(Pose2D{-1.0, -1.0, 0}, c), CellCheck::UNKNOWN);
  EXPECT_EQ(unk_free.check(Pose2D{-1.0, -1.0, 0}, c), CellCheck::FREE);
  EXPECT_EQ(unk_free.check(Pose2D{5.0, 0.0, 0}, c), CellCheck::OFF_MAP);
  EXPECT_EQ(unk_free.check(Pose2D{std::nan(""), 0.0, 0}, c), CellCheck::OFF_MAP);
}

// ------------------------------------------------------------------------------------------------
// R5 — Đích
// ------------------------------------------------------------------------------------------------

TEST(Goal, ToleranceComesFromGoalChecker)
{
  auto cm = makeCostmap();
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  ControlInput in = straightInput(0.25);
  in.goal.theta = 1.0;
  in.tol = GoalTolerance{0.30, 0.25};   // checker rộng hơn tham số dự phòng 0.15
  DwaCore a;
  EXPECT_EQ(a.compute(p, in, model, false).phase, Phase::ROTATE_TO_GOAL);
  in.tol = GoalTolerance{0.15, 0.25};
  DwaCore b;
  EXPECT_NE(b.compute(p, in, model, false).phase, Phase::ROTATE_TO_GOAL);
}

TEST(Goal, LatchReleasedOnDriftAndWhenAlignedOutsideEntry)
{
  auto cm = makeCostmap();
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in = straightInput(0.10);
  in.goal.theta = 1.5;
  in.tol = GoalTolerance{0.15, 0.25};
  ASSERT_EQ(core.compute(p, in, model, false).phase, Phase::ROTATE_TO_GOAL);
  EXPECT_TRUE(core.goalLatched());
  // Trôi tới 0.14 m (giữa ngưỡng vào 0.135 và ngưỡng ra 0.20) nhưng hướng chưa đúng: vẫn xoay.
  in.robot.x = -0.04;
  EXPECT_EQ(core.compute(p, in, model, false).phase, Phase::ROTATE_TO_GOAL);
  // Đã đúng hướng mà vẫn ngoài ngưỡng vào: KHÔNG đứng yên chờ (checker không stateful sẽ không kết thúc).
  in.robot.theta = 1.5;
  const ControlOutput o = core.compute(p, in, model, false);
  EXPECT_FALSE(core.goalLatched());
  EXPECT_NE(o.phase, Phase::ROTATE_TO_GOAL);
  // Trôi ra quá 0.20 m: bỏ latch.
  DwaCore c2;
  in.robot = Pose2D{};
  in.robot.theta = 0.0;
  ASSERT_EQ(c2.compute(p, in, model, false).phase, Phase::ROTATE_TO_GOAL);
  in.robot.x = -0.15;
  c2.compute(p, in, model, false);
  EXPECT_FALSE(c2.goalLatched());
}

TEST(Goal, YawErrorWrapsAcrossPi)
{
  auto cm = makeCostmap();
  DwbParams p;
  const CollisionModel model(&cm, CollisionMode::CENTER, {}, true);
  DwaCore core;
  ControlInput in = straightInput(0.05);
  in.goal.theta = M_PI - 0.05;
  in.robot.theta = -M_PI + 0.05;   // sai số thật = 0.10 rad, không phải 2π - 0.1
  in.tol = GoalTolerance{0.15, 0.25};
  const ControlOutput out = core.compute(p, in, model, false);
  ASSERT_TRUE(out.ok);
  EXPECT_EQ(out.phase, Phase::ROTATE_TO_GOAL);
  EXPECT_DOUBLE_EQ(out.cmd.w, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd.v, 0.0);
}

// ------------------------------------------------------------------------------------------------
// R6 — Bám đường theo tiến trình
// ------------------------------------------------------------------------------------------------

TEST(PathTracker, SelfIntersectionDoesNotJumpBranches)
{
  const auto path = sim::polyline(
    {{0, 0, 0}, {2, 0, 0}, {2, 1.2, 0}, {1, 1.2, 0}, {1, -0.8, 0}, {3.0, -0.8, 0}}, 0.0);
  PathTracker tr;
  ASSERT_EQ(tr.setPlan(path), "");
  // Robot đi dọc nhánh đầu tới giao điểm (1, 0): tiến trình ~1.0, KHÔNG nhảy sang ~5.4 (lần đi qua thứ hai).
  for (double x = 0.0; x <= 1.0001; x += 0.1) {
    tr.update(Pose2D{x, 0.0, 0.0}, 3.0, 1.5, 0.1);
  }
  EXPECT_NEAR(tr.progress(), 1.0, 0.06);
  // Đi tiếp vòng: tiến trình tăng đơn điệu, tới lần qua giao điểm thứ hai thì ~5.4.
  const std::vector<Pose2D> loop{{2.0, 0.0, 0}, {2.0, 1.2, 0}, {1.0, 1.2, 0}, {1.0, 0.5, 0},
    {1.0, 0.0, 0}};
  double last = tr.progress();
  for (const auto & q : loop) {
    tr.update(q, 3.0, 1.5, 0.1);
    EXPECT_GE(tr.progress(), last);
    last = tr.progress();
  }
  EXPECT_NEAR(tr.progress(), 5.4, 0.06);
}

TEST(PathTracker, HairpinStaysOnCurrentLeg)
{
  // Chữ U hẹp 0.3 m: robot lệch vào giữa hai nhánh, cửa sổ tìm kiếm rộng (2 m) vẫn chứa nhánh về.
  const auto path = sim::polyline({{0, 0, 0}, {1, 0, 0}, {1, 0.3, 0}, {0, 0.3, 0}}, M_PI);
  PathTracker tr;
  ASSERT_EQ(tr.setPlan(path), "");
  tr.update(Pose2D{0.5, 0.0, 0.0}, 2.0, 1.5, 0.1);
  const auto r = tr.update(Pose2D{0.7, 0.15, 0.0}, 2.0, 1.5, 0.1);
  EXPECT_NEAR(r.progress, 0.7, 0.06);
  EXPECT_NEAR(r.deviation, 0.15, 1e-6);
}

TEST(PathTracker, ProgressNeverDecreasesAndLocalPathIsBounded)
{
  const auto path = sim::polyline({{0, 0, 0}, {3, 0, 0}}, 0.0);
  PathTracker tr;
  ASSERT_EQ(tr.setPlan(path), "");
  tr.update(Pose2D{1.0, 0.0, 0.0}, 1.0, 1.5, 0.1);
  const auto r = tr.update(Pose2D{0.2, 0.0, 0.0}, 1.0, 1.5, 0.1);   // robot bị đẩy lùi
  EXPECT_NEAR(r.progress, 1.0, 1e-9);
  double len = 0.0;
  for (std::size_t i = 1; i < r.local.size(); ++i) {
    len += std::hypot(r.local[i].x - r.local[i - 1].x, r.local[i].y - r.local[i - 1].y);
  }
  EXPECT_NEAR(len, 1.5, 1e-9);
  EXPECT_NEAR(r.local.front().x, 1.0, 1e-9);
}

TEST(PathTracker, DuplicatesSinglePoseAndInvalidInputs)
{
  PathTracker tr;
  std::vector<Pose2D> dup{{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0.7}};
  ASSERT_EQ(tr.setPlan(dup), "");
  EXPECT_EQ(tr.size(), 2u);
  EXPECT_DOUBLE_EQ(tr.goal().theta, 0.7);
  EXPECT_DOUBLE_EQ(tr.length(), 1.0);

  ASSERT_EQ(tr.setPlan({Pose2D{1.0, 2.0, 0.3}}), "");
  const auto r = tr.update(Pose2D{}, 1.0, 1.5, 0.1);
  ASSERT_EQ(r.local.size(), 1u);
  EXPECT_DOUBLE_EQ(r.local[0].x, 1.0);

  EXPECT_NE(tr.setPlan({}), "");
  EXPECT_FALSE(tr.hasPlan());
  EXPECT_NE(tr.setPlan({Pose2D{0, 0, 0}, Pose2D{std::nan(""), 0, 0}}), "");
  EXPECT_NE(
    tr.setPlan({Pose2D{0, 0, 0}, Pose2D{1, std::numeric_limits<double>::infinity(), 0}}),
    "");
  EXPECT_NE(tr.setPlan({Pose2D{0, 0, std::nan("")}}), "");
}

TEST(PathTracker, CarrotIsInterpolatedAlongPath)
{
  const std::vector<Pose2D> lp{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  const Pose2D c = pickCarrot(lp, 1.5);
  EXPECT_NEAR(c.x, 1.0, 1e-12);
  EXPECT_NEAR(c.y, 0.5, 1e-12);
  EXPECT_NEAR(pickCarrot(lp, 5.0).y, 1.0, 1e-12);
}

// ------------------------------------------------------------------------------------------------
// R6 — Chống dao động
// ------------------------------------------------------------------------------------------------

TEST(Oscillation, PenalizesImmediateFlipBackAndResets)
{
  DwbParams p;
  OscillationGuard g;
  g.update(Pose2D{}, 0.3, 0.0, p);
  EXPECT_DOUBLE_EQ(g.penalty(-0.3), 0.0);   // chưa đổi chiều lần nào
  g.update(Pose2D{}, -0.3, 0.1, p);          // đổi chiều lần 1
  EXPECT_EQ(g.flips(), 1);
  EXPECT_DOUBLE_EQ(g.penalty(0.3), 1.0);     // quay lại chiều cũ ngay -> bị phạt
  EXPECT_DOUBLE_EQ(g.penalty(-0.3), 0.0);
  EXPECT_DOUBLE_EQ(g.penalty(0.0), 0.0);     // đi thẳng không bị phạt
  // Đặt lại theo thời gian (không khóa vĩnh viễn) ...
  g.update(Pose2D{}, -0.3, 0.1 + p.oscillation_reset_time + 0.01, p);
  EXPECT_DOUBLE_EQ(g.penalty(0.3), 0.0);
  // ... và theo quãng đường.
  OscillationGuard g2;
  g2.update(Pose2D{}, 0.3, 0.0, p);
  g2.update(Pose2D{}, -0.3, 0.1, p);
  g2.update(Pose2D{0.1, 0.0, 0.0}, -0.3, 0.2, p);
  EXPECT_DOUBLE_EQ(g2.penalty(0.3), 0.0);
}

// ------------------------------------------------------------------------------------------------
// R8 — Kiểm tra tham số
// ------------------------------------------------------------------------------------------------

TEST(Validate, DefaultsAreValid)
{
  EXPECT_EQ(validateParams(DwbParams{}), "");
}

TEST(Validate, RejectsNonFiniteInEveryDouble)
{
  const double bad[] = {std::nan(""), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  // Duyệt mọi trường double bằng con trỏ thành viên.
  const std::vector<double DwbParams::*> fields{
    &DwbParams::max_vel_x, &DwbParams::min_vel_x, &DwbParams::max_vel_theta,
    &DwbParams::min_in_place_vel_theta, &DwbParams::acc_lim_x, &DwbParams::decel_lim_x,
    &DwbParams::acc_lim_theta, &DwbParams::decel_lim_theta, &DwbParams::sim_time,
    &DwbParams::sim_granularity, &DwbParams::control_latency, &DwbParams::stopping_margin,
    &DwbParams::local_plan_length, &DwbParams::carrot_distance, &DwbParams::closest_search_distance,
    &DwbParams::max_path_deviation, &DwbParams::rotate_to_heading_angle, &DwbParams::rotate_gain,
    &DwbParams::xy_goal_tolerance, &DwbParams::yaw_goal_tolerance,
    &DwbParams::goal_tolerance_fraction, &DwbParams::goal_latch_release_margin,
    &DwbParams::path_distance_scale, &DwbParams::carrot_distance_scale, &DwbParams::heading_scale,
    &DwbParams::obstacle_scale, &DwbParams::speed_scale, &DwbParams::oscillation_scale,
    &DwbParams::oscillation_reset_dist, &DwbParams::oscillation_reset_angle,
    &DwbParams::oscillation_reset_time, &DwbParams::transform_tolerance, &DwbParams::max_tf_age,
    &DwbParams::max_pose_age, &DwbParams::odom_timeout, &DwbParams::debug_publish_rate};
  for (const auto f : fields) {
    for (const double b : bad) {
      DwbParams p;
      p.*f = b;
      EXPECT_NE(validateParams(p), "") << "field accepted " << b;
    }
  }
}

TEST(Validate, RangesRelationsAndLoad)
{
  const auto rejects = [](auto mutate) {
      DwbParams p;
      mutate(p);
      return !validateParams(p).empty();
    };
  EXPECT_TRUE(rejects([](DwbParams & p) {p.min_vel_x = -0.05;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.min_vel_x = 0.2;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.decel_lim_x = -0.5;}));   // Nav2 DWB dùng số âm; ở đây phải dương
  EXPECT_TRUE(rejects([](DwbParams & p) {p.vx_samples = 0;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.vtheta_samples = 1000;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.speed_scale = -1.0;}));
  EXPECT_TRUE(
    rejects(
      [](DwbParams & p) {
        p.path_distance_scale = 0.0; p.carrot_distance_scale = 0.0;
      }));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.carrot_distance = 2.0;}));   // > local_plan_length
  EXPECT_TRUE(rejects([](DwbParams & p) {p.rotate_gain = 0.0;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.rotate_to_heading_angle = 4.0;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.goal_tolerance_fraction = 1.5;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.oscillation_reset_time = 0.0;}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.collision_check = "foo";}));
  EXPECT_TRUE(rejects([](DwbParams & p) {p.sim_granularity = 0.001;}));
  // Tải: 101 x 101 mẫu, 5 s / 0.005 s -> ~10^7 pose mỗi chu kỳ.
  EXPECT_TRUE(
    rejects(
      [](DwbParams & p) {
        p.vx_samples = 101; p.vtheta_samples = 101; p.sim_time = 5.0; p.sim_granularity = 0.005;
      }));
  EXPECT_FALSE(rejects([](DwbParams & p) {p.vx_samples = 30; p.vtheta_samples = 31;}));
}

// ------------------------------------------------------------------------------------------------
// R3/R6 — Kín vòng (fixture sim_world): bất biến an toàn trên mọi kịch bản chuẩn
// ------------------------------------------------------------------------------------------------

class ClosedLoop : public ::testing::TestWithParam<std::string> {};

TEST_P(ClosedLoop, SafetyInvariantsAndExpectedOutcome)
{
  const sim::Scenario sc = sim::findScenario(GetParam());
  const DwbParams p;
  sim::RobotModel robot;
  robot.odom_vel_noise = 0.002;
  for (std::uint32_t seed = 1; seed <= 3; ++seed) {
    const sim::RunMetrics m = sim::runScenario(p, sc, robot, seed);
    SCOPED_TRACE(sc.name + " seed " + std::to_string(seed) + ": " + m.result);
    EXPECT_EQ(m.collisions, 0) << "min clearance " << m.min_clearance;
    EXPECT_EQ(m.limit_violations, 0);
    EXPECT_EQ(m.nonfinite_cmds, 0);
    EXPECT_NE(m.result, "timeout");
    if (sc.expect_success) {
      EXPECT_TRUE(m.success);
      EXPECT_LE(m.final_xy_error, 0.15 + 0.03);
    }
    if (sc.name == "blocked_wall" || sc.name == "corridor_narrow" || sc.name == "overspeed_wall") {
      EXPECT_FALSE(m.success);   // không có "thành công giả"
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Scenarios, ClosedLoop,
  ::testing::Values(
    "straight", "l_turn", "s_curve", "self_intersect", "duplicates", "single_pose", "goal_near",
    "corridor_ok", "corridor_narrow", "obstacle_bypass", "blocked_wall", "obstacle_near_path",
    "symmetric_obstacle", "start_backwards", "overspeed_start", "overspeed_wall", "goal_yaw_pi"));

TEST(ClosedLoopExtra, SluggishRobotStillDoesNotCollide)
{
  // Robot thật chậm hơn giả định (gia tốc 0.3, trễ 0.2 s) — mô hình sai lệch có chủ đích.
  const DwbParams p;
  sim::RobotModel robot;
  robot.acc_x = robot.decel_x = 0.3;
  robot.acc_theta = robot.decel_theta = 0.6;
  robot.latency = 0.2;
  for (const char * name :
    {"blocked_wall", "obstacle_near_path", "corridor_ok", "overspeed_wall"})
  {
    const sim::RunMetrics m = sim::runScenario(p, sim::findScenario(name), robot, 7);
    EXPECT_EQ(m.collisions, 0) << name << " " << m.result << " clearance " << m.min_clearance;
  }
}
