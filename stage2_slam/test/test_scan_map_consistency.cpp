// Test distance filter (scan_map_consistency) + ngưỡng blocked_ratio của localization_health.
// Dữ liệu tổng hợp: phòng 8 x 6 m (0.05 m/ô) có vách ngăn và cột (để vị trí phân biệt được),
// scan nửa trước 180° / 360 tia như /scan_filtered. Vật cản "lạ" chỉ có trong lưới sinh scan,
// không có trong bản đồ đưa cho monitor.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "stage2_slam/localization_health.hpp"
#include "stage2_slam/scan_map_consistency.hpp"

namespace stage2_slam
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRes = 0.05;

GridMap makeRoom()
{
  GridMap m;
  m.width = 160;   // 8 m
  m.height = 120;  // 6 m
  m.resolution = kRes;
  m.data.assign(static_cast<std::size_t>(m.width) * m.height, 0);
  auto set = [&m](int x, int y) {m.data[static_cast<std::size_t>(y) * m.width + x] = 100;};
  for (int x = 0; x < m.width; ++x) {set(x, 0); set(x, m.height - 1);}
  for (int y = 0; y < m.height; ++y) {set(0, y); set(m.width - 1, y);}
  for (int y = 70; y < m.height; ++y) {set(100, y);}            // vách ngăn x = 5 m, y >= 3.5 m
  for (int x = 30; x < 36; ++x) {for (int y = 20; y < 26; ++y) {set(x, y);}}  // cột 0.3 m
  for (int x = 120; x < 160; ++x) {set(x, 30);}                  // tường lửng y = 1.5 m, x >= 6 m
  return m;
}

// Thêm hình chữ nhật chiếm chỗ (toạ độ m) vào lưới.
void addBox(GridMap & m, double x0, double y0, double x1, double y1)
{
  for (int y = static_cast<int>(y0 / kRes); y <= static_cast<int>(y1 / kRes); ++y) {
    for (int x = static_cast<int>(x0 / kRes); x <= static_cast<int>(x1 / kRes); ++x) {
      m.data[static_cast<std::size_t>(y) * m.width + x] = 100;
    }
  }
}

// Scan 180° phía trước (giống /scan_filtered), điểm trong frame cảm biến. Điểm cuối đặt sâu
// 0.025 m vào ô chiếm chỗ (tia LiDAR thật dừng ở bề mặt vật, nằm trong ô tường của bản đồ).
std::vector<Point2D> simulateScan(const GridMap & world, const Pose2D & pose)
{
  const MapRaycaster rc(world, 65);
  std::vector<Point2D> pts;
  const int n = 360;
  for (int i = 0; i < n; ++i) {
    const double a = -kPi / 2 + kPi * i / (n - 1);
    const auto r = rc.cast(pose.x, pose.y, pose.yaw + a, 12.0);
    if (!r || *r < 0.10) {continue;}
    const double range = *r + 0.025;
    pts.push_back({range * std::cos(a), range * std::sin(a)});
  }
  return pts;
}

struct Fixture
{
  GridMap map = makeRoom();
  ScoreField field = *ScoreField::build(map, ScoreFieldConfig{});
  MapRaycaster raycaster{map, 65};
  DistanceFilterConfig cfg;
  HealthConfig health;

  Fixture() {health.consistency_warn = 0.45;}  // giá trị trong localization_monitor.yaml

  ConsistencyResult eval(const std::vector<Point2D> & pts, const Pose2D & estimate) const
  {
    return evaluateScanConsistency(field, raycaster, pts, estimate, cfg);
  }

  // Monitor sẽ KHÔNG báo LOCALIZED (theo riêng tiêu chí scan) khi hàm này trả về true.
  bool flagged(const ConsistencyResult & r) const
  {
    return !r.consistency || *r.consistency < health.consistency_warn ||
           r.short_ratio > health.blocked_ratio_warn;
  }
};

const Pose2D kTruth{2.5, 2.0, 0.3};

TEST(MapRaycaster, HitsWallAtExpectedDistance)
{
  const auto map = makeRoom();
  const MapRaycaster rc(map, 65);
  // Từ (2.5, 2.0) nhìn về +x: cột ở x 1.5..1.8 nằm phía sau; tường phải x = 7.95 m.
  const auto r = rc.cast(2.5, 2.0, 0.0, 12.0);
  ASSERT_TRUE(r);
  EXPECT_NEAR(*r, 7.95 - 2.5, kRes);
  // Nhìn về -y: tường dưới (ô y = 0, mép trên tại 0.05 m).
  const auto d = rc.cast(2.5, 2.0, -kPi / 2, 12.0);
  ASSERT_TRUE(d);
  EXPECT_NEAR(*d, 2.0 - 0.05, kRes);
  // Tầm quá ngắn -> không gặp gì.
  EXPECT_FALSE(rc.cast(2.5, 2.0, 0.0, 1.0));
  // Gốc nằm ngoài bản đồ -> 0.
  const auto out = rc.cast(-1.0, 2.0, 0.0, 12.0);
  ASSERT_TRUE(out);
  EXPECT_DOUBLE_EQ(*out, 0.0);
}

TEST(DistanceFilter, CorrectPoseNoObstacle)
{
  Fixture f;
  const auto r = f.eval(simulateScan(f.map, kTruth), kTruth);
  ASSERT_TRUE(r.consistency);
  EXPECT_GT(*r.consistency, 0.9);
  EXPECT_LT(r.short_ratio, 0.05);
  EXPECT_FALSE(f.flagged(r));
}

// Trường hợp robot 2026-10-17: vật lớn gần phía trước làm consistency cũ tụt dưới consistency_warn.
TEST(DistanceFilter, UnmappedObstacleInFrontIsNotLost)
{
  Fixture f;
  GridMap world = f.map;
  // Tấm chắn 1.2 m, cách LiDAR ~0.5 m, vuông góc với hướng robot (yaw 0.3).
  const double cx = kTruth.x + 0.55 * std::cos(kTruth.yaw);
  const double cy = kTruth.y + 0.55 * std::sin(kTruth.yaw);
  addBox(world, cx - 0.10, cy - 0.60, cx + 0.10, cy + 0.60);
  const auto r = f.eval(simulateScan(world, kTruth), kTruth);
  std::printf("[obstacle] all=%.3f filtered=%.3f blocked=%.3f\n",
    r.consistency_all, r.consistency.value_or(-1), r.short_ratio);
  EXPECT_LT(r.consistency_all, 0.6);   // cách cũ: tụt mạnh
  ASSERT_TRUE(r.consistency);
  EXPECT_GT(*r.consistency, 0.9);      // cách mới: vẫn khớp tường
  EXPECT_GT(r.short_ratio, 0.3);
  EXPECT_FALSE(f.flagged(r));
}

TEST(DistanceFilter, ChairBesidePathIsNotLost)
{
  Fixture f;
  GridMap world = f.map;
  addBox(world, 3.4, 2.4, 3.8, 2.8);   // ghế 0.4 x 0.4 m phía trước-trái
  const auto r = f.eval(simulateScan(world, kTruth), kTruth);
  ASSERT_TRUE(r.consistency);
  EXPECT_GT(*r.consistency, 0.9);
  EXPECT_FALSE(f.flagged(r));
}

// Vật che gần hết tầm nhìn: không còn đủ tường để kiểm tra -> phải báo (blocked_ratio).
TEST(DistanceFilter, FullyOccludedScanIsFlagged)
{
  Fixture f;
  GridMap world = f.map;
  // Hộp chữ U bao quanh nửa trước robot ở ~0.4 m.
  const double x = kTruth.x, y = kTruth.y;
  addBox(world, x - 0.1, y + 0.4, x + 0.6, y + 0.5);
  addBox(world, x - 0.1, y - 0.5, x + 0.6, y - 0.4);
  addBox(world, x + 0.5, y - 0.5, x + 0.6, y + 0.5);
  const auto r = f.eval(simulateScan(world, kTruth), kTruth);
  std::printf("[occluded] blocked=%.3f counted=%zu\n", r.short_ratio, r.counted);
  EXPECT_GT(r.short_ratio, f.health.blocked_ratio_warn);
  EXPECT_TRUE(f.flagged(r));
}

// Pose ước lượng sai (AMCL lạc), không có vật lạ: phải bị phát hiện như cách cũ.
struct LostCase
{
  const char * name;
  Pose2D error;   // estimate = truth ⊕ error (frame robot)
};

class DistanceFilterLost : public ::testing::TestWithParam<LostCase> {};

TEST_P(DistanceFilterLost, WrongPoseIsFlagged)
{
  Fixture f;
  const auto pts = simulateScan(f.map, kTruth);
  const Pose2D estimate = compose(kTruth, GetParam().error);
  const auto r = f.eval(pts, estimate);
  std::printf("[lost %s] all=%.3f filtered=%.3f blocked=%.3f through_wall=%.3f\n",
    GetParam().name, r.consistency_all, r.consistency.value_or(-1), r.short_ratio, r.long_ratio);
  EXPECT_TRUE(f.flagged(r));
}

INSTANTIATE_TEST_SUITE_P(
  Errors, DistanceFilterLost,
  ::testing::Values(
    LostCase{"yaw_+0.2", {0.0, 0.0, 0.2}},
    LostCase{"yaw_-0.35", {0.0, 0.0, -0.35}},
    LostCase{"left_0.3", {0.0, 0.3, 0.0}},
    LostCase{"right_0.5", {0.0, -0.5, 0.0}},
    LostCase{"forward_0.4", {0.4, 0.0, 0.0}},
    LostCase{"backward_0.4", {-0.4, 0.0, 0.0}},
    LostCase{"kidnap_1.5m", {1.5, -1.0, 1.0}}),
  [](const ::testing::TestParamInfo<LostCase> & i) {
    std::string n = i.param.name;
    for (auto & ch : n) {if (!std::isalnum(static_cast<unsigned char>(ch))) {ch = '_';}}
    return n;
  });

// Có nhiễu: range σ 0.02 m, pose ước lượng lệch (0.04 m, 0.03 rad) như AMCL thật, thêm vật cản.
std::vector<Point2D> addNoise(std::vector<Point2D> pts, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 0.02);
  for (auto & p : pts) {
    const double r = std::hypot(p.x, p.y);
    const double k = (r + n(rng)) / r;
    p.x *= k;
    p.y *= k;
  }
  return pts;
}

TEST(DistanceFilter, NoisyCorrectPoseWithObstacleStaysLocalized)
{
  Fixture f;
  GridMap world = f.map;
  addBox(world, 3.2, 2.0, 3.4, 2.9);   // vật 0.9 m chắn trước-trái
  const Pose2D estimate = compose(kTruth, {0.04, -0.03, 0.03});
  for (unsigned seed = 1; seed <= 5; ++seed) {
    const auto r = f.eval(addNoise(simulateScan(world, kTruth), seed), estimate);
    std::printf("[noisy ok %u] all=%.3f filtered=%.3f blocked=%.3f\n",
      seed, r.consistency_all, r.consistency.value_or(-1), r.short_ratio);
    EXPECT_FALSE(f.flagged(r)) << "seed " << seed;
  }
}

TEST(DistanceFilter, NoisyWrongPoseIsFlagged)
{
  Fixture f;
  const Pose2D errors[] = {{0.0, 0.0, 0.15}, {0.0, 0.25, 0.0}, {-0.3, 0.0, 0.0}, {0.25, 0.0, 0.0}};
  for (const auto & e : errors) {
    const auto r = f.eval(addNoise(simulateScan(f.map, kTruth), 7), compose(kTruth, e));
    std::printf("[noisy lost %.2f %.2f %.2f] filtered=%.3f blocked=%.3f\n",
      e.x, e.y, e.yaw, r.consistency.value_or(-1), r.short_ratio);
    EXPECT_TRUE(f.flagged(r));
  }
}

TEST(DistanceFilter, DisabledMatchesOldScore)
{
  Fixture f;
  f.cfg.enabled = false;
  const auto pts = simulateScan(f.map, kTruth);
  const auto r = f.eval(pts, kTruth);
  ASSERT_TRUE(r.consistency);
  EXPECT_DOUBLE_EQ(*r.consistency, scorePose(f.field, pts, kTruth));
  EXPECT_EQ(r.short_beams, 0u);
}

TEST(LocalizationHealth, BlockedRatioWarns)
{
  HealthInputs in;
  in.now = 100.0;
  in.amcl_active = true;
  in.map_received = true;
  in.amcl_pose_received = true;
  in.scan_stamp = 100.0;
  in.odom_stamp = 100.0;
  in.tf_stamp = 100.0;
  in.std_x = in.std_y = 0.05;
  in.std_yaw = 0.05;
  in.scan_consistency = 0.95;
  HealthConfig cfg;
  HealthMemory mem;

  in.blocked_ratio = 0.4;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOCALIZED);
  in.blocked_ratio = 0.9;
  const auto rep = evaluateHealth(in, cfg, mem);
  EXPECT_EQ(rep.state, HealthState::UNCERTAIN);
  ASSERT_FALSE(rep.reasons.empty());
  EXPECT_NE(rep.reasons[0].find("unmapped"), std::string::npos);
  in.blocked_ratio.reset();   // distance filter tắt -> không kiểm tra
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOCALIZED);
}

}  // namespace
}  // namespace stage2_slam
