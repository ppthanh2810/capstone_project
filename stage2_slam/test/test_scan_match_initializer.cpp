// Unit test cho lõi CSM (không ROS, không phần cứng). Bản đồ/scan tổng hợp, seed cố định.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "stage2_slam/scan_match_initializer.hpp"
#include "stage2_slam/synthetic_world.hpp"

using stage2_slam::GridMap;
using stage2_slam::InitializationSession;
using stage2_slam::MatcherConfig;
using stage2_slam::MatchStatus;
using stage2_slam::Pose2D;
using stage2_slam::ScanData;
using stage2_slam::ScanSample;
using stage2_slam::ScoreField;
using stage2_slam::SessionConfig;
namespace syn = stage2_slam::synthetic;

namespace
{

// LiDAR so với base_footprint theo URDF (lidar_joint xyz 0.15 0 ..., rpy 0; laser_joint identity).
const Pose2D kSensor{0.15, 0.0, 0.0};
constexpr double kDeg = M_PI / 180.0;
// Dung sai vị trí = 1 ô bản đồ (0.05 m): độ chính xác bị giới hạn bởi lượng tử hóa bản đồ (điểm chạm
// nằm bất kỳ đâu trong ô chiếm chỗ). Đo trên dữ liệu tổng hợp: sai số TB ~0.03-0.04 m ở res 0.05 m.
constexpr double kPosTol = 0.05;

// Origin không thẳng hàng với hình học (như bản đồ SLAM thật).
GridMap officeMap(const syn::World & w, const Pose2D & origin = {-3.013, -2.987, 0.0})
{
  return syn::rasterize(w, 0.05, origin, 26.0, 26.0);
}
GridMap corridorMap(const syn::World & w)
{
  return syn::rasterize(w, 0.05, Pose2D{-0.517, -0.013, 0.0}, 61.5, 4.5);
}

ScoreField buildField(const GridMap & map, const MatcherConfig & cfg = MatcherConfig{})
{
  std::string err;
  auto f = ScoreField::build(map, cfg.score_field, &err);
  EXPECT_TRUE(f.has_value()) << err;
  return *f;
}

stage2_slam::MatchResult runMatch(
  const ScoreField & field, const ScanData & scan, const Pose2D & guess,
  const MatcherConfig & cfg = MatcherConfig{}, const Pose2D & sensor = kSensor)
{
  const auto pre = stage2_slam::scanToPoints(scan, sensor, cfg.preprocess);
  return stage2_slam::matchScan(field, pre.points, pre.total_beams, guess, cfg);
}

double posErr(const Pose2D & a, const Pose2D & b) {return std::hypot(a.x - b.x, a.y - b.y);}
double yawErr(const Pose2D & a, const Pose2D & b)
{
  return std::abs(stage2_slam::normalizeAngle(a.yaw - b.yaw));
}

class OfficeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    world = syn::makeOfficeWorld(7);
    map = officeMap(world);
    field = buildField(map);
    std::mt19937 rng(42);
    scan = syn::simulateScan(world, truth, kSensor, syn::ScanSimConfig{}, rng);
  }
  syn::World world;
  GridMap map;
  ScoreField field;
  const Pose2D truth{3.2, 2.7, 0.4};
  ScanData scan;
};

}  // namespace

// --- Score field -----------------------------------------------------------------

TEST(ScoreField, DistanceTransformMatchesBruteForce)
{
  GridMap map = syn::makeMap(2.0, 1.5, 0.1, {});
  std::mt19937 rng(3);
  std::uniform_int_distribution<int> u(0, 9);
  for (auto & v : map.data) {v = u(rng) == 0 ? 100 : 0;}
  MatcherConfig cfg;
  cfg.score_field.sigma = 0.2;
  cfg.score_field.max_distance = 10.0;
  const ScoreField f = buildField(map, cfg);
  for (int cy = 0; cy < map.height; ++cy) {
    for (int cx = 0; cx < map.width; ++cx) {
      double best = 1e9;
      for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
          if (map.data[y * map.width + x] >= 65) {
            best = std::min(best, std::hypot(double(x - cx), double(y - cy)));
          }
        }
      }
      const double d = best * map.resolution;
      const double expected = std::exp(-d * d / (2 * 0.2 * 0.2));
      EXPECT_NEAR(f.scoreAtGrid(cx + 0.5, cy + 0.5), expected, 1e-5) << cx << "," << cy;
    }
  }
}

TEST(ScoreField, RejectsInvalidMaps)
{
  std::string err;
  GridMap empty;
  EXPECT_FALSE(ScoreField::build(empty, {}, &err).has_value());
  GridMap free_map = syn::makeMap(2.0, 2.0, 0.05, {});
  EXPECT_FALSE(ScoreField::build(free_map, {}, &err).has_value());
  EXPECT_NE(err.find("no occupied"), std::string::npos);
  GridMap bad = syn::makeMap(2.0, 2.0, 0.05, {});
  bad.data.pop_back();
  EXPECT_FALSE(ScoreField::build(bad, {}, &err).has_value());
  GridMap bad_res = syn::makeMap(2.0, 2.0, 0.05, {});
  bad_res.resolution = 0.0;
  EXPECT_FALSE(ScoreField::build(bad_res, {}, &err).has_value());
}

TEST(ScoreField, UnknownCellsAndRotatedOrigin)
{
  // Origin khác 0 và có yaw: ô occupied phải được tìm đúng chỗ trong frame map.
  const Pose2D origin{-13.3, -7.0, 0.3};
  GridMap map = syn::makeMap(4.0, 4.0, 0.05, origin);
  map.data[10 * map.width + 20] = 100;   // ô (20, 10)
  map.data[30 * map.width + 30] = -1;    // unknown
  MatcherConfig cfg;
  cfg.score_field.unknown_score = 0.0;
  const ScoreField f = buildField(map, cfg);
  const double lx = 20.5 * 0.05, ly = 10.5 * 0.05;
  const double wx = origin.x + std::cos(0.3) * lx - std::sin(0.3) * ly;
  const double wy = origin.y + std::sin(0.3) * lx + std::cos(0.3) * ly;
  EXPECT_NEAR(f.scoreAtWorld(wx, wy), 1.0, 1e-6);
  double gx, gy;
  f.worldToGrid(wx, wy, gx, gy);
  EXPECT_NEAR(gx, 20.5, 1e-9);
  EXPECT_NEAR(gy, 10.5, 1e-9);
  EXPECT_EQ(f.scoreAtGrid(30.5, 30.5), 0.0);           // unknown -> unknown_score
  EXPECT_EQ(f.scoreAtWorld(origin.x - 5.0, origin.y), 0.0);  // ngoài map
}

// --- Preprocess ------------------------------------------------------------------

TEST(Preprocess, DropsNanInfAndOutOfRange)
{
  ScanData scan;
  scan.angle_min = 0.0;
  scan.angle_increment = M_PI / 2;
  scan.range_min = 0.15;
  scan.range_max = 12.0;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  scan.ranges = {1.0f, nan, inf, -1.0f, 0.05f, 20.0f, 2.0f};
  stage2_slam::PreprocessConfig cfg;
  const auto out = stage2_slam::scanToPoints(scan, Pose2D{}, cfg);
  EXPECT_EQ(out.total_beams, 7u);
  ASSERT_EQ(out.valid_beams, 2u);
  // tia 0: góc 0 -> +X ; tia 6: góc 6*pi/2 = 3pi -> -X
  EXPECT_NEAR(out.points[0].x, 1.0, 1e-9);
  EXPECT_NEAR(out.points[0].y, 0.0, 1e-9);
  EXPECT_NEAR(out.points[1].x, -2.0, 1e-6);
  EXPECT_NEAR(out.max_point_range, 2.0, 1e-9);
}

TEST(Preprocess, NoExtraAngleOffsetAndSensorTransform)
{
  // Scan sau scan_rotate: angle 0 = phía trước laser_frame. Không được cộng thêm pi.
  ScanData scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.1;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  scan.ranges = {2.0f};
  const auto out = stage2_slam::scanToPoints(scan, kSensor, {});
  ASSERT_EQ(out.points.size(), 1u);
  EXPECT_NEAR(out.points[0].x, 2.15, 1e-9);   // phía trước robot, cộng offset LiDAR 0.15 m
  EXPECT_NEAR(out.points[0].y, 0.0, 1e-9);
}

TEST(Preprocess, Downsamples)
{
  ScanData scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.001;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  scan.ranges.assign(1000, 3.0f);
  stage2_slam::PreprocessConfig cfg;
  cfg.max_points = 100;
  const auto out = stage2_slam::scanToPoints(scan, Pose2D{}, cfg);
  EXPECT_EQ(out.valid_beams, 1000u);
  EXPECT_LE(out.points.size(), 100u);
  EXPECT_GE(out.points.size(), 90u);
}

// --- Matching --------------------------------------------------------------------

TEST_F(OfficeTest, ExactPose)
{
  const auto r = runMatch(field, scan, truth);
  ASSERT_EQ(r.status, MatchStatus::ACCEPTED) << r.reason;
  EXPECT_LT(posErr(r.pose, truth), kPosTol);
  EXPECT_LT(yawErr(r.pose, truth), 1.0 * kDeg);
  EXPECT_GT(r.score, 0.5);
}

TEST_F(OfficeTest, OffsetXY)
{
  const Pose2D guess{truth.x + 0.32, truth.y - 0.27, truth.yaw};
  const auto r = runMatch(field, scan, guess);
  ASSERT_EQ(r.status, MatchStatus::ACCEPTED) << r.reason;
  EXPECT_LT(posErr(r.pose, truth), kPosTol);
  EXPECT_LT(yawErr(r.pose, truth), 1.0 * kDeg);
  EXPECT_GT(r.score, r.guess_score);
}

TEST_F(OfficeTest, OffsetYaw)
{
  const Pose2D guess{truth.x - 0.1, truth.y + 0.1, truth.yaw + 13.0 * kDeg};
  const auto r = runMatch(field, scan, guess);
  ASSERT_EQ(r.status, MatchStatus::ACCEPTED) << r.reason;
  EXPECT_LT(posErr(r.pose, truth), kPosTol);
  EXPECT_LT(yawErr(r.pose, truth), 1.0 * kDeg);
}

TEST_F(OfficeTest, TruthOutsideWindowIsNotAccepted)
{
  for (const Pose2D guess : {Pose2D{truth.x + 0.9, truth.y, truth.yaw},
      Pose2D{truth.x, truth.y - 0.8, truth.yaw},
      Pose2D{truth.x, truth.y, truth.yaw + 35.0 * kDeg}})
  {
    const auto r = runMatch(field, scan, guess);
    EXPECT_NE(r.status, MatchStatus::ACCEPTED)
      << "guess (" << guess.x << "," << guess.y << "," << guess.yaw << ") accepted at ("
      << r.pose.x << "," << r.pose.y << "," << r.pose.yaw << ") score " << r.score;
  }
}

TEST_F(OfficeTest, CovarianceFiniteAndBounded)
{
  const auto r = runMatch(field, scan, Pose2D{truth.x + 0.2, truth.y, truth.yaw});
  ASSERT_EQ(r.status, MatchStatus::ACCEPTED) << r.reason;
  MatcherConfig cfg;
  for (double v : r.covariance) {EXPECT_TRUE(std::isfinite(v));}
  EXPECT_GE(r.covariance[0], cfg.covariance.min_std_linear * cfg.covariance.min_std_linear - 1e-12);
  EXPECT_LE(r.covariance[0], cfg.covariance.max_std_linear * cfg.covariance.max_std_linear + 1e-12);
  EXPECT_GE(r.covariance[8], cfg.covariance.min_std_angular * cfg.covariance.min_std_angular - 1e-12);
  // Đối xứng
  EXPECT_DOUBLE_EQ(r.covariance[1], r.covariance[3]);
  EXPECT_DOUBLE_EQ(r.covariance[2], r.covariance[6]);
}

TEST_F(OfficeTest, NanInfAndBadRangesInScanStillMatch)
{
  ScanData noisy = scan;
  for (std::size_t i = 0; i < noisy.ranges.size(); i += 7) {
    noisy.ranges[i] = std::numeric_limits<float>::infinity();
  }
  for (std::size_t i = 3; i < noisy.ranges.size(); i += 11) {noisy.ranges[i] = -2.0f;}
  for (std::size_t i = 5; i < noisy.ranges.size(); i += 13) {noisy.ranges[i] = 0.01f;}
  const auto r = runMatch(field, noisy, Pose2D{truth.x + 0.2, truth.y + 0.2, truth.yaw});
  ASSERT_EQ(r.status, MatchStatus::ACCEPTED) << r.reason;
  EXPECT_LT(posErr(r.pose, truth), kPosTol);
}

TEST_F(OfficeTest, InsufficientPointsGivesNoScan)
{
  ScanData sparse = scan;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  int kept = 0;
  for (auto & r : sparse.ranges) {
    if (std::isfinite(r) && kept < 20) {++kept; continue;}
    r = nan;
  }
  const auto r = runMatch(field, sparse, truth);
  EXPECT_EQ(r.status, MatchStatus::NO_SCAN) << r.reason;

  ScanData all_bad = scan;
  for (auto & v : all_bad.ranges) {v = nan;}
  EXPECT_EQ(runMatch(field, all_bad, truth).status, MatchStatus::NO_SCAN);
}

TEST(Matching, RotatedOffsetMapOrigin)
{
  // Cùng hình học văn phòng, map origin lệch và xoay 0.3 rad. Xử lý origin xoay phải cho sai số
  // tương đương bản đồ không xoay (so sánh thống kê trên nhiều pose, không phải một pose đơn lẻ:
  // phân bố sai số có đuôi ~0.08 m do lượng tử hóa ô 0.05 m).
  const syn::World world = syn::makeOfficeWorld(7);
  auto rmse = [&world](const GridMap & map, unsigned seed, double & max_err) {
      const ScoreField field = buildField(map);
      std::mt19937 rng(seed);
      double s2 = 0.0;
      int n = 0, tried = 0;
      max_err = 0.0;
      while (tried < 20) {
        Pose2D truth;
        EXPECT_TRUE(syn::sampleFreePose(map, 0.5, rng, truth));
        if (truth.x < 0.5 || truth.x > 15.5 || truth.y < 0.5 || truth.y > 9.5) {continue;}
        ++tried;
        const ScanData scan = syn::simulateScan(world, truth, kSensor, syn::ScanSimConfig{}, rng);
        const auto r = runMatch(field, scan, Pose2D{truth.x - 0.25, truth.y + 0.3, truth.yaw - 0.15});
        // AMBIGUOUS/REJECTED là hợp lệ (không phát pose); chỉ đo sai số của ca được chấp nhận.
        if (r.status != MatchStatus::ACCEPTED) {continue;}
        const double e = posErr(r.pose, truth);
        EXPECT_LT(yawErr(r.pose, truth), 1.5 * kDeg);
        s2 += e * e;
        max_err = std::max(max_err, e);
        ++n;
      }
      EXPECT_GE(n, 17);
      return std::sqrt(s2 / std::max(n, 1));
    };
  double max_rot = 0.0, max_ref = 0.0;
  const double rot = rmse(officeMap(world, Pose2D{-3.0, -8.0, 0.3}), 5, max_rot);
  const double ref = rmse(officeMap(world), 5, max_ref);
  EXPECT_LT(rot, kPosTol);
  EXPECT_LT(max_rot, 2 * kPosTol);
  EXPECT_NEAR(rot, ref, 0.02);
}

TEST(Matching, DoubleRotationIsDetected)
{
  // Scan đúng chuẩn (đã qua scan_rotate) khớp; nếu bị cộng thêm pi lần nữa thì không được ACCEPT
  // gần pose thật.
  const syn::World world = syn::makeOfficeWorld(7);
  const GridMap map = officeMap(world);
  const ScoreField field = buildField(map);
  std::mt19937 rng(9);
  const Pose2D truth{9.0, 3.5, 1.1};
  ScanData scan = syn::simulateScan(world, truth, kSensor, syn::ScanSimConfig{}, rng);
  const Pose2D guess{truth.x + 0.15, truth.y - 0.1, truth.yaw + 0.1};
  const auto ok = runMatch(field, scan, guess);
  ASSERT_EQ(ok.status, MatchStatus::ACCEPTED) << ok.reason;
  EXPECT_LT(posErr(ok.pose, truth), kPosTol);

  ScanData twice = scan;
  twice.angle_min += M_PI;
  const auto bad = runMatch(field, twice, guess);
  const bool near_truth = posErr(bad.pose, truth) < 0.1 && yawErr(bad.pose, truth) < 3 * kDeg;
  EXPECT_FALSE(bad.status == MatchStatus::ACCEPTED && near_truth);
  EXPECT_NE(bad.status, MatchStatus::ACCEPTED) << bad.reason;
}

TEST(Matching, FeaturelessCorridorIsAmbiguous)
{
  const syn::World world = syn::makeCorridorWorld();
  const GridMap map = corridorMap(world);
  const ScoreField field = buildField(map);
  std::mt19937 rng(11);
  syn::ScanSimConfig sim;
  sim.range_max = 8.0;  // không thấy hai đầu hành lang
  const Pose2D truth{30.0, 2.0, 0.0};
  const ScanData scan = syn::simulateScan(world, truth, kSensor, sim, rng);
  const auto r = runMatch(field, scan, Pose2D{truth.x + 0.1, truth.y + 0.1, 0.05});
  EXPECT_EQ(r.status, MatchStatus::AMBIGUOUS) << r.reason;
}

TEST(Matching, PeriodicStructureIsAmbiguous)
{
  const syn::World world = syn::makePeriodicCorridorWorld(0.4);
  const GridMap map = corridorMap(world);
  const ScoreField field = buildField(map);
  std::mt19937 rng(13);
  syn::ScanSimConfig sim;
  sim.range_max = 6.0;
  const Pose2D truth{30.2, 2.0, 0.0};
  const ScanData scan = syn::simulateScan(world, truth, kSensor, sim, rng);
  const auto r = runMatch(field, scan, Pose2D{truth.x + 0.05, truth.y, 0.0});
  EXPECT_EQ(r.status, MatchStatus::AMBIGUOUS) << r.reason;
}

TEST(Matching, SearchSpaceAndTimeLimits)
{
  const syn::World world = syn::makeOfficeWorld(7);
  const GridMap map = officeMap(world);
  const ScoreField field = buildField(map);
  std::mt19937 rng(1);
  const Pose2D truth{3.2, 2.7, 0.4};
  const ScanData scan = syn::simulateScan(world, truth, kSensor, syn::ScanSimConfig{}, rng);
  MatcherConfig cfg;
  cfg.search.max_candidates = 1000;
  EXPECT_EQ(runMatch(field, scan, truth, cfg).status, MatchStatus::INVALID_INPUT);
  cfg = MatcherConfig{};
  cfg.search.time_budget = 0.0;
  EXPECT_EQ(runMatch(field, scan, truth, cfg).status, MatchStatus::TIMEOUT);
  EXPECT_EQ(
    runMatch(field, scan, Pose2D{std::nan(""), 0, 0}).status, MatchStatus::INVALID_INPUT);
}

TEST(Matching, OptionalRefinementStaysNearCoarsePeak)
{
  // Refine (tắt mặc định, xem RefineConfig) không được nhảy sang đỉnh khác: |refined - coarse| <=
  // max(linear_step, resolution) và 1 bước góc.
  const syn::World world = syn::makeOfficeWorld(21);
  const GridMap map = officeMap(world);
  MatcherConfig cfg;
  cfg.refine.enabled = true;
  const ScoreField field = buildField(map, cfg);
  std::mt19937 rng(17);
  int n = 0;
  for (int i = 0; i < 8; ++i) {
    Pose2D truth;
    ASSERT_TRUE(syn::sampleFreePose(map, 0.5, rng, truth));
    const ScanData scan = syn::simulateScan(world, truth, kSensor, syn::ScanSimConfig{}, rng);
    const auto r = runMatch(field, scan, Pose2D{truth.x + 0.2, truth.y - 0.1, truth.yaw + 0.05}, cfg);
    if (r.status != MatchStatus::ACCEPTED) {continue;}
    ++n;
    EXPECT_LE(std::abs(r.pose.x - r.coarse_pose.x), 0.05 + 1e-9);
    EXPECT_LE(std::abs(r.pose.y - r.coarse_pose.y), 0.05 + 1e-9);
    EXPECT_LE(yawErr(r.pose, r.coarse_pose), r.angular_step + 1e-9);
    EXPECT_LT(posErr(r.pose, truth), kPosTol + 0.03);
  }
  EXPECT_GE(n, 5);
}

// --- Session (nhiều scan, TF, tuổi dữ liệu) --------------------------------------------

namespace
{
ScanSample sample(const ScanData & scan, double stamp, bool tf_ok = true, Pose2D odom = {})
{
  ScanSample s;
  s.stamp = stamp;
  s.scan = scan;
  if (tf_ok) {
    s.sensor_in_base = kSensor;
    s.odom_pose = odom;
  } else {
    s.tf_error = "lookup laser_frame -> base_footprint failed";
  }
  return s;
}
}  // namespace

TEST_F(OfficeTest, SessionAcceptsConsistentScans)
{
  SessionConfig sc;
  sc.num_scans = 3;
  const Pose2D guess{truth.x + 0.2, truth.y - 0.2, truth.yaw + 0.1};
  InitializationSession s(guess, 100.0, sc, MatcherConfig{});
  std::mt19937 rng(8);
  std::optional<stage2_slam::SessionResult> res;
  for (int i = 0; i < 3 && !res; ++i) {
    const ScanData sc_i = syn::simulateScan(world, truth, kSensor, syn::ScanSimConfig{}, rng);
    res = s.addScan(sample(sc_i, 100.1 + 0.1 * i), 100.15 + 0.1 * i, &field);
  }
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::ACCEPTED) << res->reason;
  EXPECT_TRUE(res->publish);
  EXPECT_EQ(res->matches.size(), 3u);
  EXPECT_LT(posErr(res->final_match.pose, truth), kPosTol);
  EXPECT_DOUBLE_EQ(res->stamp, 100.3);
}

TEST_F(OfficeTest, SessionTfErrorRejectsAndDoesNotPublish)
{
  SessionConfig sc;
  sc.max_tf_failures = 3;
  InitializationSession s(truth, 0.0, sc, MatcherConfig{});
  // Lỗi TF lẻ tẻ: bỏ qua scan; đủ max_tf_failures lần liên tiếp -> TF_ERROR.
  EXPECT_FALSE(s.addScan(sample(scan, 0.1, false), 0.12, &field).has_value());
  EXPECT_FALSE(s.addScan(sample(scan, 0.2, false), 0.22, &field).has_value());
  const auto res = s.addScan(sample(scan, 0.3, false), 0.32, &field);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::TF_ERROR);
  EXPECT_NE(res->reason.find("laser_frame"), std::string::npos);
  EXPECT_FALSE(res->publish);
}

TEST_F(OfficeTest, SessionTfErrorAtTimeout)
{
  SessionConfig sc;
  sc.timeout = 1.0;
  InitializationSession s(truth, 0.0, sc, MatcherConfig{});
  EXPECT_FALSE(s.addScan(sample(scan, 0.1, false), 0.12, &field).has_value());
  const auto res = s.checkTimeout(1.5);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::TF_ERROR);
  EXPECT_FALSE(res->publish);
}

TEST_F(OfficeTest, SessionNoMap)
{
  InitializationSession s(truth, 0.0, SessionConfig{}, MatcherConfig{});
  const auto res = s.addScan(sample(scan, 0.1), 0.12, nullptr);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::NO_MAP);
  EXPECT_FALSE(res->publish);
}

TEST_F(OfficeTest, SessionStaleAndFutureScansIgnoredThenNoScan)
{
  SessionConfig sc;
  sc.scan_max_age = 0.5;
  sc.timeout = 2.0;
  InitializationSession s(truth, 10.0, sc, MatcherConfig{});
  EXPECT_FALSE(s.addScan(sample(scan, 8.0), 10.1, &field).has_value());   // quá cũ
  EXPECT_FALSE(s.addScan(sample(scan, 11.0), 10.2, &field).has_value());  // ở tương lai
  EXPECT_FALSE(s.checkTimeout(11.0).has_value());
  const auto res = s.checkTimeout(12.5);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::NO_SCAN);
  EXPECT_EQ(res->stale_scans, 2u);
  EXPECT_FALSE(res->publish);
}

TEST_F(OfficeTest, SessionTimeoutAfterPartialScans)
{
  SessionConfig sc;
  sc.num_scans = 3;
  sc.timeout = 1.0;
  InitializationSession s(truth, 0.0, sc, MatcherConfig{});
  EXPECT_FALSE(s.addScan(sample(scan, 0.1), 0.15, &field).has_value());
  const auto res = s.checkTimeout(1.5);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::TIMEOUT);
  EXPECT_FALSE(res->publish);
}

TEST_F(OfficeTest, SessionRejectsRobotMotion)
{
  SessionConfig sc;
  sc.num_scans = 2;
  InitializationSession s(truth, 0.0, sc, MatcherConfig{});
  EXPECT_FALSE(s.addScan(sample(scan, 0.1, true, Pose2D{0, 0, 0}), 0.15, &field).has_value());
  const auto res = s.addScan(sample(scan, 0.2, true, Pose2D{0.2, 0, 0}), 0.25, &field);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::REJECTED);
  EXPECT_FALSE(res->publish);
}

TEST_F(OfficeTest, SessionRejectedMatchDoesNotPublish)
{
  // Guess nằm ngoài cửa sổ -> scan đầu tiên bị từ chối -> phiên kết thúc, không publish.
  InitializationSession s(Pose2D{truth.x + 1.0, truth.y, truth.yaw}, 0.0, SessionConfig{},
    MatcherConfig{});
  const auto res = s.addScan(sample(scan, 0.1), 0.15, &field);
  ASSERT_TRUE(res.has_value());
  EXPECT_NE(res->status, MatchStatus::ACCEPTED);
  EXPECT_FALSE(res->publish);
}

TEST(Session, AmbiguousDoesNotPublish)
{
  const syn::World world = syn::makeCorridorWorld();
  const GridMap map = corridorMap(world);
  const ScoreField field = buildField(map);
  std::mt19937 rng(11);
  syn::ScanSimConfig sim;
  sim.range_max = 8.0;
  const Pose2D truth{30.0, 2.0, 0.0};
  InitializationSession s(truth, 0.0, SessionConfig{}, MatcherConfig{});
  const auto res = s.addScan(
    sample(syn::simulateScan(world, truth, kSensor, sim, rng), 0.1), 0.15, &field);
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->status, MatchStatus::AMBIGUOUS);
  EXPECT_FALSE(res->publish);
}

TEST(Geometry, ComposeBetweenRoundTrip)
{
  const Pose2D a{1.0, -2.0, 0.7}, b{0.3, 0.4, -2.9};
  const Pose2D c = stage2_slam::compose(a, b);
  const Pose2D d = stage2_slam::between(a, c);
  EXPECT_NEAR(d.x, b.x, 1e-12);
  EXPECT_NEAR(d.y, b.y, 1e-12);
  EXPECT_NEAR(stage2_slam::normalizeAngle(d.yaw - b.yaw), 0.0, 1e-12);
}
