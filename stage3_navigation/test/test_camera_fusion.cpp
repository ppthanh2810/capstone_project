// =====================================================================================================
// test_camera_fusion.cpp — Unit test lõi hợp nhất camera + LiDAR (M2), KHÔNG cần ROS graph
// =====================================================================================================
// Frame độ sâu tổng hợp: ray-cast từ camera (cao 0.276 m như URDF camera_joint) vào sàn + hộp 3D + tường xa,
// lưới tia ~ ảnh 212×120 sau decimation của D435 (87° × 58°). Kiểm tra từng dòng của bảng luật
// (doc/obstacle_perception_plan.md mục 2) và các ca thận trọng (lỗ độ sâu, bị che, quá gần).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "stage3_navigation/camera_fusion.hpp"

using stage3_navigation::CameraFusionGrid;
using stage3_navigation::CameraObservation;
using stage3_navigation::CellState;
using stage3_navigation::FusionParams;
using stage3_navigation::Point3;

namespace
{
constexpr double kCamZ = 0.1135 + 0.1625;   // base_footprint -> base_link -> camera_link (URDF)
constexpr double kCamX = 0.17;

struct Box3
{
  double x0, y0, z0, x1, y1, z1;
};

// Khoảng cách tia tới hộp (slab), inf nếu trượt.
double rayBox(const Point3 & o, const Point3 & d, const Box3 & b)
{
  double tmin = 0.0, tmax = std::numeric_limits<double>::infinity();
  const double oo[3] = {o.x, o.y, o.z}, dd[3] = {d.x, d.y, d.z};
  const double lo[3] = {b.x0, b.y0, b.z0}, hi[3] = {b.x1, b.y1, b.z1};
  for (int k = 0; k < 3; ++k) {
    if (std::abs(dd[k]) < 1e-12) {
      if (oo[k] < lo[k] || oo[k] > hi[k]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t0 = (lo[k] - oo[k]) / dd[k], t1 = (hi[k] - oo[k]) / dd[k];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);
    if (tmin > tmax) {
      return std::numeric_limits<double>::infinity();
    }
  }
  return tmin > 1e-9 ? tmin : std::numeric_limits<double>::infinity();
}

struct Scene
{
  std::vector<Box3> boxes;
  bool wall{true};            // tường xa x = robot + 4 m (cho tia "nhìn xuyên" có điểm dội)
  bool floor{true};
  // Vùng không có độ sâu (vật đen / trong suốt): bỏ tia có phương vị trong [a0, a1] (rad, frame camera).
  double hole_az0{1.0}, hole_az1{-1.0};
};

// Robot tại (rx, ry, yaw) -> một frame camera.
CameraObservation frame(const Scene & s, double rx, double ry, double yaw, double stamp)
{
  CameraObservation obs;
  obs.stamp = stamp;
  obs.sensor = Point3{rx + kCamX * std::cos(yaw), ry + kCamX * std::sin(yaw), kCamZ};
  obs.sensor_yaw = yaw;
  obs.robot_x = rx;
  obs.robot_y = ry;
  const int naz = 212, nel = 120;
  const double hf = 87.0 * M_PI / 180.0, vf = 58.0 * M_PI / 180.0;
  std::vector<Box3> boxes = s.boxes;
  if (s.wall) {
    boxes.push_back(Box3{rx + 4.0, -50, 0, rx + 4.2, 50, 3.0});
  }
  for (int a = 0; a < naz; ++a) {
    const double az = -hf / 2 + hf * (a + 0.5) / naz;
    if (az >= s.hole_az0 && az <= s.hole_az1) {
      continue;
    }
    for (int e = 0; e < nel; ++e) {
      const double el = -vf / 2 + vf * (e + 0.5) / nel;
      const Point3 d{std::cos(el) * std::cos(az + yaw), std::cos(el) * std::sin(az + yaw), std::sin(
          el)};
      double t = std::numeric_limits<double>::infinity();
      if (s.floor && d.z < 0.0) {
        t = -obs.sensor.z / d.z;
      }
      for (const auto & b : boxes) {
        t = std::min(t, rayBox(obs.sensor, d, b));
      }
      if (!std::isfinite(t) || t > 10.0) {
        continue;   // D435: không có độ sâu
      }
      obs.points.push_back(
        Point3{obs.sensor.x + t * d.x, obs.sensor.y + t * d.y,
          obs.sensor.z + t * d.z});
    }
  }
  return obs;
}

// Điểm cuối tia LiDAR (mặt quét cao 0.347 m) chạm các hộp cắt mặt quét, nửa trước.
std::vector<std::pair<double, double>> lidar(const Scene & s, double rx, double ry, double yaw)
{
  std::vector<std::pair<double, double>> hits;
  const double lz = 0.1135 + 0.23365;
  const Point3 o{rx + 0.15 * std::cos(yaw), ry + 0.15 * std::sin(yaw), lz};
  for (int k = 0; k < 360; ++k) {
    const double a = -M_PI / 2 + M_PI * k / 360.0 + yaw;
    const Point3 d{std::cos(a), std::sin(a), 0.0};
    double t = std::numeric_limits<double>::infinity();
    for (const auto & b : s.boxes) {
      t = std::min(t, rayBox(o, d, b));
    }
    if (std::isfinite(t) && t < 2.5) {
      hits.emplace_back(o.x + t * d.x, o.y + t * d.y);
    }
  }
  return hits;
}

bool drawnNear(const CameraFusionGrid & g, double now, double x, double y, double r)
{
  for (const auto & c : g.obstacleCells(now)) {
    if (std::hypot(c.first - x, c.second - y) <= r) {
      return true;
    }
  }
  return false;
}

// Mặt bàn: z 0.45–0.65, LiDAR (0.347 m) KHÔNG thấy.
const Box3 kTableTop{1.4, -0.3, 0.45, 1.8, 0.3, 0.65};
// Thùng thấp 0.12 m: LiDAR không thấy.
const Box3 kLowBox{1.2, -0.15, 0.0, 1.4, 0.15, 0.12};
// Thùng cao 0.6 m: cả hai thấy.
const Box3 kTallBox{1.5, -0.2, 0.0, 1.8, 0.2, 0.6};
}  // namespace

TEST(CameraFusion, ParamsValidation)
{
  EXPECT_EQ(stage3_navigation::validateFusionParams(FusionParams()), "");
  FusionParams p;
  p.min_obstacle_height = 0.8;
  EXPECT_NE(stage3_navigation::validateFusionParams(p), "");
  p = FusionParams();
  p.confirm_frames = 0;
  EXPECT_NE(stage3_navigation::validateFusionParams(p), "");
  p = FusionParams();
  p.resolution = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(stage3_navigation::validateFusionParams(p), "");
  p = FusionParams();
  p.min_range = 3.0;
  EXPECT_NE(stage3_navigation::validateFusionParams(p), "");
}

TEST(CameraFusion, FloorAndWallOnlyMarkNothingInRange)
{
  CameraFusionGrid g;
  Scene s;
  for (int k = 0; k < 10; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.1 * k));
  }
  EXPECT_EQ(g.pendingCount() + g.confirmedCount(), 0u);   // sàn < min height, tường > max_range
}

// Dòng 2 của bảng luật: LiDAR không thấy, camera thấy -> vật cản sau N frame liên tiếp.
TEST(CameraFusion, CameraOnlyNeedsConsecutiveFrames)
{
  CameraFusionGrid g;   // confirm_frames 3
  Scene s;
  s.boxes = {kTableTop};
  g.setLidarHits(0.0, lidar(s, 0, 0, 0));
  EXPECT_TRUE(lidar(s, 0, 0, 0).empty());
  g.addCameraFrame(frame(s, 0, 0, 0, 0.0));
  EXPECT_EQ(g.state(1.425, 0.025), CellState::PENDING);
  EXPECT_FALSE(drawnNear(g, 0.0, 1.45, 0.0, 0.1));
  g.addCameraFrame(frame(s, 0, 0, 0, 0.066));
  EXPECT_FALSE(drawnNear(g, 0.066, 1.45, 0.0, 0.1));
  g.addCameraFrame(frame(s, 0, 0, 0, 0.133));
  EXPECT_EQ(g.state(1.425, 0.025), CellState::CONFIRMED);
  EXPECT_TRUE(drawnNear(g, 0.133, 1.425, 0.025, 0.04));
  // Không có ô nào ngoài mặt bàn (± 1 ô).
  for (const auto & c : g.obstacleCells(0.133)) {
    EXPECT_GE(c.first, kTableTop.x0 - 0.05);
    EXPECT_LE(c.first, kTableTop.x1 + 0.05);
    EXPECT_LE(std::abs(c.second), 0.35);
  }
}

TEST(CameraFusion, FlickeringNoiseNeverConfirms)
{
  CameraFusionGrid g;
  Scene with, without;
  with.boxes = {kLowBox};
  for (int k = 0; k < 12; ++k) {
    g.addCameraFrame(frame(k % 2 ? without : with, 0, 0, 0, 0.066 * k));
    EXPECT_EQ(g.confirmedCount(), 0u) << k;
  }
}

TEST(CameraFusion, SparsePointsBelowMinPointsIgnored)
{
  CameraFusionGrid g;
  for (int k = 0; k < 10; ++k) {
    CameraObservation o = frame(Scene(), 0, 0, 0, 0.066 * k);
    o.points.push_back(Point3{1.0, 0.5, 0.3});   // một điểm nhiễu cố định
    o.points.push_back(Point3{1.0, 0.5, 0.31});
    g.addCameraFrame(o);
  }
  EXPECT_EQ(g.pendingCount() + g.confirmedCount(), 0u);
}

// Dòng 1: cả hai thấy -> xác nhận ngay; ô gần điểm LiDAR không vẽ (vị trí theo LiDAR / obstacle_layer).
TEST(CameraFusion, BothSeeConfirmsImmediatelyAndDefersToLidar)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kTallBox};
  const auto hits = lidar(s, 0, 0, 0);
  ASSERT_FALSE(hits.empty());
  g.setLidarHits(0.0, hits);
  g.addCameraFrame(frame(s, 0, 0, 0, 0.0));
  // Mặt trước của thùng (x = 1.5) được camera thấy và xác nhận ngay sau 1 frame.
  EXPECT_EQ(g.state(1.525, 0.025), CellState::CONFIRMED);
  std::size_t suppressed = 0;
  const auto cells = g.obstacleCells(0.0, &suppressed);
  EXPECT_GT(suppressed, 0u);
  for (const auto & c : cells) {   // không ô nào được vẽ trong bán kính liên kết của LiDAR
    for (const auto & h : hits) {
      EXPECT_GT(std::hypot(c.first - h.first, c.second - h.second), 0.10 - 1e-9);
    }
  }
  // LiDAR cũ (> lidar_max_age): camera tự vẽ.
  EXPECT_TRUE(drawnNear(g, 1.0, 1.525, 0.025, 0.04));
}

// Dòng 4: vật bị dời -> camera nhìn xuyên -> xóa sau clear_frames.
TEST(CameraFusion, RemovedObjectClearedBySeeingThrough)
{
  CameraFusionGrid g;   // clear_frames 2
  Scene s;
  s.boxes = {kTableTop};
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  ASSERT_GT(g.confirmedCount(), 0u);
  const Scene empty;
  g.addCameraFrame(frame(empty, 0, 0, 0, 0.3));
  EXPECT_GT(g.confirmedCount(), 0u);   // 1 frame chưa đủ
  g.addCameraFrame(frame(empty, 0, 0, 0, 0.366));
  EXPECT_EQ(g.confirmedCount(), 0u);
}

TEST(CameraFusion, LowObjectMarkedAndCleared)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kLowBox};
  EXPECT_TRUE(lidar(s, 0, 0, 0).empty());   // LiDAR quét trên đầu thùng
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  EXPECT_TRUE(drawnNear(g, 0.2, 1.225, 0.0, 0.05));
  for (int k = 3; k < 5; ++k) {
    g.addCameraFrame(frame(Scene(), 0, 0, 0, 0.066 * k));
  }
  EXPECT_EQ(g.confirmedCount(), 0u);
}

// Lỗ độ sâu (vật đen / trong suốt) -> KHÔNG xóa; chỉ hết hạn decay_time.
TEST(CameraFusion, DepthHoleDoesNotClearButDecays)
{
  CameraFusionGrid g;   // decay 10 s
  Scene s;
  s.boxes = {kTableTop};
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  const std::size_t n = g.confirmedCount();
  ASSERT_GT(n, 0u);
  Scene hole;
  hole.hole_az0 = -0.4;
  hole.hole_az1 = 0.4;
  for (int k = 0; k < 20; ++k) {
    g.addCameraFrame(frame(hole, 0, 0, 0, 0.2 + 0.066 * k));
  }
  EXPECT_EQ(g.confirmedCount(), n);
  g.decay(5.0, 0, 0);
  EXPECT_EQ(g.confirmedCount(), n);
  g.decay(10.5, 0, 0);
  EXPECT_EQ(g.confirmedCount(), 0u);
}

// Vật bị che bởi vật gần hơn -> không xóa.
TEST(CameraFusion, OccludedCellNotCleared)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kTableTop};
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  const std::size_t n = g.confirmedCount();
  Scene blocker;   // tấm chắn cao ở x = 0.9 che toàn bộ mặt bàn; mặt bàn đã bị dời
  blocker.boxes = {Box3{0.9, -0.6, 0.0, 0.95, 0.6, 1.5}};
  for (int k = 0; k < 5; ++k) {
    g.addCameraFrame(frame(blocker, 0, 0, 0, 0.3 + 0.066 * k));
  }
  for (const auto & c : g.obstacleCells(0.7)) {
    (void)c;
  }
  EXPECT_GE(g.confirmedCount(), n);   // mặt bàn còn nguyên (+ tấm chắn)
  EXPECT_EQ(g.state(1.425, 0.025), CellState::CONFIRMED);
}

// Robot tới gần mặt bàn: mặt bàn ra khỏi góc nhìn đứng -> không có thông tin -> giữ nguyên.
TEST(CameraFusion, TooCloseForVerticalFovKeepsCell)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kTableTop};
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  ASSERT_EQ(g.state(1.425, 0.025), CellState::CONFIRMED);
  // Camera ở x = 0.17 + 0.9 = 1.07 -> mặt bàn cách 0.35 m, mép trên ngoài 29° (cần >= 0.68 m).
  for (int k = 0; k < 6; ++k) {
    g.addCameraFrame(frame(Scene(), 0.9, 0, 0, 0.3 + 0.066 * k));
  }
  EXPECT_EQ(g.state(1.425, 0.025), CellState::CONFIRMED);
}

// Ô ngoài góc nhìn ngang (robot quay đi) -> giữ cho tới decay_time.
TEST(CameraFusion, OutsideHorizontalFovKeptUntilDecay)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kLowBox};
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  const std::size_t n = g.confirmedCount();
  for (int k = 0; k < 10; ++k) {
    g.addCameraFrame(frame(Scene(), 0, 0, M_PI / 2, 1.0 + 0.066 * k));
  }
  EXPECT_EQ(g.confirmedCount(), n);
  g.decay(9.0, 0, 0);
  EXPECT_EQ(g.confirmedCount(), n);
  g.decay(10.2, 0, 0);
  EXPECT_EQ(g.confirmedCount(), 0u);
}

TEST(CameraFusion, SelfFilterAndRangeLimits)
{
  CameraFusionGrid g;
  for (int k = 0; k < 5; ++k) {
    CameraObservation o = frame(Scene(), 0, 0, 0, 0.066 * k);
    for (int m = 0; m < 10; ++m) {
      o.points.push_back(Point3{0.1, 0.0, 0.3});    // thân robot (< self_filter_radius)
      o.points.push_back(Point3{0.4, 0.0, 0.3});    // quá gần camera (< min_range 0.30 từ x = 0.17)
      o.points.push_back(Point3{3.5, 0.0, 0.3});    // quá xa (> max_range)
      o.points.push_back(Point3{1.0, 0.0, 0.9});    // trên max_obstacle_height
    }
    g.addCameraFrame(o);
  }
  EXPECT_EQ(g.pendingCount() + g.confirmedCount(), 0u);
}

TEST(CameraFusion, FarCellsPrunedAroundRobot)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kLowBox};
  for (int k = 0; k < 3; ++k) {
    g.addCameraFrame(frame(s, 0, 0, 0, 0.066 * k));
  }
  ASSERT_GT(g.confirmedCount(), 0u);
  g.decay(0.3, -3.0, 0.0);   // robot đã đi xa > max_cell_distance 3 m
  EXPECT_EQ(g.confirmedCount(), 0u);
}

TEST(CameraFusion, NonFiniteInputIgnored)
{
  CameraFusionGrid g;
  CameraObservation o = frame(Scene(), 0, 0, 0, 0.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  o.points.push_back(Point3{nan, 0.0, 0.3});
  o.points.push_back(Point3{1.0, std::numeric_limits<double>::infinity(), 0.3});
  g.addCameraFrame(o);
  g.setLidarHits(0.0, {{nan, 1.0}});
  EXPECT_EQ(g.pendingCount() + g.confirmedCount(), 0u);
}

// Tải: frame ~25k điểm (212×120) phải nhanh hơn nhiều so với chu kỳ 15 Hz.
TEST(CameraFusion, FrameProcessingTime)
{
  CameraFusionGrid g;
  Scene s;
  s.boxes = {kTableTop, kLowBox, Box3{0.8, 0.5, 0.0, 1.0, 0.7, 0.8}};
  std::vector<CameraObservation> frames;
  for (int k = 0; k < 30; ++k) {
    frames.push_back(frame(s, 0.01 * k, 0, 0.01 * k, 0.066 * k));
  }
  const auto t0 = std::chrono::steady_clock::now();
  for (const auto & f : frames) {
    g.addCameraFrame(f);
  }
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count() / frames.size();
  RecordProperty("ms_per_frame", std::to_string(ms));
  std::printf("points/frame %zu, %.2f ms/frame\n", frames[0].points.size(), ms);
  EXPECT_LT(ms, 20.0);
}
