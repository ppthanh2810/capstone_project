// Unit test path_blockage (kiểm tra đường A* bị chặn trên lưới costmap, dùng cho replanning).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "stage3_navigation/path_blockage.hpp"

using stage3_navigation::findBlockage;
using stage3_navigation::GridView;
using stage3_navigation::PathPoint;
using stage3_navigation::projectOnPath;

namespace
{
// Lưới 4 x 2 m, ô 0.05 m, gốc (-1, -1), toàn free.
struct Grid
{
  std::vector<std::int8_t> data;
  GridView view;
  Grid()
  : data(80 * 40, 0)
  {
    view.origin_x = -1.0;
    view.origin_y = -1.0;
    view.resolution = 0.05;
    view.width = 80;
    view.height = 40;
    view.data = data.data();
  }
  void set(double x, double y, std::int8_t v)
  {
    // +1e-6: toạ độ đúng mép ô (vd 1.8) thuộc ô bắt đầu tại đó, không rơi sang ô trước do làm tròn.
    const auto i = static_cast<unsigned>(std::floor((x + 1.0) / 0.05 + 1e-6));
    const auto j = static_cast<unsigned>(std::floor((y + 1.0) / 0.05 + 1e-6));
    data[j * 80 + i] = v;
  }
};

std::vector<PathPoint> straight(double x0, double x1, double step = 0.05)
{
  std::vector<PathPoint> p;
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    p.push_back(PathPoint{x, 0.0});
  }
  return p;
}
}  // namespace

TEST(PathBlockage, FreePathIsNotBlocked)
{
  Grid g;
  const auto b = findBlockage(straight(0.0, 2.0), g.view, 0, 0.0, 0.1, 2.5, 99);
  EXPECT_FALSE(b.blocked);
}

TEST(PathBlockage, InscribedCellAheadIsBlocked)
{
  Grid g;
  g.set(1.22, 0.0, 99);
  const auto b = findBlockage(straight(0.0, 2.0), g.view, 0, 0.0, 0.1, 2.5, 99);
  ASSERT_TRUE(b.blocked);
  EXPECT_EQ(b.cost, 99);
  EXPECT_NEAR(b.distance_ahead, 1.20, 0.03);   // đầu ô [1.20, 1.25)
}

TEST(PathBlockage, InflationBelowThresholdAndUnknownAreNotBlocked)
{
  Grid g;
  g.set(1.0, 0.0, 98);
  g.set(1.5, 0.0, -1);
  EXPECT_FALSE(findBlockage(straight(0.0, 2.0), g.view, 0, 0.0, 0.1, 2.5, 99).blocked);
  // Ngưỡng thấp hơn -> ô inflation 98 tính là chặn.
  EXPECT_TRUE(findBlockage(straight(0.0, 2.0), g.view, 0, 0.0, 0.1, 2.5, 90).blocked);
}

TEST(PathBlockage, BeyondLookaheadOrBehindRobotIsIgnored)
{
  Grid g;
  g.set(1.8, 0.0, 100);   // xa hơn lookahead
  g.set(0.3, 0.0, 100);   // phía sau robot (robot đã ở x = 0.6)
  const auto path = straight(0.0, 2.0);
  double along = 0.0;
  const auto seg = projectOnPath(path, 0.6, 0.02, 0, 2.0, along);
  EXPECT_NEAR(path[seg].x + along, 0.6, 1e-6);
  EXPECT_FALSE(findBlockage(path, g.view, seg, along, 0.1, 1.0, 99).blocked);
  const auto b = findBlockage(path, g.view, seg, along, 0.1, 1.5, 99);
  ASSERT_TRUE(b.blocked);
  EXPECT_NEAR(b.x, 1.8, 0.03);
}

TEST(PathBlockage, StartOffsetSkipsRobotCell)
{
  Grid g;
  g.set(0.02, 0.0, 99);
  EXPECT_FALSE(findBlockage(straight(0.0, 2.0), g.view, 0, 0.0, 0.1, 2.5, 99).blocked);
  EXPECT_TRUE(findBlockage(straight(0.0, 2.0), g.view, 0, 0.0, 0.0, 2.5, 99).blocked);
}

TEST(PathBlockage, SparsePathIsSampledBetweenPoses)
{
  Grid g;
  g.set(1.03, 0.0, 100);
  // Chỉ 2 pose cách nhau 2 m: ô giữa vẫn phải được phát hiện.
  const std::vector<PathPoint> path{{0.0, 0.0}, {2.0, 0.0}};
  EXPECT_TRUE(findBlockage(path, g.view, 0, 0.0, 0.1, 2.5, 99).blocked);
}

TEST(PathBlockage, GoalCellIsChecked)
{
  Grid g;
  g.set(0.5, 0.0, 100);
  const std::vector<PathPoint> path{{0.0, 0.0}, {0.5, 0.0}};
  const auto b = findBlockage(path, g.view, 0, 0.0, 0.1, 2.5, 99);
  ASSERT_TRUE(b.blocked);
  EXPECT_NEAR(b.x, 0.5, 1e-6);
}

TEST(PathBlockage, OutsideGridAndInvalidGridAreNotBlocked)
{
  Grid g;
  EXPECT_FALSE(findBlockage({{-5.0, 0.0}, {-3.0, 0.0}}, g.view, 0, 0.0, 0.0, 5.0, 99).blocked);
  GridView empty;
  EXPECT_FALSE(findBlockage(straight(0.0, 1.0), empty, 0, 0.0, 0.0, 5.0, 99).blocked);
  EXPECT_FALSE(findBlockage({}, g.view, 0, 0.0, 0.0, 5.0, 99).blocked);
}

TEST(PathBlockage, ProjectionDoesNotJumpToLaterBranch)
{
  // Chữ U: đi ra x = 1, quay về theo y = 0.3. Robot ở (0.1, 0.25): gần nhánh về hơn, nhưng tiến trình
  // đang ở đầu nhánh đi -> cửa sổ 0.5 m không cho nhảy sang nhánh về.
  std::vector<PathPoint> path;
  for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.05) {
    path.push_back({x, 0.0});
  }
  for (double x = 1.0; x >= 0.0 - 1e-9; x -= 0.05) {
    path.push_back({x, 0.3});
  }
  double along = 0.0;
  const auto seg = projectOnPath(path, 0.1, 0.25, 0, 0.5, along);
  EXPECT_LT(seg, 20u);
  EXPECT_NEAR(path[seg].y, 0.0, 1e-9);
}

TEST(PathBlockage, ProjectionIsMonotonicFromProgress)
{
  const auto path = straight(0.0, 2.0);
  double along = 0.0;
  // Robot lùi lại x = 0.2 nhưng tiến trình đã ở đoạn 20 (x = 1.0): giữ ở đoạn 20.
  const auto seg = projectOnPath(path, 0.2, 0.0, 20, 2.0, along);
  EXPECT_EQ(seg, 20u);
  EXPECT_NEAR(along, 0.0, 1e-9);
}
