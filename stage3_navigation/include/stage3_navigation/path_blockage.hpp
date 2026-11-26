// path_blockage: kiểm tra đường A* đang bám có bị vật cản MỚI chặn hay không (dùng cho replanning).
// KHÔNG phụ thuộc ROS: chỉ đọc một lưới chi phí kiểu nav_msgs/OccupancyGrid (0..100, -1 = unknown).
//
// Nguồn vật cản không nằm ở đây mà ở costmap: mọi cảm biến (LiDAR hôm nay, camera sau này) đưa vật cản vào
// global_costmap qua các layer (observation_sources). Nhờ vậy thêm cảm biến = thêm layer/nguồn trong YAML,
// không phải sửa code kiểm tra này, và A* (planner_server) thấy đúng cùng một costmap khi lập đường mới.

#ifndef STAGE3_NAVIGATION__PATH_BLOCKAGE_HPP_
#define STAGE3_NAVIGATION__PATH_BLOCKAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stage3_navigation
{

struct PathPoint
{
  double x{0.0};
  double y{0.0};
};

// Lưới chi phí (không sở hữu dữ liệu), giá trị theo quy ước OccupancyGrid của Nav2:
// 100 = lethal, 99 = nội tiếp (tâm robot ở đây là chạm vật cản), 1..98 = inflation, 0 = free, -1 = unknown.
struct GridView
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
  unsigned width{0};
  unsigned height{0};
  const std::int8_t * data{nullptr};

  bool valid() const
  {
    return data != nullptr && width > 0 && height > 0 && resolution > 0.0;
  }
  // Trả về false nếu (x, y) ngoài lưới.
  bool cost(double x, double y, int & out) const;
};

struct Blockage
{
  bool blocked{false};
  double distance_ahead{0.0};   // m, độ dài cung từ điểm chiếu của robot tới ô bị chặn đầu tiên
  double x{0.0};
  double y{0.0};
  int cost{0};
};

// Chiếu robot (x, y) lên đường, chỉ tìm TIẾN về phía trước từ đoạn `from_segment` trong cửa sổ
// `window` m độ dài cung (không nhảy sang nhánh khác của đường tự cắt / chữ U). Trả về chỉ số đoạn
// [i, i+1] gần nhất; `along` = vị trí chiếu trên đoạn, tính theo m từ poses[i].
std::size_t projectOnPath(
  const std::vector<PathPoint> & path, double x, double y, std::size_t from_segment,
  double window, double & along);

// Lấy mẫu đường (bước <= nửa ô) từ điểm chiếu (đoạn `segment`, `along`) + `start_offset` tới
// + `lookahead` m. Bị chặn khi một mẫu có chi phí >= `blocked_cost` (mặc định 99 = nội tiếp/lethal:
// NavFn cũng coi đó là vật cản). Ô unknown (-1) và mẫu ngoài lưới không tính là chặn.
Blockage findBlockage(
  const std::vector<PathPoint> & path, const GridView & grid, std::size_t segment, double along,
  double start_offset, double lookahead, int blocked_cost);

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__PATH_BLOCKAGE_HPP_
