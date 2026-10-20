// =====================================================================================================
// camera_fusion.hpp — Lõi hợp nhất camera độ sâu + LiDAR cho local costmap (M2), TÁCH KHỎI ROS
// =====================================================================================================
//
// Plugin costmap stage3_navigation::FusionLayer (fusion_layer.cpp) chỉ là lớp vỏ ROS (TF, subscription,
// tham số); toàn bộ luật hợp nhất nằm ở đây (không phụ thuộc ROS).
//
// Luật (doc/obstacle_perception_plan.md mục 2). Lớp này CHỈ giữ bằng chứng của camera; vật LiDAR thấy vẫn do
// obstacle_layer (ObstacleLayer của Nav2) đánh dấu và xóa. Hai lớp gộp bằng max trong master costmap, nên tia
// xóa của camera không bao giờ xóa vật LiDAR thấy và ngược lại.
//   LiDAR thấy + camera thấy  -> ô camera xác nhận NGAY, nhưng KHÔNG vẽ khi ô nằm trong
//                                lidar_association_radius của một điểm LiDAR mới (vị trí lấy theo LiDAR).
//   LiDAR không + camera thấy -> vật cản sau confirm_frames frame LIÊN TIẾP (lọc nhiễu độ sâu).
//   LiDAR thấy + camera không -> obstacle_layer vẽ (lớp này không làm gì).
//   Không ai thấy             -> ô camera bị xóa khi camera NHÌN XUYÊN qua ô đó clear_frames frame liên tiếp,
//                                hoặc hết hạn decay_time giây kể từ lần cuối camera thấy (khi không nhìn được).
//
// "Nhìn xuyên qua" (bằng chứng trống): ảnh độ sâu được gom thành lưới bin (phương vị × góc ngẩng) quanh trục
// camera, mỗi bin giữ khoảng cách ngang NHỎ NHẤT. Ô (đã thấy vật ở độ cao [zmin, zmax]) được coi là trống khi
// mọi bin phủ dải góc ngẩng của [zmin, zmax] tại ô đó đều có điểm và điểm gần nhất xa hơn ô. Bin rỗng (lỗ độ
// sâu: vật đen / trong suốt) hoặc vật nằm ngoài góc nhìn đứng (quá gần) -> KHÔNG xóa (thận trọng).
//
// Giả định: frame toàn cục có z = 0 ở mặt sàn (odom của EKF 2D) và sàn phẳng. Đơn vị SI.
// =====================================================================================================

#ifndef STAGE3_NAVIGATION__CAMERA_FUSION_HPP_
#define STAGE3_NAVIGATION__CAMERA_FUSION_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stage3_navigation
{

struct FusionParams
{
  double resolution{0.05};               // m, = local costmap
  // Dải độ cao coi là vật cản (so với sàn). Trên max: robot đi lọt dưới; dưới min: sàn / nhiễu sàn bóng.
  double min_obstacle_height{0.05};      // m
  double max_obstacle_height{0.70};      // m, NOT_VERIFIED: chiều cao robot kể cả laptop + 0.1 m
  // Khoảng cách NGANG từ camera. D435: độ sâu tin được từ ~0.28 m.
  double min_range{0.30};                // m
  double max_range{2.5};                 // m, đánh dấu + xóa
  // Góc nhìn dùng cho bằng chứng trống (nhỏ hơn FOV thật 87° × 58° của D435 để chừa mép ảnh).
  double horizontal_fov{1.40};           // rad (~80°)
  double vertical_fov{0.96};             // rad (~55°)
  double azimuth_bin{0.0175};            // rad (1°)
  double elevation_bin{0.035};           // rad (2°)
  int min_points_per_cell{3};            // điểm trong dải độ cao / ô / frame để coi là "thấy"
  int confirm_frames{3};                 // N frame liên tiếp cho vật chỉ camera thấy
  int clear_frames{2};                   // frame liên tiếp "nhìn xuyên" để xóa ô đã xác nhận
  double decay_time{10.0};               // s, ô đã xác nhận không được thấy lại -> xóa
  double lidar_association_radius{0.10}; // m
  double lidar_max_age{0.5};             // s, điểm LiDAR cũ hơn -> coi như LiDAR không thấy
  double self_filter_radius{0.25};       // m quanh tâm robot: bỏ điểm (thân robot)
  double max_cell_distance{3.0};         // m từ robot: ô xa hơn bị bỏ (giới hạn bộ nhớ, > cửa sổ costmap)
};

// "" nếu hợp lệ, ngược lại là lý do.
std::string validateFusionParams(const FusionParams & p);

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Một frame độ sâu đã biến đổi sang frame toàn cục (odom).
struct CameraObservation
{
  double stamp{0.0};             // s
  Point3 sensor;                 // vị trí camera
  double sensor_yaw{0.0};        // hướng trục quang học chiếu lên mặt sàn
  double robot_x{0.0};           // tâm robot (lọc thân robot)
  double robot_y{0.0};
  std::vector<Point3> points;    // điểm không hữu hạn được bỏ qua
};

struct CellKey
{
  int32_t i{0};
  int32_t j{0};
  bool operator==(const CellKey & o) const {return i == o.i && j == o.j;}
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & k) const
  {
    return (static_cast<std::size_t>(static_cast<uint32_t>(k.i)) << 32) ^
           static_cast<uint32_t>(k.j);
  }
};

enum class CellState { NONE, PENDING, CONFIRMED };

struct FusionFrameStats
{
  std::size_t points_in{0};
  std::size_t points_marking{0};   // trong dải độ cao + tầm + ngoài thân robot
  std::size_t hit_cells{0};
  std::size_t new_confirmed{0};
  std::size_t cleared{0};          // ô xác nhận bị xóa vì nhìn xuyên
  std::size_t dropped_pending{0};  // ô chưa xác nhận bị bỏ vì chuỗi frame bị đứt
};

class CameraFusionGrid
{
public:
  explicit CameraFusionGrid(const FusionParams & p = FusionParams());

  // Đổi tham số. Đổi resolution -> xóa hết bằng chứng.
  void setParams(const FusionParams & p);
  const FusionParams & params() const {return p_;}

  // Điểm cuối tia LiDAR (frame toàn cục) của scan mới nhất; thay thế scan trước.
  void setLidarHits(double stamp, const std::vector<std::pair<double, double>> & hits);

  // Cập nhật bằng chứng camera bằng một frame. Các frame phải theo thứ tự thời gian.
  FusionFrameStats addCameraFrame(const CameraObservation & obs);

  // Xóa ô xác nhận quá decay_time và ô xa robot hơn max_cell_distance. Trả về số ô bị xóa.
  std::size_t decay(double now, double robot_x, double robot_y);

  // Tâm các ô cần vẽ LETHAL lúc `now`: ô đã xác nhận, trừ ô gần điểm LiDAR còn mới.
  // `suppressed` (nếu khác null) = số ô xác nhận bị bỏ vì LiDAR đã thấy.
  std::vector<std::pair<double, double>> obstacleCells(
    double now,
    std::size_t * suppressed = nullptr) const;

  void reset();

  CellState state(double x, double y) const;
  std::size_t pendingCount() const;
  std::size_t confirmedCount() const;
  CellKey keyOf(double x, double y) const;
  std::pair<double, double> centerOf(const CellKey & k) const;

private:
  struct Cell
  {
    CellState state{CellState::PENDING};
    int hit_streak{0};
    int free_streak{0};
    uint64_t last_hit_frame{0};
    double last_hit_time{0.0};
    float zmin{0.0f};
    float zmax{0.0f};
  };

  bool lidarNear(const CellKey & k, double t) const;

  FusionParams p_;
  std::unordered_map<CellKey, Cell, CellKeyHash> cells_;
  std::unordered_set<CellKey, CellKeyHash> lidar_cells_;   // ô trong bán kính liên kết của điểm LiDAR
  double lidar_stamp_{-1e18};
  uint64_t frame_id_{0};
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__CAMERA_FUSION_HPP_
