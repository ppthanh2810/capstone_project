// Tiện ích tạo thế giới / bản đồ / scan tổng hợp có ground truth biết trước.
// Chỉ dùng cho unit test và benchmark offline (không dùng trong node chạy trên robot).
//
// Thế giới được mô tả giải tích (các hình chữ nhật vật cản). Scan được mô phỏng bằng giao tia -
// hình chữ nhật chính xác, độc lập với lưới; bản đồ được raster hóa từ cùng hình học theo quy tắc
// "ô chứa bề mặt vật cản là ô chiếm chỗ" (giống cách SLAM đánh dấu điểm chạm).

#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "stage2_slam/scan_match_initializer.hpp"

namespace stage2_slam::synthetic
{

struct Rect
{
  double x0, y0, x1, y1;
};

struct World
{
  std::vector<Rect> obstacles;
  Rect known_area{0, 0, 0, 0};  // ngoài vùng này: unknown trên bản đồ, không có phản xạ
};

// Phòng 16 x 10 m (x in [0,16], y in [0,10]) có vách ngăn, cửa và vật cản ngẫu nhiên (seed cố định).
World makeOfficeWorld(unsigned seed);
// Hành lang dài 60 m (y in [0.5,3.5]), hai tường song song -> nhập nhằng theo X.
World makeCorridorWorld();
// Hành lang có cột lặp lại theo X với chu kỳ `period` -> nhiều cực đại độc lập.
World makePeriodicCorridorWorld(double period);

// Raster hóa: ô có diện tích chồng lấn vật cản -> 100; còn lại trong known_area -> 0; ngoài -> -1.
GridMap rasterize(
  const World & world, double resolution, const Pose2D & origin, double size_x, double size_y);

// Bản đồ toàn ô free (0), kích thước theo mét (cho test score field).
GridMap makeMap(double size_x, double size_y, double resolution, const Pose2D & origin);

// Đọc bản đồ map_server (yaml + pgm P5, mode trinary). Trả về false nếu lỗi.
bool loadMapYaml(const std::string & yaml_path, GridMap & map, std::string * error = nullptr);

struct ScanSimConfig
{
  double angle_min{0.0};               // frame laser (sau scan_rotate: 0..2pi)
  double angle_increment{0.0174533};   // 1 deg
  int beams{360};
  double range_min{0.05};
  double range_max{12.0};
  double noise_std{0.01};              // m
  // Bỏ (NaN) các tia có góc trong (blank_min, blank_max) như laser_filters angular_filter.yaml
  bool blank_rear{true};
  double blank_min{1.5715927};
  double blank_max{4.7115927};
  double dropout{0.0};                 // xác suất mất tia (NaN)
};

// Scan giải tích trên World. Không phản xạ -> +inf.
ScanData simulateScan(
  const World & world, const Pose2D & robot_pose, const Pose2D & sensor_in_base,
  const ScanSimConfig & config, std::mt19937 & rng);
// Scan bằng dò tia trên lưới (dùng cho bản đồ thật không có hình học giải tích). Tia xuyên ô unknown.
ScanData simulateScan(
  const GridMap & map, const Pose2D & robot_pose, const Pose2D & sensor_in_base,
  const ScanSimConfig & config, std::mt19937 & rng);

// Pose ngẫu nhiên trên ô free cách vật cản/unknown >= clearance (m).
bool sampleFreePose(
  const GridMap & map, double clearance, std::mt19937 & rng, Pose2D & pose,
  int max_tries = 20000);

}  // namespace stage2_slam::synthetic
