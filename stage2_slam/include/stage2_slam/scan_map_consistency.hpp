// Scan-to-map consistency có lọc vật cản chưa có trong bản đồ ("distance filter").
// Logic thuần, không phụ thuộc ROS; dùng bởi localization_monitor.
//
// Ý tưởng (Fox, Burgard & Thrun 1999, "Markov Localization for Mobile Robots in Dynamic
// Environments", JAIR 11): với mỗi tia, dò tia trên bản đồ tĩnh để có khoảng cách kỳ vọng r_exp rồi
// so với khoảng đo r:
//   - r ≈ r_exp            -> MATCHED : chấm điểm likelihood field như cũ.
//   - r < r_exp - margin   -> SHORT   : có vật đứng TRƯỚC tường (ghế, người...). Nếu điểm cuối còn
//                                       xa mọi tường (score < match_score) thì BỎ khỏi trung bình.
//   - r > r_exp + margin   -> LONG    : tia "xuyên tường" -> dấu hiệu lạc, vẫn tính (thường score 0).
// Vật lạ chỉ làm tia NGẮN đi, không làm tia dài ra, nên vẫn phát hiện được lạc qua tia LONG và qua
// tia lệch ngang. Hạn chế (cũng nêu trong bài báo): lạc theo kiểu "bản đồ tưởng robot xa tường hơn
// thực tế" chỉ sinh tia SHORT -> không làm consistency tụt; bù bằng ngưỡng short_ratio và các kiểm
// tra covariance / pose jump của AMCL.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "stage2_slam/scan_match_initializer.hpp"

namespace stage2_slam
{

// Dò tia trên OccupancyGrid tĩnh. Tia dừng ở ô chiếm chỗ (>= occupied_threshold), ô unknown hoặc
// khi ra ngoài bản đồ.
class MapRaycaster
{
public:
  MapRaycaster(const GridMap & map, int occupied_threshold);

  // Khoảng cách (m) từ origin theo hướng angle (frame map) tới ô dừng đầu tiên, tối đa max_range.
  // nullopt = không gặp gì trong max_range (khoảng trống, kỳ vọng "xa hơn max_range").
  std::optional<double> cast(double x, double y, double angle, double max_range) const;

private:
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Pose2D origin_;
  double cos_o_{1.0};
  double sin_o_{0.0};
  int occupied_threshold_{65};
  std::vector<int8_t> data_;
};

struct DistanceFilterConfig
{
  bool enabled{true};             // false -> giống scorePose cũ (mọi điểm đều tính)
  double short_margin{0.15};      // m, r_exp - r > margin -> SHORT
  double long_margin{0.15};       // m, r - r_exp > margin -> LONG (chỉ để chẩn đoán)
  double match_score{0.5};        // điểm SHORT có score >= ngưỡng này vẫn tính (đang nằm sát tường)
  std::size_t min_counted_points{40};  // ít hơn -> consistency không tính được (10% của 400)
};

struct ConsistencyResult
{
  std::optional<double> consistency;  // trung bình score trên điểm được tính (nullopt = quá ít)
  double consistency_all{0.0};        // trung bình trên TẤT CẢ điểm (cách tính cũ, để so sánh)
  std::size_t total{0};
  std::size_t counted{0};
  std::size_t short_beams{0};         // số điểm SHORT bị bỏ
  std::size_t long_beams{0};
  double short_ratio{0.0};            // short_beams / total
  double long_ratio{0.0};
};

// points: điểm scan trong frame cảm biến (gốc tại LiDAR); sensor_in_map: pose LiDAR trong map.
ConsistencyResult evaluateScanConsistency(
  const ScoreField & field, const MapRaycaster & raycaster, const std::vector<Point2D> & points,
  const Pose2D & sensor_in_map, const DistanceFilterConfig & config);

}  // namespace stage2_slam
