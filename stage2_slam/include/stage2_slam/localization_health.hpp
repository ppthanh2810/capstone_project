// Đánh giá chất lượng localization (logic thuần, không phụ thuộc ROS; node: localization_monitor).
//
// Trạng thái: INITIALIZING, LOCALIZED, UNCERTAIN, LOST. Mỗi kết luận đi kèm danh sách lý do.
// Không bao giờ kết luận LOCALIZED chỉ từ covariance: bắt buộc có scan-to-map consistency tốt,
// TF map->base mới, scan/odom còn tươi và AMCL đang active.
// Module này chỉ CÔNG BỐ trạng thái; không điều khiển motor, không gọi recovery.

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace stage2_slam
{

enum class HealthState
{
  INITIALIZING,
  LOCALIZED,
  UNCERTAIN,
  LOST,
};

const char * toString(HealthState s);

struct HealthInputs
{
  double now{0.0};                          // s
  std::optional<bool> amcl_active;          // nullopt = chưa biết
  bool map_received{false};
  bool amcl_pose_received{false};
  std::optional<double> last_initialpose;   // thời điểm nhận /initialpose gần nhất
  std::optional<double> scan_stamp;         // stamp scan mới nhất
  std::optional<double> odom_stamp;         // stamp odometry mới nhất
  std::optional<double> tf_stamp;           // stamp của TF map->base mới nhất (nullopt = lookup lỗi)
  std::optional<double> std_x, std_y, std_yaw;  // từ covariance /amcl_pose
  std::optional<double> scan_consistency;   // score [0,1] của scan tại pose TF (nullopt = không tính được)
  std::optional<double> jump_linear;        // |Δpose AMCL - Δodom| giữa hai /amcl_pose (m)
  std::optional<double> jump_angular;       // rad
};

struct HealthConfig
{
  double initializing_grace{3.0};     // s sau /initialpose
  double scan_age_warn{0.5};
  double scan_age_lost{2.0};
  double odom_age_warn{0.3};
  double odom_age_lost{1.0};
  double tf_age_warn{0.5};            // now - stamp TF map->base
  double tf_age_lost{2.0};
  double std_xy_warn{0.25};           // m
  double std_xy_lost{1.0};
  double std_yaw_warn{0.17};          // rad (~10 deg)
  double std_yaw_lost{0.5};
  double consistency_warn{0.55};
  double consistency_lost{0.30};
  int consistency_lost_count{5};      // số lần liên tiếp dưới consistency_lost trước khi LOST
  double jump_linear_warn{0.30};      // m
  double jump_angular_warn{0.26};     // rad (~15 deg)
};

struct HealthMemory
{
  int low_consistency_count{0};
};

struct HealthReport
{
  HealthState state{HealthState::INITIALIZING};
  std::vector<std::string> reasons;
};

HealthReport evaluateHealth(const HealthInputs & in, const HealthConfig & cfg, HealthMemory & mem);

}  // namespace stage2_slam
