// =====================================================================================================
// fusion_layer.hpp — Plugin costmap stage3_navigation::FusionLayer (M2: camera độ sâu + LiDAR)
// =====================================================================================================
//
// Lớp vỏ ROS quanh CameraFusionGrid (camera_fusion.hpp):
//   - PointCloud2 của RealSense (cloud_topic) -> TF sang global_frame của costmap tại stamp -> addCameraFrame;
//   - LaserScan (scan_topic, mặc định /scan_filtered) -> điểm cuối tia -> setLidarHits (CHỈ để liên kết:
//     vật LiDAR thấy vẫn do obstacle_layer vẽ);
//   - updateCosts: ô camera đã xác nhận -> LETHAL (max với master), inflation_layer phía sau thổi phồng.
// isCurrent() = false khi chưa có / quá expected_update_rate giây không có cloud: controller_server (Humble)
// ngừng gọi bộ điều khiển -> robot dừng (qua timeout twist_mux) thay vì chạy "mù" camera.
// Trạng thái: DiagnosticStatus trên <costmap>/<tên lớp>/status (vd. /local_costmap/fusion_layer/status), 1 Hz.
// Tham số chỉ đọc lúc khởi tạo (đổi -> khởi động lại controller_server).
// =====================================================================================================

#ifndef STAGE3_NAVIGATION__FUSION_LAYER_HPP_
#define STAGE3_NAVIGATION__FUSION_LAYER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "stage3_navigation/camera_fusion.hpp"

namespace stage3_navigation
{

class FusionLayer : public nav2_costmap_2d::Layer
{
public:
  FusionLayer() = default;
  ~FusionLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  // Xử lý các cloud đang chờ TF (gọi khi giữ mutex_).
  void processQueueLocked();
  bool processCloudLocked(const sensor_msgs::msg::PointCloud2 & msg);
  void publishStatus(double now);

  CameraFusionGrid grid_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string cloud_topic_;
  std::string scan_topic_;
  double lidar_max_range_{2.5};
  double expected_update_rate_{1.0};
  double max_cloud_wait_{0.5};

  std::mutex mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> queue_;
  std::vector<std::pair<double, double>> draw_;
  double last_cloud_time_{-1.0};     // stamp của cloud xử lý gần nhất
  double last_status_time_{-1.0};
  bool active_{false};

  // Thống kê cho status.
  std::size_t frames_{0};
  std::size_t tf_drops_{0};
  std::size_t queue_drops_{0};
  double proc_ms_sum_{0.0};
  double proc_ms_max_{0.0};
  std::size_t proc_count_{0};
  std::size_t suppressed_{0};
  FusionFrameStats last_stats_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
    status_pub_;
};

}  // namespace stage3_navigation

#endif  // STAGE3_NAVIGATION__FUSION_LAYER_HPP_
