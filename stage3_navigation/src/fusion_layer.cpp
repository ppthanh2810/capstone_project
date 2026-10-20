// fusion_layer.cpp — Plugin costmap camera + LiDAR (M2). Giải thích: include/stage3_navigation/fusion_layer.hpp

#include "stage3_navigation/fusion_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace stage3_navigation
{

namespace
{
constexpr std::size_t kMaxQueue = 3;

diagnostic_msgs::msg::KeyValue kv(const std::string & k, const std::string & v)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = k;
  out.value = v;
  return out;
}

std::string num(double v, int prec = 3)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  return buf;
}
}  // namespace

void FusionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("FusionLayer: không lấy được node");
  }
  const FusionParams d;
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "cloud_topic",
    rclcpp::ParameterValue(std::string("/camera/camera/depth/color/points")));
  declareParameter("scan_topic", rclcpp::ParameterValue(std::string("/scan_filtered")));
  declareParameter("lidar_max_range", rclcpp::ParameterValue(2.5));
  declareParameter("expected_update_rate", rclcpp::ParameterValue(1.0));
  declareParameter("max_cloud_wait", rclcpp::ParameterValue(0.5));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(d.min_obstacle_height));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(d.max_obstacle_height));
  declareParameter("min_range", rclcpp::ParameterValue(d.min_range));
  declareParameter("max_range", rclcpp::ParameterValue(d.max_range));
  declareParameter("horizontal_fov", rclcpp::ParameterValue(d.horizontal_fov));
  declareParameter("vertical_fov", rclcpp::ParameterValue(d.vertical_fov));
  declareParameter("azimuth_bin", rclcpp::ParameterValue(d.azimuth_bin));
  declareParameter("elevation_bin", rclcpp::ParameterValue(d.elevation_bin));
  declareParameter("min_points_per_cell", rclcpp::ParameterValue(d.min_points_per_cell));
  declareParameter("confirm_frames", rclcpp::ParameterValue(d.confirm_frames));
  declareParameter("clear_frames", rclcpp::ParameterValue(d.clear_frames));
  declareParameter("decay_time", rclcpp::ParameterValue(d.decay_time));
  declareParameter("lidar_association_radius", rclcpp::ParameterValue(d.lidar_association_radius));
  declareParameter("lidar_max_age", rclcpp::ParameterValue(d.lidar_max_age));
  declareParameter("self_filter_radius", rclcpp::ParameterValue(d.self_filter_radius));
  declareParameter("max_cell_distance", rclcpp::ParameterValue(d.max_cell_distance));

  auto get = [&](const std::string & n, auto & out) {
      node->get_parameter(getFullName(n), out);
    };
  get("enabled", enabled_);
  get("cloud_topic", cloud_topic_);
  get("scan_topic", scan_topic_);
  get("lidar_max_range", lidar_max_range_);
  get("expected_update_rate", expected_update_rate_);
  get("max_cloud_wait", max_cloud_wait_);
  FusionParams p;
  p.resolution = layered_costmap_->getCostmap()->getResolution();
  get("min_obstacle_height", p.min_obstacle_height);
  get("max_obstacle_height", p.max_obstacle_height);
  get("min_range", p.min_range);
  get("max_range", p.max_range);
  get("horizontal_fov", p.horizontal_fov);
  get("vertical_fov", p.vertical_fov);
  get("azimuth_bin", p.azimuth_bin);
  get("elevation_bin", p.elevation_bin);
  get("min_points_per_cell", p.min_points_per_cell);
  get("confirm_frames", p.confirm_frames);
  get("clear_frames", p.clear_frames);
  get("decay_time", p.decay_time);
  get("lidar_association_radius", p.lidar_association_radius);
  get("lidar_max_age", p.lidar_max_age);
  get("self_filter_radius", p.self_filter_radius);
  get("max_cell_distance", p.max_cell_distance);
  const std::string err = validateFusionParams(p);
  if (!err.empty()) {
    throw std::runtime_error("FusionLayer " + name_ + ": tham số không hợp lệ: " + err);
  }
  grid_.setParams(p);
  global_frame_ = layered_costmap_->getGlobalFrameID();
  robot_base_frame_ = node->get_parameter("robot_base_frame").as_string();
  current_ = expected_update_rate_ <= 0.0;

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = callback_group_;
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FusionLayer::cloudCallback, this, std::placeholders::_1), opts);
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FusionLayer::scanCallback, this, std::placeholders::_1), opts);
  status_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    name_ + "/status", rclcpp::QoS(5));
  RCLCPP_INFO(
    logger_, "FusionLayer %s: cloud %s, scan %s, frame %s, height [%.2f, %.2f] m, range [%.2f, %.2f] m, "
    "confirm %d / clear %d frames, decay %.1f s",
    name_.c_str(), cloud_topic_.c_str(),
    scan_topic_.c_str(), global_frame_.c_str(), p.min_obstacle_height, p.max_obstacle_height,
    p.min_range, p.max_range, p.confirm_frames, p.clear_frames, p.decay_time);
}

void FusionLayer::activate()
{
  std::lock_guard<std::mutex> lk(mutex_);
  active_ = true;
  if (status_pub_) {
    status_pub_->on_activate();
  }
}

void FusionLayer::deactivate()
{
  std::lock_guard<std::mutex> lk(mutex_);
  active_ = false;
  if (status_pub_) {
    status_pub_->on_deactivate();
  }
}

void FusionLayer::reset()
{
  std::lock_guard<std::mutex> lk(mutex_);
  grid_.reset();
  queue_.clear();
  draw_.clear();
  current_ = expected_update_rate_ <= 0.0;
  last_cloud_time_ = -1.0;
}

void FusionLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (!active_ || !enabled_) {
    return;
  }
  queue_.push_back(msg);
  while (queue_.size() > kMaxQueue) {
    queue_.pop_front();
    ++queue_drops_;
  }
  processQueueLocked();
}

void FusionLayer::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (!active_ || !enabled_) {
    return;
  }
  geometry_msgs::msg::TransformStamped ts;
  try {
    ts = tf_->lookupTransform(
      global_frame_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp));
  } catch (const tf2::TransformException &) {
    try {   // TF chưa tới stamp của scan: dùng TF mới nhất (chỉ phục vụ liên kết, sai số nhỏ).
      ts = tf_->lookupTransform(global_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "FusionLayer: TF scan: %s", e.what());
      return;
    }
  }
  tf2::Transform t;
  tf2::fromMsg(ts.transform, t);
  std::vector<std::pair<double, double>> hits;
  hits.reserve(msg->ranges.size());
  const double rmax = std::min(static_cast<double>(msg->range_max), lidar_max_range_);
  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    const double r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > rmax) {
      continue;
    }
    const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const tf2::Vector3 p = t * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);
    hits.emplace_back(p.x(), p.y());
  }
  grid_.setLidarHits(rclcpp::Time(msg->header.stamp).seconds(), hits);
}

void FusionLayer::processQueueLocked()
{
  const double now = clock_->now().seconds();
  while (!queue_.empty()) {
    const auto & msg = *queue_.front();
    if (processCloudLocked(msg)) {
      queue_.pop_front();
      continue;
    }
    // TF chưa có: chờ, trừ khi cloud đã quá cũ.
    if (now - rclcpp::Time(msg.header.stamp).seconds() > max_cloud_wait_) {
      ++tf_drops_;
      queue_.pop_front();
      continue;
    }
    break;
  }
}

bool FusionLayer::processCloudLocked(const sensor_msgs::msg::PointCloud2 & msg)
{
  const auto t0 = std::chrono::steady_clock::now();
  geometry_msgs::msg::TransformStamped ts, tr;
  try {
    const auto stamp = tf2_ros::fromMsg(msg.header.stamp);
    ts = tf_->lookupTransform(global_frame_, msg.header.frame_id, stamp);
    tr = tf_->lookupTransform(global_frame_, robot_base_frame_, stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_DEBUG(logger_, "FusionLayer: TF cloud chưa có: %s", e.what());
    return false;
  }
  tf2::Transform t;
  tf2::fromMsg(ts.transform, t);
  CameraObservation obs;
  obs.stamp = rclcpp::Time(msg.header.stamp).seconds();
  obs.sensor = Point3{t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z()};
  // Frame quang học (REP-103 "_optical"): trục nhìn = +z; frame thường: +x.
  const bool optical = msg.header.frame_id.find("optical") != std::string::npos;
  const tf2::Vector3 axis = t.getBasis() *
    (optical ? tf2::Vector3(0, 0, 1) : tf2::Vector3(1, 0, 0));
  obs.sensor_yaw = std::atan2(axis.y(), axis.x());
  obs.robot_x = tr.transform.translation.x;
  obs.robot_y = tr.transform.translation.y;
  try {
    obs.points.reserve(static_cast<std::size_t>(msg.width) * msg.height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
        continue;
      }
      const tf2::Vector3 p = t * tf2::Vector3(*ix, *iy, *iz);
      obs.points.push_back(Point3{p.x(), p.y(), p.z()});
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "FusionLayer: cloud không đọc được: %s", e.what());
    return true;   // bỏ cloud hỏng
  }
  if (obs.stamp < last_cloud_time_) {
    return true;   // cloud cũ hơn frame đã xử lý: bỏ (luật frame liên tiếp cần đúng thứ tự)
  }
  last_stats_ = grid_.addCameraFrame(obs);
  last_cloud_time_ = obs.stamp;
  ++frames_;
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  proc_ms_sum_ += ms;
  proc_ms_max_ = std::max(proc_ms_max_, ms);
  ++proc_count_;
  return true;
}

void FusionLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (!enabled_) {
    current_ = true;
    draw_.clear();
    return;
  }
  processQueueLocked();
  const double now = clock_->now().seconds();
  grid_.decay(now, robot_x, robot_y);
  draw_ = grid_.obstacleCells(now, &suppressed_);
  if (expected_update_rate_ > 0.0) {
    const bool fresh = last_cloud_time_ > 0.0 && now - last_cloud_time_ <= expected_update_rate_;
    if (!fresh && current_) {
      RCLCPP_WARN(
        logger_, "FusionLayer: không có point cloud mới trên %s trong %.1f s -> costmap không "
        "current (controller dừng)", cloud_topic_.c_str(), expected_update_rate_);
    }
    current_ = fresh;
  } else {
    current_ = true;
  }
  // Rolling window nhỏ: cập nhật cả vùng quanh robot để ô vừa bị xóa cũng được làm mới.
  const double r = grid_.params().max_cell_distance;
  *min_x = std::min(*min_x, robot_x - r);
  *min_y = std::min(*min_y, robot_y - r);
  *max_x = std::max(*max_x, robot_x + r);
  *max_y = std::max(*max_y, robot_y + r);
  if (now - last_status_time_ >= 1.0) {
    last_status_time_ = now;
    publishStatus(now);
  }
}

void FusionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (!enabled_) {
    return;
  }
  for (const auto & c : draw_) {
    unsigned int mx, my;
    if (!master_grid.worldToMap(c.first, c.second, mx, my)) {
      continue;
    }
    const int i = static_cast<int>(mx), j = static_cast<int>(my);
    if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
      continue;
    }
    const unsigned char old = master_grid.getCost(mx, my);
    if (old == nav2_costmap_2d::NO_INFORMATION || old < nav2_costmap_2d::LETHAL_OBSTACLE) {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void FusionLayer::publishStatus(double now)
{
  if (!status_pub_ || !status_pub_->is_activated()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus s;
  s.name = "FusionLayer/" + name_;
  s.hardware_id = cloud_topic_;
  const double age = last_cloud_time_ > 0.0 ? now - last_cloud_time_ : -1.0;
  if (!current_) {
    s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    s.message = "STALE: không có point cloud mới";
  } else {
    s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    s.message = "OK";
  }
  s.values.push_back(kv("frames", std::to_string(frames_)));
  s.values.push_back(kv("cloud_age", num(age)));
  s.values.push_back(
    kv("proc_ms_mean", num(proc_count_ ? proc_ms_sum_ / static_cast<double>(proc_count_) : 0.0)));
  s.values.push_back(kv("proc_ms_max", num(proc_ms_max_)));
  s.values.push_back(kv("pending_cells", std::to_string(grid_.pendingCount())));
  s.values.push_back(kv("confirmed_cells", std::to_string(grid_.confirmedCount())));
  s.values.push_back(kv("drawn_cells", std::to_string(draw_.size())));
  s.values.push_back(kv("lidar_suppressed_cells", std::to_string(suppressed_)));
  s.values.push_back(kv("last_points", std::to_string(last_stats_.points_in)));
  s.values.push_back(kv("last_points_marking", std::to_string(last_stats_.points_marking)));
  s.values.push_back(kv("last_hit_cells", std::to_string(last_stats_.hit_cells)));
  s.values.push_back(kv("tf_drops", std::to_string(tf_drops_)));
  s.values.push_back(kv("queue_drops", std::to_string(queue_drops_)));
  status_pub_->publish(s);
  proc_ms_sum_ = 0.0;
  proc_ms_max_ = 0.0;
  proc_count_ = 0;
}

}  // namespace stage3_navigation

PLUGINLIB_EXPORT_CLASS(stage3_navigation::FusionLayer, nav2_costmap_2d::Layer)
