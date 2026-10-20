// localization_monitor: giám sát chất lượng localization, CHỈ công bố trạng thái.
// Không publish TF, không gửi /initialpose, không điều khiển motor, không gọi recovery.
//
// scan_consistency dùng distance filter (scan_map_consistency.hpp): tia ngắn hơn bản đồ do vật cản
// chưa có trong map bị loại, nên vật lạ không bị coi là "lạc".
// Input : map, scan_filtered, odometry/filtered, amcl_pose, initialpose, TF, /amcl/get_state
// Output: localization/status (diagnostic_msgs/DiagnosticStatus, message = LOCALIZED | UNCERTAIN |
//         LOST | INITIALIZING, values = lý do + số liệu chẩn đoán)

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "stage2_slam/localization_health.hpp"
#include "stage2_slam/scan_map_consistency.hpp"
#include "stage2_slam/scan_match_initializer.hpp"
#include "stage2_slam/scan_match_initializer_node.hpp"

namespace stage2_slam
{

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

class LocalizationMonitorNode : public rclcpp::Node
{
public:
  LocalizationMonitorNode()
  : Node("localization_monitor")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    const double rate = declare_parameter<double>("rate", 2.0);
    jump_hold_ = declare_parameter<double>("jump_hold", 5.0);
    const auto amcl_service = declare_parameter<std::string>("amcl_state_service", "/amcl/get_state");
    score_cfg_.sigma = declare_parameter<double>("score_field.sigma", score_cfg_.sigma);
    score_cfg_.max_distance =
      declare_parameter<double>("score_field.max_distance", score_cfg_.max_distance);
    pre_cfg_.max_range = declare_parameter<double>("preprocess.max_range", pre_cfg_.max_range);
    pre_cfg_.max_points = static_cast<std::size_t>(
      declare_parameter<int>("preprocess.max_points", static_cast<int>(pre_cfg_.max_points)));
    min_points_ = declare_parameter<int>("min_valid_points", 60);
    // Distance filter (Fox et al. 1999): bỏ tia ngắn hơn bản đồ (vật cản chưa có trong map) khỏi
    // consistency. enabled:=false -> cách tính cũ (mọi điểm).
    df_cfg_.enabled = declare_parameter<bool>("distance_filter.enabled", df_cfg_.enabled);
    df_cfg_.short_margin =
      declare_parameter<double>("distance_filter.short_margin", df_cfg_.short_margin);
    df_cfg_.long_margin =
      declare_parameter<double>("distance_filter.long_margin", df_cfg_.long_margin);
    df_cfg_.match_score =
      declare_parameter<double>("distance_filter.match_score", df_cfg_.match_score);
    df_cfg_.min_counted_points = static_cast<std::size_t>(declare_parameter<int>(
        "distance_filter.min_counted_points", static_cast<int>(df_cfg_.min_counted_points)));

    auto & c = cfg_;
    c.initializing_grace = declare_parameter<double>("initializing_grace", c.initializing_grace);
    c.scan_age_warn = declare_parameter<double>("scan_age_warn", c.scan_age_warn);
    c.scan_age_lost = declare_parameter<double>("scan_age_lost", c.scan_age_lost);
    c.odom_age_warn = declare_parameter<double>("odom_age_warn", c.odom_age_warn);
    c.odom_age_lost = declare_parameter<double>("odom_age_lost", c.odom_age_lost);
    c.tf_age_warn = declare_parameter<double>("tf_age_warn", c.tf_age_warn);
    c.tf_age_lost = declare_parameter<double>("tf_age_lost", c.tf_age_lost);
    c.std_xy_warn = declare_parameter<double>("std_xy_warn", c.std_xy_warn);
    c.std_xy_lost = declare_parameter<double>("std_xy_lost", c.std_xy_lost);
    c.std_yaw_warn = declare_parameter<double>("std_yaw_warn", c.std_yaw_warn);
    c.std_yaw_lost = declare_parameter<double>("std_yaw_lost", c.std_yaw_lost);
    c.consistency_warn = declare_parameter<double>("consistency_warn", c.consistency_warn);
    c.consistency_lost = declare_parameter<double>("consistency_lost", c.consistency_lost);
    c.consistency_lost_count =
      declare_parameter<int>("consistency_lost_count", c.consistency_lost_count);
    c.blocked_ratio_warn = declare_parameter<double>("blocked_ratio_warn", c.blocked_ratio_warn);
    c.jump_linear_warn = declare_parameter<double>("jump_linear_warn", c.jump_linear_warn);
    c.jump_angular_warn = declare_parameter<double>("jump_angular_warn", c.jump_angular_warn);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        std::string err;
        const GridMap grid = toGridMap(*m);
        auto f = ScoreField::build(grid, score_cfg_, &err);
        if (f) {
          field_ = std::make_shared<ScoreField>(std::move(*f));
          raycaster_ = std::make_shared<MapRaycaster>(grid, score_cfg_.occupied_threshold);
        } else {
          field_.reset();
          raycaster_.reset();
          RCLCPP_ERROR(get_logger(), "invalid map: %s", err.c_str());
        }
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {last_scan_ = m;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        odom_stamp_ = rclcpp::Time(m->header.stamp, get_clock()->get_clock_type()).seconds();
      });
    // Volatile + reliable: tương thích cả publisher volatile lẫn transient_local.
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::QoS(10),
      std::bind(&LocalizationMonitorNode::onAmclPose, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
        last_initialpose_ = now().seconds();
        prev_amcl_.reset();
      });
    status_pub_ = create_publisher<DiagnosticStatus>(
      "localization/status", rclcpp::QoS(1).transient_local());
    amcl_client_ = create_client<lifecycle_msgs::srv::GetState>(amcl_service);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
      std::bind(&LocalizationMonitorNode::evaluate, this));
    amcl_timer_ = create_wall_timer(1s, std::bind(&LocalizationMonitorNode::pollAmcl, this));
    RCLCPP_INFO(get_logger(), "localization_monitor started (status only, no actuation)");
  }

private:
  std::optional<Pose2D> lookup2D(
    const std::string & target, const std::string & source, const rclcpp::Time & t,
    double timeout, rclcpp::Time * stamp_out = nullptr)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target, source, t, rclcpp::Duration::from_seconds(timeout));
      if (stamp_out) {*stamp_out = rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type());}
      const auto & q = tf.transform.rotation;
      return Pose2D{tf.transform.translation.x, tf.transform.translation.y,
        yawFromQuaternion(q.x, q.y, q.z, q.w)};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void onAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
  {
    amcl_pose_received_ = true;
    const auto & c = m->pose.covariance;
    std_x_ = std::sqrt(std::max(0.0, c[0]));
    std_y_ = std::sqrt(std::max(0.0, c[7]));
    std_yaw_ = std::sqrt(std::max(0.0, c[35]));
    const auto & q = m->pose.pose.orientation;
    const Pose2D pose{m->pose.pose.position.x, m->pose.pose.position.y,
      yawFromQuaternion(q.x, q.y, q.z, q.w)};
    const rclcpp::Time stamp(m->header.stamp, get_clock()->get_clock_type());
    if (prev_amcl_) {
      // So sánh dịch chuyển của AMCL với dịch chuyển odom trong cùng khoảng thời gian.
      const auto o0 = lookup2D(odom_frame_, base_frame_, prev_amcl_stamp_, 0.0);
      const auto o1 = lookup2D(odom_frame_, base_frame_, stamp, 0.05);
      if (o0 && o1) {
        const Pose2D d_odom = between(*o0, *o1);
        const Pose2D d_amcl = between(*prev_amcl_, pose);
        const Pose2D diff = between(d_odom, d_amcl);
        jump_linear_ = std::hypot(diff.x, diff.y);
        jump_angular_ = std::abs(diff.yaw);
        jump_time_ = now().seconds();
      }
    }
    prev_amcl_ = pose;
    prev_amcl_stamp_ = stamp;
  }

  void pollAmcl()
  {
    if (amcl_pending_) {return;}
    if (!amcl_client_->service_is_ready()) {
      amcl_active_ = false;
      return;
    }
    amcl_pending_ = true;
    amcl_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f) {
        amcl_pending_ = false;
        amcl_active_ =
        f.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      });
  }

  void evaluate()
  {
    const rclcpp::Time t_now = now();
    HealthInputs in;
    in.now = t_now.seconds();
    in.amcl_active = amcl_active_;
    in.map_received = static_cast<bool>(field_);
    in.amcl_pose_received = amcl_pose_received_;
    in.last_initialpose = last_initialpose_;
    in.odom_stamp = odom_stamp_;
    in.std_x = std_x_;
    in.std_y = std_y_;
    in.std_yaw = std_yaw_;

    rclcpp::Time tf_stamp;
    if (lookup2D(global_frame_, base_frame_, rclcpp::Time(0, 0, get_clock()->get_clock_type()), 0.0,
      &tf_stamp))
    {
      in.tf_stamp = tf_stamp.seconds();
    }

    std::optional<double> consistency;
    std::optional<ConsistencyResult> cr;
    std::size_t points = 0;
    if (last_scan_) {
      const rclcpp::Time scan_stamp(last_scan_->header.stamp, get_clock()->get_clock_type());
      in.scan_stamp = scan_stamp.seconds();
      if (field_) {
        const auto laser_in_map =
          lookup2D(global_frame_, last_scan_->header.frame_id, scan_stamp, 0.05);
        if (laser_in_map) {
          const auto pre = scanToPoints(toScanData(*last_scan_), Pose2D{}, pre_cfg_);
          points = pre.points.size();
          if (static_cast<int>(points) >= min_points_ && raycaster_) {
            cr = evaluateScanConsistency(*field_, *raycaster_, pre.points, *laser_in_map, df_cfg_);
            consistency = df_cfg_.enabled ? cr->consistency :
              std::optional<double>(cr->consistency_all);
          }
        }
      }
    }
    in.scan_consistency = consistency;
    if (cr && df_cfg_.enabled) {in.blocked_ratio = cr->short_ratio;}
    if (jump_linear_ && in.now - jump_time_ < jump_hold_) {
      in.jump_linear = jump_linear_;
      in.jump_angular = jump_angular_;
    }

    const HealthReport rep = evaluateHealth(in, cfg_, mem_);

    DiagnosticStatus d;
    d.name = "localization_monitor";
    d.message = toString(rep.state);
    d.level = rep.state == HealthState::LOCALIZED ? DiagnosticStatus::OK :
      rep.state == HealthState::LOST ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN;
    auto kv = [&d](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        d.values.push_back(e);
      };
    auto num = [](std::optional<double> v) {
        if (!v) {return std::string("n/a");}
        std::ostringstream ss;
        ss.precision(4);
        ss << *v;
        return ss.str();
      };
    std::string reasons;
    for (const auto & r : rep.reasons) {reasons += (reasons.empty() ? "" : "; ") + r;}
    kv("reasons", reasons);
    kv("scan_age", num(in.scan_stamp ? std::optional<double>(in.now - *in.scan_stamp) : std::nullopt));
    kv("odom_age", num(in.odom_stamp ? std::optional<double>(in.now - *in.odom_stamp) : std::nullopt));
    kv("tf_age", num(in.tf_stamp ? std::optional<double>(in.now - *in.tf_stamp) : std::nullopt));
    kv("scan_consistency", num(consistency));
    kv("scan_points", std::to_string(points));
    kv("scan_consistency_all", num(cr ? std::optional<double>(cr->consistency_all) : std::nullopt));
    kv("blocked_ratio", num(cr ? std::optional<double>(cr->short_ratio) : std::nullopt));
    kv("through_wall_ratio", num(cr ? std::optional<double>(cr->long_ratio) : std::nullopt));
    kv("counted_points", cr ? std::to_string(cr->counted) : std::string("n/a"));
    kv("std_x", num(std_x_));
    kv("std_y", num(std_y_));
    kv("std_yaw", num(std_yaw_));
    kv("jump_linear", num(in.jump_linear));
    kv("jump_angular", num(in.jump_angular));
    kv("amcl_active", amcl_active_ ? (*amcl_active_ ? "true" : "false") : "unknown");
    status_pub_->publish(d);

    if (!last_state_ || *last_state_ != rep.state) {
      RCLCPP_INFO(get_logger(), "state -> %s (%s)", toString(rep.state), reasons.c_str());
      last_state_ = rep.state;
    }
  }

  std::string global_frame_, base_frame_, odom_frame_;
  double jump_hold_{5.0};
  int min_points_{60};
  ScoreFieldConfig score_cfg_;
  PreprocessConfig pre_cfg_;
  DistanceFilterConfig df_cfg_;
  HealthConfig cfg_;
  HealthMemory mem_;

  std::shared_ptr<ScoreField> field_;
  std::shared_ptr<MapRaycaster> raycaster_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::optional<double> odom_stamp_, std_x_, std_y_, std_yaw_, last_initialpose_;
  std::optional<double> jump_linear_, jump_angular_;
  double jump_time_{0.0};
  bool amcl_pose_received_{false};
  std::optional<Pose2D> prev_amcl_;
  rclcpp::Time prev_amcl_stamp_;
  std::optional<bool> amcl_active_;
  bool amcl_pending_{false};
  std::optional<HealthState> last_state_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr amcl_client_;
  rclcpp::TimerBase::SharedPtr timer_, amcl_timer_;
};

}  // namespace stage2_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stage2_slam::LocalizationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
