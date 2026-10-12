#include "stage2_slam/scan_match_initializer_node.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"

namespace stage2_slam
{

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

GridMap toGridMap(const nav_msgs::msg::OccupancyGrid & msg)
{
  GridMap map;
  map.width = static_cast<int>(msg.info.width);
  map.height = static_cast<int>(msg.info.height);
  map.resolution = msg.info.resolution;
  const auto & o = msg.info.origin;
  map.origin = {o.position.x, o.position.y,
    yawFromQuaternion(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)};
  map.data.assign(msg.data.begin(), msg.data.end());
  return map;
}

ScanData toScanData(const sensor_msgs::msg::LaserScan & msg)
{
  ScanData s;
  s.angle_min = msg.angle_min;
  s.angle_increment = msg.angle_increment;
  s.range_min = msg.range_min;
  s.range_max = msg.range_max;
  s.ranges = msg.ranges;
  return s;
}

ScanMatchInitializerNode::ScanMatchInitializerNode(const rclcpp::NodeOptions & options)
: Node("scan_match_initializer", options)
{
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
  min_publish_interval_ = declare_parameter<double>("min_publish_interval", 1.0);
  publish_initialpose_ = declare_parameter<bool>("publish_initialpose", true);
  require_amcl_active_ = declare_parameter<bool>("require_amcl_active", true);
  amcl_state_max_age_ = declare_parameter<double>("amcl_state_max_age", 3.0);
  const auto amcl_service = declare_parameter<std::string>("amcl_state_service", "/amcl/get_state");

  auto & m = matcher_cfg_;
  m.score_field.occupied_threshold =
    declare_parameter<int>("score_field.occupied_threshold", m.score_field.occupied_threshold);
  m.score_field.sigma = declare_parameter<double>("score_field.sigma", m.score_field.sigma);
  m.score_field.max_distance =
    declare_parameter<double>("score_field.max_distance", m.score_field.max_distance);
  m.score_field.unknown_score =
    declare_parameter<double>("score_field.unknown_score", m.score_field.unknown_score);
  m.preprocess.min_range = declare_parameter<double>("preprocess.min_range", m.preprocess.min_range);
  m.preprocess.max_range = declare_parameter<double>("preprocess.max_range", m.preprocess.max_range);
  m.preprocess.max_points = static_cast<std::size_t>(
    declare_parameter<int>("preprocess.max_points", static_cast<int>(m.preprocess.max_points)));
  m.search.linear_window = declare_parameter<double>("search.linear_window", m.search.linear_window);
  m.search.angular_window =
    declare_parameter<double>("search.angular_window", m.search.angular_window);
  m.search.linear_step = declare_parameter<double>("search.linear_step", m.search.linear_step);
  m.search.angular_step = declare_parameter<double>("search.angular_step", m.search.angular_step);
  m.search.max_candidates = static_cast<std::size_t>(
    declare_parameter<int>("search.max_candidates", static_cast<int>(m.search.max_candidates)));
  m.search.time_budget = declare_parameter<double>("search.time_budget", m.search.time_budget);
  m.refine.enabled = declare_parameter<bool>("refine.enabled", m.refine.enabled);
  m.validation.min_valid_points = static_cast<std::size_t>(declare_parameter<int>(
      "validation.min_valid_points", static_cast<int>(m.validation.min_valid_points)));
  m.validation.min_score = declare_parameter<double>("validation.min_score", m.validation.min_score);
  m.validation.ambiguity_ratio =
    declare_parameter<double>("validation.ambiguity_ratio", m.validation.ambiguity_ratio);
  m.validation.peak_separation_linear = declare_parameter<double>(
    "validation.peak_separation_linear", m.validation.peak_separation_linear);
  m.validation.peak_separation_angular = declare_parameter<double>(
    "validation.peak_separation_angular", m.validation.peak_separation_angular);
  m.validation.reject_at_window_boundary = declare_parameter<bool>(
    "validation.reject_at_window_boundary", m.validation.reject_at_window_boundary);
  m.covariance.min_std_linear =
    declare_parameter<double>("covariance.min_std_linear", m.covariance.min_std_linear);
  m.covariance.max_std_linear =
    declare_parameter<double>("covariance.max_std_linear", m.covariance.max_std_linear);
  m.covariance.min_std_angular =
    declare_parameter<double>("covariance.min_std_angular", m.covariance.min_std_angular);
  m.covariance.max_std_angular =
    declare_parameter<double>("covariance.max_std_angular", m.covariance.max_std_angular);

  auto & s = session_cfg_;
  s.num_scans = static_cast<std::size_t>(
    declare_parameter<int>("session.num_scans", static_cast<int>(s.num_scans)));
  s.scan_max_age = declare_parameter<double>("session.scan_max_age", s.scan_max_age);
  s.timeout = declare_parameter<double>("session.timeout", s.timeout);
  s.max_motion_linear = declare_parameter<double>("session.max_motion_linear", s.max_motion_linear);
  s.max_motion_angular =
    declare_parameter<double>("session.max_motion_angular", s.max_motion_angular);
  s.max_spread_linear = declare_parameter<double>("session.max_spread_linear", s.max_spread_linear);
  s.max_spread_angular =
    declare_parameter<double>("session.max_spread_angular", s.max_spread_angular);
  s.max_tf_failures = static_cast<std::size_t>(
    declare_parameter<int>("session.max_tf_failures", static_cast<int>(s.max_tf_failures)));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // spin_thread=true: buffer nhận TF trên thread riêng, nên lookup có timeout trong callback scan
  // (executor đơn luồng) vẫn chờ được TF odom tới đúng stamp.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  // map_server publish /map transient_local; AMCL subscribe /initialpose reliable volatile.
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&ScanMatchInitializerNode::onMap, this, std::placeholders::_1));
  guess_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose_guess", rclcpp::QoS(10),
    std::bind(&ScanMatchInitializerNode::onGuess, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SensorDataQoS(),
    std::bind(&ScanMatchInitializerNode::onScan, this, std::placeholders::_1));
  initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(10));
  candidate_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "scan_match/candidate", rclcpp::QoS(1).transient_local());
  status_pub_ = create_publisher<DiagnosticStatus>(
    "scan_match/status", rclcpp::QoS(1).transient_local());
  amcl_state_client_ = create_client<lifecycle_msgs::srv::GetState>(amcl_service);

  timer_ = create_wall_timer(100ms, std::bind(&ScanMatchInitializerNode::onTimer, this));
  if (require_amcl_active_) {
    amcl_timer_ = create_wall_timer(1s, std::bind(&ScanMatchInitializerNode::pollAmclState, this));
  }

  publishStatus(MatchStatus::NO_MAP, "waiting for map");
  RCLCPP_INFO(
    get_logger(),
    "scan_match_initializer ready | window +-%.2f m / +-%.1f deg | min_score %.2f | "
    "ambiguity_ratio %.2f | num_scans %zu | publish_initialpose %s | require_amcl_active %s",
    m.search.linear_window, m.search.angular_window * 180.0 / M_PI, m.validation.min_score,
    m.validation.ambiguity_ratio, s.num_scans, publish_initialpose_ ? "true" : "false",
    require_amcl_active_ ? "true" : "false");
}

void ScanMatchInitializerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) {
    field_.reset();
    map_error_ = "map frame '" + msg->header.frame_id + "' != global_frame '" + global_frame_ + "'";
    RCLCPP_ERROR(get_logger(), "%s", map_error_.c_str());
    return;
  }
  std::string err;
  const auto t0 = std::chrono::steady_clock::now();
  auto field = ScoreField::build(toGridMap(*msg), matcher_cfg_.score_field, &err);
  if (!field) {
    field_.reset();
    map_error_ = "invalid map: " + err;
    RCLCPP_ERROR(get_logger(), "%s", map_error_.c_str());
    return;
  }
  field_ = std::make_shared<ScoreField>(std::move(*field));
  map_error_.clear();
  RCLCPP_INFO(
    get_logger(), "map %dx%d @ %.3f m, %zu occupied cells, score field built in %.2f s",
    field_->width(), field_->height(), field_->resolution(), field_->occupiedCells(),
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  if (!session_) {
    publishStatus(MatchStatus::NO_SCAN, "map ready, waiting for initialpose_guess");
  }
}

void ScanMatchInitializerNode::onGuess(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto & p = msg->pose.pose;
  if (msg->header.frame_id != global_frame_) {
    session_.reset();
    publishStatus(
      MatchStatus::INVALID_INPUT,
      "guess frame '" + msg->header.frame_id + "' != global_frame '" + global_frame_ + "'");
    return;
  }
  const double yaw = yawFromQuaternion(
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(yaw)) {
    session_.reset();
    publishStatus(MatchStatus::INVALID_INPUT, "guess pose is not finite");
    return;
  }
  if (!field_) {
    session_.reset();
    publishStatus(MatchStatus::NO_MAP, map_error_);
    return;
  }
  if (session_) {
    RCLCPP_WARN(get_logger(), "new guess supersedes the running initialization");
  }
  const Pose2D guess{p.position.x, p.position.y, yaw};
  session_.emplace(guess, now().seconds(), session_cfg_, matcher_cfg_);
  RCLCPP_INFO(
    get_logger(), "guess (%.3f, %.3f, %.1f deg): matching next %zu scans", guess.x, guess.y,
    guess.yaw * 180.0 / M_PI, session_cfg_.num_scans);
  publishStatus(MatchStatus::NO_SCAN, "guess received, waiting for scans");
}

std::optional<Pose2D> ScanMatchInitializerNode::lookup2D(
  const std::string & target, const std::string & source, const rclcpp::Time & stamp,
  std::string & error)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      target, source, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
    const auto & q = tf.transform.rotation;
    return Pose2D{tf.transform.translation.x, tf.transform.translation.y,
      yawFromQuaternion(q.x, q.y, q.z, q.w)};
  } catch (const tf2::TransformException & e) {
    error = "TF " + target + " <- " + source + ": " + e.what();
    return std::nullopt;
  }
}

void ScanMatchInitializerNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!session_) {return;}
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  ScanSample sample;
  sample.stamp = stamp.seconds();
  sample.scan = toScanData(*msg);
  std::string err;
  sample.sensor_in_base = lookup2D(base_frame_, msg->header.frame_id, stamp, err);
  if (sample.sensor_in_base) {
    sample.odom_pose = lookup2D(odom_frame_, base_frame_, stamp, err);
  }
  sample.tf_error = err;
  auto result = session_->addScan(sample, now().seconds(), field_.get());
  if (result) {
    handleResult(*result);
  }
}

void ScanMatchInitializerNode::onTimer()
{
  if (!session_) {return;}
  auto result = session_->checkTimeout(now().seconds());
  if (result) {
    handleResult(*result);
  }
}

void ScanMatchInitializerNode::pollAmclState()
{
  if (amcl_request_pending_) {return;}
  if (!amcl_state_client_->service_is_ready()) {
    amcl_active_ = false;
    amcl_state_time_ = now();
    return;
  }
  amcl_request_pending_ = true;
  amcl_state_client_->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
    [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f) {
      amcl_request_pending_ = false;
      amcl_active_ = f.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      amcl_state_time_ = now();
    });
}

void ScanMatchInitializerNode::handleResult(const SessionResult & result)
{
  session_.reset();
  const MatchResult & m = result.final_match;
  const bool has_pose = !result.matches.empty();

  if (has_pose) {
    geometry_msgs::msg::PoseWithCovarianceStamped cand;
    cand.header.frame_id = global_frame_;
    cand.header.stamp = rclcpp::Time(static_cast<int64_t>(result.stamp * 1e9), RCL_ROS_TIME);
    cand.pose.pose.position.x = m.pose.x;
    cand.pose.pose.position.y = m.pose.y;
    cand.pose.pose.orientation.z = std::sin(m.pose.yaw / 2.0);
    cand.pose.pose.orientation.w = std::cos(m.pose.yaw / 2.0);
    // (x, y, yaw) -> chỉ số 6x6 (x=0, y=1, yaw=5).
    const int idx[3] = {0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        cand.pose.covariance[idx[i] * 6 + idx[j]] = m.covariance[i * 3 + j];
      }
    }
    candidate_pub_->publish(cand);

    if (result.publish) {
      const rclcpp::Time t_now = now();
      if (!publish_initialpose_) {
        publishStatus(result.status, "dry run: publish_initialpose=false", &result);
        return;
      }
      if (require_amcl_active_ &&
        (!amcl_active_.value_or(false) ||
        (t_now - amcl_state_time_).seconds() > amcl_state_max_age_))
      {
        publishStatus(
          MatchStatus::AMCL_INACTIVE, "AMCL lifecycle state is not active (or unknown); "
          "/initialpose not sent", &result);
        return;
      }
      if (published_once_ && (t_now - last_publish_time_).seconds() < min_publish_interval_) {
        publishStatus(
          MatchStatus::REJECTED, "rate limited (min_publish_interval); /initialpose not sent",
          &result);
        return;
      }
      initialpose_pub_->publish(cand);
      last_publish_time_ = t_now;
      published_once_ = true;
      RCLCPP_INFO(
        get_logger(),
        "ACCEPTED -> /initialpose (%.3f, %.3f, %.1f deg) | correction %.3f m, %.1f deg | "
        "score %.3f (guess %.3f) | ambiguity %.2f",
        m.pose.x, m.pose.y, m.pose.yaw * 180.0 / M_PI,
        std::hypot(m.pose.x - m.guess.x, m.pose.y - m.guess.y),
        normalizeAngle(m.pose.yaw - m.guess.yaw) * 180.0 / M_PI, m.score, m.guess_score,
        m.ambiguity);
    }
  }
  if (!result.publish) {
    RCLCPP_WARN(
      get_logger(), "%s: %s (/initialpose not sent)", toString(result.status),
      result.reason.c_str());
  }
  publishStatus(result.status, result.reason, &result);
}

void ScanMatchInitializerNode::publishStatus(
  MatchStatus status, const std::string & reason, const SessionResult * result)
{
  DiagnosticStatus d;
  d.name = "scan_match_initializer";
  d.hardware_id = "";
  d.message = toString(status);
  switch (status) {
    case MatchStatus::ACCEPTED: d.level = DiagnosticStatus::OK; break;
    case MatchStatus::AMBIGUOUS:
    case MatchStatus::REJECTED:
    case MatchStatus::NO_SCAN: d.level = DiagnosticStatus::WARN; break;
    default: d.level = DiagnosticStatus::ERROR; break;
  }
  auto kv = [&d](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      d.values.push_back(e);
    };
  auto num = [](double v) {
      std::ostringstream ss;
      ss.precision(4);
      ss << v;
      return ss.str();
    };
  kv("reason", reason);
  if (result && !result->matches.empty()) {
    const auto & m = result->final_match;
    kv("guess_x", num(m.guess.x));
    kv("guess_y", num(m.guess.y));
    kv("guess_yaw", num(m.guess.yaw));
    kv("pose_x", num(m.pose.x));
    kv("pose_y", num(m.pose.y));
    kv("pose_yaw", num(m.pose.yaw));
    kv("score", num(m.score));
    kv("guess_score", num(m.guess_score));
    kv("ambiguity", num(m.ambiguity));
    kv("inlier_fraction", num(m.inlier_fraction));
    kv("valid_points", std::to_string(m.valid_points));
    kv("candidates", std::to_string(m.candidates));
    kv("elapsed_s", num(m.elapsed));
    kv("scans_matched", std::to_string(result->matches.size()));
    kv("std_x", num(std::sqrt(m.covariance[0])));
    kv("std_y", num(std::sqrt(m.covariance[4])));
    kv("std_yaw", num(std::sqrt(m.covariance[8])));
  }
  if (result) {
    kv("stale_scans", std::to_string(result->stale_scans));
  }
  status_pub_->publish(d);
}

}  // namespace stage2_slam
