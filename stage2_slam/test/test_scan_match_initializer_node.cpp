// Test node scan_match_initializer in-process (không phần cứng, không AMCL thật).
//
// Cô lập: main() đặt ROS_DOMAIN_ID riêng + ROS_LOCALHOST_ONLY=1 trước rclcpp::init, và mọi topic
// (kể cả /tf, /tf_static) nằm trong namespace /csm_test -> không thể gửi /initialpose hay TF tới
// robot thật đang chạy trên mạng.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "stage2_slam/scan_match_initializer_node.hpp"
#include "stage2_slam/synthetic_world.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using stage2_slam::Pose2D;
namespace syn = stage2_slam::synthetic;

namespace
{

const char * kNs = "/csm_test";
const Pose2D kSensor{0.15, 0.0, 0.0};

std::vector<std::string> isolationArgs()
{
  return {"--ros-args", "-r", std::string("__ns:=") + kNs, "-r",
    std::string("/tf:=") + kNs + "/tf", "-r", std::string("/tf_static:=") + kNs + "/tf_static"};
}

nav_msgs::msg::OccupancyGrid toMsg(const stage2_slam::GridMap & m)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.width = m.width;
  g.info.height = m.height;
  g.info.resolution = static_cast<float>(m.resolution);
  g.info.origin.position.x = m.origin.x;
  g.info.origin.position.y = m.origin.y;
  g.info.origin.orientation.z = std::sin(m.origin.yaw / 2);
  g.info.origin.orientation.w = std::cos(m.origin.yaw / 2);
  g.data.assign(m.data.begin(), m.data.end());
  return g;
}

class Harness
{
public:
  Harness(
    const syn::World & world, const stage2_slam::GridMap & map, const Pose2D & truth,
    std::vector<rclcpp::Parameter> params, double range_max = 12.0)
  : world_(world), truth_(truth), rng_(123)
  {
    sim_.range_max = range_max;
    rclcpp::NodeOptions opt;
    opt.arguments(isolationArgs());
    opt.parameter_overrides(params);
    dut_ = std::make_shared<stage2_slam::ScanMatchInitializerNode>(opt);

    rclcpp::NodeOptions hopt;
    hopt.arguments(isolationArgs());
    node_ = std::make_shared<rclcpp::Node>("harness", hopt);
    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable());
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 10);
    guess_pub_ = node_->create_publisher<PoseWithCovarianceStamped>("initialpose_guess", 10);
    initialpose_sub_ = node_->create_subscription<PoseWithCovarianceStamped>(
      "initialpose", 10, [this](PoseWithCovarianceStamped::SharedPtr m) {initialposes.push_back(*m);});
    candidate_sub_ = node_->create_subscription<PoseWithCovarianceStamped>(
      "scan_match/candidate", rclcpp::QoS(1).transient_local(),
      [this](PoseWithCovarianceStamped::SharedPtr m) {candidates.push_back(*m);});
    status_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "scan_match/status", rclcpp::QoS(10).transient_local(),
      [this](diagnostic_msgs::msg::DiagnosticStatus::SharedPtr m) {statuses.push_back(m->message);});
    tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    sendStaticTf();
    map_pub_->publish(toMsg(map));
    exec_.add_node(dut_);
    exec_.add_node(node_);
  }

  void sendStaticTf()
  {
    std::vector<geometry_msgs::msg::TransformStamped> tfs(2);
    tfs[0].header.frame_id = "base_footprint";
    tfs[0].child_frame_id = "laser_frame";
    tfs[0].transform.translation.x = kSensor.x;
    tfs[0].transform.rotation.w = 1.0;
    tfs[1].header.frame_id = "odom";
    tfs[1].child_frame_id = "base_footprint";
    tfs[1].transform.rotation.w = 1.0;
    for (auto & t : tfs) {t.header.stamp = node_->now();}
    tf_static_->sendTransform(tfs);
  }

  void sendGuess(const Pose2D & g, const std::string & frame = "map")
  {
    PoseWithCovarianceStamped m;
    m.header.frame_id = frame;
    m.header.stamp = node_->now();
    m.pose.pose.position.x = g.x;
    m.pose.pose.position.y = g.y;
    m.pose.pose.orientation.z = std::sin(g.yaw / 2);
    m.pose.pose.orientation.w = std::cos(g.yaw / 2);
    guess_pub_->publish(m);
  }

  void sendScan(const std::string & frame = "laser_frame")
  {
    const auto s = syn::simulateScan(world_, truth_, kSensor, sim_, rng_);
    sensor_msgs::msg::LaserScan m;
    m.header.frame_id = frame;
    m.header.stamp = node_->now();
    m.angle_min = static_cast<float>(s.angle_min);
    m.angle_increment = static_cast<float>(s.angle_increment);
    m.angle_max = static_cast<float>(s.angle_min + s.angle_increment * (s.ranges.size() - 1));
    m.range_min = static_cast<float>(s.range_min);
    m.range_max = static_cast<float>(s.range_max);
    m.ranges = s.ranges;
    scan_pub_->publish(m);
  }

  void spinFor(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {exec_.spin_some(10ms);}
  }

  // Phát scan 10 Hz cho tới khi status kết thúc (không còn NO_SCAN chờ) hoặc hết thời gian.
  bool runUntilFinal(const std::string & scan_frame = "laser_frame", int max_iters = 80)
  {
    for (int i = 0; i < max_iters; ++i) {
      sendScan(scan_frame);
      spinFor(100ms);
      if (!statuses.empty() && statuses.back() != "NO_SCAN" && statuses.back() != "NO_MAP") {
        return true;
      }
    }
    return false;
  }

  bool waitForStatus(const std::string & s, std::chrono::milliseconds d = 3000ms)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some(10ms);
      if (!statuses.empty() && statuses.back() == s) {return true;}
    }
    return false;
  }

  std::vector<PoseWithCovarianceStamped> initialposes, candidates;
  std::vector<std::string> statuses;

private:
  syn::World world_;
  Pose2D truth_;
  std::mt19937 rng_;
  syn::ScanSimConfig sim_;
  std::shared_ptr<stage2_slam::ScanMatchInitializerNode> dut_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr guess_pub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initialpose_sub_, candidate_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;
};

std::vector<rclcpp::Parameter> baseParams()
{
  return {
    rclcpp::Parameter("require_amcl_active", false),
    rclcpp::Parameter("session.num_scans", 2),
    rclcpp::Parameter("session.scan_max_age", 2.0),
    rclcpp::Parameter("session.timeout", 6.0),
    rclcpp::Parameter("min_publish_interval", 0.0)};
}

struct Office
{
  syn::World world = syn::makeOfficeWorld(7);
  stage2_slam::GridMap map = syn::rasterize(world, 0.05, Pose2D{-3.013, -2.987, 0.0}, 26.0, 26.0);
  Pose2D truth{3.2, 2.7, 0.4};
};

}  // namespace

TEST(ScanMatchNode, AcceptedPublishesExactlyOneInitialPose)
{
  Office o;
  Harness h(o.world, o.map, o.truth, baseParams());
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));  // map đã nhận
  h.sendGuess(Pose2D{o.truth.x + 0.3, o.truth.y - 0.2, o.truth.yaw + 0.15});
  ASSERT_TRUE(h.runUntilFinal());
  EXPECT_EQ(h.statuses.back(), "ACCEPTED");
  ASSERT_EQ(h.initialposes.size(), 1u);
  const auto & p = h.initialposes.front();
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_LT(std::hypot(p.pose.pose.position.x - o.truth.x, p.pose.pose.position.y - o.truth.y), 0.05);
  const double yaw = stage2_slam::yawFromQuaternion(
    p.pose.pose.orientation.x, p.pose.pose.orientation.y, p.pose.pose.orientation.z,
    p.pose.pose.orientation.w);
  EXPECT_LT(std::abs(stage2_slam::normalizeAngle(yaw - o.truth.yaw)), 1.0 * M_PI / 180.0);
  for (int i : {0, 7, 35}) {
    EXPECT_TRUE(std::isfinite(p.pose.covariance[i]));
    EXPECT_GT(p.pose.covariance[i], 0.0);
  }
  // Scan tiếp theo không được gây /initialpose lặp lại.
  for (int i = 0; i < 10; ++i) {
    h.sendScan();
    h.spinFor(100ms);
  }
  EXPECT_EQ(h.initialposes.size(), 1u);
}

TEST(ScanMatchNode, AmbiguousDoesNotPublishInitialPose)
{
  syn::World world = syn::makeCorridorWorld();
  auto map = syn::rasterize(world, 0.05, Pose2D{-0.517, -0.013, 0.0}, 61.5, 4.5);
  Harness h(world, map, Pose2D{30.0, 2.0, 0.0}, baseParams(), 8.0);
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));
  h.sendGuess(Pose2D{30.1, 2.1, 0.05});
  ASSERT_TRUE(h.runUntilFinal());
  EXPECT_EQ(h.statuses.back(), "AMBIGUOUS");
  EXPECT_TRUE(h.initialposes.empty());
}

TEST(ScanMatchNode, OutsideWindowDoesNotPublishInitialPose)
{
  Office o;
  Harness h(o.world, o.map, o.truth, baseParams());
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));
  h.sendGuess(Pose2D{o.truth.x + 1.0, o.truth.y, o.truth.yaw});
  ASSERT_TRUE(h.runUntilFinal());
  EXPECT_NE(h.statuses.back(), "ACCEPTED");
  EXPECT_TRUE(h.initialposes.empty());
}

TEST(ScanMatchNode, MissingTfGivesTfError)
{
  Office o;
  Harness h(o.world, o.map, o.truth, baseParams());
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));
  h.sendGuess(o.truth);
  ASSERT_TRUE(h.runUntilFinal("laser_not_in_tf"));
  EXPECT_EQ(h.statuses.back(), "TF_ERROR");
  EXPECT_TRUE(h.initialposes.empty());
}

TEST(ScanMatchNode, GuessInWrongFrameIsRejected)
{
  Office o;
  Harness h(o.world, o.map, o.truth, baseParams());
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));
  h.sendGuess(o.truth, "odom");
  ASSERT_TRUE(h.waitForStatus("INVALID_INPUT"));
  h.spinFor(300ms);
  EXPECT_TRUE(h.initialposes.empty());
}

TEST(ScanMatchNode, RequiresActiveAmcl)
{
  // Không có AMCL -> không có service /amcl/get_state -> không được gửi /initialpose.
  Office o;
  auto params = baseParams();
  params[0] = rclcpp::Parameter("require_amcl_active", true);
  Harness h(o.world, o.map, o.truth, params);
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));
  h.spinFor(1500ms);  // để timer kiểm tra AMCL chạy ít nhất một lần
  h.sendGuess(Pose2D{o.truth.x + 0.2, o.truth.y, o.truth.yaw});
  ASSERT_TRUE(h.runUntilFinal());
  EXPECT_EQ(h.statuses.back(), "AMCL_INACTIVE");
  EXPECT_TRUE(h.initialposes.empty());
  EXPECT_FALSE(h.candidates.empty());
}

TEST(ScanMatchNode, DryRunPublishesCandidateOnly)
{
  Office o;
  auto params = baseParams();
  params.emplace_back("publish_initialpose", false);
  Harness h(o.world, o.map, o.truth, params);
  ASSERT_TRUE(h.waitForStatus("NO_SCAN"));
  h.sendGuess(Pose2D{o.truth.x + 0.2, o.truth.y, o.truth.yaw});
  ASSERT_TRUE(h.runUntilFinal());
  EXPECT_EQ(h.statuses.back(), "ACCEPTED");
  EXPECT_TRUE(h.initialposes.empty());
  ASSERT_FALSE(h.candidates.empty());
  EXPECT_LT(
    std::hypot(
      h.candidates.back().pose.pose.position.x - o.truth.x,
      h.candidates.back().pose.pose.position.y - o.truth.y), 0.05);
}

TEST(ScanMatchNode, NoMapGivesNoMap)
{
  // Map không có ô chiếm chỗ -> không hợp lệ -> guess bị trả NO_MAP.
  Office o;
  auto empty = syn::makeMap(5.0, 5.0, 0.05, Pose2D{});
  Harness h(o.world, empty, o.truth, baseParams());
  h.spinFor(500ms);
  h.sendGuess(o.truth);
  ASSERT_TRUE(h.waitForStatus("NO_MAP"));
  EXPECT_TRUE(h.initialposes.empty());
}

int main(int argc, char ** argv)
{
  // Cô lập khỏi mạng ROS thật (xem đầu file).
  setenv("ROS_DOMAIN_ID", "142", 1);
  setenv("ROS_LOCALHOST_ONLY", "1", 1);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
