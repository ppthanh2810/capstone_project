// =====================================================================================================
// test_dwb_controller_plugin.cpp — Test plugin CustomDWB qua giao diện nav2_core::Controller thật
// =====================================================================================================
// Fixture: LifecycleNode giả vai controller_server (KHÔNG có action server, KHÔNG publish cmd_vel),
// Costmap2DROS chỉ configure (không layer, ghi cost trực tiếp), tf2_ros::Buffer được setTransform trực tiếp
// (không listener), odometry publish trong cùng process. CMake đặt ROS_DOMAIN_ID riêng + ROS_LOCALHOST_ONLY=1.
// Yêu cầu: R1 (lifecycle/plugin), R4 (dữ liệu cũ/hỏng), R5 (GoalChecker), R8 (tham số), R2 (speed limit).

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "stage3_navigation/dwb_controller.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

using stage3_navigation::DwbController;
using namespace std::chrono_literals;

namespace
{
class FakeGoalChecker : public nav2_core::GoalChecker
{
public:
  FakeGoalChecker(double xy, double yaw)
  : xy_(xy), yaw_(yaw) {}
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & pt, geometry_msgs::msg::Twist &) override
  {
    pt.position.x = pt.position.y = xy_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    pt.orientation = tf2::toMsg(q);
    return true;
  }
  double xy_, yaw_;
};

geometry_msgs::msg::Quaternion yawQ(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

nav_msgs::msg::Path straightPath(
  double len, double goal_yaw = 0.0,
  const std::string & frame = "map")
{
  nav_msgs::msg::Path p;
  p.header.frame_id = frame;
  for (int k = 0; k <= static_cast<int>(len / 0.05); ++k) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.pose.position.x = 0.05 * k;
    ps.pose.orientation = yawQ(0.0);
    p.poses.push_back(ps);
  }
  p.poses.back().pose.orientation = yawQ(goal_yaw);
  return p;
}
}  // namespace

class PluginFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    // Như controller_server: odom_topic đến từ YAML (parameter override) nhưng CHƯA được khai báo khi plugin
    // configure (server khai báo sau). controller_frequency đã được server khai báo trước.
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("odom_topic", "dwb_test_odom")});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server_test", opts);
    node_->declare_parameter("controller_frequency", 10.0);
    costmap_ =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", "local_costmap");
    costmap_->set_parameters(
      {rclcpp::Parameter("global_frame", "odom"),
        rclcpp::Parameter("robot_base_frame", "base_footprint"),
        rclcpp::Parameter("rolling_window", true), rclcpp::Parameter("width", 2),
        rclcpp::Parameter("height", 2), rclcpp::Parameter("resolution", 0.05),
        rclcpp::Parameter("robot_radius", 0.20), rclcpp::Parameter(
          "plugins",
          std::vector<std::string>{})});
    costmap_->configure();
    costmap_->getCostmap()->updateOrigin(-1.0, -1.0);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    setMapToOdom(node_->now());
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("dwb_test_odom", 10);
    odom_pub_->on_activate();
  }

  void TearDown() override
  {
    ctrl_.reset();
    costmap_.reset();
    node_.reset();
  }

  void setMapToOdom(const rclcpp::Time & stamp, double x = 0.0)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.header.stamp = stamp;
    t.child_frame_id = "odom";
    t.transform.translation.x = x;
    t.transform.rotation.w = 1.0;
    tf_->setTransform(t, "test", false);
  }

  void makeController()
  {
    ctrl_ = std::make_shared<DwbController>();
    ctrl_->configure(node_, "CustomDWB", tf_, costmap_);
    ctrl_->activate();
  }

  // Publish odometry và để subscription của plugin nhận.
  void feedOdom()
  {
    nav_msgs::msg::Odometry o;
    o.header.stamp = node_->now();
    for (int i = 0; i < 20; ++i) {
      odom_pub_->publish(o);
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(5ms);
    }
  }

  geometry_msgs::msg::PoseStamped robotPose(double x = 0.0, double y = 0.0, double yaw = 0.0)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "odom";
    p.header.stamp = node_->now();
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = yawQ(yaw);
    return p;
  }

  geometry_msgs::msg::TwistStamped compute(
    const geometry_msgs::msg::PoseStamped & pose, double v = 0.0, double w = 0.0,
    nav2_core::GoalChecker * gc = nullptr)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    return ctrl_->computeVelocityCommands(pose, t, gc);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<DwbController> ctrl_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

TEST_F(PluginFixture, PluginlibLoadsTheClass)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto inst = loader.createSharedInstance("stage3_navigation::DwbController");
  ASSERT_NE(inst, nullptr);
  inst->configure(node_, "LoadedDWB", tf_, costmap_);
  inst->activate();
  inst->deactivate();
  inst->cleanup();
}

TEST_F(PluginFixture, LifecycleAndNominalCommand)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  auto cmd = compute(robotPose());
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_LE(cmd.twist.linear.x, 0.05 + 1e-9);   // từ đứng yên: acc 0.5 * dt 0.1
  EXPECT_EQ(cmd.header.frame_id, "base_footprint");
  // deactivate -> activate: trạng thái được đặt lại, vẫn tính được; cleanup -> configure lại (tham số đã khai báo).
  ctrl_->deactivate();
  ctrl_->activate();
  cmd = compute(robotPose());
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  ctrl_->deactivate();
  ctrl_->cleanup();
  EXPECT_THROW(compute(robotPose()), nav2_core::PlannerException);   // không còn plan
  ctrl_->configure(node_, "CustomDWB", tf_, costmap_);
  ctrl_->activate();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  EXPECT_NO_THROW(compute(robotPose()));
}

TEST_F(PluginFixture, InvalidYamlFailsConfigure)
{
  node_->declare_parameter("CustomDWB.min_vel_x", -0.1);
  ctrl_ = std::make_shared<DwbController>();
  EXPECT_THROW(ctrl_->configure(node_, "CustomDWB", tf_, costmap_), std::runtime_error);
}

TEST_F(PluginFixture, InvalidPlansAreRejected)
{
  makeController();
  EXPECT_THROW(ctrl_->setPlan(nav_msgs::msg::Path()), nav2_core::PlannerException);
  auto p = straightPath(1.0);
  p.header.frame_id = "";
  EXPECT_THROW(ctrl_->setPlan(p), nav2_core::PlannerException);
  p = straightPath(1.0);
  p.poses[3].pose.position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ctrl_->setPlan(p), nav2_core::PlannerException);
  p = straightPath(1.0);
  p.poses.back().pose.orientation = geometry_msgs::msg::Quaternion();
  p.poses.back().pose.orientation.w = 0.0;
  EXPECT_THROW(ctrl_->setPlan(p), nav2_core::PlannerException);
  // Sau plan hỏng không còn plan cũ: không được tiếp tục lái theo plan trước.
  feedOdom();
  EXPECT_THROW(compute(robotPose()), nav2_core::PlannerException);
}

TEST_F(PluginFixture, StaleOrMissingInputsStopTheController)
{
  makeController();
  ctrl_->setPlan(straightPath(1.5));
  // Chưa có odometry.
  EXPECT_THROW(compute(robotPose()), nav2_core::PlannerException);
  feedOdom();
  EXPECT_NO_THROW(compute(robotPose()));
  // Pose (TF odom -> base) cũ 2 s.
  auto old = robotPose();
  old.header.stamp = node_->now() - rclcpp::Duration::from_seconds(2.0);
  EXPECT_THROW(compute(old), nav2_core::PlannerException);
  // Vận tốc / pose không hữu hạn, sai frame.
  EXPECT_THROW(compute(robotPose(), std::nan("")), nav2_core::PlannerException);
  auto nanpose = robotPose();
  nanpose.pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(compute(nanpose), nav2_core::PlannerException);
  auto wrong = robotPose();
  wrong.header.frame_id = "map";
  EXPECT_THROW(compute(wrong), nav2_core::PlannerException);
  // Odometry ngừng > odom_timeout (0.5 s).
  std::this_thread::sleep_for(700ms);
  EXPECT_THROW(compute(robotPose()), nav2_core::PlannerException);
  feedOdom();
  EXPECT_NO_THROW(compute(robotPose()));
}

TEST_F(PluginFixture, MapToOdomAgeIsCheckedButFutureStampsAreFine)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  // AMCL đóng dấu TF vào tương lai (+1.0 s): hợp lệ.
  tf_->clear();
  setMapToOdom(node_->now() + rclcpp::Duration::from_seconds(1.0));
  EXPECT_NO_THROW(compute(robotPose()));
  // TF map -> odom mới nhất đã 3 s (AMCL/scan ngừng): dừng.
  tf_->clear();
  setMapToOdom(node_->now() - rclcpp::Duration::from_seconds(3.0));
  EXPECT_THROW(compute(robotPose()), nav2_core::PlannerException);
  // Không có TF: dừng (chờ tối đa transform_tolerance).
  tf_->clear();
  EXPECT_THROW(compute(robotPose()), nav2_core::PlannerException);
}

TEST_F(PluginFixture, OldPathStampUsesLatestTransform)
{
  // Path được lập 40 s trước (bridge cho phép tới max_path_age 30 s; buffer TF chỉ giữ ~10 s): code cũ tra TF
  // tại stamp của pose cuối -> extrapolation -> abort. Mới: dùng TF mới nhất cho cả đường và đích.
  makeController();
  feedOdom();
  auto p = straightPath(1.5);
  const auto old_stamp = node_->now() - rclcpp::Duration::from_seconds(40.0);
  p.header.stamp = old_stamp;
  for (auto & ps : p.poses) {
    ps.header.stamp = old_stamp;
  }
  ctrl_->setPlan(p);
  EXPECT_NO_THROW(compute(robotPose()));
}

TEST_F(PluginFixture, RobotFarFromPathIsRefused)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  EXPECT_THROW(compute(robotPose(0.5, 0.9)), nav2_core::PlannerException);   // lệch 0.9 m > 0.8
}

TEST_F(PluginFixture, ParameterUpdatesAreAtomicAndValidated)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  // Lô có một giá trị sai -> cả lô bị từ chối, max_vel_x giữ 0.15.
  auto res = node_->set_parameters_atomically(
    {rclcpp::Parameter("CustomDWB.max_vel_x", 0.05), rclcpp::Parameter(
        "CustomDWB.speed_scale",
        -1.0)});
  EXPECT_FALSE(res.successful);
  auto cmd = compute(robotPose(), 0.12, 0.0);
  EXPECT_GT(cmd.twist.linear.x, 0.05 + 1e-6);
  // NaN / sai quan hệ bị từ chối.
  EXPECT_FALSE(
    node_->set_parameter(
      rclcpp::Parameter(
        "CustomDWB.max_vel_x", std::nan(
          ""))).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("CustomDWB.min_vel_x", -0.01)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("CustomDWB.vx_samples", 0)).successful);
  EXPECT_FALSE(
    node_->set_parameter(
      rclcpp::Parameter(
        "CustomDWB.collision_check",
        "foo")).successful);
  // Giá trị hợp lệ được áp dụng ở chu kỳ sau.
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("CustomDWB.max_vel_x", 0.08)).successful);
  cmd = compute(robotPose(), 0.08, 0.0);
  EXPECT_LE(cmd.twist.linear.x, 0.08 + 1e-9);
}

TEST_F(PluginFixture, SpeedLimitHumbleContract)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  const auto v_at = [&]() {return compute(robotPose(), 0.15, 0.0).twist.linear.x;};
  EXPECT_NEAR(v_at(), 0.15, 1e-9);
  ctrl_->setSpeedLimit(0.05, false);   // tuyệt đối
  EXPECT_LE(v_at(), 0.05 + 1e-9);
  ctrl_->setSpeedLimit(50.0, true);    // 50 % của 0.15
  EXPECT_LE(v_at(), 0.075 + 1e-9);
  ctrl_->setSpeedLimit(-1.0, false);   // không hợp lệ: bỏ qua, giữ 50 %
  EXPECT_LE(v_at(), 0.075 + 1e-9);
  ctrl_->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);   // 0.0 = BỎ giới hạn (không phải dừng)
  EXPECT_NEAR(v_at(), 0.15, 1e-9);
}

TEST_F(PluginFixture, GoalCheckerTolerancesAreUsed)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(0.25, 1.2));
  FakeGoalChecker wide(0.30, 0.25);
  // Cách đích 0.25 m: với checker 0.30 m -> đã tới vị trí, chỉ xoay (v = 0, ω != 0).
  auto cmd = compute(robotPose(), 0.0, 0.0, &wide);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_GT(std::fabs(cmd.twist.angular.z), 0.0);
  // Checker 0.15 m: vẫn tiến tới.
  ctrl_->setPlan(straightPath(0.25, 1.2));
  FakeGoalChecker tight(0.15, 0.25);
  cmd = compute(robotPose(), 0.0, 0.0, &tight);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

TEST_F(PluginFixture, ObstacleAheadBlocksForwardMotion)
{
  makeController();
  feedOdom();
  ctrl_->setPlan(straightPath(1.5));
  auto * cm = costmap_->getCostmap();
  // Vùng INSCRIBED từ x = 0.10 m: đang chạy 0.15 m/s thì không phanh kịp trong mọi ứng viên -> PlannerException.
  for (unsigned int j = 0; j < cm->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < cm->getSizeInCellsX(); ++i) {
      double wx, wy;
      cm->mapToWorld(i, j, wx, wy);
      if (wx > 0.10) {
        cm->setCost(i, j, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
    }
  }
  EXPECT_THROW(compute(robotPose(), 0.15, 0.0), nav2_core::PlannerException);
  // Đứng yên: chỉ còn lệnh (0, ω) hợp lệ -> không tiến.
  const auto cmd = compute(robotPose(), 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}
