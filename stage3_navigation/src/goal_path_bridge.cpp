// goal_path_bridge: RViz "Goal Pose" (/goal_pose) -> action ComputePathToPose (planner_server, A*)
// -> nav_msgs/Path (/global_path) cho RViz.
// CHỈ lập kế hoạch: không publish cmd_vel, không gọi NavigateToPose/FollowPath, không publish TF.
//
// Input : goal_pose (PoseStamped), map (kiểm tra có bản đồ), global_costmap/costmap (giải thích lỗi),
//         TF global_frame -> robot_base_frame (map -> base_footprint)
// Output: global_path (nav_msgs/Path, transient_local; path rỗng khi lập kế hoạch thất bại để RViz
//         không giữ đường cũ)

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace stage3_navigation
{

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class GoalPathBridge : public rclcpp::Node
{
public:
  GoalPathBridge()
  : Node("goal_path_bridge")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.5);
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const auto path_topic = declare_parameter<std::string>("path_topic", "/global_path");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    const auto costmap_topic =
      declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    const auto action_name =
      declare_parameter<std::string>("action_name", "/compute_path_to_pose");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, latched);

    // Chỉ ghi nhận đã có bản đồ (không giữ dữ liệu): planner_server tự đọc /map qua global_costmap.
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (!map_received_) {
          RCLCPP_INFO(
            get_logger(), "map received: %ux%u cells, %.3f m/cell, frame '%s'",
            msg->info.width, msg->info.height, msg->info.resolution, msg->header.frame_id.c_str());
        }
        map_received_ = true;
      });

    // Costmap đã inflate (100 lethal, 99 nội tiếp, -1 unknown): chỉ dùng để giải thích khi thất bại.
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {costmap_ = msg;});

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {onGoal(*msg);});

    client_ = rclcpp_action::create_client<ComputePathToPose>(this, action_name);

    RCLCPP_INFO(
      get_logger(), "goal_path_bridge started (planning only, no velocity commands): %s -> %s -> %s",
      goal_topic.c_str(), action_name.c_str(), path_topic.c_str());
  }

private:
  void onGoal(geometry_msgs::msg::PoseStamped goal)
  {
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = global_frame_;
    }
    RCLCPP_INFO(
      get_logger(), "goal received: (%.2f, %.2f) in '%s'",
      goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());

    if (!map_received_) {
      fail(
        "no map on /map. Start Stage 2 first: mapping.launch.py (slam_toolbox) or "
        "localization.launch.py (map_server + AMCL)");
      return;
    }

    geometry_msgs::msg::TransformStamped robot;
    try {
      robot = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      fail(
        "no robot pose: TF " + global_frame_ + " -> " + base_frame_ + " unavailable (" + ex.what() +
        "). localization: set 2D Pose Estimate (AMCL publishes map -> odom only after an initial "
        "pose); mapping: check slam_toolbox. odom -> base_footprint comes from the EKF");
      return;
    }
    const double tf_age = (now() - rclcpp::Time(robot.header.stamp, get_clock()->get_clock_type()))
      .seconds();
    if (tf_age > tf_timeout_) {
      RCLCPP_WARN(
        get_logger(), "TF %s -> %s is %.2f s old: robot pose may be stale", global_frame_.c_str(),
        base_frame_.c_str(), tf_age);
    }

    if (!client_->action_server_is_ready()) {
      fail(
        "planner_server action not available. Is planning.launch.py running and planner_server "
        "active? (it activates only once TF " + global_frame_ + " -> " + base_frame_ + " exists)");
      return;
    }

    ComputePathToPose::Goal request;
    request.goal = goal;
    request.planner_id = planner_id_;
    request.use_start = false;  // start = pose robot hiện tại (planner_server đọc TF)

    const std::uint64_t id = ++request_id_;
    const auto start = robot.transform.translation;
    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, id](const GoalHandle::SharedPtr & handle) {
        if (!handle && id == request_id_) {
          fail("planner_server rejected the ComputePathToPose goal");
        }
      };
    options.result_callback =
      [this, id, goal, start](const GoalHandle::WrappedResult & result) {
        if (id != request_id_) {
          return;  // đã có goal mới hơn
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          !result.result->path.poses.empty())
        {
          const auto & path = result.result->path;
          double length = 0.0;
          for (std::size_t i = 1; i < path.poses.size(); ++i) {
            length += std::hypot(
              path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
              path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
          }
          path_pub_->publish(path);
          RCLCPP_INFO(
            get_logger(), "path found (A*): %zu poses, %.2f m, planning time %.1f ms",
            path.poses.size(), length,
            rclcpp::Duration(result.result->planning_time).seconds() * 1e3);
          return;
        }
        const char * code = result.code == rclcpp_action::ResultCode::ABORTED ? "aborted" :
          result.code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "empty path";
        fail(
          std::string("no path found (") + code + ") from (" + fmt(start.x, start.y) + ") to (" +
          fmt(goal.pose.position.x, goal.pose.position.y) + ")" + explain(goal, start));
      };
    client_->async_send_goal(request, options);
  }

  // Giải thích lỗi từ global costmap (chỉ là gợi ý; planner_server mới là kết luận).
  std::string explain(
    const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::Vector3 & start) const
  {
    if (!costmap_) {
      return ". No global costmap received yet";
    }
    geometry_msgs::msg::PoseStamped g = goal;
    if (goal.header.frame_id != costmap_->header.frame_id) {
      try {
        g = tf_buffer_->transform(goal, costmap_->header.frame_id);
      } catch (const tf2::TransformException & ex) {
        return std::string(". Goal frame not transformable to costmap frame: ") + ex.what();
      }
    }
    std::string out;
    const auto goal_cost = cellCost(g.pose.position.x, g.pose.position.y);
    const auto start_cost = cellCost(start.x, start.y);
    out += ". Goal " + describe(goal_cost);
    out += "; start " + describe(start_cost);
    if (goal_cost && *goal_cost >= 0 && *goal_cost < 99 && start_cost && *start_cost >= 0 &&
      *start_cost < 99)
    {
      out += ". Both cells are free: goal is probably enclosed / not connected to the robot "
        "(or connected only through unknown space with allow_unknown:=false)";
    }
    return out;
  }

  std::optional<int> cellCost(double x, double y) const
  {
    const auto & info = costmap_->info;
    const double res = info.resolution;
    const int mx = static_cast<int>(std::floor((x - info.origin.position.x) / res));
    const int my = static_cast<int>(std::floor((y - info.origin.position.y) / res));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
      my >= static_cast<int>(info.height))
    {
      return std::nullopt;
    }
    return costmap_->data[static_cast<std::size_t>(my) * info.width + mx];
  }

  static std::string describe(const std::optional<int> & cost)
  {
    if (!cost) {
      return "is outside the map";
    }
    if (*cost == 100) {
      return "is in an obstacle (lethal)";
    }
    if (*cost == 99) {
      return "is closer than the robot radius to an obstacle (inscribed)";
    }
    if (*cost < 0) {
      return "is in unknown space";
    }
    return "is free (cost " + std::to_string(*cost) + ")";
  }

  static std::string fmt(double x, double y)
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f, %.2f", x, y);
    return buf;
  }

  // Báo lỗi + xóa đường cũ trên RViz.
  void fail(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "planning failed: %s", reason.c_str());
    nav_msgs::msg::Path empty;
    empty.header.frame_id = global_frame_;
    empty.header.stamp = now();
    path_pub_->publish(empty);
  }

  std::string global_frame_;
  std::string base_frame_;
  std::string planner_id_;
  double tf_timeout_{0.5};
  bool map_received_{false};
  std::uint64_t request_id_{0};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;
};

}  // namespace stage3_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stage3_navigation::GoalPathBridge>());
  rclcpp::shutdown();
  return 0;
}
