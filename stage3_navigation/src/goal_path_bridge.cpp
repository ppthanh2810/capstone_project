// goal_path_bridge: RViz "Goal Pose" (/goal_pose) -> action ComputePathToPose (planner_server, A*)
// -> nav_msgs/Path (/global_path) cho RViz.
// Stage 3.2 (tùy chọn): chuyển đường A* hợp lệ cuối cùng tới action FollowPath (controller_server, DWB)
// CHỈ khi được kích hoạt có chủ đích: service ~/follow, hoặc auto_follow:=true.
// Node này không bao giờ publish cmd_vel hay TF: vận tốc do controller_server phát, và topic đầu ra
// của nó (cách ly /cmd_vel_dwb hay /cmd_vel_nav) do planning.launch.py quyết định.
//
// Input : goal_pose (PoseStamped), map (kiểm tra có bản đồ), global_costmap/costmap (giải thích lỗi),
//         localization/status (DiagnosticStatus của localization_monitor, khi require_localized),
//         TF global_frame -> robot_base_frame (map -> base_footprint)
// Output: global_path (nav_msgs/Path, transient_local; path rỗng khi lập kế hoạch thất bại để RViz
//         không giữ đường cũ), ~/follow_state (std_msgs/String, transient_local)
// Service: ~/follow (std_srvs/Trigger): gửi đường hợp lệ cuối cùng tới FollowPath
//          ~/cancel (std_srvs/Trigger): hủy FollowPath đang chạy (controller_server phát vận tốc 0)
// Hủy FollowPath đang chạy khi: goal mới, ~/cancel, quá follow_timeout, localization không còn OK,
// node tắt (SIGINT/SIGTERM).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace stage3_navigation
{

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
using FollowPath = nav2_msgs::action::FollowPath;
using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using Trigger = std_srvs::srv::Trigger;

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

    // ---- FollowPath (Stage 3.2). Các giá trị dưới đây đọc lại mỗi lần dùng -> chỉnh được bằng
    // `ros2 param set /goal_path_bridge <name> <value>` khi đang chạy.
    const auto follow_action =
      declare_parameter<std::string>("follow_action_name", "/follow_path");
    declare_parameter<std::string>("controller_id", "FollowPath");
    declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    declare_parameter<bool>("auto_follow", false);
    declare_parameter<double>("max_path_age", 30.0);
    declare_parameter<double>("max_start_distance", 0.5);
    declare_parameter<double>("follow_timeout", 120.0);
    declare_parameter<bool>("require_localized", true);
    declare_parameter<double>("localization_status_timeout", 2.0);
    const auto status_topic =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, latched);
    state_pub_ = create_publisher<std_msgs::msg::String>("~/follow_state", latched);

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

    // localization_monitor publish transient_local mỗi chu kỳ (2 Hz).
    status_sub_ = create_subscription<DiagnosticStatus>(
      status_topic, latched,
      [this](DiagnosticStatus::ConstSharedPtr msg) {
        loc_status_ = msg;
        loc_status_time_ = now();
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {onGoal(*msg);});

    client_ = rclcpp_action::create_client<ComputePathToPose>(this, action_name);
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_action);

    follow_srv_ = create_service<Trigger>(
      "~/follow",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        const auto [ok, msg] = startFollow("service ~/follow");
        res->success = ok;
        res->message = msg;
      });
    cancel_srv_ = create_service<Trigger>(
      "~/cancel",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = true;
        res->message = cancelFollow("service ~/cancel") ?
        "FollowPath cancel requested" : "nothing to cancel (no active FollowPath)";
      });

    // Giám sát FollowPath đang chạy: timeout + localization.
    watch_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {watchFollow();});

    publishState("IDLE");
    RCLCPP_INFO(
      get_logger(),
      "goal_path_bridge started: %s -> %s -> %s | FollowPath %s only on ~/follow or auto_follow=%s "
      "(this node never publishes velocity)",
      goal_topic.c_str(), action_name.c_str(), path_topic.c_str(), follow_action.c_str(),
      get_parameter("auto_follow").as_bool() ? "true" : "false");
  }

  // Gọi khi tắt node (trước rclcpp::shutdown): hủy FollowPath để controller_server dừng robot.
  bool cancelOnShutdown()
  {
    return cancelFollow("node shutdown");
  }

private:
  // ======================= A* (ComputePathToPose) =======================

  void onGoal(geometry_msgs::msg::PoseStamped goal)
  {
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = global_frame_;
    }
    RCLCPP_INFO(
      get_logger(), "goal received: (%.2f, %.2f) in '%s'",
      goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());

    // Goal mới -> đường đang bám không còn là ý định của người dùng: dừng trước khi lập kế hoạch lại.
    cancelFollow("new goal received");
    last_path_.reset();

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
          last_path_ = path;
          last_path_time_ = now();
          RCLCPP_INFO(
            get_logger(), "path found (A*): %zu poses, %.2f m, planning time %.1f ms",
            path.poses.size(), length,
            rclcpp::Duration(result.result->planning_time).seconds() * 1e3);
          if (get_parameter("auto_follow").as_bool()) {
            const auto [ok, msg] = startFollow("auto_follow");
            if (!ok) {
              RCLCPP_WARN(get_logger(), "auto_follow: %s", msg.c_str());
            }
          }
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

  // ======================= DWB (FollowPath) =======================

  // Kiểm tra trước khi gửi; mọi điều kiện không đạt -> từ chối kèm lý do (không gửi gì).
  std::pair<bool, std::string> startFollow(const std::string & trigger)
  {
    const auto refuse = [this, &trigger](const std::string & why) {
        RCLCPP_WARN(get_logger(), "FollowPath refused (%s): %s", trigger.c_str(), why.c_str());
        publishState("REFUSED: " + why);
        return std::make_pair(false, why);
      };

    if (following_) {
      return refuse("a FollowPath goal is already active (call ~/cancel first)");
    }
    if (!last_path_) {
      return refuse("no valid A* path: send a 2D Goal Pose first (failed/new goals clear the path)");
    }
    const double max_age = get_parameter("max_path_age").as_double();
    const double age = (now() - last_path_time_).seconds();
    if (max_age > 0.0 && age > max_age) {
      return refuse(
        "path is stale (" + fmt1(age) + " s > max_path_age " + fmt1(max_age) +
        " s): send the goal again");
    }

    geometry_msgs::msg::TransformStamped robot;
    try {
      robot = tf_buffer_->lookupTransform(
        last_path_->header.frame_id.empty() ? global_frame_ : last_path_->header.frame_id,
        base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return refuse(std::string("no robot pose (TF): ") + ex.what());
    }
    const double tf_age = (now() - rclcpp::Time(robot.header.stamp, get_clock()->get_clock_type()))
      .seconds();
    if (tf_age > tf_timeout_) {
      return refuse("TF robot pose is " + fmt1(tf_age) + " s old (> tf_timeout)");
    }
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & p : last_path_->poses) {
      nearest = std::min(
        nearest, std::hypot(
          p.pose.position.x - robot.transform.translation.x,
          p.pose.position.y - robot.transform.translation.y));
    }
    const double max_start = get_parameter("max_start_distance").as_double();
    if (nearest > max_start) {
      return refuse(
        "robot is " + fmt1(nearest) + " m from the path (> max_start_distance " + fmt1(max_start) +
        " m): path is stale, send the goal again");
    }

    std::string loc_reason;
    if (!localizationOk(loc_reason)) {
      return refuse(loc_reason);
    }

    if (!follow_client_->action_server_is_ready()) {
      return refuse(
        "controller_server FollowPath action not available (run planning.launch.py "
        "controller:=true and wait until controller_server is active)");
    }

    FollowPath::Goal request;
    request.path = *last_path_;
    request.controller_id = get_parameter("controller_id").as_string();
    request.goal_checker_id = get_parameter("goal_checker_id").as_string();

    const std::uint64_t id = ++follow_id_;
    following_ = true;
    follow_handle_.reset();
    follow_start_ = now();

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, id](const FollowHandle::SharedPtr & handle) {
        if (id != follow_id_) {
          // Đã bị hủy trước khi server chấp nhận -> hủy luôn goal vừa được chấp nhận.
          if (handle) {
            follow_client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          following_ = false;
          publishState("REJECTED");
          RCLCPP_ERROR(get_logger(), "controller_server rejected the FollowPath goal");
          return;
        }
        follow_handle_ = handle;
        publishState("FOLLOWING");
        RCLCPP_INFO(get_logger(), "FollowPath accepted: DWB is following the A* path");
      };
    options.feedback_callback =
      [this, id](FollowHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> fb) {
        if (id == follow_id_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000, "following: %.2f m to goal, speed %.2f m/s",
            fb->distance_to_goal, fb->speed);
        }
      };
    options.result_callback =
      [this, id](const FollowHandle::WrappedResult & result) {
        if (id != follow_id_) {
          return;
        }
        following_ = false;
        follow_handle_.reset();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            publishState("SUCCEEDED");
            RCLCPP_INFO(get_logger(), "FollowPath succeeded: goal reached (goal checker)");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            publishState("CANCELED");
            RCLCPP_WARN(get_logger(), "FollowPath canceled");
            break;
          default:
            publishState("ABORTED");
            RCLCPP_ERROR(
              get_logger(), "FollowPath aborted by controller_server (no valid DWB trajectory, "
              "no progress, TF or costmap error; see controller_server log). It publishes zero "
              "velocity when it aborts");
            break;
        }
      };

    RCLCPP_INFO(
      get_logger(), "FollowPath requested (%s): %zu poses, path age %.1f s, robot %.2f m from path",
      trigger.c_str(), last_path_->poses.size(), age, nearest);
    publishState("FOLLOW_REQUESTED");
    follow_client_->async_send_goal(request, options);
    return {true, "FollowPath sent to controller_server"};
  }

  // Trả về true nếu có FollowPath đang chạy (đã gửi yêu cầu hủy).
  bool cancelFollow(const std::string & reason)
  {
    if (!following_) {
      return false;
    }
    following_ = false;
    ++follow_id_;  // bỏ qua callback muộn của goal cũ
    if (follow_handle_) {
      follow_client_->async_cancel_goal(follow_handle_);
    }
    // Goal chưa được chấp nhận: goal_response_callback (id cũ) sẽ hủy nó khi tới.
    follow_handle_.reset();
    publishState("CANCELED: " + reason);
    RCLCPP_WARN(get_logger(), "FollowPath canceled: %s", reason.c_str());
    return true;
  }

  void watchFollow()
  {
    if (!following_) {
      return;
    }
    const double timeout = get_parameter("follow_timeout").as_double();
    const double elapsed = (now() - follow_start_).seconds();
    if (timeout > 0.0 && elapsed > timeout) {
      cancelFollow("follow_timeout (" + fmt1(timeout) + " s) exceeded");
      return;
    }
    std::string reason;
    if (!localizationOk(reason)) {
      cancelFollow(reason);
    }
  }

  bool localizationOk(std::string & reason) const
  {
    if (!get_parameter("require_localized").as_bool()) {
      return true;
    }
    if (!loc_status_) {
      reason = "require_localized: no /localization/status received (run monitor:=true, or set "
        "require_localized false)";
      return false;
    }
    const double age = (now() - loc_status_time_).seconds();
    const double max_age = get_parameter("localization_status_timeout").as_double();
    if (age > max_age) {
      reason = "require_localized: /localization/status is " + fmt1(age) + " s old";
      return false;
    }
    if (loc_status_->level != DiagnosticStatus::OK) {
      reason = "require_localized: localization is " + loc_status_->message;
      return false;
    }
    return true;
  }

  void publishState(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  // ======================= helpers =======================

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

  static std::string fmt1(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    return buf;
  }

  // Báo lỗi + xóa đường cũ trên RViz.
  void fail(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "planning failed: %s", reason.c_str());
    last_path_.reset();
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

  // FollowPath
  std::optional<nav_msgs::msg::Path> last_path_;
  rclcpp::Time last_path_time_;
  bool following_{false};
  std::uint64_t follow_id_{0};
  rclcpp::Time follow_start_;
  FollowHandle::SharedPtr follow_handle_;
  DiagnosticStatus::ConstSharedPtr loc_status_;
  rclcpp::Time loc_status_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<DiagnosticStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp::Service<Trigger>::SharedPtr follow_srv_;
  rclcpp::Service<Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr watch_timer_;
};

}  // namespace stage3_navigation

namespace
{
std::atomic<bool> g_stop{false};
void onSignal(int) {g_stop = true;}
}  // namespace

int main(int argc, char ** argv)
{
  // Tự xử lý SIGINT/SIGTERM: cần context ROS còn sống để gửi yêu cầu hủy FollowPath trước khi thoát.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto node = std::make_shared<stage3_navigation::GoalPathBridge>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !g_stop) {
    exec.spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (rclcpp::ok() && node->cancelOnShutdown()) {
    // Cho yêu cầu hủy kịp gửi đi.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_some(std::chrono::milliseconds(50));
    }
  }
  rclcpp::shutdown();
  return 0;
}
