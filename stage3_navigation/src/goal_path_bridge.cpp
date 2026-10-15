// goal_path_bridge: RViz "Goal Pose" (/goal_pose) -> action ComputePathToPose (planner_server, A*)
// -> nav_msgs/Path (/global_path) cho RViz.
// Stage 3.2 (tùy chọn): chuyển đường A* hợp lệ cuối cùng tới action FollowPath (controller_server, DWB)
// CHỈ khi được kích hoạt có chủ đích: service ~/follow, hoặc auto_follow:=true.
// Node này không bao giờ publish cmd_vel hay TF: vận tốc do controller_server phát, và topic đầu ra
// của nó (cách ly /cmd_vel_dwb hay /cmd_vel_nav) do navigation.launch.py quyết định.
//
// Replanning (replan.enabled, Stage 3.3): trong một "phiên điều hướng" (từ lúc FollowPath được kích hoạt tới khi
// tới đích / bị hủy / bỏ cuộc), node kiểm tra đoạn đường A* phía trước robot trên global costmap. global_costmap
// có obstacle layer (LiDAR, sau này thêm camera) -> vật cản mới làm ô trên đường thành nội tiếp/lethal ->
// gọi lại ComputePathToPose (A*) tới CÙNG đích -> gửi đường mới cho FollowPath (preempt, DWB bám đường mới).
// Lặp lại mỗi lần đường hiện tại bị chặn. FollowPath bị abort (DWB kẹt) -> cũng lập lại (giới hạn số lần).
// Không có đường -> hủy FollowPath (controller_server phát 0), chờ và thử lại tới replan.wait_timeout rồi bỏ cuộc.
// Logic kiểm tra chỉ đọc costmap (path_blockage.cpp), không đọc trực tiếp cảm biến nào.
//
// Input : goal_pose (PoseStamped), map (kiểm tra có bản đồ), global_costmap/costmap (giải thích lỗi + kiểm tra
//         đường bị chặn), localization/status (DiagnosticStatus của localization_monitor, khi require_localized),
//         TF global_frame -> robot_base_frame (map -> base_footprint)
// Output: global_path (nav_msgs/Path, transient_local; path rỗng khi lập kế hoạch thất bại để RViz
//         không giữ đường cũ), ~/follow_state (std_msgs/String, transient_local)
// Service: ~/follow (std_srvs/Trigger): gửi đường hợp lệ cuối cùng tới FollowPath (bắt đầu phiên)
//          ~/cancel (std_srvs/Trigger): kết thúc phiên, hủy FollowPath đang chạy (controller_server phát 0)
// Client : replan.clear_costmap_service (ClearCostmapExceptRegion, tùy chọn) khi lập lại thất bại.
// Kết thúc phiên (hủy FollowPath) khi: goal mới, ~/cancel, quá follow_timeout, localization không còn OK,
// không còn đường trong replan.wait_timeout, quá số lần lập lại, node tắt (SIGINT/SIGTERM).

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
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "stage3_navigation/path_blockage.hpp"
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
using ClearExcept = nav2_msgs::srv::ClearCostmapExceptRegion;
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
    // Đang bám đường: WARN (UNCERTAIN / INITIALIZING) chỉ hủy khi kéo dài liên tục quá mức này;
    // ERROR (LOST) và status cũ vẫn hủy ngay. Bắt đầu bám đường vẫn đòi OK.
    declare_parameter<double>("localization_uncertain_grace", 2.0);
    const auto status_topic =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");

    // ---- Replanning (đọc lại mỗi lần dùng, chỉnh được bằng ros2 param set) ----
    declare_parameter<bool>("replan.enabled", true);
    declare_parameter<double>("replan.lookahead_distance", 2.5);
    declare_parameter<double>("replan.start_offset", 0.10);
    declare_parameter<int>("replan.blocked_cost", 99);
    declare_parameter<double>("replan.confirm_time", 0.5);
    declare_parameter<double>("replan.min_interval", 2.0);
    declare_parameter<int>("replan.max_obstacle_replans", 20);
    declare_parameter<int>("replan.max_abort_replans", 3);
    declare_parameter<double>("replan.wait_timeout", 15.0);
    declare_parameter<double>("replan.costmap_timeout", 5.0);
    declare_parameter<bool>("replan.clear_costmap_on_failure", false);
    declare_parameter<double>("replan.clear_keep_distance", 5.0);
    const auto clear_service = declare_parameter<std::string>(
      "replan.clear_costmap_service", "/global_costmap/clear_except_global_costmap");

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

    // Costmap đã inflate (100 lethal, 99 nội tiếp, -1 unknown): giải thích khi thất bại + kiểm tra
    // đường đang bám có bị vật cản (obstacle layer) chặn không.
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        costmap_ = msg;
        costmap_time_ = now();
      });

    // localization_monitor publish transient_local mỗi chu kỳ (2 Hz).
    status_sub_ = create_subscription<DiagnosticStatus>(
      status_topic, latched,
      [this](DiagnosticStatus::ConstSharedPtr msg) {
        loc_status_ = msg;
        loc_status_time_ = now();
        if (msg->level != DiagnosticStatus::WARN) {
          loc_warn_since_.reset();
        } else if (!loc_warn_since_) {
          loc_warn_since_ = loc_status_time_;
        }
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {onGoal(*msg);});

    client_ = rclcpp_action::create_client<ComputePathToPose>(this, action_name);
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_action);
    clear_client_ = create_client<ClearExcept>(clear_service);

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
        "navigation canceled (FollowPath cancel requested)" :
        "nothing to cancel (no active navigation)";
      });

    // Giám sát phiên điều hướng: timeout, localization, đường bị chặn -> lập lại.
    watch_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {watchFollow();});

    publishState("IDLE");
    RCLCPP_INFO(
      get_logger(),
      "goal_path_bridge started: %s -> %s -> %s | FollowPath %s only on ~/follow or auto_follow=%s, "
      "replan=%s (this node never publishes velocity)",
      goal_topic.c_str(), action_name.c_str(), path_topic.c_str(), follow_action.c_str(),
      get_parameter("auto_follow").as_bool() ? "true" : "false",
      get_parameter("replan.enabled").as_bool() ? "true" : "false");
  }

  // Gọi khi tắt node (trước rclcpp::shutdown): hủy FollowPath để controller_server dừng robot.
  bool cancelOnShutdown()
  {
    return cancelFollow("node shutdown");
  }

private:
  // Một phiên điều hướng tới một đích (từ lúc kích hoạt FollowPath tới khi xong / hủy / bỏ cuộc).
  struct Session
  {
    std::uint64_t id{0};
    geometry_msgs::msg::PoseStamped goal;
    rclcpp::Time start;
    int obstacle_replans{0};
    int abort_replans{0};
    bool replan_in_flight{false};
    bool costmap_cleared{false};                 // đã xóa obstacle layer một lần trong đợt chờ hiện tại
    std::optional<rclcpp::Time> waiting_since;   // không có FollowPath, đang chờ đường mới
    std::optional<rclcpp::Time> blocked_since;   // đầu chuỗi "đường bị chặn" liên tục hiện tại
    std::optional<rclcpp::Time> last_attempt;    // lần gửi ComputePathToPose lập lại gần nhất
    std::vector<PathPoint> path;                 // đường đang bám (frame của path)
    std::string path_frame;
    std::size_t progress_segment{0};
  };

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
        if (succeeded(result)) {
          const auto & path = result.result->path;
          path_pub_->publish(path);
          last_path_ = path;
          last_path_time_ = now();
          last_goal_ = goal;
          RCLCPP_INFO(
            get_logger(), "path found (A*): %zu poses, %.2f m, planning time %.1f ms",
            path.poses.size(), pathLength(path),
            rclcpp::Duration(result.result->planning_time).seconds() * 1e3);
          if (get_parameter("auto_follow").as_bool()) {
            const auto [ok, msg] = startFollow("auto_follow");
            if (!ok) {
              RCLCPP_WARN(get_logger(), "auto_follow: %s", msg.c_str());
            }
          }
          return;
        }
        fail(
          "no path found (" + resultCode(result) + ") from (" + fmt(start.x, start.y) + ") to (" +
          fmt(goal.pose.position.x, goal.pose.position.y) + ")" + explain(goal, start));
      };
    client_->async_send_goal(request, options);
  }

  // ======================= DWB (FollowPath) =======================

  // Kích hoạt có chủ đích (~/follow, auto_follow): kiểm tra rồi mở một phiên điều hướng mới.
  std::pair<bool, std::string> startFollow(const std::string & trigger)
  {
    const auto refuse = [this, &trigger](const std::string & why) {
        RCLCPP_WARN(get_logger(), "FollowPath refused (%s): %s", trigger.c_str(), why.c_str());
        publishState("REFUSED: " + why);
        return std::make_pair(false, why);
      };

    if (session_) {
      return refuse(
        "a navigation is already active (following or replanning; call ~/cancel first)");
    }
    if (!last_path_ || !last_goal_) {
      return refuse("no valid A* path: send a 2D Goal Pose first (failed/new goals clear the path)");
    }
    const double max_age = get_parameter("max_path_age").as_double();
    const double age = (now() - last_path_time_).seconds();
    if (max_age > 0.0 && age > max_age) {
      return refuse(
        "path is stale (" + fmt1(age) + " s > max_path_age " + fmt1(max_age) +
        " s): send the goal again");
    }
    double nearest = 0.0;
    const std::string why = checkFollowable(*last_path_, false, nearest);
    if (!why.empty()) {
      return refuse(why);
    }

    session_.emplace();
    session_->id = ++session_counter_;
    session_->goal = *last_goal_;
    session_->start = now();
    RCLCPP_INFO(
      get_logger(), "FollowPath requested (%s): %zu poses, path age %.1f s, robot %.2f m from path",
      trigger.c_str(), last_path_->poses.size(), age, nearest);
    sendFollow(*last_path_);
    return {true, "FollowPath sent to controller_server"};
  }

  // Điều kiện gửi một đường tới FollowPath. Rỗng = được phép. `in_session`: đang trong phiên và FollowPath
  // còn chạy -> WARN localization ngắn (<= grace) được chấp nhận như khi đang bám đường.
  std::string checkFollowable(const nav_msgs::msg::Path & path, bool in_session, double & nearest)
  {
    geometry_msgs::msg::TransformStamped robot;
    try {
      robot = tf_buffer_->lookupTransform(
        path.header.frame_id.empty() ? global_frame_ : path.header.frame_id,
        base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return std::string("no robot pose (TF): ") + ex.what();
    }
    const double tf_age = (now() - rclcpp::Time(robot.header.stamp, get_clock()->get_clock_type()))
      .seconds();
    if (tf_age > tf_timeout_) {
      return "TF robot pose is " + fmt1(tf_age) + " s old (> tf_timeout)";
    }
    nearest = std::numeric_limits<double>::infinity();
    for (const auto & p : path.poses) {
      nearest = std::min(
        nearest, std::hypot(
          p.pose.position.x - robot.transform.translation.x,
          p.pose.position.y - robot.transform.translation.y));
    }
    const double max_start = get_parameter("max_start_distance").as_double();
    if (nearest > max_start) {
      return "robot is " + fmt1(nearest) + " m from the path (> max_start_distance " +
             fmt1(max_start) + " m): path is stale, send the goal again";
    }
    std::string loc_reason;
    if (!localizationOk(loc_reason) && !(in_session && following_ && withinGrace())) {
      return loc_reason;
    }
    if (!follow_client_->action_server_is_ready()) {
      return "controller_server FollowPath action not available (run navigation.launch.py "
             "and wait until controller_server is active)";
    }
    return {};
  }

  // Gửi `path` tới FollowPath trong phiên hiện tại. Nếu FollowPath đang chạy, controller_server (SimpleActionServer)
  // thay goal cũ bằng goal mới (preempt): goal cũ trả ABORTED (id cũ -> bỏ qua), DWB nhận setPlan mới.
  void sendFollow(const nav_msgs::msg::Path & path)
  {
    FollowPath::Goal request;
    request.path = path;
    request.controller_id = get_parameter("controller_id").as_string();
    request.goal_checker_id = get_parameter("goal_checker_id").as_string();

    const std::uint64_t id = ++follow_id_;
    following_ = true;
    // follow_handle_ giữ handle cũ (nếu có) tới khi goal mới được chấp nhận: hủy trong lúc đó hủy cả hai.
    session_->path.clear();
    for (const auto & p : path.poses) {
      session_->path.push_back(PathPoint{p.pose.position.x, p.pose.position.y});
    }
    session_->path_frame = path.header.frame_id.empty() ? global_frame_ : path.header.frame_id;
    session_->progress_segment = 0;
    session_->blocked_since.reset();
    session_->waiting_since.reset();
    session_->costmap_cleared = false;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, id](const FollowHandle::SharedPtr & handle) {
        if (id != follow_id_) {
          // Đã bị hủy / thay trước khi server chấp nhận -> hủy luôn goal vừa được chấp nhận
          // (trừ khi đã có goal mới hơn đang chạy: goal mới thay nó trên server).
          if (handle && !following_) {
            follow_client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          following_ = false;
          RCLCPP_ERROR(get_logger(), "controller_server rejected the FollowPath goal");
          endSession("REJECTED");
          return;
        }
        follow_handle_ = handle;
        publishState(
          session_ && (session_->obstacle_replans + session_->abort_replans) > 0 ?
          "FOLLOWING (replanned " + std::to_string(
            session_->obstacle_replans + session_->abort_replans) + "x)" : "FOLLOWING");
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
            RCLCPP_INFO(get_logger(), "FollowPath succeeded: goal reached (goal checker)");
            endSession("SUCCEEDED");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "FollowPath canceled");
            endSession("CANCELED");
            break;
          default:
            RCLCPP_ERROR(
              get_logger(), "FollowPath aborted by controller_server (no valid DWB trajectory, "
              "no progress, TF or costmap error; see controller_server log). It publishes zero "
              "velocity when it aborts");
            onFollowAborted();
            break;
        }
      };

    publishState("FOLLOW_REQUESTED");
    follow_client_->async_send_goal(request, options);
  }

  // FollowPath bị abort trong phiên: lập lại A* (có giới hạn) thay vì kết thúc ngay.
  void onFollowAborted()
  {
    if (!session_ || !get_parameter("replan.enabled").as_bool()) {
      endSession("ABORTED");
      return;
    }
    const int max_abort = static_cast<int>(get_parameter("replan.max_abort_replans").as_int());
    if (session_->abort_replans >= max_abort) {
      endSession(
        "ABORTED: FollowPath aborted " + std::to_string(session_->abort_replans + 1) +
        " times (replan.max_abort_replans " + std::to_string(max_abort) + ")");
      return;
    }
    ++session_->abort_replans;
    if (!session_->waiting_since) {
      session_->waiting_since = now();
    }
    if (!session_->replan_in_flight) {
      requestReplan(
        "FollowPath aborted (" + std::to_string(session_->abort_replans) + "/" +
        std::to_string(max_abort) + ")");
    }
  }

  // Hủy goal FollowPath đang chạy (nếu có), KHÔNG kết thúc phiên. controller_server phát vận tốc 0.
  bool stopFollowGoal()
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
    return true;
  }

  void endSession(const std::string & state)
  {
    stopFollowGoal();
    session_.reset();
    publishState(state);
  }

  // Kết thúc phiên (nếu có) + hủy FollowPath. Trả về true nếu có gì để hủy.
  bool cancelFollow(const std::string & reason)
  {
    if (!following_ && !session_) {
      return false;
    }
    endSession("CANCELED: " + reason);
    RCLCPP_WARN(get_logger(), "navigation canceled: %s", reason.c_str());
    return true;
  }

  void watchFollow()
  {
    if (!session_) {
      return;
    }
    const double timeout = get_parameter("follow_timeout").as_double();
    const double elapsed = (now() - session_->start).seconds();
    if (timeout > 0.0 && elapsed > timeout) {
      cancelFollow("follow_timeout (" + fmt1(timeout) + " s) exceeded");
      return;
    }
    if (following_) {
      std::string reason;
      if (!localizationOk(reason)) {
        // Robot 2026-10-14: scan-to-map consistency tụt dưới ngưỡng 0.5-1 s khi đang quay/đi rồi về LOCALIZED;
        // hủy ngay làm 6/8 lượt FollowPath dừng giữa đường. WARN ngắn được bỏ qua, LOST / status cũ thì không.
        const double grace = get_parameter("localization_uncertain_grace").as_double();
        if (withinGrace()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "following despite %s (%.1f s <= localization_uncertain_grace %.1f s)",
            reason.c_str(), (now() - *loc_warn_since_).seconds(), grace);
        } else {
          if (warnFresh()) {
            reason += " for " + fmt1((now() - *loc_warn_since_).seconds()) +
              " s (> localization_uncertain_grace " + fmt1(grace) + " s)";
          }
          cancelFollow(reason);
          return;
        }
      }
    }
    if (session_->replan_in_flight) {
      return;
    }
    if (!get_parameter("replan.enabled").as_bool()) {
      if (!following_) {
        endSession("FAILED: replan disabled while waiting for a path");
      }
      return;
    }
    if (following_) {
      checkBlockage();
    } else if (session_->waiting_since) {
      retryWhileWaiting();
    }
  }

  // ======================= Replanning =======================

  // Đoạn đường phía trước robot có ô >= replan.blocked_cost trên global costmap -> (xác nhận) lập lại A*.
  void checkBlockage()
  {
    auto & s = *session_;
    if (!costmap_ || s.path.empty()) {
      return;
    }
    const double costmap_age = (now() - costmap_time_).seconds();
    if (costmap_age > get_parameter("replan.costmap_timeout").as_double()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "replan: global costmap is %.1f s old, path blockage not checked", costmap_age);
      s.blocked_since.reset();
      return;
    }
    if (costmap_->header.frame_id != s.path_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "replan: costmap frame '%s' != path frame '%s', path blockage not checked",
        costmap_->header.frame_id.c_str(), s.path_frame.c_str());
      return;
    }
    geometry_msgs::msg::TransformStamped robot;
    try {
      robot = tf_buffer_->lookupTransform(s.path_frame, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;   // localization / TF đã được giám sát ở nơi khác
    }
    double along = 0.0;
    s.progress_segment = projectOnPath(
      s.path, robot.transform.translation.x, robot.transform.translation.y, s.progress_segment, 2.0,
      along);

    const auto & info = costmap_->info;
    GridView grid;
    grid.origin_x = info.origin.position.x;
    grid.origin_y = info.origin.position.y;
    grid.resolution = info.resolution;
    grid.width = info.width;
    grid.height = info.height;
    grid.data = costmap_->data.size() == static_cast<std::size_t>(info.width) * info.height ?
      costmap_->data.data() : nullptr;
    const auto b = findBlockage(
      s.path, grid, s.progress_segment, along,
      get_parameter("replan.start_offset").as_double(),
      get_parameter("replan.lookahead_distance").as_double(),
      static_cast<int>(get_parameter("replan.blocked_cost").as_int()));
    if (!b.blocked) {
      if (s.blocked_since) {
        RCLCPP_INFO(get_logger(), "replan: path is clear again (blockage not confirmed)");
        s.blocked_since.reset();
      }
      return;
    }
    const std::string where = "obstacle on the path " + fmt2(b.distance_ahead) + " m ahead at (" +
      fmt(b.x, b.y) + "), cost " + std::to_string(b.cost);
    if (!s.blocked_since) {
      s.blocked_since = now();
      RCLCPP_WARN(get_logger(), "replan: %s, confirming", where.c_str());
    }
    if ((now() - *s.blocked_since).seconds() < get_parameter("replan.confirm_time").as_double()) {
      return;
    }
    if (s.last_attempt &&
      (now() - *s.last_attempt).seconds() < get_parameter("replan.min_interval").as_double())
    {
      return;
    }
    const int max_replans = static_cast<int>(get_parameter("replan.max_obstacle_replans").as_int());
    if (max_replans > 0 && s.obstacle_replans >= max_replans) {
      RCLCPP_ERROR(
        get_logger(), "replan: %s, but replan.max_obstacle_replans (%d) reached: stopping",
        where.c_str(), max_replans);
      endSession("FAILED: path blocked, replan.max_obstacle_replans reached");
      return;
    }
    ++s.obstacle_replans;
    requestReplan(where);
  }

  void retryWhileWaiting()
  {
    auto & s = *session_;
    const double waited = (now() - *s.waiting_since).seconds();
    const double wait_timeout = get_parameter("replan.wait_timeout").as_double();
    if (waited > wait_timeout) {
      RCLCPP_ERROR(
        get_logger(), "replan: no followable path for %.1f s (> replan.wait_timeout %.1f s): giving up",
        waited, wait_timeout);
      endSession("FAILED: no free path to the goal for " + fmt1(waited) + " s");
      return;
    }
    if (s.last_attempt &&
      (now() - *s.last_attempt).seconds() < get_parameter("replan.min_interval").as_double())
    {
      return;
    }
    requestReplan("retry, waiting " + fmt1(waited) + " s");
  }

  // ComputePathToPose tới đích của phiên, start = pose robot hiện tại. Thành công -> FollowPath đường mới.
  void requestReplan(const std::string & reason)
  {
    auto & s = *session_;
    s.last_attempt = now();
    RCLCPP_WARN(get_logger(), "replan: %s -> computing a new A* path", reason.c_str());
    publishState("REPLANNING: " + reason);
    if (!client_->action_server_is_ready()) {
      onReplanFailed("planner_server action not available");
      return;
    }
    ComputePathToPose::Goal request;
    request.goal = s.goal;
    request.planner_id = planner_id_;
    request.use_start = false;
    s.replan_in_flight = true;

    const std::uint64_t id = ++request_id_;
    const std::uint64_t sid = s.id;
    const auto current = [this, id, sid] {
        return id == request_id_ && session_ && session_->id == sid;
      };
    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, current](const GoalHandle::SharedPtr & handle) {
        if (!handle && current()) {
          session_->replan_in_flight = false;
          onReplanFailed("planner_server rejected the ComputePathToPose goal");
        }
      };
    options.result_callback =
      [this, current](const GoalHandle::WrappedResult & result) {
        if (!current()) {
          return;   // goal mới / phiên đã kết thúc
        }
        session_->replan_in_flight = false;
        if (!succeeded(result)) {
          geometry_msgs::msg::Vector3 start;
          try {
            start = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero)
              .transform.translation;
          } catch (const tf2::TransformException &) {
          }
          onReplanFailed("no path (" + resultCode(result) + ")" + explain(session_->goal, start));
          return;
        }
        const auto & path = result.result->path;
        path_pub_->publish(path);
        last_path_ = path;
        last_path_time_ = now();
        RCLCPP_INFO(
          get_logger(), "replan: new A* path %zu poses, %.2f m (planning %.1f ms)",
          path.poses.size(), pathLength(path),
          rclcpp::Duration(result.result->planning_time).seconds() * 1e3);
        double nearest = 0.0;
        const std::string why = checkFollowable(path, true, nearest);
        if (!why.empty()) {
          onReplanFailed("new path not followable: " + why);
          return;
        }
        sendFollow(path);
      };
    client_->async_send_goal(request, options);
  }

  // Lập lại thất bại: dừng (hủy FollowPath, giữ phiên), chờ rồi thử lại. Xóa obstacle layer ngoài vùng quanh
  // robot một lần (vết vật cản cũ phía sau robot không bao giờ được LiDAR xóa vì nửa sau là NaN).
  void onReplanFailed(const std::string & why)
  {
    auto & s = *session_;
    if (stopFollowGoal()) {
      RCLCPP_WARN(get_logger(), "replan: stopping the robot (FollowPath canceled) while waiting");
    }
    if (!s.waiting_since) {
      s.waiting_since = now();
    }
    RCLCPP_ERROR(get_logger(), "replan failed: %s", why.c_str());
    publishState(
      "WAITING: " + why + " (retry every " + fmt1(
        get_parameter(
          "replan.min_interval").as_double()) +
      " s, give up after " + fmt1(get_parameter("replan.wait_timeout").as_double()) + " s)");
    nav_msgs::msg::Path empty;
    empty.header.frame_id = global_frame_;
    empty.header.stamp = now();
    path_pub_->publish(empty);

    if (get_parameter("replan.clear_costmap_on_failure").as_bool() && !s.costmap_cleared) {
      s.costmap_cleared = true;
      if (clear_client_->service_is_ready()) {
        auto req = std::make_shared<ClearExcept::Request>();
        req->reset_distance =
          static_cast<float>(get_parameter("replan.clear_keep_distance").as_double());
        clear_client_->async_send_request(req);
        RCLCPP_WARN(
          get_logger(), "replan: clearing global costmap obstacles outside a %.1f m square around "
          "the robot (stale marks)", req->reset_distance);
      } else {
        RCLCPP_WARN(
          get_logger(), "replan: costmap clear service %s not available",
          clear_client_->get_service_name());
      }
    }
  }

  // ======================= localization =======================

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

  // Status mới, mức WARN, đã biết thời điểm bắt đầu chuỗi WARN.
  bool warnFresh() const
  {
    return loc_status_ && loc_warn_since_ && loc_status_->level == DiagnosticStatus::WARN &&
           (now() - loc_status_time_).seconds() <=
           get_parameter("localization_status_timeout").as_double();
  }

  bool withinGrace() const
  {
    return warnFresh() &&
           (now() - *loc_warn_since_).seconds() <=
           get_parameter("localization_uncertain_grace").as_double();
  }

  void publishState(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  // ======================= helpers =======================

  static bool succeeded(const GoalHandle::WrappedResult & result)
  {
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
           !result.result->path.poses.empty();
  }

  static std::string resultCode(const GoalHandle::WrappedResult & result)
  {
    return result.code == rclcpp_action::ResultCode::ABORTED ? "aborted" :
           result.code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "empty path";
  }

  static double pathLength(const nav_msgs::msg::Path & path)
  {
    double length = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      length += std::hypot(
        path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
        path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    }
    return length;
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

  static std::string fmt1(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    return buf;
  }

  static std::string fmt2(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
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
  rclcpp::Time costmap_time_;

  // FollowPath
  std::optional<nav_msgs::msg::Path> last_path_;
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;   // đích của last_path_
  rclcpp::Time last_path_time_;
  bool following_{false};          // có goal FollowPath đang chạy (hoặc đang chờ được chấp nhận)
  std::uint64_t follow_id_{0};
  FollowHandle::SharedPtr follow_handle_;
  std::optional<Session> session_;
  std::uint64_t session_counter_{0};
  DiagnosticStatus::ConstSharedPtr loc_status_;
  rclcpp::Time loc_status_time_;
  std::optional<rclcpp::Time> loc_warn_since_;   // đầu chuỗi WARN liên tục hiện tại

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
  rclcpp::Client<ClearExcept>::SharedPtr clear_client_;
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
