
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"

class zlac8015d_odom : public rclcpp::Node
{
public:
  zlac8015d_odom()
  : Node("zlac8015d_odom")
  {
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0535);

    wheel_distance_ = declare_parameter<double>("wheel_distance", 0.34);

    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    feedback_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/wheel_feedback", 10,
        std::bind(
          &zlac8015d_odom::feedbackCallback,
          this,
          std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>( "/wheel_odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>( this);

    reset_service_ = create_service<std_srvs::srv::Empty>( "/reset_odom",
        std::bind(
          &zlac8015d_odom::resetOdom,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "zlac8015d_odom started");
  }

private:

  // ============================================================
  // WHEEL FEEDBACK
  // ============================================================

  void feedbackCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid wheel feedback size.");

      return;
    }

    double left_rpm = msg->data[0];
    double right_rpm = msg->data[1];

    double left_travelled = msg->data[2];
    double right_travelled = msg->data[3];

    if (!std::isfinite(left_rpm) ||
        !std::isfinite(right_rpm) ||
        !std::isfinite(left_travelled) ||
        !std::isfinite(right_travelled)) {

      RCLCPP_WARN(
        get_logger(),
        "Invalid wheel feedback.");

      return;
    }

    updateVelocity(left_rpm, right_rpm);

    rclcpp::Time current_time = now();

    if (!wheel_initialized_) {
      last_left_travelled_ = left_travelled;
      last_right_travelled_ = right_travelled;

      wheel_initialized_ = true;

      publishOdom(current_time);

      return;
    }

    double left_distance = left_travelled - last_left_travelled_;

    double right_distance = right_travelled - last_right_travelled_;

    if (std::abs(left_distance) < distance_deadband_ &&
        std::abs(right_distance) < distance_deadband_) {

      publishOdom(current_time);

      return;
    }

    last_left_travelled_ = left_travelled;
    last_right_travelled_ = right_travelled;

    // ==========================================================
    // DIFFERENTIAL DRIVE ODOMETRY
    // ==========================================================

    double distance = (left_distance + right_distance) / 2.0;

    double delta_yaw = (right_distance - left_distance) / wheel_distance_;

    double middle_yaw = yaw_ + delta_yaw / 2.0;

    x_ += distance * std::cos(middle_yaw);
    y_ += distance * std::sin(middle_yaw);

    yaw_ += delta_yaw;

    yaw_ = std::atan2(
      std::sin(yaw_),
      std::cos(yaw_));

    publishOdom(current_time);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "X: %.3f | Y: %.3f | Yaw: %.3f | V: %.3f | W: %.3f",
      x_,
      y_,
      yaw_,
      linear_velocity_,
      angular_velocity_);
  }

  // ============================================================
  // VELOCITY
  // ============================================================

  void updateVelocity(
    double left_rpm,
    double right_rpm)
  {
    double left_velocity = left_rpm * wheel_radius_ / rpm_factor_;

    double right_velocity = right_rpm * wheel_radius_ / rpm_factor_;

    double linear_velocity = (left_velocity + right_velocity) / 2.0;

    double angular_velocity = (right_velocity - left_velocity) / wheel_distance_;

    linear_velocity_ = filter_alpha_ * linear_velocity + (1.0 - filter_alpha_) * linear_velocity_;

    angular_velocity_ = filter_alpha_ * angular_velocity + (1.0 - filter_alpha_) * angular_velocity_;

    if (std::abs(linear_velocity_) < linear_deadband_) {
      linear_velocity_ = 0.0;
    }

    if (std::abs(angular_velocity_) < angular_deadband_) {
      angular_velocity_ = 0.0;
    }
  }

  // ============================================================
  // PUBLISH ODOMETRY
  // ============================================================

  void publishOdom(
    const rclcpp::Time & stamp)
  {
    double quaternion_z = std::sin(yaw_ / 2.0);

    double quaternion_w = std::cos(yaw_ / 2.0);

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // ==========================================================
    // POSITION
    // ==========================================================

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // ==========================================================
    // ORIENTATION
    // ==========================================================

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;

    odom.pose.pose.orientation.z = quaternion_z;
    odom.pose.pose.orientation.w = quaternion_w;

    // ==========================================================
    // LINEAR VELOCITY
    // ==========================================================

    odom.twist.twist.linear.x = linear_velocity_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    // ==========================================================
    // ANGULAR VELOCITY
    // ==========================================================

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    odom.twist.twist.angular.z = angular_velocity_;

    // ==========================================================
    // POSE COVARIANCE
    // ==========================================================

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;

    odom.pose.covariance[14] = 1000000.0;
    odom.pose.covariance[21] = 1000000.0;
    odom.pose.covariance[28] = 1000000.0;

    odom.pose.covariance[35] = 0.05;

    // ==========================================================
    // TWIST COVARIANCE
    // ==========================================================

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;

    odom.twist.covariance[14] = 1000000.0;
    odom.twist.covariance[21] = 1000000.0;
    odom.twist.covariance[28] = 1000000.0;

    odom.twist.covariance[35] = 0.05;

    // ==========================================================
    // PUBLISH
    // ==========================================================

    odom_pub_->publish(odom);

    if (publish_tf_) {
      publishTf( stamp, quaternion_z, quaternion_w);
    }
  }

  // ============================================================
  // PUBLISH TF
  // ============================================================

  void publishTf(
    const rclcpp::Time & stamp,
    double quaternion_z,
    double quaternion_w)
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;

    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    // ==========================================================
    // TRANSLATION
    // ==========================================================

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    // ==========================================================
    // ROTATION
    // ==========================================================

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;

    transform.transform.rotation.z = quaternion_z;
    transform.transform.rotation.w = quaternion_w;

    tf_broadcaster_->sendTransform(transform);
  }

  // ============================================================
  // RESET ODOMETRY
  // ============================================================

  void resetOdom(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;

    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;

    last_left_travelled_ = 0.0;
    last_right_travelled_ = 0.0;

    wheel_initialized_ = false;

    publishOdom(now());

    RCLCPP_INFO(
      get_logger(),
      "Odometry reset.");
  }

  // ============================================================
  // PARAMETERS
  // ============================================================

  static constexpr double pi_ = 3.14159265358979323846;

  static constexpr double rpm_factor_ = 30.0 / pi_;

  static constexpr double filter_alpha_ = 0.2;

  static constexpr double linear_deadband_ = 0.004;
  static constexpr double angular_deadband_ = 0.004;

  static constexpr double distance_deadband_ = 0.0005;

  double wheel_radius_ = 0.0535;
  double wheel_distance_ = 0.34;

  // ============================================================
  // ROBOT STATE
  // ============================================================

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;

  double last_left_travelled_ = 0.0;
  double last_right_travelled_ = 0.0;

  bool publish_tf_ = true;
  bool wheel_initialized_ = false;

  // ============================================================
  // ROS2 INTERFACES
  // ============================================================

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr feedback_sub_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Service<
    std_srvs::srv::Empty>::SharedPtr reset_service_;

  std::unique_ptr<
    tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<zlac8015d_odom>());

  rclcpp::shutdown();

  return 0;
}