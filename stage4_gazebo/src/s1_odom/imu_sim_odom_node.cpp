// imu_sim_odom: thay imu_odom (UART ESP32 + BNO055) trong Gazebo.
// Phần xuất bản chép từ stage1_odom/src/imu_odom.cpp: cùng topic, frame, covariance, logic yaw
// (zero_yaw_on_start, service /imu_odom/zero_yaw), đóng dấu thời gian lúc nhận (now(), theo sim time).
// Khác: nguồn là sensor IMU Gazebo (/sim/imu, rad/s sẵn) thay vì dòng CSV; không có /imu/mag
// (không mô phỏng từ kế, không node nào trong stage1–3 dùng).
//
//   /sim/imu (Gazebo, 50 Hz) ─► /imu/data (Imu, imu_link) + /imu_odom (Odometry: yaw + vyaw)

#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"

class ImuSimOdom : public rclcpp::Node
{
public:
  ImuSimOdom()
  : Node("imu_odom")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    // Giống imu_odom: false (mặc định) = yaw "tuyệt đối" (ở sim: theo trục X của world Gazebo,
    // thay cho hướng từ trường của NDOF); EKF tự xử lý bằng odom1_relative.
    zero_yaw_on_start_ = declare_parameter<bool>("zero_yaw_on_start", false);
    const auto input_topic = declare_parameter<std::string>("input_topic", "/sim/imu");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/imu_odom", 10);

    zero_yaw_service_ = create_service<std_srvs::srv::Empty>(
      "/imu_odom/zero_yaw",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        yaw_offset_initialized_ = false;
        zero_yaw_on_start_ = true;
        RCLCPP_INFO(get_logger(), "IMU yaw will be zeroed on next sample.");
      });

    sim_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuSimOdom::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "IMU SIM ODOM started | input=%s", input_topic.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr in)
  {
    const auto & q = in->orientation;
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(norm) || norm < 0.5 || norm > 1.5) return;

    const double w = q.w / norm;
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;

    const auto stamp = now();

    // ---------------- /imu/data ----------------
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame_id_;

    imu.orientation.w = w;
    imu.orientation.x = x;
    imu.orientation.y = y;
    imu.orientation.z = z;
    imu.angular_velocity = in->angular_velocity;
    imu.linear_acceleration = in->linear_acceleration;

    // Covariance: giống imu_odom (giá trị cố định, không lấy từ Gazebo)
    imu.orientation_covariance[0] = 0.01;
    imu.orientation_covariance[4] = 0.01;
    imu.orientation_covariance[8] = 0.01;

    imu.angular_velocity_covariance[0] = 0.0004;
    imu.angular_velocity_covariance[4] = 0.0004;
    imu.angular_velocity_covariance[8] = 0.0004;

    imu.linear_acceleration_covariance[0] = 0.04;
    imu.linear_acceleration_covariance[4] = 0.04;
    imu.linear_acceleration_covariance[8] = 0.04;

    imu_pub_->publish(imu);

    // ---------------- quaternion -> yaw ----------------
    const double raw_yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    if (zero_yaw_on_start_ && !yaw_offset_initialized_) {
      yaw_offset_ = raw_yaw;
      yaw_offset_initialized_ = true;
      RCLCPP_INFO(get_logger(), "IMU yaw zeroed | offset (absolute) = %.3f rad", yaw_offset_);
    }

    const double offset = zero_yaw_on_start_ ? yaw_offset_ : 0.0;
    const double yaw = std::atan2(std::sin(raw_yaw - offset), std::cos(raw_yaw - offset));

    // ---------------- /imu_odom ----------------
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.twist.twist.angular.z = imu.angular_velocity.z;

    constexpr double UNKNOWN = 1e6;

    odom.pose.covariance[0] = UNKNOWN;
    odom.pose.covariance[7] = UNKNOWN;
    odom.pose.covariance[14] = UNKNOWN;
    odom.pose.covariance[21] = UNKNOWN;
    odom.pose.covariance[28] = UNKNOWN;
    odom.pose.covariance[35] = imu.orientation_covariance[8];

    odom.twist.covariance[0] = UNKNOWN;
    odom.twist.covariance[7] = UNKNOWN;
    odom.twist.covariance[14] = UNKNOWN;
    odom.twist.covariance[21] = UNKNOWN;
    odom.twist.covariance[28] = UNKNOWN;
    odom.twist.covariance[35] = imu.angular_velocity_covariance[8];

    odom_pub_->publish(odom);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU ODOM | Yaw: %.3f rad (raw %.3f) | Wz: %.3f rad/s",
      yaw, raw_yaw, odom.twist.twist.angular.z);
  }

  std::string frame_id_;
  bool zero_yaw_on_start_ = false;
  bool yaw_offset_initialized_ = false;
  double yaw_offset_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sim_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_yaw_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSimOdom>());
  rclcpp::shutdown();
  return 0;
}
