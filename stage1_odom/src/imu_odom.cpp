
#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class ImuOdom : public rclcpp::Node
{
public:
  ImuOdom() : Node("imu_odom")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB1");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS());

    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "/imu/mag", rclcpp::SensorDataQoS());

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/imu_odom", 10);

    timer_ = create_wall_timer(5ms, [this] { readSerial(); });

    RCLCPP_INFO(
      get_logger(), "IMU ODOM started | port=%s | baud=115200",
      port_.c_str());
  }

  ~ImuOdom() override
  {
    closeSerial();
  }

private:

  // ============================================================
  // SERIAL CONNECTION
  // ============================================================

  bool openSerial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot open %s: %s",
        port_.c_str(), std::strerror(errno));

      return false;
    }

    termios tty{};

    if (tcgetattr(fd_, &tty) != 0) {
      closeSerial();
      return false;
    }

    cfmakeraw(&tty);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      closeSerial();
      return false;
    }

    tcflush(fd_, TCIFLUSH);

    RCLCPP_INFO(
      get_logger(), "Connected to %s at 115200 8N1",
      port_.c_str());

    return true;
  }

  void closeSerial()
  {
    if (fd_ >= 0) ::close(fd_);

    fd_ = -1;
    buffer_.clear();
    discard_ = false;
  }

  // ============================================================
  // PROCESS UART PACKET
  // ============================================================

  void processLine(const std::string &line)
  {
    if (line.compare(0, 4, "IMU,") != 0) return;

    std::array<std::string, 15> fields{};
    std::array<double, 13> v{};

    std::stringstream ss(line);
    std::string token;

    size_t count = 0;

    while (std::getline(ss, token, ',')) {
      if (count >= fields.size()) return;
      fields[count++] = token;
    }

    if (count != 15 || fields[0] != "IMU") return;

    // 0  IMU
    // 1  ESP32 millis
    // 2  qw   3 qx   4 qy   5 qz
    // 6  gx   7 gy   8 gz
    // 9  ax  10 ay  11 az
    // 12 mx  13 my  14 mz

    char *end = nullptr;

    errno = 0;

    const unsigned long timestamp =
      std::strtoul(fields[1].c_str(), &end, 10);

    if (errno != 0 ||
        end == fields[1].c_str() ||
        *end != '\0' ||
        timestamp > std::numeric_limits<uint32_t>::max())
      return;

    for (size_t i = 0; i < v.size(); ++i) {
      errno = 0;

      v[i] = std::strtod(fields[i + 2].c_str(), &end);

      if (errno != 0 ||
          end == fields[i + 2].c_str() ||
          *end != '\0' ||
          !std::isfinite(v[i]))
        return;
    }

    const double qw = v[0], qx = v[1];
    const double qy = v[2], qz = v[3];

    const double gx = v[4], gy = v[5], gz = v[6];

    const double ax = v[7], ay = v[8], az = v[9];

    const double mx = v[10], my = v[11], mz = v[12];

    const double norm = std::sqrt(
      qw*qw + qx*qx + qy*qy + qz*qz);

    if (norm < 0.5 || norm > 1.5) return;

    constexpr double DEG_TO_RAD =
      3.14159265358979323846 / 180.0;

    const double w = qw / norm;
    const double x = qx / norm;
    const double y = qy / norm;
    const double z = qz / norm;

    const double gyro_x = gx * DEG_TO_RAD;
    const double gyro_y = gy * DEG_TO_RAD;
    const double gyro_z = gz * DEG_TO_RAD;

    const auto stamp = now();

    // ========================================================
    // PUBLISH IMU DATA
    // ========================================================

    sensor_msgs::msg::Imu imu;

    imu.header.stamp = stamp;
    imu.header.frame_id = frame_id_;

    // Orientation

    imu.orientation.w = w;
    imu.orientation.x = x;
    imu.orientation.y = y;
    imu.orientation.z = z;

    // Angular velocity

    imu.angular_velocity.x = gyro_x;
    imu.angular_velocity.y = gyro_y;
    imu.angular_velocity.z = gyro_z;

    // Linear acceleration

    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;

    // Covariance

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

    // ========================================================
    // PUBLISH MAGNETOMETER
    // ========================================================

    sensor_msgs::msg::MagneticField mag;

    mag.header = imu.header;

    mag.magnetic_field.x = mx * 1e-6;
    mag.magnetic_field.y = my * 1e-6;
    mag.magnetic_field.z = mz * 1e-6;

    mag_pub_->publish(mag);

    // ========================================================
    // QUATERNION -> YAW
    // ========================================================

    const double yaw = std::atan2(
      2.0 * (w*z + x*y),
      1.0 - 2.0 * (y*y + z*z));

    // ========================================================
    // PUBLISH IMU ODOMETRY
    // ========================================================

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Position - not measured

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    // Orientation - yaw only

    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);

    // Linear velocity - not measured

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    // Angular velocity - gyroscope

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = gyro_z;

    // Covariance

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
    odom.twist.covariance[35] =
      imu.angular_velocity_covariance[8];

    odom_pub_->publish(odom);

    // ========================================================
    // PRINT IMU ODOMETRY
    // ========================================================

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU ODOM | Yaw: %.3f rad | Wz: %.3f rad/s",
      yaw,
      odom.twist.twist.angular.z);
  }

  // ============================================================
  // READ SERIAL
  // ============================================================

  void readSerial()
  {
    if (fd_ < 0) {

      if (std::chrono::steady_clock::now() < next_retry_)
        return;

      next_retry_ = std::chrono::steady_clock::now() + 1s;

      if (!openSerial()) return;
    }

    char data[512];

    for (int attempt = 0; attempt < 16; ++attempt) {

      const ssize_t n = ::read(fd_, data, sizeof(data));

      if (n > 0) {

        for (ssize_t i = 0; i < n; ++i) {

          const char c = data[i];

          if (c == '\n') {

            if (!discard_) {

              if (!buffer_.empty() && buffer_.back() == '\r')
                buffer_.pop_back();

              processLine(buffer_);
            }

            buffer_.clear();
            discard_ = false;
          }

          else if (!discard_) {

            if (buffer_.size() < 255)
              buffer_ += c;

            else {
              buffer_.clear();
              discard_ = true;
            }
          }
        }
      }

      else if (n == 0) {
        break;
      }

      else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      else if (errno == EINTR) {
        continue;
      }

      else {

        RCLCPP_WARN(
          get_logger(), "Serial error: %s",
          std::strerror(errno));

        closeSerial();

        next_retry_ = std::chrono::steady_clock::now() + 1s;

        break;
      }
    }
  }

  // ============================================================
  // VARIABLES
  // ============================================================

  std::string port_, frame_id_, buffer_;

  int fd_ = -1;
  bool discard_ = false;

  std::chrono::steady_clock::time_point next_retry_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOdom>());
  rclcpp::shutdown();
  return 0;
}