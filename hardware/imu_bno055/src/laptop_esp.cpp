#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class LaptopEsp : public rclcpp::Node
{
public:
  LaptopEsp() : Node("laptop_esp")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    timer_ = create_wall_timer(5ms, [this] { readSerial(); });

    RCLCPP_INFO(get_logger(), "BNO055: %s -> /imu/data", port_.c_str());
  }

  ~LaptopEsp() override { closeSerial(); }

private:
  bool openSerial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot open %s: %s", port_.c_str(), std::strerror(errno));
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

    RCLCPP_INFO(get_logger(), "Connected to %s at 115200 8N1", port_.c_str());

    return true;
  }

  void closeSerial()
  {
    if (fd_ >= 0) ::close(fd_);

    fd_ = -1;
    buffer_.clear();
    discard_ = false;
  }

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

    // fields:
    // 0  IMU
    // 1  ESP32 millis
    // 2  qw   3 qx   4 qy   5 qz
    // 6  gx   7 gy   8 gz
    // 9  ax  10 ay  11 az
    // 12 mx  13 my  14 mz

    char *end = nullptr;

    errno = 0;
    const unsigned long esp_ms =
      std::strtoul(fields[1].c_str(), &end, 10);

    if (errno != 0 || end == fields[1].c_str() || *end != '\0')
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

    const double qw = v[0], qx = v[1], qy = v[2], qz = v[3];
    const double gx = v[4], gy = v[5], gz = v[6];
    const double ax = v[7], ay = v[8], az = v[9];
    const double mx = v[10], my = v[11], mz = v[12];

    const double norm = std::sqrt(
      qw*qw + qx*qx + qy*qy + qz*qz);

    if (norm < 0.5 || norm > 1.5) return;

    constexpr double DEG_TO_RAD =
      3.14159265358979323846 / 180.0;

    sensor_msgs::msg::Imu msg;

    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.orientation.w = qw / norm;
    msg.orientation.x = qx / norm;
    msg.orientation.y = qy / norm;
    msg.orientation.z = qz / norm;

    msg.angular_velocity.x = gx * DEG_TO_RAD;
    msg.angular_velocity.y = gy * DEG_TO_RAD;
    msg.angular_velocity.z = gz * DEG_TO_RAD;

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    imu_pub_->publish(msg);

    ++received_;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU packets=%lu ESP_time=%lu ms | "
      "Q=(%.3f %.3f %.3f %.3f) | "
      "GYRO=(%.3f %.3f %.3f) rad/s | "
      "ACC=(%.3f %.3f %.3f) m/s2 | "
      "MAG=(%.3f %.3f %.3f) uT",
      static_cast<unsigned long>(received_), esp_ms,
      msg.orientation.w, msg.orientation.x,
      msg.orientation.y, msg.orientation.z,
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z,
      ax, ay, az, mx, my, mz);
  }

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
      else if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      else if (errno == EINTR) {
        continue;
      }
      else {
        RCLCPP_WARN(
          get_logger(), "Serial error: %s", std::strerror(errno));

        closeSerial();
        next_retry_ = std::chrono::steady_clock::now() + 1s;
        break;
      }
    }
  }

  std::string port_, frame_id_, buffer_;

  int fd_ = -1;
  bool discard_ = false;
  uint64_t received_ = 0;

  std::chrono::steady_clock::time_point next_retry_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaptopEsp>());
  rclcpp::shutdown();
  return 0;
}