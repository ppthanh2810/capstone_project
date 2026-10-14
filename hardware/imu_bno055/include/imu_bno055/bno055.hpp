#pragma once

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <initializer_list>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace imu_bno055 {

struct ImuSample {
  uint32_t esp_ms{};
  double qw{}, qx{}, qy{}, qz{};
  double gx{}, gy{}, gz{};  // degrees/second, from ESP32
  double ax{}, ay{}, az{};  // m/s^2, accelerometer including gravity
};

class Bno055 {
public:
  explicit Bno055(std::string port = "/dev/ttyUSB0") : port_(std::move(port)) {}
  ~Bno055() { closePort(); }
  Bno055(const Bno055 &) = delete;
  Bno055 &operator=(const Bno055 &) = delete;

  bool connected() const { return fd_ >= 0; }
  const std::string &port() const { return port_; }

  bool openPort()
  {
    if (connected()) return true;
    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return false;

    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }
    ::cfmakeraw(&tty);
    if (::cfsetispeed(&tty, B115200) != 0 || ::cfsetospeed(&tty, B115200) != 0) {
      ::close(fd); return false;
    }
    tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | CSTOPB)) | CS8 | CLOCAL | CREAD;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }
    ::tcflush(fd, TCIFLUSH);
    fd_ = fd;
    line_.clear();
    return true;
  }

  void closePort()
  {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    line_.clear();
  }

  // Drain available UART bytes; return the newest complete, valid IMU packet.
  bool readLatest(ImuSample &latest)
  {
    if (!connected()) return false;
    bool got = false;
    char buffer[256];
    for (;;) {
      const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          const char c = buffer[i];
          if (c == '\n') {
            if (!line_.empty() && line_.back() == '\r') line_.pop_back();
            ImuSample sample;
            if (parse(line_, sample)) { latest = sample; got = true; }
            line_.clear();
          } else if (line_.size() < 220) {
            line_ += c;
          } else {
            line_.clear();  // reject overlong/corrupt line
          }
        }
        continue;
      }
      if (n == 0) break;
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      closePort();  // USB unplugged or serial read error: node will retry
      break;
    }
    return got;
  }

private:
  static bool parse(const std::string &line, ImuSample &s)
  {
    unsigned long ms = 0;
    int consumed = 0;
    const int count = std::sscanf(line.c_str(),
      "IMU,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%n",
      &ms, &s.qw, &s.qx, &s.qy, &s.qz,
      &s.gx, &s.gy, &s.gz, &s.ax, &s.ay, &s.az, &consumed);
    if (count != 11 || consumed != static_cast<int>(line.size()) ||
        ms > std::numeric_limits<uint32_t>::max()) return false;
    for (const double v : {s.qw, s.qx, s.qy, s.qz, s.gx, s.gy, s.gz, s.ax, s.ay, s.az})
      if (!std::isfinite(v)) return false;
    const double norm = std::sqrt(s.qw*s.qw + s.qx*s.qx + s.qy*s.qy + s.qz*s.qz);
    if (norm < 0.1) return false;
    s.qw /= norm; s.qx /= norm; s.qy /= norm; s.qz /= norm;
    s.esp_ms = static_cast<uint32_t>(ms);
    return true;
  }

  std::string port_;
  int fd_{-1};
  std::string line_;
};

}  // namespace imu_bno055