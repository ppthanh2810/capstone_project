#include "imu_bno055/bno055.hpp"

#include <chrono>
#include <fcntl.h>
#include <functional>
#include <linux/i2c-dev.h>
#include <memory>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

namespace imu_bno055
{

Bno055::Bno055(const std::string & device, uint8_t address)
: fd_(open(device.c_str(), O_RDWR)), address_(address)
{
  if (fd_ < 0)
    throw std::runtime_error("Cannot open I2C");

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    close(fd_);
    fd_ = -1;
    throw std::runtime_error("Cannot connect BNO055");
  }
}


Bno055::~Bno055()
{
  if (fd_ >= 0)
    close(fd_);
}


void Bno055::writeReg(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};

  if (write(fd_, data, 2) != 2)
    throw std::runtime_error("I2C write failed");
}


uint8_t Bno055::readReg(uint8_t reg)
{
  uint8_t value;

  if (write(fd_, &reg, 1) != 1)
    throw std::runtime_error("I2C register select failed");

  if (read(fd_, &value, 1) != 1)
    throw std::runtime_error("I2C read failed");

  return value;
}


void Bno055::readRegs(uint8_t reg, uint8_t * data, int length)
{
  if (write(fd_, &reg, 1) != 1)
    throw std::runtime_error("I2C register select failed");

  if (read(fd_, data, length) != length)
    throw std::runtime_error("I2C block read failed");
}


int16_t Bno055::toInt16(uint8_t lsb, uint8_t msb)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(lsb) |
    (static_cast<uint16_t>(msb) << 8));
}


void Bno055::restoreCalibration()
{
  const uint8_t cal[22] = {
    0xF2, 0xFF,
    0xFC, 0xFF,
    0xFF, 0xFF,
    0x95, 0xFF,
    0x29, 0x03,
    0x89, 0xFB,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0x00, 0x00,
    0xE8, 0x03,
    0xD7, 0x02
  };

  for (int i = 0; i < 22; ++i)
    writeReg(static_cast<uint8_t>(0x55 + i), cal[i]);
}


void Bno055::configure()
{
  // Page 0
  writeReg(0x07, 0x00);

  // CONFIGMODE
  writeReg(0x3D, 0x00);
  std::this_thread::sleep_for(100ms);

  // Normal power mode
  writeReg(0x3E, 0x00);
  std::this_thread::sleep_for(100ms);

  // Restore calibration
  restoreCalibration();

  // NDOF mode
  writeReg(0x3D, 0x0C);
  std::this_thread::sleep_for(1s);
}


uint8_t Bno055::getChipId()
{
  return readReg(0x00);
}


uint8_t Bno055::getMode()
{
  return readReg(0x3D);
}


bool Bno055::readImu(ImuData & d)
{
  uint8_t q[8];
  uint8_t g[6];
  uint8_t a[6];

  // Quaternion
  readRegs(0x20, q, 8);

  // Gyroscope
  readRegs(0x14, g, 6);

  // Linear acceleration
  readRegs(0x28, a, 6);

  constexpr double QUAT_SCALE = 1.0 / 16384.0;
  constexpr double GYRO_SCALE =
    3.14159265358979323846 / (180.0 * 16.0);
  constexpr double ACC_SCALE = 1.0 / 100.0;

  d.qw = toInt16(q[0], q[1]) * QUAT_SCALE;
  d.qx = toInt16(q[2], q[3]) * QUAT_SCALE;
  d.qy = toInt16(q[4], q[5]) * QUAT_SCALE;
  d.qz = toInt16(q[6], q[7]) * QUAT_SCALE;

  d.gx = toInt16(g[0], g[1]) * GYRO_SCALE;
  d.gy = toInt16(g[2], g[3]) * GYRO_SCALE;
  d.gz = toInt16(g[4], g[5]) * GYRO_SCALE;

  d.ax = toInt16(a[0], a[1]) * ACC_SCALE;
  d.ay = toInt16(a[2], a[3]) * ACC_SCALE;
  d.az = toInt16(a[4], a[5]) * ACC_SCALE;

  return true;
}

}  // namespace imu_bno055


class Bno055Node : public rclcpp::Node
{
public:
  Bno055Node()
  : Node("bno055_node"),
    imu_("/dev/i2c-1", 0x29)
  {
    imu_.configure();

    RCLCPP_INFO(
      get_logger(),
      "BNO055 CHIP_ID: 0x%02X",
      imu_.getChipId());

    RCLCPP_INFO(
      get_logger(),
      "BNO055 MODE: 0x%02X",
      imu_.getMode());

    pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data",
      10);

    timer_ = create_wall_timer(
      20ms,
      std::bind(&Bno055Node::publishImu, this));

    RCLCPP_INFO(
      get_logger(),
      "BNO055 started");
  }

private:
  void publishImu()
  {
    try {
      imu_bno055::ImuData d;

      if (!imu_.readImu(d))
        return;

      sensor_msgs::msg::Imu msg;

      msg.header.stamp = now();
      msg.header.frame_id = "imu_link";

      // Orientation
      msg.orientation.w = d.qw;
      msg.orientation.x = d.qx;
      msg.orientation.y = d.qy;
      msg.orientation.z = d.qz;

      // Angular velocity [rad/s]
      msg.angular_velocity.x = d.gx;
      msg.angular_velocity.y = d.gy;
      msg.angular_velocity.z = d.gz;

      // Linear acceleration [m/s^2]
      msg.linear_acceleration.x = d.ax;
      msg.linear_acceleration.y = d.ay;
      msg.linear_acceleration.z = d.az;

      pub_->publish(msg);
    }
    catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "BNO055 read failed: %s",
        e.what());
    }
  }

  imu_bno055::Bno055 imu_;

  rclcpp::Publisher<
    sensor_msgs::msg::Imu>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<Bno055Node>());
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("bno055_node"),
      "%s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}