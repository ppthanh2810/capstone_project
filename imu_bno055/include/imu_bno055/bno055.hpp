#pragma once

#include <cstdint>
#include <string>

namespace imu_bno055
{

struct ImuData
{
  double qw, qx, qy, qz;
  double gx, gy, gz;
  double ax, ay, az;
};

class Bno055
{
public:
  Bno055(
    const std::string & device = "/dev/i2c-1",
    uint8_t address = 0x29);

  ~Bno055();

  void configure();
  bool readImu(ImuData & data);

  uint8_t getChipId();
  uint8_t getMode();

private:
  void writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);
  void readRegs(uint8_t reg, uint8_t * data, int length);

  void restoreCalibration();

  static int16_t toInt16(
    uint8_t lsb,
    uint8_t msb);

  int fd_;
  uint8_t address_;
};

}  // namespace imu_bno055