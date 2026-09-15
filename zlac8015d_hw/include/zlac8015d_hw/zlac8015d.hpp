#pragma once

#include <cstdint>
#include <string>
#include <utility>

struct _modbus;

namespace zlac8015d_hw
{

class Controller
{
public:
  explicit Controller(
    const std::string & port = "/dev/ttyUSB0",
    int baudrate = 115200,
    int slave_id = 1);

  ~Controller();

  Controller(const Controller &) = delete;
  Controller & operator=(const Controller &) = delete;

  void setMode(int mode);

  void enableMotor();
  void disableMotor();

  void setAccelTime(int left_ms, int right_ms);
  void setDecelTime(int left_ms, int right_ms);

  void setRpm(int left_rpm, int right_rpm);

  std::pair<double, double> getRpm();

private:
  void writeRegister(uint16_t address, uint16_t value);
  void writeTwoRegisters(
    uint16_t address,
    uint16_t first,
    uint16_t second);

  static uint16_t toRegister(int value);
  static int16_t toSigned(uint16_t value);

  _modbus * ctx_;

  // ZLAC8015D registers
  static constexpr uint16_t CONTROL_REG = 0x200E;
  static constexpr uint16_t OPR_MODE    = 0x200D;

  static constexpr uint16_t L_ACL_TIME = 0x2080;
  static constexpr uint16_t L_DCL_TIME = 0x2082;

  static constexpr uint16_t L_CMD_RPM = 0x2088;
  static constexpr uint16_t L_FB_RPM  = 0x20AB;

  // Control commands
  static constexpr uint16_t DOWN_TIME = 0x07;
  static constexpr uint16_t ENABLE    = 0x08;
};

}  // namespace zlac8015d_hardware