#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

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

  // Common
  void setMode(int mode);
  int getMode();

  void enableMotor();
  void disableMotor();
  void clearAlarm();

  std::pair<
    std::pair<bool, uint16_t>,
    std::pair<bool, uint16_t>>
  getFaultCode();

  void setAccelTime(int left_ms, int right_ms);
  void setDecelTime(int left_ms, int right_ms);

  // Velocity
  void setRpm(int left_rpm, int right_rpm);
  std::pair<double, double> getRpm();
  std::pair<double, double> getLinearVelocities();

  // Position
  void setMaxRpmPos(int left_rpm, int right_rpm);
  void setPositionAsyncControl();

  void moveLeftWheel();
  void moveRightWheel();

  void setRelativeAngle(double left_deg, double right_deg);

  // Odometry
  std::pair<double, double> getWheelsTravelled();
  std::pair<int32_t, int32_t> getWheelsTick();

  // Conversion
  double rpmToRadPerSec(double rpm) const;
  double rpmToLinear(double rpm) const;

private:
  std::vector<uint16_t> readRegisters(
    uint16_t address,
    int words,
    int retries = 5);

  void writeRegister(
    uint16_t address,
    uint16_t value);

  void writeRegisters(
    uint16_t address,
    const std::vector<uint16_t> & values);

  void writeTwoRegisters(
    uint16_t address,
    uint16_t first,
    uint16_t second);

  static uint16_t toRegister(int value);
  static int16_t toSigned(uint16_t value);

  static double mapValue(
    double value,
    double in_min,
    double in_max,
    double out_min,
    double out_max);

  static std::array<uint16_t, 2> degTo32bitArray(double deg);

  static int32_t combine32(
    uint16_t high,
    uint16_t low);

  bool isFault(uint16_t code) const;

  _modbus * ctx_;

  // ======================================================
  // Register Address
  // ======================================================

  // Common
  static constexpr uint16_t CONTROL_REG = 0x200E;
  static constexpr uint16_t OPR_MODE    = 0x200D;

  static constexpr uint16_t L_ACL_TIME = 0x2080;
  static constexpr uint16_t R_ACL_TIME = 0x2081;
  static constexpr uint16_t L_DCL_TIME = 0x2082;
  static constexpr uint16_t R_DCL_TIME = 0x2083;

  // Velocity control
  static constexpr uint16_t L_CMD_RPM = 0x2088;
  static constexpr uint16_t R_CMD_RPM = 0x2089;

  static constexpr uint16_t L_FB_RPM = 0x20AB;
  static constexpr uint16_t R_FB_RPM = 0x20AC;

  // Position control
  static constexpr uint16_t POS_CONTROL_TYPE = 0x200F;

  static constexpr uint16_t L_MAX_RPM_POS = 0x208E;
  static constexpr uint16_t R_MAX_RPM_POS = 0x208F;

  static constexpr uint16_t L_CMD_REL_POS_HI = 0x208A;
  static constexpr uint16_t L_CMD_REL_POS_LO = 0x208B;
  static constexpr uint16_t R_CMD_REL_POS_HI = 0x208C;
  static constexpr uint16_t R_CMD_REL_POS_LO = 0x208D;

  static constexpr uint16_t L_FB_POS_HI = 0x20A7;
  static constexpr uint16_t L_FB_POS_LO = 0x20A8;
  static constexpr uint16_t R_FB_POS_HI = 0x20A9;
  static constexpr uint16_t R_FB_POS_LO = 0x20AA;

  // Fault
  static constexpr uint16_t L_FAULT = 0x20A5;
  static constexpr uint16_t R_FAULT = 0x20A6;

  // ======================================================
  // Control Commands
  // ======================================================

  static constexpr uint16_t EMER_STOP   = 0x05;
  static constexpr uint16_t ALRM_CLR    = 0x06;
  static constexpr uint16_t DOWN_TIME   = 0x07;
  static constexpr uint16_t ENABLE      = 0x08;

  static constexpr uint16_t POS_SYNC    = 0x10;
  static constexpr uint16_t POS_L_START = 0x11;
  static constexpr uint16_t POS_R_START = 0x12;

  // ======================================================
  // Operation Mode
  // ======================================================

  static constexpr uint16_t POS_REL_CONTROL = 1;
  static constexpr uint16_t POS_ABS_CONTROL = 2;
  static constexpr uint16_t VEL_CONTROL     = 3;

  static constexpr uint16_t ASYNC = 0;
  static constexpr uint16_t SYNC  = 1;

  // ======================================================
  // Fault Codes
  // ======================================================

  static constexpr uint16_t NO_FAULT       = 0x0000;
  static constexpr uint16_t OVER_VOLT      = 0x0001;
  static constexpr uint16_t UNDER_VOLT     = 0x0002;
  static constexpr uint16_t OVER_CURR      = 0x0004;
  static constexpr uint16_t OVER_LOAD      = 0x0008;
  static constexpr uint16_t CURR_OUT_TOL   = 0x0010;
  static constexpr uint16_t ENCOD_OUT_TOL  = 0x0020;
  static constexpr uint16_t MOTOR_BAD      = 0x0040;
  static constexpr uint16_t REF_VOLT_ERROR = 0x0080;
  static constexpr uint16_t EEPROM_ERROR   = 0x0100;
  static constexpr uint16_t WALL_ERROR     = 0x0200;
  static constexpr uint16_t HIGH_TEMP      = 0x0400;

  // ======================================================
  // Odometry
  // ======================================================

  static constexpr double TRAVEL_IN_ONE_REV = 0.336;
  static constexpr int CPR = 4096;
  static constexpr double WHEEL_RADIUS = 0.0535;
};

}  // namespace zlac8015d_hw