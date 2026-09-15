#include "zlac8015d_hw/zlac8015d.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <modbus/modbus.h>

namespace zlac8015d_hw
{

Controller::Controller(
  const std::string & port,
  int baudrate,
  int slave_id)
: ctx_(modbus_new_rtu(port.c_str(), baudrate, 'N', 8, 1))
{
  if (!ctx_)
    throw std::runtime_error("Cannot create Modbus context");

  if (modbus_set_slave(ctx_, slave_id) == -1)
    throw std::runtime_error("Cannot set slave ID");

  // 20 ms response timeout
  modbus_set_response_timeout(ctx_, 0, 20000);

  if (modbus_connect(ctx_) == -1)
    throw std::runtime_error(
      "Cannot connect ZLAC: " +
      std::string(modbus_strerror(errno)));

  std::cout
    << "[ZLAC] Connected: "
    << port
    << std::endl;
}


Controller::~Controller()
{
  if (ctx_) {
    modbus_close(ctx_);
    modbus_free(ctx_);
  }
}


// ==========================================================
// Read helper
// ==========================================================

std::vector<uint16_t> Controller::readRegisters(
  uint16_t address,
  int words,
  int retries)
{
  std::vector<uint16_t> registers(words);

  for (int i = 0; i < retries; ++i) {
    int rc = modbus_read_registers(
      ctx_,
      address,
      words,
      registers.data());

    if (rc == words)
      return registers;
  }

  throw std::runtime_error(
    "Modbus read failed: " +
    std::string(modbus_strerror(errno)));
}


// ==========================================================
// Basic control
// ==========================================================

void Controller::setMode(int mode)
{
  if (mode < 1 || mode > 3)
    throw std::invalid_argument(
      "Mode must be 1, 2 or 3");

  writeRegister(
    OPR_MODE,
    static_cast<uint16_t>(mode));
}


int Controller::getMode()
{
  auto reg = readRegisters(OPR_MODE, 1);
  return reg[0];
}


void Controller::enableMotor()
{
  writeRegister(CONTROL_REG, ENABLE);
}


void Controller::disableMotor()
{
  writeRegister(CONTROL_REG, DOWN_TIME);
}


void Controller::clearAlarm()
{
  writeRegister(CONTROL_REG, ALRM_CLR);
}


// ==========================================================
// Fault
// ==========================================================

bool Controller::isFault(uint16_t code) const
{
  switch (code) {
    case OVER_VOLT:
    case UNDER_VOLT:
    case OVER_CURR:
    case OVER_LOAD:
    case CURR_OUT_TOL:
    case ENCOD_OUT_TOL:
    case MOTOR_BAD:
    case REF_VOLT_ERROR:
    case EEPROM_ERROR:
    case WALL_ERROR:
    case HIGH_TEMP:
      return true;

    default:
      return false;
  }
}


std::pair<
  std::pair<bool, uint16_t>,
  std::pair<bool, uint16_t>>
Controller::getFaultCode()
{
  auto reg = readRegisters(L_FAULT, 2);

  uint16_t left_fault = reg[0];
  uint16_t right_fault = reg[1];

  return {
    {isFault(left_fault), left_fault},
    {isFault(right_fault), right_fault}
  };
}


// ==========================================================
// Acceleration / Deceleration
// ==========================================================

void Controller::setAccelTime(
  int left_ms,
  int right_ms)
{
  left_ms  = std::clamp(left_ms, 0, 32767);
  right_ms = std::clamp(right_ms, 0, 32767);

  writeTwoRegisters(
    L_ACL_TIME,
    static_cast<uint16_t>(left_ms),
    static_cast<uint16_t>(right_ms));
}


void Controller::setDecelTime(
  int left_ms,
  int right_ms)
{
  left_ms  = std::clamp(left_ms, 0, 32767);
  right_ms = std::clamp(right_ms, 0, 32767);

  writeTwoRegisters(
    L_DCL_TIME,
    static_cast<uint16_t>(left_ms),
    static_cast<uint16_t>(right_ms));
}


// ==========================================================
// Velocity control
// ==========================================================

void Controller::setRpm(
  int left_rpm,
  int right_rpm)
{
  left_rpm  = std::clamp(left_rpm, -3000, 3000);
  right_rpm = std::clamp(right_rpm, -3000, 3000);

  writeTwoRegisters(
    L_CMD_RPM,
    toRegister(left_rpm),
    toRegister(right_rpm));
}


std::pair<double, double> Controller::getRpm()
{
  auto reg = readRegisters(L_FB_RPM, 2);

  double left_rpm =
    static_cast<double>(toSigned(reg[0])) / 10.0;

  double right_rpm =
    static_cast<double>(toSigned(reg[1])) / 10.0;

  return {
    left_rpm,
    right_rpm
  };
}


double Controller::rpmToRadPerSec(double rpm) const
{
  constexpr double PI =
    3.14159265358979323846;

  return rpm * 2.0 * PI / 60.0;
}


double Controller::rpmToLinear(double rpm) const
{
  return rpmToRadPerSec(rpm) * WHEEL_RADIUS;
}


std::pair<double, double>
Controller::getLinearVelocities()
{
  auto [left_rpm, right_rpm] = getRpm();

  double left =
    rpmToLinear(left_rpm);

  // Same sign convention as Python
  double right =
    rpmToLinear(-right_rpm);

  return {
    left,
    right
  };
}


// ==========================================================
// Position control
// ==========================================================

void Controller::setMaxRpmPos(
  int left_rpm,
  int right_rpm)
{
  left_rpm  = std::clamp(left_rpm, 1, 1000);
  right_rpm = std::clamp(right_rpm, 1, 1000);

  writeTwoRegisters(
    L_MAX_RPM_POS,
    static_cast<uint16_t>(left_rpm),
    static_cast<uint16_t>(right_rpm));
}


void Controller::setPositionAsyncControl()
{
  writeRegister(
    POS_CONTROL_TYPE,
    ASYNC);
}


void Controller::moveLeftWheel()
{
  writeRegister(
    CONTROL_REG,
    POS_L_START);
}


void Controller::moveRightWheel()
{
  writeRegister(
    CONTROL_REG,
    POS_R_START);
}


double Controller::mapValue(
  double value,
  double in_min,
  double in_max,
  double out_min,
  double out_max)
{
  return
    (value - in_min) *
    (out_max - out_min) /
    (in_max - in_min) +
    out_min;
}


std::array<uint16_t, 2>
Controller::degTo32bitArray(double deg)
{
  int32_t value = static_cast<int32_t>(
    mapValue(
      deg,
      -1440.0,
      1440.0,
      -65536.0,
      65536.0));

  uint16_t high =
    static_cast<uint16_t>(
      (static_cast<uint32_t>(value) >> 16) &
      0xFFFF);

  uint16_t low =
    static_cast<uint16_t>(
      static_cast<uint32_t>(value) &
      0xFFFF);

  return {
    high,
    low
  };
}


void Controller::setRelativeAngle(
  double left_deg,
  double right_deg)
{
  auto left =
    degTo32bitArray(left_deg);

  auto right =
    degTo32bitArray(right_deg);

  std::vector<uint16_t> values = {
    left[0],
    left[1],
    right[0],
    right[1]
  };

  writeRegisters(
    L_CMD_REL_POS_HI,
    values);
}


// ==========================================================
// Odometry / Encoder
// ==========================================================

int32_t Controller::combine32(
  uint16_t high,
  uint16_t low)
{
  uint32_t raw =
    (static_cast<uint32_t>(high) << 16) |
    static_cast<uint32_t>(low);

  return static_cast<int32_t>(raw);
}


std::pair<int32_t, int32_t>
Controller::getWheelsTick()
{
  auto reg =
    readRegisters(L_FB_POS_HI, 4);

  int32_t left_tick =
    combine32(reg[0], reg[1]);

  int32_t right_tick =
    combine32(reg[2], reg[3]);

  return {
    left_tick,
    right_tick
  };
}


std::pair<double, double>
Controller::getWheelsTravelled()
{
  auto [left_tick, right_tick] =
    getWheelsTick();

  double left_travelled =
    (static_cast<double>(left_tick) / CPR) *
    TRAVEL_IN_ONE_REV;

  double right_travelled =
    (static_cast<double>(right_tick) / CPR) *
    TRAVEL_IN_ONE_REV;

  return {
    left_travelled,
    right_travelled
  };
}


// ==========================================================
// Modbus write helpers
// ==========================================================

void Controller::writeRegister(
  uint16_t address,
  uint16_t value)
{
  if (modbus_write_register(
      ctx_,
      address,
      value) != 1)
  {
    std::cerr
      << "[ZLAC] Write warning: "
      << modbus_strerror(errno)
      << std::endl;
  }
}


void Controller::writeRegisters(
  uint16_t address,
  const std::vector<uint16_t> & values)
{
  int expected =
    static_cast<int>(values.size());

  if (modbus_write_registers(
      ctx_,
      address,
      expected,
      values.data()) != expected)
  {
    std::cerr
      << "[ZLAC] Write warning: "
      << modbus_strerror(errno)
      << std::endl;
  }
}


void Controller::writeTwoRegisters(
  uint16_t address,
  uint16_t first,
  uint16_t second)
{
  std::vector<uint16_t> values = {
    first,
    second
  };

  writeRegisters(
    address,
    values);
}


// ==========================================================
// Conversion
// ==========================================================

uint16_t Controller::toRegister(int value)
{
  return static_cast<uint16_t>(
    static_cast<int16_t>(value));
}


int16_t Controller::toSigned(uint16_t value)
{
  return static_cast<int16_t>(value);
}

}  // namespace zlac8015d_hw