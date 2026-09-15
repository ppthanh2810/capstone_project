#include "zlac8015d_hw/zlac8015d.hpp"

#include <algorithm>
#include <cerrno>
#include <iostream>
#include <stdexcept>

#include <modbus/modbus.h>

namespace zlac8015d_hw
{

Controller::Controller(const std::string & port, int baudrate, int slave_id)
: ctx_(modbus_new_rtu(port.c_str(), baudrate, 'N', 8, 1))
{
  if (!ctx_) throw std::runtime_error("Cannot create Modbus context");

  if (modbus_set_slave(ctx_, slave_id) == -1)
    throw std::runtime_error("Cannot set slave ID");

  modbus_set_response_timeout(ctx_, 0, 20000);

  if (modbus_connect(ctx_) == -1)
    throw std::runtime_error(
      "Cannot connect ZLAC: " + std::string(modbus_strerror(errno)));

  std::cout << "[ZLAC] Connected: " << port << std::endl;
}


Controller::~Controller()
{
  if (ctx_) {
    modbus_close(ctx_);
    modbus_free(ctx_);
  }
}


void Controller::setMode(int mode)
{
  if (mode < 1 || mode > 3)
    throw std::invalid_argument("Mode must be 1, 2 or 3");

  writeRegister(OPR_MODE, mode);
}


void Controller::enableMotor()
{
  writeRegister(CONTROL_REG, ENABLE);
}


void Controller::disableMotor()
{
  writeRegister(CONTROL_REG, DOWN_TIME);
}


void Controller::setAccelTime(int left_ms, int right_ms)
{
  left_ms  = std::clamp(left_ms, 0, 32767);
  right_ms = std::clamp(right_ms, 0, 32767);

  writeTwoRegisters(L_ACL_TIME, left_ms, right_ms);
}


void Controller::setDecelTime(int left_ms, int right_ms)
{
  left_ms  = std::clamp(left_ms, 0, 32767);
  right_ms = std::clamp(right_ms, 0, 32767);

  writeTwoRegisters(L_DCL_TIME, left_ms, right_ms);
}


void Controller::setRpm(int left_rpm, int right_rpm)
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
  uint16_t reg[2];

  if (modbus_read_registers(ctx_, L_FB_RPM, 2, reg) != 2)
    throw std::runtime_error(
      "Read RPM failed: " + std::string(modbus_strerror(errno)));

  return {
    static_cast<double>(toSigned(reg[0])) / 10.0,
    static_cast<double>(toSigned(reg[1])) / 10.0
  };
}


void Controller::writeRegister(uint16_t address, uint16_t value)
{
  if (modbus_write_register(ctx_, address, value) != 1)
    std::cerr << "[ZLAC] Write warning: "
              << modbus_strerror(errno) << std::endl;
}


void Controller::writeTwoRegisters(
  uint16_t address,
  uint16_t first,
  uint16_t second)
{
  uint16_t values[2] = {first, second};

  if (modbus_write_registers(ctx_, address, 2, values) != 2)
    std::cerr << "[ZLAC] Write warning: "
              << modbus_strerror(errno) << std::endl;
}


uint16_t Controller::toRegister(int value)
{
  return static_cast<uint16_t>(static_cast<int16_t>(value));
}


int16_t Controller::toSigned(uint16_t value)
{
  return static_cast<int16_t>(value);
}

}  // namespace zlac8015d_hw