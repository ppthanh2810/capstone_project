#include "stage4_gazebo/zlac_driver_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stage4_gazebo
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
// Bỏ qua bước thời gian lớn bất thường (pause, reset world) thay vì ramp một bước khổng lồ.
constexpr double kMaxStep = 0.5;
}  // namespace

bool parseRampMode(const std::string & text, RampMode & mode)
{
  if (text == "time_to_target") {
    mode = RampMode::TimeToTarget;
    return true;
  }
  if (text == "rate") {
    mode = RampMode::Rate;
    return true;
  }
  return false;
}

std::string validateParams(const ZlacModelParams & p)
{
  if (!(p.accel_time_s > 0.0) || !(p.decel_time_s > 0.0)) {
    return "accel/decel time must be > 0";
  }
  if (!(p.ramp_reference_rpm > 0.0)) {
    return "ramp_reference_rpm must be > 0";
  }
  if (!(p.command_delay_s >= 0.0) || p.command_delay_s > 1.0) {
    return "command_delay must be in [0, 1] s";
  }
  if (!(p.speed_loop_tau_s >= 0.0) || p.speed_loop_tau_s > 1.0) {
    return "speed_loop_tau must be in [0, 1] s";
  }
  if (p.max_rpm <= 0) {
    return "max_rpm must be > 0";
  }
  return "";
}

// ============================================================
// ZlacMotorModel
// ============================================================

ZlacMotorModel::ZlacMotorModel(const ZlacModelParams & params)
: p_(params) {}

void ZlacMotorModel::setTarget(double rpm)
{
  if (rpm == target_) {
    return;  // lệnh lặp lại không khởi động lại ramp
  }
  target_ = rpm;

  // Tăng tốc khi độ lớn tăng cùng chiều (hoặc đi từ 0); còn lại (giảm, đổi chiều) dùng decel.
  const bool accelerating =
    std::abs(target_) > std::abs(ramped_) && (ramped_ == 0.0 || (target_ > 0.0) == (ramped_ > 0.0));
  const double time_s = accelerating ? p_.accel_time_s : p_.decel_time_s;

  if (p_.ramp_mode == RampMode::TimeToTarget) {
    rate_ = std::abs(target_ - ramped_) / time_s;
  } else {
    rate_ = p_.ramp_reference_rpm / time_s;
  }
}

double ZlacMotorModel::update(double dt)
{
  if (dt <= 0.0) {
    return output_;
  }
  const double max_change = rate_ * dt;
  const double diff = target_ - ramped_;
  ramped_ = std::abs(diff) <= max_change ? target_ : ramped_ + std::copysign(max_change, diff);

  if (p_.speed_loop_tau_s > 0.0) {
    output_ += (ramped_ - output_) * dt / (p_.speed_loop_tau_s + dt);
  } else {
    output_ = ramped_;
  }
  return output_;
}

void ZlacMotorModel::reset()
{
  target_ = ramped_ = output_ = rate_ = 0.0;
}

// ============================================================
// ZlacDriverModel
// ============================================================

ZlacDriverModel::ZlacDriverModel(const ZlacModelParams & params)
: p_(params), left_(params), right_(params) {}

void ZlacDriverModel::setRpm(double now_s, int left_raw, int right_raw)
{
  left_raw = std::clamp(left_raw, -p_.max_rpm, p_.max_rpm);
  right_raw = std::clamp(right_raw, -p_.max_rpm, p_.max_rpm);
  queue_.push_back({now_s + p_.command_delay_s, left_raw, right_raw});
}

std::pair<double, double> ZlacDriverModel::update(double now_s)
{
  if (!has_time_ || now_s < last_time_) {
    // lần đầu hoặc sim time lùi (reset world): bắt đầu lại từ thời điểm này
    last_time_ = now_s;
    has_time_ = true;
  }

  // Áp các lệnh đã hết trễ theo đúng thứ tự thời gian, ramp từng đoạn giữa chúng.
  while (!queue_.empty() && queue_.front().apply_time <= now_s) {
    const PendingCommand cmd = queue_.front();
    queue_.pop_front();
    const double t = std::max(cmd.apply_time, last_time_);
    const double dt = std::min(t - last_time_, kMaxStep);
    left_.update(dt);
    right_.update(dt);
    last_time_ = t;
    left_.setTarget(cmd.left);
    right_.setTarget(cmd.right);
  }

  const double dt = std::min(now_s - last_time_, kMaxStep);
  last_time_ = now_s;
  return {left_.update(dt), right_.update(dt)};
}

void ZlacDriverModel::reset()
{
  left_.reset();
  right_.reset();
  queue_.clear();
  has_time_ = false;
}

// ============================================================
// Feedback
// ============================================================

double feedbackRpm(double joint_velocity_rad_s)
{
  const double rpm = joint_velocity_rad_s * 30.0 / kPi;
  // thanh ghi int16, đơn vị 0.1 rpm -> getRpm() chia 10
  const double reg = std::clamp(
    std::round(rpm * 10.0),
    static_cast<double>(std::numeric_limits<int16_t>::min()),
    static_cast<double>(std::numeric_limits<int16_t>::max()));
  return reg / 10.0;
}

int32_t encoderTicks(double joint_position_rad)
{
  const double ticks = std::floor(joint_position_rad / (2.0 * kPi) * kEncoderCpr);
  // int32 của driver tràn sau ~5e5 vòng; ở đây chỉ kẹp để tránh UB
  return static_cast<int32_t>(std::clamp(
    ticks,
    static_cast<double>(std::numeric_limits<int32_t>::min()),
    static_cast<double>(std::numeric_limits<int32_t>::max())));
}

double ticksToTravelled(int32_t ticks)
{
  return static_cast<double>(ticks) / kEncoderCpr * kTravelInOneRev;
}

double rpmToRadPerSec(double rpm)
{
  return rpm * kPi / 30.0;
}

}  // namespace stage4_gazebo
