// Unit test mô hình driver ZLAC8015D (không cần ROS / Gazebo).

#include <gtest/gtest.h>

#include <cmath>

#include "stage4_gazebo/zlac_driver_model.hpp"

using stage4_gazebo::RampMode;
using stage4_gazebo::ZlacDriverModel;
using stage4_gazebo::ZlacModelParams;

namespace
{
constexpr double kPi = 3.14159265358979323846;

// Chạy model với bước dt tới thời điểm t_end, trả về đầu ra bánh trái.
double runUntil(ZlacDriverModel & m, double & t, double t_end, double dt = 0.005)
{
  double out = 0.0;
  while (t < t_end - 1e-9) {
    t += dt;
    out = m.update(t).first;
  }
  return out;
}
}  // namespace

TEST(ZlacDriverModel, TimeToTargetReachesStepInAccelTime)
{
  ZlacModelParams p;  // 500 ms, time_to_target
  ZlacDriverModel m(p);
  double t = 0.0;
  m.update(t);
  m.setRpm(t, 54, -54);
  EXPECT_NEAR(runUntil(m, t, 0.25), 27.0, 0.6);   // nửa thời gian -> nửa đường
  EXPECT_NEAR(runUntil(m, t, 0.50), 54.0, 1e-9);
  EXPECT_NEAR(m.right().output(), -54.0, 1e-9);
}

TEST(ZlacDriverModel, DecelUsesDecelTime)
{
  ZlacModelParams p;
  p.accel_time_s = 0.2;
  p.decel_time_s = 1.0;
  ZlacDriverModel m(p);
  double t = 0.0;
  m.update(t);
  m.setRpm(t, 50, 50);
  runUntil(m, t, 0.3);
  m.setRpm(t, 0, 0);
  EXPECT_NEAR(runUntil(m, t, 0.8), 25.0, 0.6);
  EXPECT_NEAR(runUntil(m, t, 1.3), 0.0, 1e-9);
}

TEST(ZlacDriverModel, RateModeUsesFixedSlope)
{
  ZlacModelParams p;
  p.ramp_mode = RampMode::Rate;
  p.ramp_reference_rpm = 3000.0;  // 3000 rpm / 0.5 s = 6000 rpm/s
  ZlacDriverModel m(p);
  double t = 0.0;
  m.update(t);
  m.setRpm(t, 600, 600);
  EXPECT_NEAR(runUntil(m, t, 0.05), 300.0, 1.0);
  EXPECT_NEAR(runUntil(m, t, 0.2), 600.0, 1e-9);
}

TEST(ZlacDriverModel, CommandDelayHoldsOutput)
{
  ZlacModelParams p;
  p.command_delay_s = 0.1;
  p.accel_time_s = 0.05;
  ZlacDriverModel m(p);
  double t = 0.0;
  m.update(t);
  m.setRpm(t, 30, 30);
  EXPECT_NEAR(runUntil(m, t, 0.095), 0.0, 1e-9);
  EXPECT_NEAR(runUntil(m, t, 0.2), 30.0, 1e-9);
}

TEST(ZlacDriverModel, ClampsToMaxRpm)
{
  ZlacModelParams p;
  p.accel_time_s = 0.05;
  ZlacDriverModel m(p);
  double t = 0.0;
  m.update(t);
  m.setRpm(t, 5000, -5000);
  runUntil(m, t, 0.1);
  EXPECT_DOUBLE_EQ(m.left().output(), 3000.0);
  EXPECT_DOUBLE_EQ(m.right().output(), -3000.0);
}

TEST(ZlacDriverModel, RepeatedCommandDoesNotRestartRamp)
{
  // DWB / twist_mux gửi lại cùng lệnh mỗi chu kỳ: ramp không được kéo dài vô hạn
  ZlacModelParams p;
  ZlacDriverModel m(p);
  double t = 0.0;
  m.update(t);
  for (int i = 0; i < 10; ++i) {
    m.setRpm(t, 40, 40);
    runUntil(m, t, t + 0.1);
  }
  EXPECT_NEAR(m.left().output(), 40.0, 1e-9);
}

TEST(ZlacDriverModel, SimTimeJumpBackResets)
{
  ZlacModelParams p;
  ZlacDriverModel m(p);
  m.update(10.0);
  m.setRpm(10.0, 20, 20);
  m.update(9.0);  // không được ramp với dt âm
  EXPECT_NEAR(m.left().output(), 0.0, 1e-9);
}

TEST(ZlacFeedback, RpmQuantizedToTenth)
{
  const double w = 15.04 * kPi / 30.0;
  EXPECT_DOUBLE_EQ(stage4_gazebo::feedbackRpm(w), 15.0);
  EXPECT_DOUBLE_EQ(stage4_gazebo::feedbackRpm(-w), -15.0);
}

TEST(ZlacFeedback, OneRevolutionIs4096TicksAnd0336m)
{
  const int32_t ticks = stage4_gazebo::encoderTicks(2.0 * kPi + 1e-9);
  EXPECT_EQ(ticks, 4096);
  EXPECT_NEAR(stage4_gazebo::ticksToTravelled(ticks), 0.336, 1e-12);
  EXPECT_EQ(stage4_gazebo::encoderTicks(-2.0 * kPi - 1e-9), -4097);  // floor như bộ đếm
}

TEST(ZlacFeedback, InvalidParamsRejected)
{
  ZlacModelParams p;
  p.accel_time_s = 0.0;
  EXPECT_FALSE(stage4_gazebo::validateParams(p).empty());
  p = ZlacModelParams{};
  p.command_delay_s = -0.1;
  EXPECT_FALSE(stage4_gazebo::validateParams(p).empty());
  EXPECT_TRUE(stage4_gazebo::validateParams(ZlacModelParams{}).empty());
}
