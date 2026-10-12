#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "stage2_slam/localization_health.hpp"

using stage2_slam::evaluateHealth;
using stage2_slam::HealthConfig;
using stage2_slam::HealthInputs;
using stage2_slam::HealthMemory;
using stage2_slam::HealthState;

namespace
{

HealthInputs good()
{
  HealthInputs in;
  in.now = 100.0;
  in.amcl_active = true;
  in.map_received = true;
  in.amcl_pose_received = true;
  in.scan_stamp = 99.95;
  in.odom_stamp = 99.98;
  in.tf_stamp = 99.9;
  in.std_x = 0.05;
  in.std_y = 0.05;
  in.std_yaw = 0.03;
  in.scan_consistency = 0.85;
  return in;
}

bool hasReason(const stage2_slam::HealthReport & r, const std::string & s)
{
  for (const auto & x : r.reasons) {
    if (x.find(s) != std::string::npos) {return true;}
  }
  return false;
}

}  // namespace

TEST(Health, AllGoodIsLocalized)
{
  HealthMemory mem;
  const auto r = evaluateHealth(good(), HealthConfig{}, mem);
  EXPECT_EQ(r.state, HealthState::LOCALIZED);
}

TEST(Health, InitializingStates)
{
  HealthConfig cfg;
  HealthMemory mem;
  auto in = good();
  in.amcl_active = false;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::INITIALIZING);
  in = good();
  in.amcl_active.reset();
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::INITIALIZING);
  in = good();
  in.amcl_pose_received = false;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::INITIALIZING);
  in = good();
  in.last_initialpose = 99.0;  // trong grace 3 s
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::INITIALIZING);
  in.last_initialpose = 90.0;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOCALIZED);
  in = good();
  in.map_received = false;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::INITIALIZING);
}

TEST(Health, LowCovarianceAloneIsNotLocalized)
{
  // Covariance rất nhỏ nhưng scan không khớp map / không có consistency -> không được LOCALIZED.
  HealthMemory mem;
  auto in = good();
  in.std_x = in.std_y = 0.001;
  in.std_yaw = 0.001;
  in.scan_consistency.reset();
  auto r = evaluateHealth(in, HealthConfig{}, mem);
  EXPECT_EQ(r.state, HealthState::UNCERTAIN);
  EXPECT_TRUE(hasReason(r, "consistency unavailable"));

  in.scan_consistency = 0.1;
  HealthConfig cfg;
  for (int i = 0; i < cfg.consistency_lost_count - 1; ++i) {
    EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::UNCERTAIN);
  }
  r = evaluateHealth(in, cfg, mem);
  EXPECT_EQ(r.state, HealthState::LOST);
  EXPECT_TRUE(hasReason(r, "scan-to-map consistency"));
  // Phục hồi khi consistency tốt trở lại
  in.scan_consistency = 0.9;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOCALIZED);
  EXPECT_EQ(mem.low_consistency_count, 0);
}

TEST(Health, StaleDataAndTf)
{
  HealthConfig cfg;
  HealthMemory mem;
  auto in = good();
  in.scan_stamp = 99.0;  // 1 s > warn 0.5
  auto r = evaluateHealth(in, cfg, mem);
  EXPECT_EQ(r.state, HealthState::UNCERTAIN);
  EXPECT_TRUE(hasReason(r, "scan age"));
  in.scan_stamp = 97.0;  // 3 s > lost 2
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOST);
  in = good();
  in.odom_stamp.reset();
  r = evaluateHealth(in, cfg, mem);
  EXPECT_EQ(r.state, HealthState::LOST);
  EXPECT_TRUE(hasReason(r, "no odometry"));
  in = good();
  in.tf_stamp.reset();
  r = evaluateHealth(in, cfg, mem);
  EXPECT_EQ(r.state, HealthState::LOST);
  EXPECT_TRUE(hasReason(r, "TF map->base unavailable"));
  in = good();
  in.tf_stamp = 99.0;  // TF trễ 1 s
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::UNCERTAIN);
}

TEST(Health, CovarianceThresholds)
{
  HealthConfig cfg;
  HealthMemory mem;
  auto in = good();
  in.std_x = 0.4;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::UNCERTAIN);
  in.std_x = 2.0;
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOST);
  in = good();
  in.std_yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluateHealth(in, cfg, mem).state, HealthState::LOST);
}

TEST(Health, PoseJumpIsUncertain)
{
  HealthMemory mem;
  auto in = good();
  in.jump_linear = 0.5;
  in.jump_angular = 0.0;
  auto r = evaluateHealth(in, HealthConfig{}, mem);
  EXPECT_EQ(r.state, HealthState::UNCERTAIN);
  EXPECT_TRUE(hasReason(r, "pose jump"));
}
