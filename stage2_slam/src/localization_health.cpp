#include "stage2_slam/localization_health.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace stage2_slam
{

const char * toString(HealthState s)
{
  switch (s) {
    case HealthState::INITIALIZING: return "INITIALIZING";
    case HealthState::LOCALIZED: return "LOCALIZED";
    case HealthState::UNCERTAIN: return "UNCERTAIN";
    case HealthState::LOST: return "LOST";
  }
  return "UNKNOWN";
}

namespace
{
std::string fmt(const char * what, double v, const char * op, double thr)
{
  std::ostringstream ss;
  ss.precision(3);
  ss << what << " " << v << " " << op << " " << thr;
  return ss.str();
}
}  // namespace

HealthReport evaluateHealth(const HealthInputs & in, const HealthConfig & cfg, HealthMemory & mem)
{
  HealthReport r;
  std::vector<std::string> init, lost, warn;

  // --- Điều kiện khởi tạo ---
  if (!in.map_received) {init.push_back("no map received");}
  if (!in.amcl_active.has_value()) {
    init.push_back("AMCL lifecycle state unknown");
  } else if (!*in.amcl_active) {
    init.push_back("AMCL is not active");
  }
  if (!in.amcl_pose_received) {init.push_back("no /amcl_pose yet (initial pose not set?)");}
  if (in.last_initialpose && in.now - *in.last_initialpose < cfg.initializing_grace) {
    init.push_back("within grace period after /initialpose");
  }

  // --- Tuổi dữ liệu ---
  if (!in.scan_stamp) {
    lost.push_back("no scan received");
  } else {
    const double age = in.now - *in.scan_stamp;
    if (age > cfg.scan_age_lost) {
      lost.push_back(fmt("scan age", age, ">", cfg.scan_age_lost));
    } else if (age > cfg.scan_age_warn) {
      warn.push_back(fmt("scan age", age, ">", cfg.scan_age_warn));
    }
  }
  if (!in.odom_stamp) {
    lost.push_back("no odometry received");
  } else {
    const double age = in.now - *in.odom_stamp;
    if (age > cfg.odom_age_lost) {
      lost.push_back(fmt("odom age", age, ">", cfg.odom_age_lost));
    } else if (age > cfg.odom_age_warn) {
      warn.push_back(fmt("odom age", age, ">", cfg.odom_age_warn));
    }
  }
  if (!in.tf_stamp) {
    lost.push_back("TF map->base unavailable");
  } else {
    const double age = in.now - *in.tf_stamp;
    if (age > cfg.tf_age_lost) {
      lost.push_back(fmt("TF map->base age", age, ">", cfg.tf_age_lost));
    } else if (age > cfg.tf_age_warn) {
      warn.push_back(fmt("TF map->base age", age, ">", cfg.tf_age_warn));
    }
  }

  // --- Covariance AMCL (chỉ là một tín hiệu) ---
  if (in.std_x && in.std_y) {
    const double sxy = std::max(*in.std_x, *in.std_y);
    if (!std::isfinite(sxy) || sxy > cfg.std_xy_lost) {
      lost.push_back(fmt("AMCL std_xy", sxy, ">", cfg.std_xy_lost));
    } else if (sxy > cfg.std_xy_warn) {
      warn.push_back(fmt("AMCL std_xy", sxy, ">", cfg.std_xy_warn));
    }
  }
  if (in.std_yaw) {
    if (!std::isfinite(*in.std_yaw) || *in.std_yaw > cfg.std_yaw_lost) {
      lost.push_back(fmt("AMCL std_yaw", *in.std_yaw, ">", cfg.std_yaw_lost));
    } else if (*in.std_yaw > cfg.std_yaw_warn) {
      warn.push_back(fmt("AMCL std_yaw", *in.std_yaw, ">", cfg.std_yaw_warn));
    }
  }

  // --- Scan-to-map consistency (bắt buộc cho LOCALIZED) ---
  if (!in.scan_consistency) {
    warn.push_back("scan-to-map consistency unavailable");
  } else {
    const double c = *in.scan_consistency;
    if (c < cfg.consistency_lost) {
      ++mem.low_consistency_count;
      if (mem.low_consistency_count >= cfg.consistency_lost_count) {
        lost.push_back(
          fmt("scan-to-map consistency", c, "<", cfg.consistency_lost) + " for " +
          std::to_string(mem.low_consistency_count) + " checks");
      } else {
        warn.push_back(fmt("scan-to-map consistency", c, "<", cfg.consistency_lost));
      }
    } else {
      mem.low_consistency_count = 0;
      if (c < cfg.consistency_warn) {
        warn.push_back(fmt("scan-to-map consistency", c, "<", cfg.consistency_warn));
      }
    }
  }

  // --- Nhảy pose (AMCL dịch khác odom) ---
  if (in.jump_linear && *in.jump_linear > cfg.jump_linear_warn) {
    warn.push_back(fmt("pose jump", *in.jump_linear, "m >", cfg.jump_linear_warn));
  }
  if (in.jump_angular && *in.jump_angular > cfg.jump_angular_warn) {
    warn.push_back(fmt("pose jump", *in.jump_angular, "rad >", cfg.jump_angular_warn));
  }

  if (!init.empty()) {
    r.state = HealthState::INITIALIZING;
    r.reasons = init;
    r.reasons.insert(r.reasons.end(), lost.begin(), lost.end());
    r.reasons.insert(r.reasons.end(), warn.begin(), warn.end());
  } else if (!lost.empty()) {
    r.state = HealthState::LOST;
    r.reasons = lost;
    r.reasons.insert(r.reasons.end(), warn.begin(), warn.end());
  } else if (!warn.empty()) {
    r.state = HealthState::UNCERTAIN;
    r.reasons = warn;
  } else {
    r.state = HealthState::LOCALIZED;
    r.reasons.push_back("all checks passed");
  }
  return r;
}

}  // namespace stage2_slam
