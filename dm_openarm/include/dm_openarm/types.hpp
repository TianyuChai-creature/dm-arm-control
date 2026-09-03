#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace dm_openarm {

enum class MotorModel {
  DM4310,
  DM8009,
};

enum class ControlMode {
  MIT,
};

struct MitCommand {
  double kp{0.0};
  double kd{0.0};
  double q{0.0};
  double dq{0.0};
  double tau{0.0};
};

struct MotorState {
  std::uint16_t can_id{0};
  std::uint16_t mst_id{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double feedback_interval_s{0.0};
  double last_rx_age_s{std::numeric_limits<double>::infinity()};
  std::uint64_t rx_sequence{0};
  std::uint8_t error_code{0};

  bool feedback_fresh(double max_age_s = 0.1) const noexcept
  {
    return max_age_s > 0.0 && rx_sequence > 0 && std::isfinite(last_rx_age_s) &&
           last_rx_age_s <= max_age_s;
  }
};

}  // namespace dm_openarm
