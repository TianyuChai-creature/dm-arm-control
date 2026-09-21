#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace dm_openarm {

enum class MotorModel {
  DM4310,
  DM8009,
  DM4340P,
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

// Feedback status 0x1 means enabled, not a fault; preserve all other codes.
inline std::uint8_t motor_error_code(std::uint8_t status) noexcept
{
  return status == 1 ? 0 : status;
}

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
  double feedback_hz{0.0};

  bool feedback_fresh(double max_age_s = 0.1) const noexcept
  {
    return max_age_s > 0.0 && rx_sequence > 0 && std::isfinite(last_rx_age_s) &&
           last_rx_age_s <= max_age_s;
  }
};

struct MotorTimingStats {
  std::uint16_t can_id{0};
  double rx_hz{0.0};
  double rx_interval_s{0.0};
  double last_rx_age_s{std::numeric_limits<double>::infinity()};
  std::uint64_t rx_frames{0};
};

struct TimingStats {
  bool connected{false};
  bool running{false};
  double window_s{1.0};
  double target_tx_hz{0.0};
  double actual_tx_hz{0.0};
  std::uint64_t tx_cycles{0};
  std::uint64_t deadline_misses{0};
  std::vector<MotorTimingStats> motors;
};

}  // namespace dm_openarm
