#pragma once

#include "dm_openarm/gravity_model.hpp"
#include "dm_openarm/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace dm_openarm {

struct MotorConfig {
  std::string name;
  MotorModel model{MotorModel::DM4310};
  ControlMode mode{ControlMode::MIT};
  std::uint16_t can_id{0};
  std::uint16_t mst_id{0};
};

struct GravityConfig {
  bool enabled{false};
  double scale{1.0};
  /// If true, g(q) uses measured joint positions; else uses MIT command q.
  bool use_measured_q{true};
  /// Coupled gravity only: basis + weights[n_limb_joints][K].
  CoupledGravityParam coupled;
};

/// One side of a dual-arm bus: contiguous slice into ArmConfig::motors.
struct LimbSpec {
  std::size_t begin{0};
  std::size_t count{0};
  GravityConfig gravity;

  bool present() const noexcept { return count > 0; }
};

struct ArmConfig {
  std::string usb_serial;
  std::uint32_t nom_baud{1000000};
  std::uint32_t dat_baud{1000000};
  /// Classic CAN when false (station default). Matches resources/u2canfd.
  bool canfd{false};
  bool brs{false};
  int device_index{0};
  std::chrono::milliseconds loop_period{1};
  /// Flattened motor table: left motors then right motors.
  std::vector<MotorConfig> motors;
  LimbSpec left;
  LimbSpec right;

  /// Convenience: true if both limbs present.
  bool is_dual() const noexcept { return left.present() && right.present(); }
};

}  // namespace dm_openarm
