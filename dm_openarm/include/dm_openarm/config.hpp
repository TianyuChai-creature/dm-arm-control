#pragma once

#include "dm_openarm/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace dm_openarm {

inline constexpr double kDefaultControlHz = 1000.0;

struct MotorConfig {
  std::string name;
  MotorModel model{MotorModel::DM4310};
  ControlMode mode{ControlMode::MIT};
  std::uint16_t can_id{0};
  std::uint16_t mst_id{0};
};

/// One side of a dual-arm bus: contiguous slice into ArmConfig::motors.
struct LimbSpec {
  std::size_t begin{0};
  std::size_t count{0};
};

struct ArmConfig {
  std::string usb_serial;
  std::uint32_t nom_baud{1000000};
  std::uint32_t dat_baud{1000000};
  /// Classic CAN when false (station default). Matches resources/u2canfd.
  bool canfd{false};
  bool brs{false};
  int device_index{0};
  /// Flattened motor table: left motors then right motors.
  std::vector<MotorConfig> motors;
  // Full registration table for disjoint owners sharing one USB transport.
  std::vector<MotorConfig> bus_motors;
  LimbSpec left;
  LimbSpec right;

};

}  // namespace dm_openarm
