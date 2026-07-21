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
  GravityMode mode{GravityMode::Decoupled};
  double scale{1.0};
  /// If true, g(q) uses measured joint positions; else uses MIT command q.
  bool use_measured_q{true};
  /// Used when mode == Decoupled (and as optional fallback table).
  std::vector<JointGravityParam> joints;
  /// Used when mode == Coupled.
  CoupledGravityParam coupled;
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
  std::vector<MotorConfig> motors;
  GravityConfig gravity;
};

}  // namespace dm_openarm
