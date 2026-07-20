#pragma once

#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/gravity_model.hpp"
#include "dm_openarm/types.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace dm_openarm {

class MitLoopController {
public:
  explicit MitLoopController(DmArm& arm);
  ~MitLoopController();

  MitLoopController(const MitLoopController&) = delete;
  MitLoopController& operator=(const MitLoopController&) = delete;

  void start(double hz = 1000.0);
  void stop();
  bool running() const noexcept;

  void set_command(std::uint16_t can_id, MitCommand command);
  void set_all_commands(std::vector<MitCommand> commands);
  std::vector<MitCommand> commands() const;

  void set_gravity_enabled(bool enabled);
  bool gravity_enabled() const noexcept;
  void set_gravity_scale(double scale);
  double gravity_scale() const noexcept;
  void set_gravity_use_measured_q(bool use_measured);
  bool gravity_use_measured_q() const noexcept;

  /// Gravity torques for given q (motor order). Uses current scale.
  std::vector<double> gravity_torques(const std::vector<double>& q) const;

private:
  std::size_t motor_index(std::uint16_t can_id) const;
  void worker_loop(double hz);
  std::vector<MitCommand> apply_gravity(std::vector<MitCommand> commands) const;

  DmArm& arm_;
  std::vector<std::uint16_t> can_ids_;
  GravityModel gravity_model_;
  mutable std::mutex mutex_;
  std::vector<MitCommand> commands_;
  std::atomic<bool> running_{false};
  std::atomic<bool> gravity_enabled_{false};
  std::atomic<double> gravity_scale_{1.0};
  std::atomic<bool> gravity_use_measured_q_{true};
  std::thread worker_;
  std::exception_ptr worker_exception_{nullptr};
};

}  // namespace dm_openarm
