#pragma once

#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/gravity_model.hpp"
#include "dm_openarm/types.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <mutex>
#include <string>
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

  /// Legacy: toggles **left** gravity only (backward compatible single-arm).
  void set_gravity_enabled(bool enabled);
  bool gravity_enabled() const noexcept;
  void set_gravity_scale(double scale);
  double gravity_scale() const noexcept;
  void set_gravity_use_measured_q(bool use_measured);
  bool gravity_use_measured_q() const noexcept;

  /// Per-limb gravity. *side* is "left" or "right".
  void set_limb_gravity_enabled(const std::string& side, bool enabled);
  bool limb_gravity_enabled(const std::string& side) const;
  void set_limb_gravity_scale(const std::string& side, double scale);
  double limb_gravity_scale(const std::string& side) const;
  void set_limb_gravity_use_measured_q(const std::string& side, bool use_measured);
  bool limb_gravity_use_measured_q(const std::string& side) const;

  /// Gravity torques for one limb given limb-local q. Uses that limb's scale.
  std::vector<double> limb_gravity_torques(
    const std::string& side, const std::vector<double>& q) const;

  /// Legacy full-vector API: left limb only (q length must match left.count).
  std::vector<double> gravity_torques(const std::vector<double>& q) const;

  std::size_t left_begin() const noexcept { return left_begin_; }
  std::size_t left_count() const noexcept { return left_count_; }
  std::size_t right_begin() const noexcept { return right_begin_; }
  std::size_t right_count() const noexcept { return right_count_; }

private:
  struct LimbRuntime {
    std::size_t begin{0};
    std::size_t count{0};
    GravityModel model;
    bool has_model{false};
    std::atomic<bool> enabled{false};
    std::atomic<double> scale{1.0};
    std::atomic<bool> use_measured_q{true};
  };

  std::size_t motor_index(std::uint16_t can_id) const;
  LimbRuntime& limb_mut(const std::string& side);
  const LimbRuntime& limb_ref(const std::string& side) const;
  void worker_loop(double hz);
  std::vector<MitCommand> apply_gravity(std::vector<MitCommand> commands) const;
  void apply_limb_gravity(
    LimbRuntime const& limb,
    std::vector<MitCommand>& commands,
    const std::vector<double>* measured_q_full) const;

  DmArm& arm_;
  std::vector<std::uint16_t> can_ids_;
  LimbRuntime left_;
  LimbRuntime right_;
  std::size_t left_begin_{0};
  std::size_t left_count_{0};
  std::size_t right_begin_{0};
  std::size_t right_count_{0};
  mutable std::mutex mutex_;
  std::vector<MitCommand> commands_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::exception_ptr worker_exception_{nullptr};
};

}  // namespace dm_openarm
