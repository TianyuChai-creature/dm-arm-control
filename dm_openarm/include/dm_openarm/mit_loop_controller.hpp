#pragma once

#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/command_guard.hpp"
#include "dm_openarm/types.hpp"
#include "unit/sliding_rate.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <map>
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

  void start(double hz = kDefaultControlHz);
  void enable_seeded(std::vector<MitCommand> gains, double hz, double command_timeout, double feedback_timeout);
  void start_seeded(std::vector<MitCommand> gains, double hz, double command_timeout, double feedback_timeout);
  void hold(bool reset_fault = false);
  std::vector<MitCommand> hold_command();
  std::string safety_state() const;
  std::string fault() const;
  std::uint64_t accepted_sequence() const;
  std::uint64_t sent_sequence() const;
  void stop();
  bool running() const noexcept;
  std::uint64_t deadline_misses() const noexcept;
  TimingStats timing_stats() const;

  void set_command(std::uint16_t can_id, MitCommand command);
  void set_commands(const std::map<std::uint16_t, MitCommand>& commands);
  void set_all_commands(std::vector<MitCommand> commands);
  std::vector<MitCommand> commands() const;
  std::vector<MitCommand> sent_commands() const;

private:
  std::size_t motor_index(std::uint16_t can_id) const;
  void worker_loop(double hz);
  void wait_for_hold(std::unique_lock<std::mutex>& lock, std::uint64_t request);

  CommandGuard guard_;
  DmArm& arm_;
  std::vector<std::uint16_t> can_ids_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::uint64_t hold_request_{0}, hold_sent_{0};
  std::chrono::steady_clock::time_point hold_deadline_;
  std::mutex stop_mutex_;
  std::vector<MitCommand> commands_;
  std::vector<MitCommand> sent_commands_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> deadline_misses_{0};
  double target_tx_hz_{0.0};
  std::uint64_t tx_cycles_{0};
  damiao::detail::SlidingRate tx_rate_;
  std::thread worker_;
  std::exception_ptr worker_exception_{nullptr};
};

}  // namespace dm_openarm
