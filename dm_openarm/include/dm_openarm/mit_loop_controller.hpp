#pragma once

#include "dm_openarm/dm_arm.hpp"
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

  void start(double hz = kDefaultControlHz);
  void stop();
  bool running() const noexcept;
  std::uint64_t deadline_misses() const noexcept;

  void set_command(std::uint16_t can_id, MitCommand command);
  void set_all_commands(std::vector<MitCommand> commands);
  std::vector<MitCommand> commands() const;

private:
  std::size_t motor_index(std::uint16_t can_id) const;
  void worker_loop(double hz);

  DmArm& arm_;
  std::vector<std::uint16_t> can_ids_;
  mutable std::mutex mutex_;
  std::vector<MitCommand> commands_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> deadline_misses_{0};
  std::thread worker_;
  std::exception_ptr worker_exception_{nullptr};
};

}  // namespace dm_openarm
