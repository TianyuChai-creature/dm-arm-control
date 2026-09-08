#include "dm_openarm/mit_loop_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace dm_openarm {
namespace {
double monotonic_seconds() {
 return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}


MitLoopController::MitLoopController(DmArm& arm)
  : arm_(arm)
{
  const auto& cfg = arm_.config();
  const auto& motors = cfg.motors;
  can_ids_.reserve(motors.size());
  commands_.resize(motors.size());
  sent_commands_.resize(motors.size());
  for(const auto& motor : motors)
  {
    can_ids_.push_back(motor.can_id);
  }

}

MitLoopController::~MitLoopController()
{
  try
  {
    stop();
  }
  catch(...)
  {
  }
}

void MitLoopController::start(double hz)
{
  if(!std::isfinite(hz) || hz <= 0.0)
  {
    throw std::invalid_argument("MIT loop frequency must be positive");
  }
  if(running_)
  {
    return;
  }

  if(worker_.joinable())
  {
    worker_.join();
  }

  if(worker_exception_)
  {
    auto exception = worker_exception_;
    worker_exception_ = nullptr;
    std::rethrow_exception(exception);
  }

  deadline_misses_ = 0;
  running_ = true;
  worker_ = std::thread([this, hz]() { worker_loop(hz); });
}

void MitLoopController::enable_seeded(std::vector<MitCommand> gains, double hz,
                                          double command_timeout, double feedback_timeout) {
  if (!std::isfinite(hz) || hz <= 0 || running_) throw std::invalid_argument("invalid startup frequency/state");
  if (gains.size() != commands_.size()) throw std::invalid_argument("hold gain count mismatch");
  CommandGuard validation;
  validation.command_timeout = command_timeout; validation.feedback_timeout = feedback_timeout;
  validation.start(gains, monotonic_seconds()); // validate before energizing
  try {
    arm_.enable();
    start_seeded(std::move(gains), hz, command_timeout, feedback_timeout);
    const double deadline = monotonic_seconds() + 3.0;
    while (safety_state() == "STARTING" && monotonic_seconds() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (safety_state() != "HOLD") throw std::runtime_error("native startup failed: " + fault());
  } catch (...) {
    try { stop(); } catch (...) {}
    try { arm_.disable(); } catch (...) {}
    throw;
  }
}

void MitLoopController::start_seeded(std::vector<MitCommand> gains, double hz,
                                         double command_timeout, double feedback_timeout)
{
  if (running_) throw std::runtime_error("MIT loop already running");
  if (gains.size() != commands_.size()) throw std::invalid_argument("hold gain count mismatch");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    guard_.command_timeout = command_timeout; guard_.feedback_timeout = feedback_timeout;
    guard_.start(std::move(gains), monotonic_seconds());
  }
  start(hz);
}
std::vector<MitCommand> MitLoopController::hold_command() {
  auto states = arm_.states();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_) throw std::runtime_error("MIT loop not running");
  guard_.hold_command(states, sent_commands_, monotonic_seconds());
  return guard_.holding;
}

void MitLoopController::hold(bool reset_fault) {
  auto states = arm_.states();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || !guard_.enabled) throw std::runtime_error("guarded MIT loop not running");
  if (guard_.state == "STARTING") throw std::runtime_error("MIT startup incomplete");
  guard_.hold(states, sent_commands_, monotonic_seconds(), guard_.latched ? guard_.fault : "");
  if (reset_fault) { guard_.latched = false; guard_.fault.clear(); guard_.state = "HOLD"; }
}
std::string MitLoopController::safety_state() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.state; }
std::string MitLoopController::fault() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.fault; }
std::uint64_t MitLoopController::accepted_sequence() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.accepted_seq; }
std::uint64_t MitLoopController::sent_sequence() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.sent_seq; }

void MitLoopController::stop()
{
  const bool was_running = running_.exchange(false);
  if(worker_.joinable())
  {
    worker_.join();
  }

  std::exception_ptr stop_exception;
  if(was_running)
  {
    try
    {
      arm_.send_zero_mit_all();
    }
    catch(...)
    {
      stop_exception = std::current_exception();
      try
      {
        arm_.disable();
      }
      catch(...)
      {
      }
    }
  }

  if(worker_exception_)
  {
    auto exception = worker_exception_;
    worker_exception_ = nullptr;
    std::rethrow_exception(exception);
  }
  if(stop_exception)
  {
    std::rethrow_exception(stop_exception);
  }
}

bool MitLoopController::running() const noexcept
{
  return running_;
}

std::uint64_t MitLoopController::deadline_misses() const noexcept
{
  return deadline_misses_.load();
}

void MitLoopController::set_command(std::uint16_t can_id, MitCommand command)
{
  const auto index = motor_index(can_id);
  std::lock_guard<std::mutex> lock(mutex_);
  auto updated = commands_; updated[index] = command;
  guard_.accept(updated, monotonic_seconds());
  commands_ = std::move(updated);
}

void MitLoopController::set_all_commands(std::vector<MitCommand> commands)
{
  if(commands.size() != commands_.size())
  {
    throw std::invalid_argument("MIT command count does not match motor count");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  guard_.accept(commands, monotonic_seconds());
  commands_ = std::move(commands);
}

std::vector<MitCommand> MitLoopController::commands() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return commands_;
}

std::vector<MitCommand> MitLoopController::sent_commands() const {
  std::lock_guard<std::mutex> lock(mutex_); return sent_commands_;
}

std::size_t MitLoopController::motor_index(std::uint16_t can_id) const
{
  for(std::size_t i = 0; i < can_ids_.size(); ++i)
  {
    if(can_ids_[i] == can_id)
    {
      return i;
    }
  }

  throw std::invalid_argument("unknown motor CAN ID");
}

void MitLoopController::worker_loop(double hz)
{
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  auto next_tick = clock::now();

  try
  {
    while(running_)
    {
      const auto states = guard_.enabled ? arm_.states() : std::vector<MotorState>{};
      std::vector<MitCommand> snapshot;
      std::uint64_t sending_seq = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = guard_.step(states, commands_, monotonic_seconds());
        sending_seq = (!guard_.enabled || guard_.state == "ACTIVE") ? guard_.accepted_seq : 0;
      }

      arm_.send_mit_all(snapshot);
      { std::lock_guard<std::mutex> lock(mutex_); guard_.sent_seq = sending_seq; sent_commands_ = snapshot; }

      next_tick += std::chrono::duration_cast<clock::duration>(period);
      const auto now = clock::now();
      if(now > next_tick)
      {
        ++deadline_misses_;
        next_tick = now + std::chrono::duration_cast<clock::duration>(period);
      }
      std::this_thread::sleep_until(next_tick);
    }
  }
  catch(...)
  {
    const auto exception = std::current_exception();
    running_ = false;
    try
    {
      arm_.send_zero_mit_all();
    }
    catch(...)
    {
    }
    try
    {
      arm_.disable();
    }
    catch(...)
    {
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      guard_.state = "FAULT"; guard_.latched = true;
      try { std::rethrow_exception(exception); }
      catch (const std::exception& e) { guard_.fault = e.what(); }
      catch (...) { guard_.fault = "MIT worker failure"; }
    }
    worker_exception_ = exception;
  }
}

}  // namespace dm_openarm
