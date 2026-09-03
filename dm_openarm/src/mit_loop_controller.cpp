#include "dm_openarm/mit_loop_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace dm_openarm {

MitLoopController::MitLoopController(DmArm& arm)
  : arm_(arm)
{
  const auto& cfg = arm_.config();
  const auto& motors = cfg.motors;
  can_ids_.reserve(motors.size());
  commands_.resize(motors.size());
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
  if(hz <= 0.0)
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
  commands_[index] = command;
}

void MitLoopController::set_all_commands(std::vector<MitCommand> commands)
{
  if(commands.size() != commands_.size())
  {
    throw std::invalid_argument("MIT command count does not match motor count");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  commands_ = std::move(commands);
}

std::vector<MitCommand> MitLoopController::commands() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return commands_;
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
      std::vector<MitCommand> snapshot;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = commands_;
      }

      arm_.send_mit_all(snapshot);

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
    worker_exception_ = exception;
  }
}

}  // namespace dm_openarm
