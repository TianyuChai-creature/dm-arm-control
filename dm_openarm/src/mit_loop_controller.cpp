#include "dm_openarm/mit_loop_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace dm_openarm {

MitLoopController::MitLoopController(DmArm& arm)
  : arm_(arm)
{
  const auto& motors = arm_.config().motors;
  can_ids_.reserve(motors.size());
  commands_.resize(motors.size());
  for(const auto& motor : motors)
  {
    can_ids_.push_back(motor.can_id);
  }

  const auto& gcfg = arm.config().gravity;
  std::vector<JointGravityParam> joints = gcfg.joints;
  if(joints.size() != motors.size())
  {
    std::vector<JointGravityParam> padded(motors.size());
    for(std::size_t i = 0; i < padded.size() && i < joints.size(); ++i)
    {
      padded[i] = joints[i];
    }
    joints = std::move(padded);
  }

  gravity_model_ = GravityModel(gcfg.mode, std::move(joints), gcfg.coupled);

  gravity_enabled_ = gcfg.enabled;
  gravity_scale_ = gcfg.scale;
  gravity_use_measured_q_ = gcfg.use_measured_q;
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

  worker_exception_ = nullptr;
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

  if(was_running)
  {
    arm_.send_zero_mit_all();
  }

  if(worker_exception_)
  {
    auto exception = worker_exception_;
    worker_exception_ = nullptr;
    std::rethrow_exception(exception);
  }
}

bool MitLoopController::running() const noexcept
{
  return running_;
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

void MitLoopController::set_gravity_enabled(bool enabled)
{
  gravity_enabled_ = enabled;
}

bool MitLoopController::gravity_enabled() const noexcept
{
  return gravity_enabled_;
}

void MitLoopController::set_gravity_scale(double scale)
{
  gravity_scale_ = scale;
}

double MitLoopController::gravity_scale() const noexcept
{
  return gravity_scale_;
}

void MitLoopController::set_gravity_use_measured_q(bool use_measured)
{
  gravity_use_measured_q_ = use_measured;
}

bool MitLoopController::gravity_use_measured_q() const noexcept
{
  return gravity_use_measured_q_;
}

std::vector<double> MitLoopController::gravity_torques(const std::vector<double>& q) const
{
  return gravity_model_.compute(q, gravity_scale_.load());
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

std::vector<MitCommand> MitLoopController::apply_gravity(std::vector<MitCommand> commands) const
{
  if(!gravity_enabled_.load() || gravity_model_.size() == 0)
  {
    return commands;
  }

  std::vector<double> q(commands.size(), 0.0);
  if(gravity_use_measured_q_.load())
  {
    const auto states = arm_.states();
    if(states.size() != commands.size())
    {
      throw std::runtime_error("gravity: state count does not match command count");
    }
    for(std::size_t i = 0; i < states.size(); ++i)
    {
      q[i] = states[i].position;
    }
  }
  else
  {
    for(std::size_t i = 0; i < commands.size(); ++i)
    {
      q[i] = commands[i].q;
    }
  }

  const auto tau_g = gravity_model_.compute(q, gravity_scale_.load());
  for(std::size_t i = 0; i < commands.size(); ++i)
  {
    commands[i].tau += tau_g[i];
  }
  return commands;
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

      snapshot = apply_gravity(std::move(snapshot));
      arm_.send_mit_all(snapshot);

      next_tick += std::chrono::duration_cast<clock::duration>(period);
      std::this_thread::sleep_until(next_tick);
    }
  }
  catch(...)
  {
    worker_exception_ = std::current_exception();
    running_ = false;
  }
}

}  // namespace dm_openarm
