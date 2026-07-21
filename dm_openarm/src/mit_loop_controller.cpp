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

  left_begin_ = cfg.left.begin;
  left_count_ = cfg.left.count;
  right_begin_ = cfg.right.begin;
  right_count_ = cfg.right.count;

  left_.begin = cfg.left.begin;
  left_.count = cfg.left.count;
  left_.has_model = false;
  left_.enabled = false;
  left_.scale = cfg.left.gravity.scale;
  left_.use_measured_q = cfg.left.gravity.use_measured_q;
  if(cfg.left.present() &&
     (cfg.left.gravity.enabled || !cfg.left.gravity.coupled.basis.empty()))
  {
    if(cfg.left.gravity.enabled &&
       (cfg.left.gravity.coupled.basis.empty() ||
        cfg.left.gravity.coupled.weights.empty()))
    {
      throw std::runtime_error(
        "left gravity enabled but coupled basis/weights missing");
    }
    if(!cfg.left.gravity.coupled.basis.empty())
    {
      if(cfg.left.gravity.coupled.weights.size() != cfg.left.count)
      {
        throw std::runtime_error(
          "left gravity.coupled.weights length must match left motor count");
      }
      left_.model = GravityModel(cfg.left.gravity.coupled);
      left_.has_model = true;
      left_.enabled = cfg.left.gravity.enabled;
    }
  }

  right_.begin = cfg.right.begin;
  right_.count = cfg.right.count;
  right_.has_model = false;
  right_.enabled = false;
  right_.scale = cfg.right.gravity.scale;
  right_.use_measured_q = cfg.right.gravity.use_measured_q;
  if(cfg.right.present() &&
     (cfg.right.gravity.enabled || !cfg.right.gravity.coupled.basis.empty()))
  {
    if(cfg.right.gravity.enabled &&
       (cfg.right.gravity.coupled.basis.empty() ||
        cfg.right.gravity.coupled.weights.empty()))
    {
      throw std::runtime_error(
        "right gravity enabled but coupled basis/weights missing");
    }
    if(!cfg.right.gravity.coupled.basis.empty())
    {
      if(cfg.right.gravity.coupled.weights.size() != cfg.right.count)
      {
        throw std::runtime_error(
          "right gravity.coupled.weights length must match right motor count");
      }
      right_.model = GravityModel(cfg.right.gravity.coupled);
      right_.has_model = true;
      right_.enabled = cfg.right.gravity.enabled;
    }
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

MitLoopController::LimbRuntime& MitLoopController::limb_mut(const std::string& side)
{
  if(side == "left")
  {
    return left_;
  }
  if(side == "right")
  {
    return right_;
  }
  throw std::invalid_argument("limb side must be 'left' or 'right'");
}

const MitLoopController::LimbRuntime& MitLoopController::limb_ref(
  const std::string& side) const
{
  if(side == "left")
  {
    return left_;
  }
  if(side == "right")
  {
    return right_;
  }
  throw std::invalid_argument("limb side must be 'left' or 'right'");
}

void MitLoopController::set_gravity_enabled(bool enabled)
{
  left_.enabled = enabled;
}

bool MitLoopController::gravity_enabled() const noexcept
{
  return left_.enabled.load();
}

void MitLoopController::set_gravity_scale(double scale)
{
  left_.scale = scale;
}

double MitLoopController::gravity_scale() const noexcept
{
  return left_.scale.load();
}

void MitLoopController::set_gravity_use_measured_q(bool use_measured)
{
  left_.use_measured_q = use_measured;
}

bool MitLoopController::gravity_use_measured_q() const noexcept
{
  return left_.use_measured_q.load();
}

void MitLoopController::set_limb_gravity_enabled(const std::string& side, bool enabled)
{
  limb_mut(side).enabled = enabled;
}

bool MitLoopController::limb_gravity_enabled(const std::string& side) const
{
  return limb_ref(side).enabled.load();
}

void MitLoopController::set_limb_gravity_scale(const std::string& side, double scale)
{
  limb_mut(side).scale = scale;
}

double MitLoopController::limb_gravity_scale(const std::string& side) const
{
  return limb_ref(side).scale.load();
}

void MitLoopController::set_limb_gravity_use_measured_q(
  const std::string& side, bool use_measured)
{
  limb_mut(side).use_measured_q = use_measured;
}

bool MitLoopController::limb_gravity_use_measured_q(const std::string& side) const
{
  return limb_ref(side).use_measured_q.load();
}

std::vector<double> MitLoopController::limb_gravity_torques(
  const std::string& side, const std::vector<double>& q) const
{
  const auto& limb = limb_ref(side);
  if(!limb.has_model || limb.count == 0)
  {
    return std::vector<double>(q.size(), 0.0);
  }
  if(q.size() != limb.count)
  {
    throw std::invalid_argument("limb gravity q size must match limb motor count");
  }
  return limb.model.compute(q, limb.scale.load());
}

std::vector<double> MitLoopController::gravity_torques(const std::vector<double>& q) const
{
  return limb_gravity_torques("left", q);
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

void MitLoopController::apply_limb_gravity(
  LimbRuntime const& limb,
  std::vector<MitCommand>& commands,
  const std::vector<double>* measured_q_full) const
{
  if(!limb.enabled.load() || !limb.has_model || limb.count == 0)
  {
    return;
  }
  if(limb.begin + limb.count > commands.size())
  {
    throw std::runtime_error("limb index range exceeds command count");
  }

  std::vector<double> q(limb.count, 0.0);
  if(limb.use_measured_q.load())
  {
    if(measured_q_full == nullptr || measured_q_full->size() != commands.size())
    {
      throw std::runtime_error("gravity: measured q unavailable or size mismatch");
    }
    for(std::size_t i = 0; i < limb.count; ++i)
    {
      q[i] = (*measured_q_full)[limb.begin + i];
    }
  }
  else
  {
    for(std::size_t i = 0; i < limb.count; ++i)
    {
      q[i] = commands[limb.begin + i].q;
    }
  }

  const auto tau_g = limb.model.compute(q, limb.scale.load());
  for(std::size_t i = 0; i < limb.count; ++i)
  {
    commands[limb.begin + i].tau += tau_g[i];
  }
}

std::vector<MitCommand> MitLoopController::apply_gravity(
  std::vector<MitCommand> commands) const
{
  const bool need_meas =
    (left_.enabled.load() && left_.has_model && left_.use_measured_q.load()) ||
    (right_.enabled.load() && right_.has_model && right_.use_measured_q.load());

  std::vector<double> measured;
  const std::vector<double>* measured_ptr = nullptr;
  if(need_meas)
  {
    const auto states = arm_.states();
    if(states.size() != commands.size())
    {
      throw std::runtime_error("gravity: state count does not match command count");
    }
    measured.resize(states.size());
    for(std::size_t i = 0; i < states.size(); ++i)
    {
      measured[i] = states[i].position;
    }
    measured_ptr = &measured;
  }

  apply_limb_gravity(left_, commands, measured_ptr);
  apply_limb_gravity(right_, commands, measured_ptr);
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
