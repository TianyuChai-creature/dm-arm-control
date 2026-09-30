#include "dm_openarm/backend/dm_serial_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace dm_openarm::backend {

struct DmSerialBackend::SharedBus {
  std::vector<damiao::DmActData> data;
  std::shared_ptr<damiao::Motor_Control> control;
  std::set<std::uint16_t> owners;
  std::string layout;
};
namespace {
std::mutex registry_mutex;
}

DmSerialBackend::DmSerialBackend(ArmConfig config)
  : config_(std::move(config))
{
}

DmSerialBackend::~DmSerialBackend()
{
  try { disconnect(); }
  catch (...) {
    // Fail-closed for the remainder of this process: retain the shared bus
    // and its owner reservation instead of handing an unknown motor to a new owner.
    std::lock_guard<std::mutex> registry_lock(registry_mutex);
    static auto* quarantined = new std::vector<std::shared_ptr<SharedBus>>;
    if (bus_) quarantined->push_back(bus_);
    control_.reset(); bus_.reset();
  }
}

damiao::DM_Motor_Type DmSerialBackend::to_damiao_model(MotorModel model)
{
  switch(model)
  {
  case MotorModel::DM4310:
    return damiao::DM4310;
  case MotorModel::DM8009:
    return damiao::DM8009;
  case MotorModel::DM4340P:
    // Station-confirmed PMAX=12.5, VMAX=20, TMAX=28.
    // Reuse the vendor profile with these exact ranges (not a voltage assertion).
    return damiao::DM4340_48V;
  }
  throw std::invalid_argument("unsupported motor model");
}

void DmSerialBackend::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(control_)
  {
    return;
  }

  std::lock_guard<std::mutex> registry_lock(registry_mutex);
  static std::map<std::string, std::weak_ptr<SharedBus>> buses;
  const std::string key = std::to_string(config_.device_index);
  const auto& motors = config_.bus_motors.empty() ? config_.motors : config_.bus_motors;
  std::ostringstream identity;
  identity << config_.usb_serial << ':' << config_.nom_baud << ':' << config_.dat_baud << ':' << config_.canfd << ':' << config_.brs;
  std::set<std::uint16_t> registered;
  for (const auto& m : motors) {
    identity << ':' << m.can_id << ',' << m.mst_id << ',' << static_cast<int>(m.model);
    registered.insert(m.can_id);
  }
  auto bus = buses[key].lock();
  if (bus && bus->layout != identity.str())
    throw std::runtime_error("shared USB bus configuration differs");
  std::set<std::uint16_t> selected;
  for (const auto& m : config_.motors) {
    auto definition = std::find_if(motors.begin(), motors.end(), [&m](const auto& candidate) {
      return candidate.can_id == m.can_id && candidate.mst_id == m.mst_id && candidate.model == m.model;
    });
    if (definition == motors.end() || !registered.count(m.can_id) || !selected.insert(m.can_id).second ||
        (bus && bus->owners.count(m.can_id)))
      throw std::runtime_error("motor is missing, duplicated, or already owned on shared USB bus");
  }
  if (!bus) {
    bus = std::make_shared<SharedBus>();
    bus->layout = identity.str();
    for (const auto& m : motors)
      bus->data.push_back(damiao::DmActData{to_damiao_model(m.model), damiao::MIT_MODE, m.can_id, m.mst_id});
    bus->control = std::make_shared<damiao::Motor_Control>(
      config_.nom_baud, config_.dat_baud, config_.usb_serial, &bus->data,
      config_.canfd, config_.brs, config_.device_index, false, DMCAN_USB2CANFD, false);
    buses[key] = bus;
  }
  bus->owners.insert(selected.begin(), selected.end());
  bus_ = bus;
  control_ = bus->control;
}

void DmSerialBackend::enable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!control_)
  {
    throw std::runtime_error("DmSerialBackend is not connected");
  }
  if (maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("motor state unknown; enable blocked pending recovery");
  enable_baseline_.clear();
  enabled_ = true;  // A partially enabled side must still be disabled on failure.
  try {
    for (const auto& m : config_.motors) {
      enable_baseline_.push_back(control_->getMotorFeedback(m.can_id).rx_sequence);
      control_->enable_motor(*control_->getMotor(m.can_id));
    }
  } catch (...) { shutdown_unknown_ = true; throw; }
}

void DmSerialBackend::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(control_ && enabled_)
  {
    std::exception_ptr first;
    std::vector<std::uint64_t> before;
    for (const auto& m : config_.motors) {
      // Snapshot immediately before this axis's FD frames, not before the
      // whole batch; otherwise an earlier unsolicited status0 can qualify.
      before.push_back(control_->getMotorFeedback(m.can_id).rx_sequence);
      try { control_->disable_motor(*control_->getMotor(m.can_id)); }
      catch (...) { if (!first) first = std::current_exception(); }
    }
    if (first) { shutdown_unknown_ = true; std::rethrow_exception(first); }
    // Vendor has no separate FC/FD ACK. A new, matching motion status 0 is
    // required; some firmware may cease feedback after FD, then fail closed.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    bool all_disabled = false;
    do {
      all_disabled = true;
      for (std::size_t i=0; i<config_.motors.size(); ++i) {
        const auto f = control_->getMotorFeedback(config_.motors[i].can_id);
        all_disabled &= f.rx_sequence > before[i] && f.last_rx_age_s <= 0.2 &&
                        f.error_code == 0;
      }
      if (all_disabled) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    if (!all_disabled) {
      shutdown_unknown_ = true;
      throw std::runtime_error("disable status unconfirmed; motor state unknown");
    }
    enabled_ = false;
    shutdown_unknown_ = false;
    enable_baseline_.clear();
  }
}

void DmSerialBackend::disconnect()
{
  // Never release a CAN owner while its state is unknown. A successful,
  // status-confirmed disable is required first; a failed maintenance zero
  // requires manual investigation and process restart.
  disable();
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_ || maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("cannot release USB owner while motor state is unknown");
  enabled_ = false;
  enable_baseline_.clear();
  std::lock_guard<std::mutex> registry_lock(registry_mutex);
  if (bus_) for (const auto& m : config_.motors) bus_->owners.erase(m.can_id);
  control_.reset();
  bus_.reset();
}

bool DmSerialBackend::connected() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<bool>(control_);
}

std::vector<std::uint16_t> DmSerialBackend::probe_status(double timeout_s)
{
  if(!std::isfinite(timeout_s) || timeout_s <= 0)
    throw std::invalid_argument("status probe timeout must be positive and finite");
  std::lock_guard<std::mutex> lock(mutex_);
  if(!control_) throw std::runtime_error("USB bus is not connected");
  std::vector<std::uint64_t> before;
  before.reserve(config_.motors.size());
  for(const auto& motor : config_.motors)
    before.push_back(control_->status_probe_sequence(motor.can_id));
  for(const auto& motor : config_.motors)
    control_->refresh_motor_status(*control_->getMotor(motor.can_id));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  std::vector<std::uint16_t> responded;
  do {
    responded.clear();
    for(std::size_t i = 0; i < config_.motors.size(); ++i)
      if(control_->status_probe_sequence(config_.motors[i].can_id) > before[i])
        responded.push_back(config_.motors[i].can_id);
    if(responded.size() == config_.motors.size()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while(std::chrono::steady_clock::now() < deadline);
  return responded;
}

void DmSerialBackend::send_mit_all(const std::vector<MitCommand>& commands)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!control_)
  {
    throw std::runtime_error("DmSerialBackend is not connected");
  }
  if (maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("motor state unknown; MIT commands blocked");
  if(commands.size() != config_.motors.size())
  {
    throw std::invalid_argument("MIT command count does not match motor count");
  }

  // Validate the complete batch before emitting any CAN frame. Transmission
  // errors can still leave a prefix applied; CAN provides no batch transaction.
  std::vector<std::shared_ptr<damiao::Motor>> motors;
  motors.reserve(config_.motors.size());
  for(std::size_t i = 0; i < config_.motors.size(); ++i)
  {
    const auto motor = control_->getMotor(config_.motors[i].can_id);
    if(!motor)
    {
      throw std::runtime_error("motor is not registered");
    }

    const auto& command = commands[i];
    control_->validate_mit(*motor, static_cast<float>(command.kp), static_cast<float>(command.kd),
                           static_cast<float>(command.q), static_cast<float>(command.dq),
                           static_cast<float>(command.tau));
    motors.push_back(motor);
  }
  for(std::size_t i = 0; i < motors.size(); ++i) {
    const auto& command = commands[i];
    try {
      control_->control_mit(
        *motors[i], static_cast<float>(command.kp), static_cast<float>(command.kd),
        static_cast<float>(command.q), static_cast<float>(command.dq),
        static_cast<float>(command.tau));
    } catch (const std::exception& e) {
      shutdown_unknown_ = true;
      throw std::runtime_error("MIT batch send failed at CAN ID=" +
        std::to_string(config_.motors[i].can_id) + "; prior frames sent=" +
        std::to_string(i) + "; motor state unknown: " + e.what());
    }
  }
}

void DmSerialBackend::send_zero_mit_all()
{
  send_mit_all(std::vector<MitCommand>(config_.motors.size()));
}

void DmSerialBackend::validate_mit_all(const std::vector<MitCommand>& commands) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (commands.size() != config_.motors.size())
    throw std::invalid_argument("MIT command count does not match motor count");
  for (std::size_t i=0; i<commands.size(); ++i) {
    const auto& c = commands[i];
    const auto model = to_damiao_model(config_.motors[i].model);
    const auto limits = control_ ? control_->getMotor(config_.motors[i].can_id)->get_limit_param()
                                 : damiao::limit_param[model];
    const auto check = [](double value, double lo, double hi, int bits) {
      if (!std::isfinite(value) || value < lo || value > hi)
        throw std::out_of_range("MIT field value is outside motor limits");
      (void)damiao::encode_mit_field(static_cast<float>(value), static_cast<float>(lo),
                                     static_cast<float>(hi), static_cast<std::uint8_t>(bits));
    };
    check(c.kp, 0, 500, 12); check(c.kd, 0, 5, 12);
    check(c.q, -limits.Q_MAX, limits.Q_MAX, 16);
    check(c.dq, -limits.DQ_MAX, limits.DQ_MAX, 12);
    check(c.tau, -limits.TAU_MAX, limits.TAU_MAX, 12);
  }
}

void DmSerialBackend::set_zero(std::uint16_t can_id, bool persist)
{
  std::lock_guard<std::mutex> lock(mutex_);
  set_zero_locked(can_id, persist, true);
}

void DmSerialBackend::set_zero_locked(std::uint16_t can_id, bool persist, bool verify)
{
  if(!control_)
  {
    throw std::runtime_error("DmSerialBackend is not connected");
  }

  if (std::none_of(config_.motors.begin(), config_.motors.end(),
                   [can_id](const auto& m) { return m.can_id == can_id; }))
    throw std::invalid_argument("motor is outside this owner's scope");
  const auto motor = control_->getMotor(can_id);
  if(!motor)
  {
    throw std::runtime_error("motor CAN ID is not registered");
  }
  if (maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("set_zero blocked while motor state is unknown");
  if(persist && verify) verify_set_zero_ready(can_id);
  try {
    control_->set_zero_position(*motor);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(persist) {
    // save_motor_param disables only the target. A passively connected owner
    // must stay passive; do not energize every motor on this side.
    const bool restore_target = enabled_;
    control_->save_motor_param(*motor);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (restore_target) {
      const auto target_index = static_cast<std::size_t>(std::distance(config_.motors.begin(),
        std::find_if(config_.motors.begin(), config_.motors.end(),
          [can_id](const auto& m) { return m.can_id == can_id; })));
      enable_baseline_[target_index] = control_->getMotorFeedback(can_id).rx_sequence;
      control_->enable_motor(*motor);
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      bool confirmed = false;
      do {
        const auto f = control_->getMotorFeedback(can_id);
        confirmed = f.rx_sequence > enable_baseline_[target_index] &&
                    f.last_rx_age_s <= 0.2 && f.error_code == 1;
        if (confirmed) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } while (std::chrono::steady_clock::now() < deadline);
      if (!confirmed) throw std::runtime_error("set_zero: target re-enable status unconfirmed");
    }
    }
  } catch (const std::exception& e) {
    maintenance_unknown_ = true;
    throw std::runtime_error("set_zero failed at CAN ID=" + std::to_string(can_id) +
      "; coordinate/state unknown: " + e.what());
  }
}

void DmSerialBackend::set_zero_all(bool persist)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!control_) throw std::runtime_error("USB bus is not connected");
  if (persist) {
    // Preflight all motors before changing any Flash coordinate zero.
    for (const auto& motor : config_.motors) verify_set_zero_ready(motor.can_id);
  }
  std::size_t completed = 0;
  for(const auto& motor : config_.motors)
  {
    try { set_zero_locked(motor.can_id, persist, true); ++completed; }
    catch (const std::exception& e) {
      if (completed > 0) maintenance_unknown_ = true;
      throw std::runtime_error("set_zero_all failed at CAN ID=" + std::to_string(motor.can_id) +
                               "; prior motors may already be changed: " + e.what());
    }
  }
}

void DmSerialBackend::verify_set_zero_ready(std::uint16_t can_id) const
{
  const auto target_index = std::distance(config_.motors.begin(),
    std::find_if(config_.motors.begin(), config_.motors.end(),
      [can_id](const auto& m) { return m.can_id == can_id; }));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  const auto before = control_->getMotorFeedback(can_id).rx_sequence;
  bool confirmed = false;
  do {
    const auto feedback = control_->getMotorFeedback(can_id);
    confirmed = enabled_
      ? static_cast<std::size_t>(target_index) < enable_baseline_.size() &&
        feedback.rx_sequence > before &&
        feedback.rx_sequence > enable_baseline_[target_index] &&
        feedback.last_rx_age_s <= 0.2 && feedback.error_code == 1
      : feedback.rx_sequence > before && feedback.last_rx_age_s <= 0.2 &&
        feedback.error_code == 0;
    if (confirmed) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
  if (!confirmed)
    throw std::runtime_error("set_zero: prior state unconfirmed for CAN ID=" + std::to_string(can_id));
}

std::vector<MotorState> DmSerialBackend::states() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!control_)
  {
    throw std::runtime_error("DmSerialBackend is not connected");
  }

  std::vector<MotorState> result;
  result.reserve(config_.motors.size());
  for(std::size_t i = 0; i < config_.motors.size(); ++i)
  {
    const auto& motor_config = config_.motors[i];
    const auto feedback = control_->getMotorFeedback(motor_config.can_id);

    result.push_back(MotorState{
      motor_config.can_id,
      motor_config.mst_id,
      feedback.position,
      feedback.velocity,
      feedback.torque,
      feedback.feedback_interval_s,
      feedback.last_rx_age_s,
      feedback.rx_sequence,
      motor_error_code(feedback.error_code),
      feedback.feedback_hz,
      feedback.error_code,
      enabled_ && i < enable_baseline_.size() && feedback.rx_sequence > enable_baseline_[i] && feedback.error_code == 1});
  }

  return result;
}

}  // namespace dm_openarm::backend
