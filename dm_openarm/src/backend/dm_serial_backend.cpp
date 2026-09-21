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
  disconnect();
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
  enabled_ = true;  // A partially enabled side must still be disabled on failure.
  for (const auto& m : config_.motors) control_->enable_motor(*control_->getMotor(m.can_id));
}

void DmSerialBackend::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(control_ && enabled_)
  {
    for (const auto& m : config_.motors) control_->disable_motor(*control_->getMotor(m.can_id));
    enabled_ = false;
  }
}

void DmSerialBackend::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (control_ && enabled_) for (const auto& m : config_.motors) control_->disable_motor(*control_->getMotor(m.can_id));
  enabled_ = false;
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
    before.push_back(control_->response_sequence(motor.can_id));
  for(const auto& motor : config_.motors)
    control_->refresh_motor_status(*control_->getMotor(motor.can_id));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  std::vector<std::uint16_t> responded;
  do {
    responded.clear();
    for(std::size_t i = 0; i < config_.motors.size(); ++i)
      if(control_->response_sequence(config_.motors[i].can_id) > before[i])
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
  if(commands.size() != config_.motors.size())
  {
    throw std::invalid_argument("MIT command count does not match motor count");
  }

  for(std::size_t i = 0; i < config_.motors.size(); ++i)
  {
    const auto motor = control_->getMotor(config_.motors[i].can_id);
    if(!motor)
    {
      throw std::runtime_error("motor is not registered");
    }

    const auto& command = commands[i];
    control_->control_mit(
      *motor,
      static_cast<float>(command.kp),
      static_cast<float>(command.kd),
      static_cast<float>(command.q),
      static_cast<float>(command.dq),
      static_cast<float>(command.tau));
  }
}

void DmSerialBackend::send_zero_mit_all()
{
  send_mit_all(std::vector<MitCommand>(config_.motors.size()));
}

void DmSerialBackend::set_zero(std::uint16_t can_id, bool persist)
{
  std::lock_guard<std::mutex> lock(mutex_);
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

  control_->set_zero_position(*motor);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if(persist)
  {
    control_->save_motor_param(*motor);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    for (const auto& m : config_.motors) control_->enable_motor(*control_->getMotor(m.can_id));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void DmSerialBackend::set_zero_all(bool persist)
{
  for(const auto& motor : config_.motors)
  {
    set_zero(motor.can_id, persist);
  }
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
  for(const auto& motor_config : config_.motors)
  {
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
      feedback.feedback_hz});
  }

  return result;
}

}  // namespace dm_openarm::backend
