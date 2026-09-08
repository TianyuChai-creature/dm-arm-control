#include "dm_openarm/backend/dm_serial_backend.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace dm_openarm::backend {

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

  init_data_.clear();
  init_data_.reserve(config_.motors.size());
  for(const auto& motor : config_.motors)
  {
    init_data_.push_back(
      damiao::DmActData{to_damiao_model(motor.model), damiao::MIT_MODE, motor.can_id, motor.mst_id});
  }

  control_ = std::make_shared<damiao::Motor_Control>(
    config_.nom_baud,
    config_.dat_baud,
    config_.usb_serial,
    &init_data_,
    config_.canfd,
    config_.brs,
    config_.device_index,
    /*auto_enable=*/false);
}

void DmSerialBackend::enable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!control_)
  {
    throw std::runtime_error("DmSerialBackend is not connected");
  }
  control_->enable_all();
}

void DmSerialBackend::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(control_)
  {
    control_->disable_all();
  }
}

void DmSerialBackend::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  control_.reset();
}

bool DmSerialBackend::connected() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<bool>(control_);
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
    control_->enable_all();
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
      motor_error_code(feedback.error_code)});
  }

  return result;
}

}  // namespace dm_openarm::backend
