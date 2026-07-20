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
  }
  throw std::invalid_argument("unsupported motor model");
}

void DmSerialBackend::connect()
{
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
    /*auto_enable=*/true);
}

void DmSerialBackend::disconnect()
{
  control_.reset();
}

bool DmSerialBackend::connected() const noexcept
{
  return static_cast<bool>(control_);
}

void DmSerialBackend::send_mit_all(const std::vector<MitCommand>& commands)
{
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
  if(!control_)
  {
    throw std::runtime_error("DmSerialBackend is not connected");
  }

  std::vector<MotorState> result;
  result.reserve(config_.motors.size());
  for(const auto& motor_config : config_.motors)
  {
    const auto motor = control_->getMotor(motor_config.can_id);
    if(!motor)
    {
      throw std::runtime_error("motor is not registered");
    }

    result.push_back(MotorState{
      motor_config.can_id,
      motor_config.mst_id,
      motor->Get_Position(),
      motor->Get_Velocity(),
      motor->Get_tau(),
      motor->getTimeInterval()});
  }

  return result;
}

}  // namespace dm_openarm::backend
