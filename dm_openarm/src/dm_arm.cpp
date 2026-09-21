#include "dm_openarm/dm_arm.hpp"

#include <utility>

namespace dm_openarm {

DmArm::DmArm(ArmConfig config)
  : config_(std::move(config))
  , backend_(config_)
{
}

DmArm::~DmArm()
{
  try
  {
    disable();
  }
  catch(...)
  {
  }
  try
  {
    disconnect();
  }
  catch(...)
  {
  }
}

void DmArm::connect()
{
  backend_.connect();
}

void DmArm::enable()
{
  connect();
  try
  {
    backend_.enable();
  }
  catch(...)
  {
    backend_.disconnect();
    throw;
  }
}

void DmArm::disable()
{
  try
  {
    backend_.disable();
  }
  catch(...)
  {
    backend_.disconnect();
    throw;
  }
}

void DmArm::disconnect()
{
  backend_.disconnect();
}

bool DmArm::connected() const noexcept
{
  return backend_.connected();
}

void DmArm::send_mit_all(const std::vector<MitCommand>& commands)
{
  backend_.send_mit_all(commands);
}

void DmArm::send_zero_mit_all()
{
  backend_.send_zero_mit_all();
}

void DmArm::set_zero(std::uint16_t can_id, bool persist)
{
  backend_.set_zero(can_id, persist);
}

void DmArm::set_zero_all(bool persist)
{
  backend_.set_zero_all(persist);
}

std::vector<MotorState> DmArm::states() const
{
  return backend_.states();
}

std::vector<std::uint16_t> DmArm::probe_status(double timeout_s)
{
  return backend_.probe_status(timeout_s);
}

const ArmConfig& DmArm::config() const noexcept
{
  return config_;
}

}  // namespace dm_openarm
