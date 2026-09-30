#include "dm_openarm/dm_arm.hpp"

#include <utility>
#include <stdexcept>
#include <string>

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
  std::lock_guard<std::mutex> lock(maintenance_mutex_);
  if (loop_active_) throw std::runtime_error("enable requires a stopped MIT loop");
  connect();
  try
  {
    backend_.enable();
  }
  catch(...)
  {
    const auto original = std::current_exception();
    std::string cleanup_error;
    try { backend_.disable(); }
    catch (const std::exception& e) { cleanup_error = e.what(); }
    catch (...) { cleanup_error = "unknown shutdown failure"; }
    if (!cleanup_error.empty()) {
      std::string original_text = "enable failed";
      try { std::rethrow_exception(original); }
      catch (const std::exception& e) { original_text = e.what(); }
      catch (...) {}
      throw std::runtime_error(original_text + "; shutdown unknown: " + cleanup_error);
    }
    backend_.disconnect();
    std::rethrow_exception(original);
  }
}

void DmArm::disable()
{
  std::lock_guard<std::mutex> lock(maintenance_mutex_);
  if (loop_active_) throw std::runtime_error("disable requires a stopped MIT loop");
  backend_.disable();
}

void DmArm::disable_for_loop()
{
  backend_.disable();
}

void DmArm::disconnect()
{
  std::lock_guard<std::mutex> lock(maintenance_mutex_);
  if (loop_active_) throw std::runtime_error("disconnect requires a stopped MIT loop");
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

void DmArm::validate_mit_all(const std::vector<MitCommand>& commands) const
{
  backend_.validate_mit_all(commands);
}

void DmArm::send_zero_mit_all()
{
  backend_.send_zero_mit_all();
}

void DmArm::set_zero(std::uint16_t can_id, bool persist)
{
  std::lock_guard<std::mutex> lock(maintenance_mutex_);
  if (loop_active_) throw std::runtime_error("set_zero requires a stopped MIT loop");
  backend_.set_zero(can_id, persist);
}

void DmArm::set_zero_all(bool persist)
{
  std::lock_guard<std::mutex> lock(maintenance_mutex_);
  if (loop_active_) throw std::runtime_error("set_zero_all requires a stopped MIT loop");
  backend_.set_zero_all(persist);
}

void DmArm::set_loop_active(bool active)
{
  std::lock_guard<std::mutex> lock(maintenance_mutex_);
  if (active && loop_active_) throw std::runtime_error("MIT loop already active for this arm");
  loop_active_ = active;
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
