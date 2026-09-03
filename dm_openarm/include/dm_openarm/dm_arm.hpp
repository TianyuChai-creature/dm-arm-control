#pragma once

#include "dm_openarm/backend/dm_serial_backend.hpp"
#include "dm_openarm/config.hpp"
#include "dm_openarm/types.hpp"

#include <cstdint>
#include <vector>

namespace dm_openarm {

class DmArm {
public:
  explicit DmArm(ArmConfig config);
  ~DmArm();

  DmArm(const DmArm&) = delete;
  DmArm& operator=(const DmArm&) = delete;

  void connect();
  void enable();
  void disable();
  void disconnect();
  bool connected() const noexcept;

  void send_mit_all(const std::vector<MitCommand>& commands);
  void send_zero_mit_all();
  void set_zero(std::uint16_t can_id, bool persist = true);
  void set_zero_all(bool persist = true);
  std::vector<MotorState> states() const;

  const ArmConfig& config() const noexcept;

private:
  ArmConfig config_;
  backend::DmSerialBackend backend_;
};

}  // namespace dm_openarm
