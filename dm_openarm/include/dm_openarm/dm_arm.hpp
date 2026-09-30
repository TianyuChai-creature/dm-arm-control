#pragma once

#include "dm_openarm/backend/dm_serial_backend.hpp"
#include "dm_openarm/config.hpp"
#include "dm_openarm/types.hpp"

#include <cstdint>
#include <mutex>
#include <vector>

namespace dm_openarm {
class MitLoopController;

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
  void validate_mit_all(const std::vector<MitCommand>& commands) const;
  void send_zero_mit_all();
  void set_zero(std::uint16_t can_id, bool persist = true);
  void set_zero_all(bool persist = true);
  std::vector<MotorState> states() const;
  std::vector<std::uint16_t> probe_status(double timeout_s);

  const ArmConfig& config() const noexcept;

private:
  friend class MitLoopController;
  void set_loop_active(bool active);
  void disable_for_loop();
  mutable std::mutex maintenance_mutex_;
  bool loop_active_{false};
  ArmConfig config_;
  backend::DmSerialBackend backend_;
};

}  // namespace dm_openarm
