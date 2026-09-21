#pragma once

#include "dm_openarm/config.hpp"
#include "dm_openarm/types.hpp"

#include "protocol/damiao.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace dm_openarm::backend {

class DmSerialBackend {
public:
  explicit DmSerialBackend(ArmConfig config);
  ~DmSerialBackend();

  DmSerialBackend(const DmSerialBackend&) = delete;
  DmSerialBackend& operator=(const DmSerialBackend&) = delete;

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
  std::vector<std::uint16_t> probe_status(double timeout_s);

private:
  static damiao::DM_Motor_Type to_damiao_model(MotorModel model);

  ArmConfig config_;
  struct SharedBus;
  std::shared_ptr<SharedBus> bus_;
  mutable std::mutex mutex_;
  std::shared_ptr<damiao::Motor_Control> control_;
  bool enabled_{false};
};

}  // namespace dm_openarm::backend
