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

private:
  static damiao::DM_Motor_Type to_damiao_model(MotorModel model);

  ArmConfig config_;
  std::vector<damiao::DmActData> init_data_;
  mutable std::mutex mutex_;
  std::shared_ptr<damiao::Motor_Control> control_;
};

}  // namespace dm_openarm::backend
