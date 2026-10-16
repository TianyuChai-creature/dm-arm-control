#pragma once

#include "dm_openarm/config.hpp"
#include "dm_openarm/types.hpp"

#include "protocol/damiao.h"

#include <cstdint>
#include <atomic>
#include <functional>
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
  void validate_mit_all(const std::vector<MitCommand>& commands) const;
  void send_zero_mit_all();
  void set_zero(std::uint16_t can_id, bool persist = true);
  void set_zero_all(bool persist = true);
  void set_zero_many(const std::vector<std::uint16_t>& can_ids, bool persist = true);
  std::vector<MotorState> states() const;
  std::vector<std::uint16_t> probe_status(double timeout_s);

  std::vector<double> verify_calibration_configuration();
  void commit_calibration(const std::vector<double>& zeros, const std::atomic<bool>& cancel,
                          const std::function<void(std::size_t, const std::string&)>& progress);

private:
  std::vector<double> read_disabled_positions_locked(const std::vector<double>& expected = {});
  static damiao::DM_Motor_Type to_damiao_model(MotorModel model);
  void verify_set_zero_ready(const std::vector<std::uint16_t>& can_ids) const; // mutex_ held

  ArmConfig config_;
  struct SharedBus;
  std::shared_ptr<SharedBus> bus_;
  mutable std::mutex mutex_;
  std::shared_ptr<damiao::Motor_Control> control_;
  bool enabled_{false};
  bool disabled_confirmed_{false};
  bool shutdown_unknown_{false};
  bool maintenance_unknown_{false};
  std::vector<std::uint64_t> enable_baseline_;
};

}  // namespace dm_openarm::backend
