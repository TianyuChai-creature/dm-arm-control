#include "dm_openarm/backend/dm_serial_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <limits>
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
  try { disconnect(); }
  catch (...) {
    // Fail-closed for the remainder of this process: retain the shared bus
    // and its owner reservation instead of handing an unknown motor to a new owner.
    std::lock_guard<std::mutex> registry_lock(registry_mutex);
    static auto* quarantined = new std::vector<std::shared_ptr<SharedBus>>;
    if (bus_) quarantined->push_back(bus_);
    control_.reset(); bus_.reset();
  }
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

  disabled_confirmed_ = false;
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
  if (maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("motor state unknown; enable blocked pending recovery");
  enable_baseline_.clear();
  disabled_confirmed_ = false;
  enabled_ = true;  // A partially enabled side must still be disabled on failure.
  try {
    for (const auto& m : config_.motors) {
      enable_baseline_.push_back(control_->getMotorFeedback(m.can_id).rx_sequence);
      control_->enable_motor(*control_->getMotor(m.can_id));
    }
  } catch (...) { shutdown_unknown_ = true; throw; }
}

void DmSerialBackend::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(control_ && enabled_)
  {
    std::exception_ptr first;
    std::vector<std::uint64_t> before;
    for (const auto& m : config_.motors) {
      // Snapshot immediately before this axis's FD frames, not before the
      // whole batch; otherwise an earlier unsolicited status0 can qualify.
      before.push_back(control_->getMotorFeedback(m.can_id).rx_sequence);
      try { control_->disable_motor(*control_->getMotor(m.can_id)); }
      catch (...) { if (!first) first = std::current_exception(); }
    }
    if (first) { shutdown_unknown_ = true; std::rethrow_exception(first); }
    // Vendor has no separate FC/FD ACK. A new, matching motion status 0 is
    // required; some firmware may cease feedback after FD, then fail closed.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    bool all_disabled = false;
    do {
      all_disabled = true;
      for (std::size_t i=0; i<config_.motors.size(); ++i) {
        const auto f = control_->getMotorFeedback(config_.motors[i].can_id);
        all_disabled &= f.rx_sequence > before[i] && f.last_rx_age_s <= 0.2 &&
                        f.error_code == 0;
      }
      if (all_disabled) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    if (!all_disabled) {
      shutdown_unknown_ = true;
      throw std::runtime_error("disable status unconfirmed; motor state unknown");
    }
    enabled_ = false;
    disabled_confirmed_ = true;
    shutdown_unknown_ = false;
    enable_baseline_.clear();
  }
}

void DmSerialBackend::disconnect()
{
  // Never release a CAN owner while its state is unknown. A successful,
  // status-confirmed disable is required first; a failed maintenance zero
  // requires manual investigation and process restart.
  disable();
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_ || maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("cannot release USB owner while motor state is unknown");
  enabled_ = false;
  enable_baseline_.clear();
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
  for(const auto& motor : config_.motors) {
    // Snapshot per axis immediately before its query; an earlier axis's reply
    // must not make cached feedback for a later axis count as a new response.
    before.push_back(control_->getMotorFeedback(motor.can_id).rx_sequence);
    control_->refresh_motor_status(*control_->getMotor(motor.can_id));
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  std::vector<std::uint16_t> responded;
  do {
    responded.clear();
    for(std::size_t i = 0; i < config_.motors.size(); ++i)
      if(control_->getMotorFeedback(config_.motors[i].can_id).rx_sequence > before[i])
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
  if (maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("motor state unknown; MIT commands blocked");
  if(commands.size() != config_.motors.size())
  {
    throw std::invalid_argument("MIT command count does not match motor count");
  }

  // Validate the complete batch before emitting any CAN frame. Transmission
  // errors can still leave a prefix applied; CAN provides no batch transaction.
  std::vector<std::shared_ptr<damiao::Motor>> motors;
  motors.reserve(config_.motors.size());
  for(std::size_t i = 0; i < config_.motors.size(); ++i)
  {
    const auto motor = control_->getMotor(config_.motors[i].can_id);
    if(!motor)
    {
      throw std::runtime_error("motor is not registered");
    }

    const auto& command = commands[i];
    control_->validate_mit(*motor, static_cast<float>(command.kp), static_cast<float>(command.kd),
                           static_cast<float>(command.q), static_cast<float>(command.dq),
                           static_cast<float>(command.tau));
    motors.push_back(motor);
  }
  for(std::size_t i = 0; i < motors.size(); ++i) {
    const auto& command = commands[i];
    try {
      control_->control_mit(
        *motors[i], static_cast<float>(command.kp), static_cast<float>(command.kd),
        static_cast<float>(command.q), static_cast<float>(command.dq),
        static_cast<float>(command.tau));
    } catch (const std::exception& e) {
      shutdown_unknown_ = true;
      throw std::runtime_error("MIT batch send failed at CAN ID=" +
        std::to_string(config_.motors[i].can_id) + "; prior frames sent=" +
        std::to_string(i) + "; motor state unknown: " + e.what());
    }
  }
}

void DmSerialBackend::send_zero_mit_all()
{
  send_mit_all(std::vector<MitCommand>(config_.motors.size()));
}

void DmSerialBackend::validate_mit_all(const std::vector<MitCommand>& commands) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (commands.size() != config_.motors.size())
    throw std::invalid_argument("MIT command count does not match motor count");
  for (std::size_t i=0; i<commands.size(); ++i) {
    const auto& c = commands[i];
    const auto model = to_damiao_model(config_.motors[i].model);
    const auto limits = control_ ? control_->getMotor(config_.motors[i].can_id)->get_limit_param()
                                 : damiao::limit_param[model];
    const auto check = [](double value, double lo, double hi, int bits) {
      if (!std::isfinite(value) || value < lo || value > hi)
        throw std::out_of_range("MIT field value is outside motor limits");
      (void)damiao::encode_mit_field(static_cast<float>(value), static_cast<float>(lo),
                                     static_cast<float>(hi), static_cast<std::uint8_t>(bits));
    };
    check(c.kp, 0, 500, 12); check(c.kd, 0, 5, 12);
    check(c.q, -limits.Q_MAX, limits.Q_MAX, 16);
    check(c.dq, -limits.DQ_MAX, limits.DQ_MAX, 12);
    check(c.tau, -limits.TAU_MAX, limits.TAU_MAX, 12);
  }
}

void DmSerialBackend::set_zero(std::uint16_t can_id, bool persist)
{
  set_zero_many({can_id}, persist);
}

void DmSerialBackend::set_zero_all(bool persist)
{
  std::vector<std::uint16_t> can_ids;
  for (const auto& motor : config_.motors) can_ids.push_back(motor.can_id);
  set_zero_many(can_ids, persist);
}

void DmSerialBackend::set_zero_many(const std::vector<std::uint16_t>& can_ids, bool persist)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!control_) throw std::runtime_error("USB bus is not connected");
  if (maintenance_unknown_ || shutdown_unknown_)
    throw std::runtime_error("set_zero blocked while motor state is unknown");
  if (enabled_) throw std::runtime_error("set_zero requires disabled motors");
  if (can_ids.empty()) throw std::invalid_argument("set_zero target list must not be empty");
  std::set<std::uint16_t> selected;
  for (const auto can_id : can_ids) {
    if (std::none_of(config_.motors.begin(), config_.motors.end(),
                    [can_id](const auto& m) { return m.can_id == can_id; }))
      throw std::invalid_argument("motor is outside this owner's scope");
    if (!selected.insert(can_id).second)
      throw std::invalid_argument("duplicate set_zero CAN ID");
  }

  // Preflight the complete selection for both runtime and persistent zeros.
  verify_set_zero_ready(can_ids);
  std::size_t completed = 0;
  std::ostringstream completed_ids;
  bool attempted = false;
  for (const auto can_id : can_ids) {
    try {
      // Earlier writes take time: establish a new stationary window for this axis.
      verify_set_zero_ready({can_id});
      attempted = true; // Even a failed send may have reached the device.
      const auto motor = control_->getMotor(can_id);
      control_->set_zero_position(*motor);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (persist) {
        control_->save_motor_param(*motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
      if (completed > 0) completed_ids << ',';
      completed_ids << can_id;
      ++completed;
    }
    catch (const std::exception& e) {
      if (attempted) maintenance_unknown_ = true;
      throw std::runtime_error("set_zero failed at CAN ID=" + std::to_string(can_id) +
        "; completed=" + std::to_string(completed) +
        "; completed IDs=[" + completed_ids.str() + "]" +
        (attempted ? "; coordinate/state unknown: " : "; no coordinates changed: ") + e.what());
    }
    catch (...) {
      if (attempted) maintenance_unknown_ = true;
      throw;
    }
  }
}

void DmSerialBackend::verify_set_zero_ready(const std::vector<std::uint16_t>& can_ids) const
{
  using clock = std::chrono::steady_clock;
  struct Observation {
    std::uint64_t sequence;
    unsigned samples{0};
    clock::time_point first{}, last{};
    double minimum{std::numeric_limits<double>::infinity()};
    double maximum{-std::numeric_limits<double>::infinity()};
  };
  std::vector<Observation> observations;
  for (const auto id : can_ids)
    observations.push_back({control_->getMotorFeedback(id).rx_sequence});
  const auto deadline = clock::now() + std::chrono::seconds(1);
  auto next_query = clock::now();
  std::uint16_t missing = can_ids.front();
  while (clock::now() < deadline) {
    bool all_ready = true;
    for (std::size_t i = 0; i < can_ids.size(); ++i) {
      const auto id = can_ids[i];
      auto& observed = observations[i];
      const auto f = control_->getMotorFeedback(id);
      if (f.last_rx_age_s > 0.2) {
        observed.samples = 0;
        observed.minimum = std::numeric_limits<double>::infinity();
        observed.maximum = -std::numeric_limits<double>::infinity();
      }
      if (f.rx_sequence > observed.sequence && f.last_rx_age_s <= 0.2) {
        observed.sequence = f.rx_sequence;
        if (f.error_code != 0 || !std::isfinite(f.position) ||
            !std::isfinite(f.velocity) || !std::isfinite(f.torque) || std::abs(f.velocity) > 0.05)
          throw std::runtime_error("set_zero requires stationary status-0 feedback: CAN ID=" + std::to_string(id));
        const auto received = clock::now() - std::chrono::duration_cast<clock::duration>(
          std::chrono::duration<double>(f.last_rx_age_s));
        observed.minimum = std::min(observed.minimum, static_cast<double>(f.position));
        observed.maximum = std::max(observed.maximum, static_cast<double>(f.position));
        if (observed.maximum - observed.minimum > 0.005)
          throw std::runtime_error("set_zero position drift: CAN ID=" + std::to_string(id));
        // Distinct frames in one USB reception batch do not prove stability.
        if (observed.samples == 0 || received - observed.last >= std::chrono::milliseconds(20)) {
          if (observed.samples++ == 0) observed.first = received;
          observed.last = received;
        }
      }
      const bool ready = f.last_rx_age_s <= 0.2 && observed.samples >= 3 &&
        observed.last - observed.first >= std::chrono::milliseconds(300);
      if (!ready) {
        if (all_ready) missing = id;
        all_ready = false;
      }
    }
    if (all_ready) return;
    if (clock::now() >= next_query) {
      for (const auto id : can_ids) control_->refresh_motor_status(*control_->getMotor(id));
      next_query = clock::now() + std::chrono::milliseconds(50);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("set_zero stationary feedback timeout: CAN ID=" + std::to_string(missing));
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
  for(std::size_t i = 0; i < config_.motors.size(); ++i)
  {
    const auto& motor_config = config_.motors[i];
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
      feedback.feedback_hz,
      feedback.error_code,
      enabled_ && i < enable_baseline_.size() && feedback.rx_sequence > enable_baseline_[i] && feedback.error_code == 1});
  }

  return result;
}

std::vector<double> DmSerialBackend::verify_calibration_configuration() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!control_ || enabled_ || shutdown_unknown_ || maintenance_unknown_)
    throw std::runtime_error("calibration requires disabled motors with known state");
  // Encoding ranges come from the station-confirmed model configuration.
  return read_disabled_positions_locked();
}

std::vector<double> DmSerialBackend::read_disabled_positions_locked(const std::vector<double>& expected) {
  constexpr double tolerance = 3.14159265358979323846 / 180;
  if (!expected.empty() && expected.size() != config_.motors.size())
    throw std::invalid_argument("angle readback target count mismatch");
  std::vector<std::uint64_t> before;
  for (const auto& m : config_.motors)
    before.push_back(control_->getMotorFeedback(m.can_id).rx_sequence);
  std::vector<double> positions(config_.motors.size());
  std::vector<bool> received(config_.motors.size(), false);
  auto next_query = std::chrono::steady_clock::now();
  const auto deadline = next_query + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    for (std::size_t i=0; i<config_.motors.size(); ++i) {
      if (received[i]) continue;
      const auto& m = config_.motors[i];
      const auto feedback = control_->getMotorFeedback(m.can_id);
      if (feedback.rx_sequence <= before[i] || feedback.last_rx_age_s > .2) continue;
      if (feedback.error_code != 0 || !std::isfinite(feedback.position))
        throw std::runtime_error("disabled angle feedback invalid: CAN ID=" + std::to_string(m.can_id));
      if (!expected.empty() && std::abs(feedback.position - expected[i]) >= tolerance)
        throw std::runtime_error("angle readback position mismatch: CAN ID=" + std::to_string(m.can_id));
      positions[i] = feedback.position;
      received[i] = true;
    }
    if (std::chrono::steady_clock::now() >= deadline) break;
    if (std::all_of(received.begin(), received.end(), [](bool value){return value;})) return positions;
    if (std::chrono::steady_clock::now() >= next_query) {
      for (std::size_t i=0; i<config_.motors.size(); ++i)
        if (!received[i]) control_->refresh_motor_status(*control_->getMotor(config_.motors[i].can_id));
      next_query = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("angle readback timed out: missing motor feedback within 1s window");
}

void DmSerialBackend::commit_calibration(const std::vector<double>& zeros,
    const std::atomic<bool>& cancel,
    const std::function<void(std::size_t, const std::string&)>& progress) {
  std::lock_guard<std::mutex> lock(mutex_);
  constexpr double tolerance = 3.14159265358979323846 / 180;
  if (!control_ || enabled_ || !disabled_confirmed_ || shutdown_unknown_ || maintenance_unknown_)
    throw std::runtime_error("zero commit requires confirmed disabled motors");
  if (zeros.size() != config_.motors.size() ||
      std::any_of(zeros.begin(), zeros.end(), [](double q){ return !std::isfinite(q); }))
    throw std::invalid_argument("invalid zero positions");
  auto expected = zeros;
  bool changed = false, checking_stability = true;
  std::vector<double> minimum(zeros.size(), std::numeric_limits<double>::infinity());
  std::vector<double> maximum(zeros.size(), -std::numeric_limits<double>::infinity());
  std::vector<std::chrono::steady_clock::time_point> first_seen(zeros.size());
  std::vector<double> observed_for(zeros.size(), 0);
  auto check = [&] {
    if (cancel) throw std::runtime_error("zero calibration cancelled");
    const auto positions = read_disabled_positions_locked(expected);
    for (std::size_t i=0; i<expected.size(); ++i) {
      if (cancel) throw std::runtime_error("zero calibration cancelled");
      const auto& m = config_.motors[i];
      const double actual = positions[i];
      if (checking_stability) {
        const auto now = std::chrono::steady_clock::now();
        if (std::isinf(minimum[i])) first_seen[i] = now;
        observed_for[i] = std::chrono::duration<double>(now-first_seen[i]).count();
        minimum[i] = std::min(minimum[i], actual); maximum[i] = std::max(maximum[i], actual);
      }
      if (std::abs(actual - expected[i]) >= tolerance || maximum[i]-minimum[i] >= tolerance)
        throw std::runtime_error("disabled zero position drift: CAN ID=" + std::to_string(m.can_id));
    }
    if (cancel) throw std::runtime_error("zero calibration cancelled");
  };
  try {
    do { check(); std::this_thread::sleep_for(std::chrono::milliseconds(20)); }
    while (std::any_of(observed_for.begin(), observed_for.end(), [](double t){return t < 1;}));
    checking_stability = false;
    for (std::size_t i=0; i<expected.size(); ++i) {
      // Recheck the entire station after each axis changes coordinate systems.
      check();
      auto motor = control_->getMotor(config_.motors[i].can_id);
      progress(i, "UNKNOWN"); changed = true;
      control_->set_zero_position(*motor);
      control_->save_motor_param(*motor);
      expected[i] = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    check();
    for (std::size_t i=0; i<expected.size(); ++i) progress(i, "SAVED");
  } catch (...) {
    if (changed) maintenance_unknown_ = true;
    throw;
  }
}
}  // namespace dm_openarm::backend
