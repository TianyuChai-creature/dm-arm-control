#include "dm_openarm/mit_loop_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>
#include <set>

namespace dm_openarm {
namespace {
double monotonic_seconds() {
 return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string exception_text(std::exception_ptr p) {
  try { if (p) std::rethrow_exception(p); }
  catch (const std::exception& e) { return e.what(); }
  catch (...) { return "unknown exception"; }
  return {};
}
void validate_frequency(double hz) {
  if (!std::isfinite(hz) || hz < kMinControlHz || hz > kMaxControlHz ||
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / hz)).count() <= 0)
    throw std::invalid_argument("MIT loop frequency outside bounded scheduler range");
}
}


MitLoopController::MitLoopController(DmArm& arm)
  : arm_(arm)
{
  const auto& cfg = arm_.config();
  const auto& motors = cfg.motors;
  can_ids_.reserve(motors.size());
  commands_.resize(motors.size());
  sent_commands_.resize(motors.size());
  for(const auto& motor : motors)
  {
    can_ids_.push_back(motor.can_id);
  }

}

MitLoopController::~MitLoopController()
{
  try
  {
    stop();
  }
  catch(...)
  {
  }
}

void MitLoopController::require_idle() const {
  if (calibration_active_ || calibration_executing_) throw std::runtime_error("zero calibration owns this loop");
}

void MitLoopController::start(double hz) {
  require_idle();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (guard_.latched) throw std::runtime_error("fault latched; use a fresh guarded startup");
    if (guard_.enabled && !running_)
      throw std::runtime_error("incomplete guarded startup; stop before a legacy start");
  }
  start_impl(hz, false);
}

void MitLoopController::start_impl(double hz, bool calibration)
{
  validate_frequency(hz);
  if(running_)
  {
    return;
  }

  if(worker_.joinable())
  {
    worker_.join();
  }

  if(worker_exception_)
  {
    auto exception = worker_exception_;
    worker_exception_ = nullptr;
    std::rethrow_exception(exception);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_tx_hz_ = hz;
    tx_cycles_ = 0;
    tx_rate_.reset();
  }
  deadline_misses_ = 0;
  arm_.set_loop_active(true, calibration);
  running_ = true;
  try { worker_ = std::thread([this, hz]() { worker_loop(hz); }); }
  catch (...) { running_ = false; arm_.set_loop_active(false); throw; }
}

void MitLoopController::enable_seeded(std::vector<MitCommand> gains, double hz,
                                          double command_timeout, double feedback_timeout) {
  require_idle();
  validate_frequency(hz);
  if (running_) throw std::invalid_argument("MIT loop already running");
  if (gains.size() != commands_.size()) throw std::invalid_argument("hold gain count mismatch");
  CommandGuard validation;
  validation.command_timeout = command_timeout; validation.feedback_timeout = feedback_timeout;
  validation.start(gains, monotonic_seconds()); // validate before energizing
  try {
    arm_.enable();
    start_seeded(std::move(gains), hz, command_timeout, feedback_timeout);
    std::unique_lock<std::mutex> lock(mutex_);
    const bool confirmed = changed_.wait_until(lock, hold_deadline_, [this] {
      return hold_sent_ >= 1 || !running_;
    });
    if (!confirmed || !running_ || guard_.state != "HOLD" || hold_sent_ < 1)
      throw std::runtime_error("native startup failed: " +
        (guard_.fault.empty() ? "first hold batch unconfirmed" : guard_.fault));
  } catch (...) {
    const auto original = std::current_exception();
    std::string stop_error;
    try { stop(); } catch (...) { stop_error = exception_text(std::current_exception()); }
    try { arm_.disable(); }
    catch (...) {
      const auto message = exception_text(original) + "; shutdown unknown: " +
                           exception_text(std::current_exception()) +
                           (stop_error.empty() ? "" : "; loop stop failed: " + stop_error);
      { std::lock_guard<std::mutex> lock(mutex_);
        guard_.state = "SHUTDOWN_UNKNOWN"; guard_.fault = message; guard_.latched = true; }
      throw std::runtime_error(message);
    }
    if (!stop_error.empty())
      throw std::runtime_error(exception_text(original) + "; loop stop failed: " + stop_error);
    std::rethrow_exception(original);
  }
}

void MitLoopController::start_seeded(std::vector<MitCommand> gains, double hz,
                                         double command_timeout, double feedback_timeout)
{
  require_idle();
  validate_frequency(hz);
  if (running_) throw std::runtime_error("MIT loop already running");
  if (gains.size() != commands_.size()) throw std::invalid_argument("hold gain count mismatch");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    guard_.command_timeout = command_timeout; guard_.feedback_timeout = feedback_timeout;
    guard_.start(std::move(gains), monotonic_seconds());
    commands_ = guard_.holding;
    hold_request_ = 1; hold_sent_ = 0;
    hold_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  }
  start_impl(hz, false);
}
std::vector<MitCommand> MitLoopController::hold_command() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!running_) throw std::runtime_error("MIT loop not running");
  require_idle();
  if (hold_request_ != hold_sent_) throw std::runtime_error("hold confirmation pending");
  guard_.hold_command(arm_.states(), sent_commands_, monotonic_seconds());
  commands_ = guard_.holding;
  const auto result = commands_;
  wait_for_hold(lock, ++hold_request_);
  return result;
}

void MitLoopController::hold(bool reset_fault) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!running_ || !guard_.enabled) throw std::runtime_error("guarded MIT loop not running");
  if (guard_.state == "STARTING") throw std::runtime_error("MIT startup incomplete");
  require_idle();
  if (hold_request_ != hold_sent_) throw std::runtime_error("hold confirmation pending");
  guard_.hold(arm_.states(), sent_commands_, monotonic_seconds(), guard_.latched ? guard_.fault : "");
  if (reset_fault) { guard_.latched = false; guard_.fault.clear(); guard_.state = "HOLD"; }
  commands_ = guard_.holding;
  wait_for_hold(lock, ++hold_request_);
}

void MitLoopController::wait_for_hold(std::unique_lock<std::mutex>& lock, std::uint64_t request) {
  hold_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(guard_.feedback_timeout));
  changed_.notify_all();
  const bool confirmed = changed_.wait_until(lock, hold_deadline_,
    [this, request] { return hold_sent_ >= request || !running_; });
  if (confirmed && running_ && hold_sent_ >= request) return;
  const auto message = guard_.fault.empty() ? "hold batch unconfirmed" : guard_.fault;
  if (!confirmed && running_) {
    guard_.state = "FAULT"; guard_.fault = message; guard_.latched = true;
  }
  lock.unlock();
  try { stop(); }
  catch (...) { throw std::runtime_error(message + "; stop failed: " + exception_text(std::current_exception())); }
  throw std::runtime_error(message);
}
std::string MitLoopController::safety_state() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.state; }
std::string MitLoopController::fault() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.fault; }
std::uint64_t MitLoopController::accepted_sequence() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.accepted_seq; }
std::uint64_t MitLoopController::sent_sequence() const { std::lock_guard<std::mutex> lock(mutex_); return guard_.sent_seq; }

void MitLoopController::stop() {
  calibration_cancelled_ = true;
  stop_impl();
}

void MitLoopController::finish_stop(const std::string& failure, bool shutdown_unknown) {
  std::lock_guard<std::mutex> lock(mutex_);
  guard_.enabled = false;
  guard_.preserve_command = false;
  guard_.holding.clear(); guard_.gains.clear(); guard_.seeded.clear();
  commands_.assign(can_ids_.size(), {});
  hold_request_ = hold_sent_ = 0;
  if (!failure.empty()) {
    if (guard_.fault.empty() || failure.find(guard_.fault) != std::string::npos)
      guard_.fault = failure;
    else if (guard_.fault.find(failure) == std::string::npos)
      guard_.fault += "; " + failure;
  }
  if (shutdown_unknown || guard_.state == "SHUTDOWN_UNKNOWN")
    guard_.state = "SHUTDOWN_UNKNOWN";
  else
    guard_.state = guard_.latched || !guard_.fault.empty() ? "FAULT" : "DISABLED";
  guard_.latched = guard_.state != "DISABLED";
}

void MitLoopController::shutdown_failed(const std::string& reason) {
  finish_stop(reason, true);
}

void MitLoopController::stop_impl()
{
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  const bool was_running = running_.exchange(false);
  changed_.notify_all();
  if(worker_.joinable())
  {
    worker_.join();
  }

  std::exception_ptr stop_exception;
  std::exception_ptr disable_exception;
  if(was_running)
  {
    try
    {
      arm_.backend_.send_zero_mit_all();
    }
    catch(...)
    {
      stop_exception = std::current_exception();
    }
    // A zero-torque MIT frame does not disable the motor. Always send FD and
    // require fresh status-0 feedback before reporting a safe loop stop.
    try { arm_.disable_for_loop(); }
    catch(...) { disable_exception = std::current_exception(); }
  }

  if (was_running) arm_.set_loop_active(false);
  if (!calibration_executing_) release_calibration();

  const auto worker_exception = worker_exception_;
  worker_exception_ = nullptr;
  std::string failure = exception_text(worker_exception);
  if (stop_exception) failure += (failure.empty() ? "" : "; ") +
    std::string("zero send failed: ") + exception_text(stop_exception);
  if (disable_exception) failure += (failure.empty() ? "" : "; ") +
    std::string("shutdown unknown: ") + exception_text(disable_exception);
  finish_stop(failure, static_cast<bool>(disable_exception));
  if (!failure.empty()) throw std::runtime_error(failure);
}

bool MitLoopController::running() const noexcept
{
  return running_;
}

std::uint64_t MitLoopController::deadline_misses() const noexcept
{
  return deadline_misses_.load();
}

TimingStats MitLoopController::timing_stats() const
{
  TimingStats result;
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result.running = running_.load();
    result.target_tx_hz = target_tx_hz_;
    result.actual_tx_hz = tx_rate_.hz(now);
    result.tx_cycles = tx_cycles_;
    result.deadline_misses = deadline_misses_.load();
  }

  result.connected = arm_.connected();
  if(!result.connected)
  {
    result.motors.reserve(can_ids_.size());
    for(const auto can_id : can_ids_)
    {
      result.motors.push_back(MotorTimingStats{can_id});
    }
    return result;
  }

  std::vector<MotorState> states;
  try
  {
    states = arm_.states();
  }
  catch(...)
  {
    if(arm_.connected())
    {
      throw;
    }
    result.connected = false;
    result.motors.reserve(can_ids_.size());
    for(const auto can_id : can_ids_)
    {
      result.motors.push_back(MotorTimingStats{can_id});
    }
    return result;
  }

  result.motors.reserve(states.size());
  for(const auto& state : states)
  {
    result.motors.push_back(MotorTimingStats{
      state.can_id,
      state.feedback_hz,
      state.feedback_interval_s,
      state.last_rx_age_s,
      state.rx_sequence});
  }
  return result;
}

void MitLoopController::set_command(std::uint16_t can_id, MitCommand command)
{
  set_commands({{can_id, command}});
}

void MitLoopController::set_commands(const std::map<std::uint16_t, MitCommand>& commands)
{
  if (commands.empty()) throw std::invalid_argument("MIT update must not be empty");
  std::lock_guard<std::mutex> lock(mutex_);
  require_idle();
  if (hold_request_ != hold_sent_) throw std::runtime_error("hold confirmation pending");
  const auto now = monotonic_seconds();
  auto updated = (guard_.state == "HOLD" || guard_.state == "FAULT_HOLD")
    ? guard_.step(arm_.states(), commands_, now) : commands_;
  for (const auto& entry : commands) updated[motor_index(entry.first)] = entry.second;
  arm_.validate_mit_all(updated);
  guard_.accept(updated, now);
  commands_ = std::move(updated);
}

void MitLoopController::set_all_commands(std::vector<MitCommand> commands)
{
  if (commands.empty()) throw std::invalid_argument("MIT update must not be empty");
  if(commands.size() != commands_.size())
  {
    throw std::invalid_argument("MIT command count does not match motor count");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  require_idle();
  if (hold_request_ != hold_sent_) throw std::runtime_error("hold confirmation pending");
  arm_.validate_mit_all(commands);
  guard_.accept(commands, monotonic_seconds());
  commands_ = std::move(commands);
}

std::vector<MitCommand> MitLoopController::commands() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return commands_;
}

std::vector<MitCommand> MitLoopController::sent_commands() const {
  std::lock_guard<std::mutex> lock(mutex_); return sent_commands_;
}

std::size_t MitLoopController::motor_index(std::uint16_t can_id) const
{
  for(std::size_t i = 0; i < can_ids_.size(); ++i)
  {
    if(can_ids_[i] == can_id)
    {
      return i;
    }
  }

  throw std::invalid_argument("unknown motor CAN ID");
}

void MitLoopController::worker_loop(double hz)
{
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  auto next_tick = clock::now();

  try
  {
    while(running_)
    {
      {
        // Keep a snapshot and its complete send on the same side of a hold request.
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) break;
        const auto states = guard_.enabled ? arm_.states() : std::vector<MotorState>{};
        const auto snapshot = guard_.step(states, commands_, monotonic_seconds());
        const bool holding = guard_.enabled &&
          (guard_.state == "HOLD" || guard_.state == "FAULT_HOLD");
        if (guard_.enabled && guard_.state != "ACTIVE") commands_ = snapshot;
        const auto sending_seq = (!guard_.enabled || guard_.state == "ACTIVE") ? guard_.accepted_seq : 0;
        arm_.backend_.send_mit_all(snapshot);
        const auto sent_at = clock::now();
        if (holding && hold_request_ != hold_sent_ && sent_at > hold_deadline_)
          throw std::runtime_error("hold confirmation timeout after batch submission");
        tx_rate_.record(sent_at);
        ++tx_cycles_;
        guard_.sent_seq = sending_seq;
        sent_commands_ = snapshot;
        if (holding) hold_sent_ = hold_request_;
        changed_.notify_all();
      }

      next_tick += std::chrono::duration_cast<clock::duration>(period);
      const auto now = clock::now();
      if(now > next_tick)
      {
        ++deadline_misses_;
        next_tick = now + std::chrono::duration_cast<clock::duration>(period);
      }
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait_until(lock, next_tick, [this] {
        return !running_ || (guard_.state != "STARTING" && hold_request_ != hold_sent_);
      });
    }
  }
  catch(...)
  {
    const auto exception = std::current_exception();
    running_ = false;
    std::string cleanup_error;
    bool shutdown_unknown = false;
    try
    {
      arm_.backend_.send_zero_mit_all();
    }
    catch(...) { cleanup_error = "zero send failed: " + exception_text(std::current_exception()); }
    try
    {
      arm_.disable_for_loop();
    }
    catch(...) {
      shutdown_unknown = true;
      cleanup_error += (cleanup_error.empty() ? "" : "; ");
      cleanup_error += "shutdown unknown: " + exception_text(std::current_exception());
    }
    finish_stop(exception_text(exception) + (cleanup_error.empty() ? "" : "; " + cleanup_error),
                shutdown_unknown);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_exception_ = cleanup_error.empty() ? exception :
        std::make_exception_ptr(std::runtime_error(exception_text(exception) + "; " + cleanup_error));
    }
    arm_.set_loop_active(false);
    changed_.notify_all();
  }
}

void MitLoopController::release_calibration() {
  if (calibration_active_) {
    arm_.end_calibration();
    calibration_active_ = false;
  }
}

void MitLoopController::begin_calibration(std::vector<MitCommand> gains) {
  CommandGuard validation;
  validation.start(gains, monotonic_seconds());
  const auto& c = arm_.config();
  if (gains.size()!=10 || c.motors.size()!=10 || c.left.begin!=0 || c.left.count!=5 ||
      c.right.begin!=5 || c.right.count!=5)
    throw std::invalid_argument("calibration requires the complete dual-arm station");
  std::set<unsigned> ids;
  for (std::size_t i=0;i<10;++i) {
    const auto& m=c.motors[i];
    if (m.can_id>=10 || !ids.insert(m.can_id).second || (i<5)!=(m.can_id<5) ||
        m.model!=(m.can_id%5<2 ? MotorModel::DM8009 : MotorModel::DM4340P) ||
        gains[i].kd<=0 || gains[i].dq!=0 || gains[i].tau!=0)
      throw std::invalid_argument("invalid calibration station model/ID/gains");
  }
  bool expected=false;
  if (!calibration_executing_.compare_exchange_strong(expected,true))
    throw std::runtime_error("calibration already executing");
  bool owned=false;
  try {
    if (running_ && calibration_active_) {
      std::uint64_t request;
      {
        std::lock_guard<std::mutex> stop_lock(stop_mutex_);
        std::lock_guard<std::mutex> lock(mutex_);
        if (guard_.state!="FAULT_HOLD" || !guard_.enabled ||
            calibration_saves_.size()!=10 || std::any_of(calibration_saves_.begin(),calibration_saves_.end(),
              [](const std::string& s){return s!="NOT_ATTEMPTED";}))
          throw std::runtime_error("calibration retry requires an unwritten healthy fault hold");
        const auto states=arm_.states();
        for(const auto& state:states)
          if(!CommandGuard::fresh(state,.3)) throw std::runtime_error("calibration retry feedback invalid");
        calibration_cancelled_=false;
        guard_.start(gains,monotonic_seconds());
        guard_.hold(states,gains,monotonic_seconds());
        commands_=guard_.holding;
        request=++hold_request_;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      wait_for_hold(lock,request);
      return;
    }
    if (running_ || calibration_active_) throw std::runtime_error("stop MIT loop before calibration");
    stop_impl();
    calibration_cancelled_=false;
    const auto positions=arm_.begin_calibration();
    owned=true; calibration_active_=true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      calibration_saves_.assign(10,"NOT_ATTEMPTED");
      guard_.command_timeout=.5;guard_.feedback_timeout=.3;
      guard_.start(std::move(gains),monotonic_seconds());
      commands_=guard_.holding;
      hold_request_=1;hold_sent_=0;
      hold_deadline_=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    }
    {
      std::lock_guard<std::mutex> stop_lock(stop_mutex_);
      if (calibration_cancelled_) throw std::runtime_error("zero calibration cancelled");
      arm_.backend_.enable();
      start_impl(250,true);
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (!changed_.wait_until(lock,hold_deadline_,[this]{return hold_sent_>=1 || !running_;}) ||
        !running_ || guard_.state!="HOLD")
      throw std::runtime_error("calibration startup failed: "+guard_.fault);
    const auto states=arm_.states();
    for (std::size_t i=0;i<10;++i)
      if (std::abs(states[i].position-positions[i])>.02)
        throw std::runtime_error("calibration startup position mismatch");
  } catch (...) {
    const auto original=std::current_exception();
    std::string cleanup;
    if (owned) {
      try { stop_impl(); arm_.backend_.disable(); }
      catch (...) { cleanup=exception_text(std::current_exception()); }
    }
    calibration_executing_=false;
    if (owned && !running_) release_calibration();
    if (!cleanup.empty()) throw std::runtime_error(exception_text(original)+"; cleanup: "+cleanup);
    std::rethrow_exception(original);
  }
}

void MitLoopController::calibration_update(std::vector<MitCommand> commands) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!calibration_executing_ || !calibration_active_ || calibration_cancelled_ || !running_)
    throw std::runtime_error("zero calibration cancelled or stopped");
  if (guard_.latched) throw std::runtime_error("calibration control failed: "+guard_.fault);
  if (commands.size()!=commands_.size()) throw std::invalid_argument("calibration command count mismatch");
  for (const auto& c:commands)
    if (c.kp<=0 || c.kd<=0 || c.dq!=0 || c.tau!=0)
      throw std::invalid_argument("calibration requires position MIT without feedforward");
  arm_.validate_mit_all(commands);
  guard_.accept(commands,monotonic_seconds());
  guard_.gains=commands;
  commands_=std::move(commands);
}

void MitLoopController::calibration_fail(const std::string& reason) {
  if (!calibration_active_ || !running_) return;
  std::unique_lock<std::mutex> lock(mutex_);
  // Keep an already latched hold anchor, rather than repeatedly reseeding it.
  if (!guard_.latched) {
    guard_.hold(arm_.states(),commands_,monotonic_seconds(),reason);
    commands_=guard_.holding;
  }
  wait_for_hold(lock,++hold_request_);
}

void MitLoopController::calibration_commit(const std::vector<double>& zeros) {
  if (!calibration_active_ || !calibration_executing_)
    throw std::runtime_error("no calibration owns this loop");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || guard_.latched || calibration_cancelled_)
      throw std::runtime_error("calibration control is not ready to commit: "+guard_.fault);
    const auto states=arm_.states();
    if (zeros.size()!=states.size()) throw std::invalid_argument("zero position count mismatch");
    for (std::size_t i=0;i<states.size();++i)
      if (!std::isfinite(zeros[i]) || !CommandGuard::fresh(states[i],.3) ||
          std::abs(states[i].position-zeros[i])>=3.14159265358979323846/180)
        throw std::runtime_error("calibration zero is not ready to commit");
  }
  try {
    stop_impl();
    arm_.backend_.commit_calibration(zeros,calibration_cancelled_,[&](std::size_t i,const std::string& state){
      std::lock_guard<std::mutex> lock(mutex_);calibration_saves_[i]=state;
    });
    std::lock_guard<std::mutex> lock(mutex_);
    guard_=CommandGuard{};commands_.assign(can_ids_.size(),{});sent_commands_=commands_;
    hold_request_=hold_sent_=0;
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (guard_.state!="SHUTDOWN_UNKNOWN") guard_.state="FAULT";
    guard_.latched=true;guard_.fault=exception_text(std::current_exception());
    throw;
  }
}

void MitLoopController::finish_calibration() {
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  calibration_executing_=false;
  if (!running_) release_calibration();
}
std::vector<std::string> MitLoopController::calibration_saves() const {
  std::lock_guard<std::mutex> lock(mutex_);return calibration_saves_;
}
}  // namespace dm_openarm
