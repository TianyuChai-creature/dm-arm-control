#pragma once
#include "dm_openarm/types.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_openarm {
// The same deterministic step runs in the MIT worker and in offline fault tests.
class CommandGuard {
public:
  double command_timeout{0.1}, feedback_timeout{0.2}, startup_timeout{2.5};
  bool enabled{false}, latched{false};
  bool preserve_command{false};
  std::string state{"DISABLED"}, fault;
  std::vector<MitCommand> holding, gains;
  std::vector<bool> seeded;
  double updated{0}, started{0}, held_at{0};
  std::uint64_t accepted_seq{0}, sent_seq{0};

  static void validate(const std::vector<MitCommand>& commands) {
    for (const auto& c : commands)
      if (!std::isfinite(c.q) || !std::isfinite(c.dq) || !std::isfinite(c.tau) ||
          !std::isfinite(c.kp) || !std::isfinite(c.kd) || c.kp < 0 || c.kp > 500 || c.kd < 0 || c.kd > 5)
        throw std::invalid_argument("invalid MIT command");
  }
  void start(std::vector<MitCommand> hold_gains, double now) {
    validate(hold_gains);
    if (!(command_timeout > 0 && feedback_timeout > 0 && startup_timeout > 0) ||
        !std::isfinite(command_timeout + feedback_timeout + startup_timeout))
      throw std::invalid_argument("invalid safety timing");
    for (const auto& c : hold_gains)
      if (c.kp <= 0) throw std::invalid_argument("seed hold requires positive kp");
    gains = std::move(hold_gains); holding.assign(gains.size(), {});
    seeded.assign(gains.size(), false); enabled = true; latched = false;
    preserve_command = false;
    fault.clear(); state = "STARTING"; started = updated = held_at = now;
    accepted_seq = sent_seq = 0;
  }
  static bool fresh(const MotorState& s, double age) {
    return s.feedback_fresh(age) && s.raw_status == 1 && s.enabled_confirmed &&
           s.error_code == 0 && std::isfinite(s.position) &&
           std::isfinite(s.velocity) && std::isfinite(s.torque);
  }
  void hold(const std::vector<MotorState>& states, const std::vector<MitCommand>& commands,
            double now, const std::string& reason = "") {
    preserve_command = false;
    if (states.size() != gains.size()) throw std::runtime_error("feedback count mismatch");
    for (std::size_t i=0; i<states.size(); ++i) {
      if (!fresh(states[i], feedback_timeout)) throw std::runtime_error("feedback invalid during hold");
      holding[i] = gains[i]; holding[i].q = states[i].position;
      holding[i].dq = 0; holding[i].tau = commands[i].tau;
    }
    held_at = now; state = "HOLD";
    if (!reason.empty()) { fault = reason; latched = true; state = "FAULT_HOLD"; }
  }
  void hold_command(const std::vector<MotorState>& states,
                    const std::vector<MitCommand>& commands, double now) {
    if (!enabled || latched || (state != "ACTIVE" && state != "HOLD"))
      throw std::runtime_error("command hold requires healthy position control");
    if (states.size() != gains.size() || commands.size() != gains.size())
      throw std::runtime_error("command hold count mismatch");
    validate(commands);
    for (std::size_t i=0; i<states.size(); ++i)
      if (!fresh(states[i], feedback_timeout) || commands[i].kp <= 0 || commands[i].dq != 0)
        throw std::runtime_error("command hold requires fresh feedback, positive kp and zero dq");
    holding = commands; held_at = now; state = "HOLD"; preserve_command = true;
  }
  void accept(const std::vector<MitCommand>& commands, double now) {
    validate(commands);
    if (enabled && (latched || state == "STARTING" || state == "FAULT"))
      throw std::runtime_error("MIT safety latched or not ready");
    updated = now; ++accepted_seq;
    preserve_command = false;
    if (enabled) state = "ACTIVE";
  }
  std::vector<MitCommand> step(const std::vector<MotorState>& states,
                               const std::vector<MitCommand>& commands, double now) {
    if (!enabled) return commands;
    if (states.size() != gains.size()) throw std::runtime_error("feedback count mismatch");
    if (state == "STARTING") {
      for (std::size_t i=0; i<states.size(); ++i) {
        if (states[i].error_code != 0) throw std::runtime_error("motor fault during startup: CAN ID=" + std::to_string(states[i].can_id) + " error_code=" + std::to_string(states[i].error_code));
        if (!seeded[i] && fresh(states[i], feedback_timeout)) {
          holding[i] = gains[i]; holding[i].q = states[i].position;
          holding[i].dq = holding[i].tau = 0; seeded[i] = true;
        }
        if (seeded[i] && !fresh(states[i], feedback_timeout))
          throw std::runtime_error("seeded motor feedback lost");
      }
      if (std::all_of(seeded.begin(), seeded.end(), [](bool v){return v;})) {
        state = "HOLD"; held_at = now;
      } else if (now-started >= startup_timeout) throw std::runtime_error("startup feedback timeout");
      return holding;
    }
    for (const auto& s : states)
      if (!fresh(s, feedback_timeout)) throw std::runtime_error("motor feedback stale, invalid or faulted");
    if (state == "ACTIVE" && now-updated >= command_timeout)
      hold(states, commands, now, "command_timeout");
    if (state == "HOLD" || state == "FAULT_HOLD") {
      auto out = holding;
      // Fixed position plus validated holding gains supports the load as old FF fades.
      const double alpha = preserve_command ? 1.0 : std::max(0.0, 1.0-(now-held_at));
      for (auto& c : out) c.tau *= alpha;
      return out;
    }
    return commands;
  }
};
}
