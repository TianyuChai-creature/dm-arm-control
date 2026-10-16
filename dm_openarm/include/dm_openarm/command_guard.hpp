#pragma once
#include "dm_openarm/types.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_openarm {
// One deterministic policy, shared by the native worker and offline clock tests.
class CommandGuard {
public:
  double command_timeout{0.1}, feedback_timeout{0.2}, feedback_hard_timeout{0.5};
  double hold_timeout{0.2}, startup_timeout{2.5};
  bool strict_timeouts{false}; // Internal calibration policy; never a consumer toggle.
  bool enabled{false}, latched{false}, preserve_command{false}, feedback_paused{false};
  std::string state{"DISABLED"}, fault;
  std::vector<ControlIssue> warnings, faults;
  std::vector<MitCommand> holding, gains;
  std::vector<bool> seeded;
  double updated{0}, started{0}, held_at{0};
  std::uint64_t accepted_seq{0}, sent_seq{0};

  void configure(double command, double feedback, std::optional<double> hard, double hold_wait) {
    command_timeout=command; feedback_timeout=feedback; hold_timeout=hold_wait;
    feedback_hard_timeout=hard.value_or(std::max(0.5, 2.5*feedback));
  }
  static void validate(const std::vector<MitCommand>& commands) {
    for (const auto& c : commands)
      if (!std::isfinite(c.q) || !std::isfinite(c.dq) || !std::isfinite(c.tau) ||
          !std::isfinite(c.kp) || !std::isfinite(c.kd) || c.kp < 0 || c.kp > 500 || c.kd < 0 || c.kd > 5)
        throw std::invalid_argument("invalid MIT command");
  }
  void start(std::vector<MitCommand> hold_gains, double now) {
    validate(hold_gains);
    if (!(command_timeout>0 && feedback_timeout>0 && feedback_hard_timeout>feedback_timeout &&
          hold_timeout>0 && startup_timeout>0) ||
        !std::isfinite(command_timeout+feedback_timeout+feedback_hard_timeout+hold_timeout+startup_timeout))
      throw std::invalid_argument("invalid safety timing");
    for (const auto& c : hold_gains)
      if (c.kp<=0) throw std::invalid_argument("seed hold requires positive kp");
    gains=std::move(hold_gains); holding.assign(gains.size(), {});
    seeded.assign(gains.size(), false); enabled=true; latched=false;
    preserve_command=feedback_paused=false;
    fault.clear(); warnings.clear(); faults.clear(); state="STARTING";
    started=updated=held_at=now; accepted_seq=sent_seq=0;
  }
  static bool fresh(const MotorState& s, double age) {
    return s.feedback_fresh(age) && s.raw_status==1 && s.enabled_confirmed &&
           s.error_code==0 && std::isfinite(s.position) &&
           std::isfinite(s.velocity) && std::isfinite(s.torque);
  }
  bool warned(const std::string& code) const {
    return std::any_of(warnings.begin(),warnings.end(),[&](const auto& i){return i.code==code;});
  }
  void clear_warning(const std::string& code) {
    warnings.erase(std::remove_if(warnings.begin(),warnings.end(),
      [&](const auto& i){return i.code==code;}),warnings.end());
  }
  void warn(ControlIssue issue) {
    const auto it=std::find_if(warnings.begin(),warnings.end(),[&](const auto& i){
      return i.code==issue.code && i.can_id==issue.can_id;
    });
    if(it==warnings.end()) warnings.push_back(std::move(issue)); else *it=std::move(issue);
  }
  void record_fault(ControlIssue issue) {
    const auto duplicate=std::any_of(faults.begin(),faults.end(),[&](const auto& i){
      return i.code==issue.code && i.can_id==issue.can_id;
    });
    if(!duplicate) faults.push_back(std::move(issue));
  }
  [[noreturn]] void fail(const std::string& code, const std::string& message,
                         const MotorState* motor=nullptr) {
    ControlIssue issue{code,message};
    if(motor) { issue.can_id=motor->can_id; issue.age_s=motor->last_rx_age_s; issue.rx_sequence=motor->rx_sequence; }
    record_fault(issue); fault=message; latched=true; state="FAULT";
    throw ControlFailure(std::move(issue));
  }
  void check_feedback(const std::vector<MotorState>& states) {
    if(states.size()!=gains.size()) fail("FEEDBACK_INVALID","feedback count mismatch");
    clear_warning("FEEDBACK_STALE");
    for(const auto& s:states) {
      if(!s.rx_sequence || !std::isfinite(s.last_rx_age_s) || s.last_rx_age_s<0 ||
         !std::isfinite(s.position) || !std::isfinite(s.velocity) || !std::isfinite(s.torque))
        fail("FEEDBACK_INVALID","invalid motor feedback: CAN ID="+std::to_string(s.can_id),&s);
      if(s.error_code) fail("MOTOR_FAULT","motor fault: CAN ID="+std::to_string(s.can_id)+
                            " error_code="+std::to_string(s.error_code),&s);
      if(s.raw_status!=1 || !s.enabled_confirmed)
        fail("MOTOR_DISABLED","motor unexpectedly disabled: CAN ID="+std::to_string(s.can_id),&s);
      if((strict_timeouts && s.last_rx_age_s>feedback_timeout) || s.last_rx_age_s>=feedback_hard_timeout)
        fail("FEEDBACK_LOST","motor feedback lost: CAN ID="+std::to_string(s.can_id),&s);
      if(s.last_rx_age_s>feedback_timeout)
        warn({"FEEDBACK_STALE","motor feedback temporarily stale",s.can_id,s.last_rx_age_s,s.rx_sequence});
    }
  }
  void require_fresh(const std::vector<MotorState>& states) {
    check_feedback(states);
    if(warned("FEEDBACK_STALE")) throw FeedbackStaleError("fresh feedback required; holding without accepting motion");
  }
  void hold(const std::vector<MotorState>& states, const std::vector<MitCommand>& commands,
            double now, const std::string& reason="") {
    if(commands.size()!=gains.size()) fail("FEEDBACK_INVALID","feedback/command count mismatch");
    require_fresh(states); preserve_command=false;
    for(std::size_t i=0;i<states.size();++i) {
      holding[i]=gains[i]; holding[i].q=states[i].position;
      holding[i].dq=0; holding[i].tau=commands[i].tau;
    }
    held_at=now; state="HOLD";
    if(!reason.empty()) {
      fault=reason; latched=true; state="FAULT_HOLD";
      record_fault({reason=="command_timeout" ? "COMMAND_TIMEOUT" : "CALIBRATION_FAILED",reason});
    }
  }
  void hold_command(const std::vector<MotorState>& states,
                    const std::vector<MitCommand>& commands, double now) {
    if(!enabled || latched || (state!="ACTIVE" && state!="HOLD"))
      throw std::runtime_error("command hold requires healthy position control");
    if(commands.size()!=gains.size()) fail("FEEDBACK_INVALID","command hold count mismatch");
    validate(commands); require_fresh(states);
    for(const auto& c:commands)
      if(c.kp<=0 || c.dq!=0) throw std::runtime_error("command hold requires positive kp and zero dq");
    holding=commands; held_at=now; state="HOLD"; preserve_command=true;
  }
  void accept(const std::vector<MitCommand>& commands, double now) {
    validate(commands);
    if(latched || (enabled && (state=="STARTING" || state=="FAULT")))
      throw std::runtime_error("MIT safety latched or not ready");
    if(warned("FEEDBACK_STALE")) throw FeedbackStaleError("feedback stale; command not accepted");
    updated=now; ++accepted_seq; preserve_command=false; clear_warning("COMMAND_TIMEOUT");
    if(enabled) state="ACTIVE";
  }
  std::vector<MitCommand> hold_output(double now) const {
    auto out=holding;
    const double alpha=preserve_command ? 1.0 : std::max(0.0,1.0-(now-held_at));
    for(auto& c:out) c.tau*=alpha;
    return out;
  }
  void pause_output(const std::vector<MotorState>& states, const std::vector<MitCommand>& executed, double now) {
    holding=executed;
    for(std::size_t i=0;i<holding.size();++i) {
      if(holding[i].kp<=0 || holding[i].dq!=0) {
        const double tau=holding[i].tau;
        holding[i]=gains[i]; holding[i].q=states[i].position;
        holding[i].dq=0; holding[i].tau=tau;
      }
    }
    held_at=now; state="HOLD"; preserve_command=true;
  }
  std::vector<MitCommand> step(const std::vector<MotorState>& states,
      const std::vector<MitCommand>& commands, double now,
      const std::vector<MitCommand>& executed={}) {
    if(!enabled) return commands;
    if(state=="FAULT" && latched && !faults.empty()) throw ControlFailure(faults.front());
    if(states.size()!=gains.size()) fail("FEEDBACK_INVALID","feedback count mismatch");
    if(state=="STARTING") {
      for(std::size_t i=0;i<states.size();++i) {
        if(states[i].error_code) fail("MOTOR_FAULT","motor fault during startup: CAN ID="+
          std::to_string(states[i].can_id)+" error_code="+std::to_string(states[i].error_code),&states[i]);
        if(!seeded[i] && fresh(states[i],feedback_timeout)) {
          holding[i]=gains[i]; holding[i].q=states[i].position;
          holding[i].dq=holding[i].tau=0; seeded[i]=true;
        }
        if(seeded[i] && !fresh(states[i],feedback_timeout))
          fail("STARTUP_FAILED","seeded motor feedback lost",&states[i]);
      }
      if(std::all_of(seeded.begin(),seeded.end(),[](bool v){return v;})) {state="HOLD";held_at=now;}
      else if(now-started>=startup_timeout) fail("STARTUP_FAILED","startup feedback timeout");
      return holding;
    }
    check_feedback(states);
    const bool overdue=state=="ACTIVE" && now-updated>=command_timeout;
    const auto& sent=executed.size()==commands.size() ? executed : commands;
    if(strict_timeouts) {
      if(overdue) hold(states,commands,now,"command_timeout");
    } else {
      const bool stale=warned("FEEDBACK_STALE");
      if(stale && !feedback_paused) {
        // An explicit HOLD may already be awaiting its first submission.
        pause_output(states,state=="HOLD" ? hold_output(now) : sent,now);
      }
      feedback_paused=stale;
      if(overdue) {
        if(!stale) pause_output(states,sent,now);
        warn({"COMMAND_TIMEOUT","command_timeout; holding last valid output",std::nullopt,now-updated});
      }
      if(warned("COMMAND_TIMEOUT"))
        warn({"COMMAND_TIMEOUT","command_timeout; holding last valid output",std::nullopt,now-updated});
    }
    if(state=="HOLD" || state=="FAULT_HOLD") return hold_output(now);
    return commands;
  }
};
}
