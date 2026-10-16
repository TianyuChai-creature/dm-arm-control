#include "dm_openarm/command_guard.hpp"
#include <cassert>
#include <chrono>
#include "protocol/damiao.h"
#include <limits>
using namespace dm_openarm;
int main() {
  using rate_clock = damiao::detail::SlidingRate::clock;
  const auto t0 = rate_clock::time_point{};
  damiao::detail::SlidingRate rate;
  assert(rate.hz(t0) == 0.0);
  for(int i = 0; i <= 1000; ++i) {
    rate.record(t0 + std::chrono::microseconds(i * 1000));
  }
  assert(std::abs(rate.hz(t0 + std::chrono::seconds(1)) - 1000.0) < 0.000001);
  assert(rate.hz(t0 + std::chrono::seconds(3)) == 0.0);
  rate.reset();
  assert(rate.hz(t0) == 0.0);

  // Mode-write ACK 04 00 55 0A 01 00 00 00 decoded as the logged bogus pose.
  const double bogus_q = 85.0 / 65535 * 25 - 12.5;
  assert(std::abs(bogus_q - (-12.467574119567871)) < 0.000002);
  for (uint8_t opcode : {0x33, 0x55, 0xAA}) {
    for (uint16_t id : {0, 2, 4}) {
      uint8_t reply[8] = {static_cast<uint8_t>(id), 0, opcode, 10, 1, 0, 0, 0};
      assert(damiao::is_param_reply(reply, 8, id));
      assert(!damiao::is_param_reply(reply, 6, id));
      assert(!damiao::is_param_reply(reply, 8, id+1));
      reply[0] |= 0x10; // enabled feedback with position low byte equal to opcode
      assert(!damiao::is_param_reply(reply, 8, id));
    }
  }
  // Exercise the same status conversion as the backend before the native guard.
  for (int status=0; status<16; ++status) {
    assert(motor_error_code(status) == (status == 1 ? 0 : status));
    CommandGuard startup;
    std::vector<MitCommand> gain(1, MitCommand{60,4,0,0,0});
    startup.start(gain, 0);
    MotorState s{}; s.can_id=4; s.rx_sequence=1; s.last_rx_age_s=0.01;
    s.error_code=motor_error_code(status); s.raw_status=status; s.enabled_confirmed=(status==1);
    bool fault=false;
    try { startup.step({s}, gain, 0.01); }
    catch (const std::runtime_error& e) {
      fault=true;
      assert(std::string(e.what()).find("CAN ID=4 error_code=" + std::to_string(status)) != std::string::npos);
    }
    assert(fault == (status != 0 && status != 1));
    if(status==0) assert(startup.state=="STARTING");
  }
  const std::vector<MitCommand> gains(2,MitCommand{60,4,0,0,0});
  MotorState state{};state.position=.3;state.velocity=0;state.torque=1;
  state.rx_sequence=1;state.raw_status=1;state.enabled_confirmed=true;state.last_rx_age_s=.01;
  std::vector<MotorState> feedback(2,state);feedback[0].can_id=1;feedback[1].can_id=2;
  auto start=[&](CommandGuard& guard){guard.start(gains,0);guard.step(feedback,gains,.01);};
  auto sent=gains;sent[0].q=1;sent[0].tau=.5;
  {
    CommandGuard guard;start(guard);
    guard.hold_command(feedback,sent,.015);
    const auto held=guard.step(feedback,sent,10,sent);
    assert(held[0].q==1 && held[0].kp==60 && held[0].tau==.5);
    guard.hold(feedback,sent,10);
    assert(guard.step(feedback,sent,12,sent)[0].tau==0); // explicit measured HOLD keeps its old fade
  }
  {
    CommandGuard guard;start(guard);
    auto queued=sent;queued[0].q=2;queued[0].tau=.9;
    guard.accept(queued,.02);
    const auto held=guard.step(feedback,queued,.13,sent);
    assert(guard.state=="HOLD" && !guard.latched && guard.fault.empty() && guard.faults.empty());
    assert(guard.warned("COMMAND_TIMEOUT") && held[0].q==1 && held[0].tau==.5);
    assert(guard.step(feedback,queued,2,sent)[0].tau==.5);
    guard.accept(sent,2.01);
    assert(guard.state=="ACTIVE" && !guard.warned("COMMAND_TIMEOUT") && guard.accepted_seq==2);
  }
  {
    CommandGuard guard;start(guard);
    auto moving=sent;moving[0].dq=.2;moving[1].kp=0;moving[1].tau=.4;
    guard.accept(moving,.02);
    const auto held=guard.step(feedback,moving,.13,moving);
    assert(held[0].q==state.position && held[0].kp==60 && held[0].dq==0 && held[0].tau==.5);
    assert(held[1].q==state.position && held[1].kp==60 && held[1].dq==0 && held[1].tau==.4);
  }
  {
    CommandGuard guard;guard.command_timeout=10;start(guard);guard.accept(sent,.02);
    auto aging=feedback;aging[0].last_rx_age_s=.2;
    guard.step(aging,sent,.1,sent);assert(!guard.warned("FEEDBACK_STALE"));
    aging[0].last_rx_age_s=.200001;
    auto held=guard.step(aging,sent,.11,sent);
    assert(guard.state=="HOLD" && guard.warned("FEEDBACK_STALE") && held[0].q==1 && held[0].tau==.5);
    assert(guard.fault.empty() && guard.faults.empty());
    assert(guard.warnings[0].can_id==1 && guard.warnings[0].rx_sequence==1);
    bool rejected=false;try{guard.accept(sent,.12);}catch(const FeedbackStaleError&){rejected=true;}
    assert(rejected && guard.accepted_seq==1 && guard.updated==.02);
    aging[0].last_rx_age_s=.499999;assert(guard.step(aging,sent,.2,sent)[0].q==1);
    aging[0].last_rx_age_s=.5;
    rejected=false;try{guard.step(aging,sent,.21,sent);}catch(const ControlFailure& e){
      rejected=e.issue.code=="FEEDBACK_LOST" && e.issue.can_id==1 && e.issue.age_s==.5;}
    assert(rejected && guard.latched && guard.state=="FAULT");
  }
  {
    CommandGuard guard;guard.command_timeout=10;start(guard);guard.accept(sent,.02);
    auto aging=feedback;for(auto& s:aging)s.last_rx_age_s=.21;
    guard.step(aging,sent,.1,sent);assert(guard.warnings.size()==2);
    auto queued=sent;queued[0].q=9;
    auto recovered=feedback;for(auto& s:recovered)++s.rx_sequence;
    const auto held=guard.step(recovered,queued,.11,sent);
    assert(guard.state=="HOLD" && !guard.warned("FEEDBACK_STALE") && held[0].q==1);
    guard.accept(sent,.12);assert(guard.state=="ACTIVE" && guard.accepted_seq==2);
  }
  for(int mode=0;mode<4;++mode) {
    CommandGuard guard;start(guard);auto bad=feedback;
    if(mode==0)bad[0].error_code=8;
    if(mode==1)bad[0].raw_status=0;
    if(mode==2)bad[0].position=std::numeric_limits<double>::quiet_NaN();
    if(mode==3)bad[0].enabled_confirmed=false;
    bool failed=false;try{guard.step(bad,sent,.02,sent);}catch(const ControlFailure& e){
      failed=e.issue.code==(mode==0 ? "MOTOR_FAULT" : mode==2 ? "FEEDBACK_INVALID" : "MOTOR_DISABLED");}
    assert(failed && guard.latched);
  }
  {
    CommandGuard guard;guard.strict_timeouts=true;start(guard);guard.accept(sent,.02);
    auto held=guard.step(feedback,sent,.13,sent);
    assert(guard.latched && guard.state=="FAULT_HOLD" && guard.fault=="command_timeout");
    assert(held[0].q==state.position && guard.step(feedback,sent,2,sent)[0].tau==0);
    bool rejected=false;try{guard.accept(sent,2.1);}catch(const std::runtime_error&){rejected=true;}
    assert(rejected);
    auto stale=feedback;stale[0].last_rx_age_s=.201;
    rejected=false;try{guard.step(stale,sent,2.2,sent);}catch(const ControlFailure&){rejected=true;}
    assert(rejected); // calibration has no grace period
  }
  {
    CommandGuard guard;guard.configure(.1,.5,std::nullopt,.2);
    assert(guard.feedback_hard_timeout==1.25);start(guard);
    for(double hard:{0.,.2,-1.,std::numeric_limits<double>::infinity()}) {
      guard.configure(.1,.2,hard,.2);bool rejected=false;
      try{guard.start(gains,0);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
    }
    for(double wait:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
      guard.configure(.1,.2,std::nullopt,wait);bool rejected=false;
      try{guard.start(gains,0);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
    }
  }
}
