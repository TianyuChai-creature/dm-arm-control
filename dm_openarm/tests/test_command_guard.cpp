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
  CommandGuard guard;
  std::vector<MitCommand> gains(2, MitCommand{60,4,0,0,0});
  guard.start(gains, 0);
  MotorState state{}; state.position=0.3; state.velocity=0; state.torque=1;
  state.rx_sequence=1; state.error_code=0; state.raw_status=1; state.enabled_confirmed=true; state.last_rx_age_s=0.01;
  std::vector<MotorState> feedback(2,state);
  auto output=guard.step(feedback,gains,0.01);
  assert(guard.state=="HOLD" && output[0].q==0.3);
  auto target=gains; target[0].q=1; target[0].tau=0.5;
  guard.hold_command(feedback, target, 0.015);
  auto held = guard.step(feedback, gains, 10);
  assert(held[0].q == 1 && held[0].kp == 60 && held[0].tau == 0.5);
  assert(held[0].kp*(held[0].q-state.position)+held[0].tau ==
         target[0].kp*(target[0].q-state.position)+target[0].tau);
  auto stale = feedback; stale[0].last_rx_age_s=1;
  bool lost=false;
  try { guard.step(stale, gains, 10); } catch (...) { lost=true; }
  assert(lost);
  guard.hold(feedback, target, 10);
  held = guard.step(feedback, target, 12);
  assert(held[0].q == state.position && held[0].tau == 0);
  guard.start(gains, 0);
  guard.step(feedback, gains, 0.01);
  guard.accept(target,0.02);
  assert(guard.step(feedback,target,0.03)[0].q==1);
  output=guard.step(feedback,target,0.13);
  assert(guard.latched && guard.fault=="command_timeout");
  assert(output[0].q==0.3 && output[0].kp==60);
  feedback[0].position=0.31;
  output=guard.step(feedback,target,1.2);
  assert(output[0].q==0.3 && output[0].tau==0); // no repeated reseed or stale FF
  bool rejected=false;
  try {guard.accept(target,1.3);} catch(...) {rejected=true;}
  assert(rejected);
  feedback[0].last_rx_age_s=1;
  rejected=false;
  try {guard.step(feedback,target,1.3);} catch(...) {rejected=true;}
  assert(rejected);
  guard.start(gains,2);
  feedback.assign(2,state); feedback[1].rx_sequence=0;
  output=guard.step(feedback,target,2.01);
  assert(output[0].kp==60 && output[1].kp==0);
  rejected=false;
  try {guard.step(feedback,target,5);} catch(...) {rejected=true;}
  assert(rejected);
  target[0].q=std::numeric_limits<double>::quiet_NaN();
  rejected=false; try {CommandGuard::validate(target);} catch(...) {rejected=true;}
  assert(rejected);
}
