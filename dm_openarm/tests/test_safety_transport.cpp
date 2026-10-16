#include "dm_openarm/backend/dm_serial_backend.hpp"
#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/mit_loop_controller.hpp"
#include <cassert>
#include <atomic>
#include <array>
#include <chrono>
#include <cstring>
#include <condition_variable>
#include <future>
#include <functional>
#include <mutex>
#include <thread>
#include <set>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cmath>
struct dmcan_context {};
struct dmcan_device_handle {};
static dmcan_context context;
static dmcan_device_handle device;
static dmcan_frame_callback callback=nullptr;
static std::vector<std::pair<unsigned,unsigned>> frames;
static std::atomic<int> failed_op{-1};
static bool probe_motion=false, probe_reply=false;
static std::atomic<bool> emit_status{true}, mit_feedback{false}, fail_hold_send{false};
static std::mutex frames_mutex;
static std::condition_variable frames_changed;
static int blocked_id=-1, blocked_kp=-1;
static bool tx_entered=false, release_tx=false;
static void inject_status(unsigned id,unsigned status);
static bool preempt_status2=false;
static std::atomic<int> failed_mit_id{-1};
static std::atomic<bool> flip_status2_after_save1{false};
static bool flip_on_save1=false;
static std::array<std::uint8_t,8> last_mit{};
static std::atomic<bool> calibration_transport{false};
static std::array<float,10> register_positions{};
static std::array<dm_openarm::MotorModel,10> register_models{};
static int missing_save_ack=-1, wrong_mapping=-1;
static bool drift_after_save=false;
static bool stale_save_ack_on_fd=false;
static unsigned position_calibration_frames=0;
static unsigned calibration_save_commands=0;
static int drop_final_angle_queries=0;
static bool final_angle_outside_zero=false;
static std::array<double,10> plant_velocity{}, plant_torque{};
extern "C" {
void dmcan_context_create(dmcan_context** p){*p=&context;}
void dmcan_context_destroy(dmcan_context*){}
int dmcan_find_devices_with_type(dmcan_context*,int){return 16;}
bool dmcan_device_get(dmcan_context*,dmcan_device_handle** p,int){*p=&device;return true;}
void dmcan_device_get_info(dmcan_device_handle*,int* index,int*,int*,char** serial){
  *index=0;*serial=const_cast<char*>("fake");
}
bool dmcan_device_open(dmcan_device_handle*){return true;}
void dmcan_device_enable_channel(dmcan_device_handle*,uint8_t){}
void dmcan_device_disable_channel(dmcan_device_handle*,uint8_t){}
bool dmcan_device_set_channel_baudrate(dmcan_device_handle*,uint8_t,dmcan_channel_can_info){return true;}
void dmcan_device_hook_recv_callback(dmcan_device_handle*,dmcan_frame_callback f){callback=f;}
bool dmcan_device_send_can(dmcan_device_handle*,uint8_t,uint32_t id,bool,bool,bool,bool,uint8_t n,const uint8_t* d){
  const unsigned op=(id==0x7ff ? d[2] : n==8 && d[0]==255 ? d[7] : 0);
  {
    std::unique_lock<std::mutex> lock(frames_mutex);
    frames.emplace_back(id,op);
    if(op==0 && id==1 && n==8) std::memcpy(last_mit.data(),d,8);
    if (calibration_transport && op==0 && id<10 && n==8) {
      position_calibration_frames += (((d[3]&15)<<8)|d[4]) != 0 || ((d[5]<<4)|(d[6]>>4)) != 0;
    }
    frames_changed.notify_all();
    if(op==0 && static_cast<int>(id)==blocked_id &&
       (blocked_kp<0 || (((d[3]&15)<<8)|d[4])==blocked_kp)){
      tx_entered=true;frames_changed.notify_all();
      frames_changed.wait(lock,[]{return release_tx;});blocked_id=-1;
    }
  }
  if(op==0 && static_cast<int>(id)==failed_mit_id.load())return false;
  if(op==0 && id==2 && fail_hold_send && (((d[3]&15)<<8)|d[4])==204)return false;
  if(static_cast<int>(op)==failed_op)return false;
  if (calibration_transport && callback) {
    if (id==0x7ff && op==0xcc && emit_status) {
      if (calibration_save_commands==2 && d[0]==2 && drop_final_angle_queries!=0) {
        if (drop_final_angle_queries>0) --drop_final_angle_queries;
      } else inject_status(d[0],0);
    }
    if (id==2 && op==0xfd && stale_save_ack_on_fd) {
      usb_rx_frame old{};old.head.can_id=2;old.head.dlc=4;old.payload[0]=2;old.payload[2]=0xaa;old.payload[3]=1;
      callback(&device,&old);
    }
    if (id<10 && op==0xfe) register_positions[id]=0;
    if (id==0x7ff && op==0x33) {
      usb_rx_frame reply{}; reply.head.can_id=d[0]; reply.head.dlc=8;
      reply.payload[0]=d[0]; reply.payload[1]=d[1]; reply.payload[2]=0x33; reply.payload[3]=d[3];
      const bool large=register_models[d[0]]==dm_openarm::MotorModel::DM8009;
      float value=d[3]==21 ? (wrong_mapping==d[0] ? 10.0f : 12.5f) : d[3]==22 ? (large ? 45 : 20) :
                  d[3]==23 ? (large ? 54 : 28) : register_positions[d[0]];
      if(d[3]==10) { const std::uint32_t mode=1; std::memcpy(reply.payload+4,&mode,4); }
      else std::memcpy(reply.payload+4,&value,4);
      callback(&device,&reply);
    }
    if (id==0x7ff && op==0xaa) {
      ++calibration_save_commands;
      if (calibration_save_commands==2 && final_angle_outside_zero) register_positions[1]=.1f;
      if (drift_after_save && d[0]==1) register_positions[2]+=.1f;
      if (missing_save_ack!=d[0]) {
        usb_rx_frame reply{}; reply.head.can_id=d[0]; reply.head.dlc=4;
        reply.payload[0]=d[0]; reply.payload[1]=d[1]; reply.payload[2]=0xaa; reply.payload[3]=1;
        callback(&device,&reply);
      }
    }
  }
  if(flip_on_save1 && id==0x7ff && op==0xaa && d[0]==1)
    flip_status2_after_save1=true;
  if(callback && id==0x7ff && n==4 && op==0xcc){
    if(probe_motion){usb_rx_frame f{};f.head.can_id=d[0];f.head.dlc=8;f.payload[0]=static_cast<uint8_t>(0x10|d[0]);callback(&device,&f);}
    if(probe_reply){usb_rx_frame f{};f.head.can_id=d[0];f.head.dlc=4;f.payload[0]=d[0];f.payload[1]=d[1];f.payload[2]=0xcc;callback(&device,&f);}
  }
  if(callback && emit_status && (op==0xfc || op==0xfd)){
    if (!(preempt_status2 && op==0xfd && id==2)) {
      usb_rx_frame f{};f.head.can_id=id;f.head.dlc=8;
      f.payload[0]=static_cast<uint8_t>((op==0xfc ? 0x10 : 0)|id);
      f.payload[1]=0x80;f.payload[3]=0x80;f.payload[4]=0x08;callback(&device,&f);
    }
    if (preempt_status2 && op==0xfd && id==1) {
      usb_rx_frame f{};f.head.can_id=2;f.head.dlc=8;f.payload[0]=2;
      f.payload[1]=0x80;f.payload[3]=0x80;f.payload[4]=0x08;callback(&device,&f);
    }
  }
  if(calibration_transport && emit_status && (op==0xfc || op==0xfd))inject_status(id,op==0xfc ? 1 : 0);
  if(callback && mit_feedback && op==0)inject_status(id,1);
  return true;
}
}
static dm_openarm::ArmConfig config(){
  dm_openarm::ArmConfig c;c.usb_serial="fake";
  dm_openarm::MotorConfig a,b;a.can_id=1;a.mst_id=11;b.can_id=2;b.mst_id=12;
  c.motors={a,b};return c;
}
static unsigned count(unsigned id,unsigned op){std::lock_guard<std::mutex> lock(frames_mutex);unsigned n=0;for(auto f:frames)n+=f.first==id&&f.second==op;return n;}
static void inject_status(unsigned id,unsigned status){
  assert(callback);usb_rx_frame f{};f.head.can_id=id;f.head.dlc=8;
  f.payload[0]=static_cast<uint8_t>((status<<4)|id);
  f.payload[1]=0x80;f.payload[3]=0x80;f.payload[4]=0x08;
  if(calibration_transport && id<10) {
    const bool large=register_models[id]==dm_openarm::MotorModel::DM8009;
    const auto encode=[](double value,double range,unsigned bits){return static_cast<unsigned>((value+range)/(2*range)*((1u<<bits)-1));};
    const auto q=encode(register_positions[id],12.5,16);
    const auto dq=encode(plant_velocity[id],large ? 45 : 20,12);
    const auto tau=encode(plant_torque[id],large ? 54 : 28,12);
    f.payload[1]=q>>8;f.payload[2]=q&255;f.payload[3]=dq>>4;
    f.payload[4]=((dq&15)<<4)|(tau>>8);f.payload[5]=tau&255;
  }
  callback(&device,&f);
}

static void block_send(int id,int kp=-1){
  std::lock_guard<std::mutex> lock(frames_mutex);
  blocked_id=id;blocked_kp=kp;tx_entered=release_tx=false;
}
static void wait_blocked(){
  std::unique_lock<std::mutex> lock(frames_mutex);
  assert(frames_changed.wait_for(lock,std::chrono::seconds(2),[]{return tx_entered;}));
}
static void release_send(){
  std::lock_guard<std::mutex> lock(frames_mutex);release_tx=true;frames_changed.notify_all();
}
static void await_loop(const std::function<bool()>& done){
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!done()){
    std::unique_lock<std::mutex> lock(frames_mutex);
    const auto before=frames.size();
    assert(frames_changed.wait_until(lock,deadline,[before]{return frames.size()>before;}));
  }
}
static void test_guarded_updates(){
  using namespace dm_openarm;
  mit_feedback=true;
  DmArm arm(config());MitLoopController loop(arm);
  loop.enable_seeded(std::vector<MitCommand>(2,MitCommand{25,1,0,0,0}),250,.05,.5);
  const auto seeded=loop.commands();
  assert(seeded[0].kp==25 && seeded[1].kp==25);
  loop.set_command(1,MitCommand{10,.5,.6,0,.4});
  await_loop([&]{return loop.sent_sequence()==1;});
  assert(loop.commands()[1].q==seeded[1].q && loop.commands()[1].kp==25);
  loop.set_commands({{2,MitCommand{10,.5,.9,0,.3}}});
  assert(loop.accepted_sequence()==2);
  await_loop([&]{return loop.sent_sequence()==2;});
  for(const auto& bad:std::vector<std::map<std::uint16_t,MitCommand>>{
      {{1,MitCommand{10,.5,.7,0,0}},{9,MitCommand{}}},
      {{1,MitCommand{10,.5,.7,0,0}},{2,MitCommand{10,.5,99,0,0}}}}){
    bool rejected=false;try{loop.set_commands(bad);}catch(const std::exception&){rejected=true;}
    assert(rejected && loop.accepted_sequence()==2 && loop.commands()[0].q==.6);
  }
  const auto kept=loop.hold_command();
  assert(kept[1].q==.9 && kept[1].tau==.3 && loop.commands()[1].tau==.3);
  loop.set_command(1,MitCommand{10,.5,.7,0,.4});
  await_loop([&]{return loop.sent_sequence()==3;});
  assert(loop.commands()[1].q==.9 && loop.commands()[1].tau==.3);
  loop.hold();
  await_loop([&]{return loop.commands()[1].tau==0;});
  loop.set_command(1,MitCommand{10,.5,.8,0,0});
  assert(loop.commands()[1].q==seeded[1].q && loop.commands()[1].kp==25 && loop.commands()[1].tau==0);
  await_loop([&]{return loop.safety_state()=="FAULT_HOLD";});
  bool rejected=false;try{loop.set_command(1,MitCommand{});}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
  loop.hold(true);loop.set_commands({{1,MitCommand{10,.5,.4,0,0}}});
  assert(loop.commands()[1].q==seeded[1].q && loop.commands()[1].kp==25);
  loop.stop();arm.disconnect();mit_feedback=false;frames.clear();
}
static void test_startup_and_hold_boundary(){
  using namespace dm_openarm;
  mit_feedback=true;
  DmArm arm(config());MitLoopController loop(arm);
  const std::vector<MitCommand> gains(2,MitCommand{25,1,0,0,0});
  block_send(2);
  auto startup=std::async(std::launch::async,[&]{loop.enable_seeded(gains,250,1,.5);});
  wait_blocked();
  assert(startup.wait_for(std::chrono::milliseconds(30))==std::future_status::timeout);
  release_send();startup.get();
  block_send(2);loop.set_all_commands(std::vector<MitCommand>(2,MitCommand{10,.5,.9,0,0}));
  wait_blocked();
  std::promise<void> requesting;
  auto held=std::async(std::launch::async,[&]{requesting.set_value();loop.hold();});
  requesting.get_future().wait();
  assert(held.wait_for(std::chrono::milliseconds(30))==std::future_status::timeout);
  release_send();held.get();
  assert(loop.sent_commands()[0].q!=.9 && loop.sent_commands()[1].kp==25);
  const auto q=loop.sent_commands()[0].q;
  const auto cycles=loop.timing_stats().tx_cycles;
  await_loop([&]{return loop.timing_stats().tx_cycles>cycles+2;});
  assert(loop.sent_commands()[0].q==q && loop.sent_sequence()==0);
  loop.stop();arm.disconnect();mit_feedback=false;frames.clear();
}
static void test_hold_send_failure(bool shutdown_failure){
  using namespace dm_openarm;
  mit_feedback=true;
  DmArm arm(config());MitLoopController loop(arm);
  loop.enable_seeded(std::vector<MitCommand>(2,MitCommand{25,1,0,0,0}),250,1,.5);
  loop.set_all_commands(std::vector<MitCommand>(2,MitCommand{10,.5,.9,0,0}));
  await_loop([&]{return loop.sent_sequence()==1;});
  fail_hold_send=true;if(shutdown_failure)emit_status=false;
  bool failed=false;try{loop.hold();}catch(const std::runtime_error&){failed=true;}
  assert(failed);
  try{loop.stop();}catch(const std::runtime_error&){}
  assert(loop.sent_commands()[0].q==.9 && loop.sent_commands()[1].q==.9);
  assert(loop.safety_state()==(shutdown_failure ? "SHUTDOWN_UNKNOWN" : "FAULT"));
  assert(count(1,0xfd)>=5 && count(2,0xfd)>=5);
  bool rejected=false;try{loop.hold(true);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
  fail_hold_send=false;emit_status=true;arm.disable();arm.disconnect();
  mit_feedback=false;frames.clear();
}
static void test_startup_send_failure(){
  using namespace dm_openarm;
  mit_feedback=true;failed_mit_id=2;
  DmArm arm(config());MitLoopController loop(arm);
  bool failed=false;try{loop.enable_seeded(std::vector<MitCommand>(2,MitCommand{25,1,0,0,0}),250,1,.5);}
  catch(const std::runtime_error&){failed=true;}
  assert(failed && loop.safety_state()=="FAULT" && loop.sent_commands()[0].kp==0);
  failed_mit_id=-1;arm.disconnect();mit_feedback=false;frames.clear();
}
static void test_pending_updates_and_confirmation_timeout(){
  using namespace dm_openarm;
  {
    emit_status=false;
    DmArm arm(config());arm.enable();MitLoopController loop(arm);
    loop.start_seeded(std::vector<MitCommand>(2,MitCommand{25,1,0,0,0}),10,1,.5);
    bool rejected=false;try{loop.set_commands({{1,MitCommand{10,.5,.3,0,0}}});}
    catch(const std::runtime_error& e){rejected=std::string(e.what()).find("pending")!=std::string::npos;}
    assert(rejected && loop.accepted_sequence()==0);
    emit_status=true;loop.stop();arm.disconnect();frames.clear();
  }
  mit_feedback=true;
  DmArm arm(config());MitLoopController loop(arm);
  loop.enable_seeded(std::vector<MitCommand>(2,MitCommand{25,1,0,0,0}),250,1,.1);
  loop.set_all_commands(std::vector<MitCommand>(2,MitCommand{10,.5,.9,0,0}));
  await_loop([&]{return loop.sent_sequence()==1;});
  block_send(2,204);
  auto held=std::async(std::launch::async,[&]{loop.hold();});
  wait_blocked();
  assert(held.wait_for(std::chrono::milliseconds(150))==std::future_status::timeout);
  release_send();
  bool failed=false;try{held.get();}catch(const std::runtime_error& e){
    failed=std::string(e.what()).find("timeout")!=std::string::npos;}
  assert(failed && loop.safety_state()=="FAULT" && loop.sent_commands()[0].q==.9);
  arm.disconnect();mit_feedback=false;frames.clear();
}
static void test_single_limb_batch(){
  using namespace dm_openarm;
  auto c=config();
  auto third=c.motors[0],fourth=c.motors[1];
  third.can_id=3;third.mst_id=13;fourth.can_id=4;fourth.mst_id=14;
  c.motors.push_back(third);c.motors.push_back(fourth);
  c.left.count=c.right.count=2;c.right.begin=2;
  mit_feedback=true;
  DmArm arm(c);MitLoopController loop(arm);
  loop.enable_seeded(std::vector<MitCommand>(4,MitCommand{25,1,0,0,0}),250,1,.5);
  const auto seeded=loop.commands();
  for(double target : {.3,.6}){
    loop.set_commands({{1,MitCommand{10,.5,target,0,0}},{2,MitCommand{10,.5,-target,0,0}}});
    const auto current=loop.commands();
    assert(current[2].kp==25 && current[3].q==seeded[3].q);
    loop.hold();
  }
  assert(loop.accepted_sequence()==2);
  loop.stop();arm.disconnect();mit_feedback=false;frames.clear();
}

static void prepare_calibration_transport(const dm_openarm::ArmConfig& c) {
  register_positions.fill(0);
  plant_velocity.fill(0);plant_torque.fill(0);
  for (const auto& m : c.motors) register_models[m.can_id]=m.model;
  position_calibration_frames=0; missing_save_ack=wrong_mapping=-1; drift_after_save=false;
  stale_save_ack_on_fd=false;
  calibration_save_commands=0;drop_final_angle_queries=0;final_angle_outside_zero=false;
  calibration_transport=true; frames.clear();
}


static void test_calibration_transport() {
  using namespace dm_openarm;
  auto c=config();
  for(auto& m:c.motors)m.model=MotorModel::DM4340P;
  const std::vector<double> zeros{.4,-.3};
  std::atomic<bool> cancel{false};
  std::vector<std::string> saved(2,"NOT_ATTEMPTED");
  auto progress=[&](std::size_t i,const std::string& state){saved[i]=state;};
  {
    prepare_calibration_transport(c);
    backend::DmSerialBackend b(c);b.connect();wrong_mapping=1;
    register_positions[1]=.4f;register_positions[2]=-.3f;
    auto pos=b.verify_calibration_configuration();assert(std::abs(pos[0]-.4)<.001);
    assert(count(0x7ff,0x33)==0 && count(1,0xfc)==0 && count(1,0xfe)==0);
    assert(b.states()[0].rx_sequence>0);
    emit_status=false;
    bool failed=false;try{b.verify_calibration_configuration();}catch(const std::runtime_error&){failed=true;}
    assert(failed);emit_status=true;wrong_mapping=-1;
    b.enable();b.disable();b.commit_calibration(zeros,cancel,progress);
    assert(saved[0]=="SAVED" && saved[1]=="SAVED");
    assert(count(1,0xfe)==1 && count(2,0xfe)==1 && count(0x7ff,0xaa)==2);
    b.disconnect();
  }
  {
    prepare_calibration_transport(c);
    backend::DmSerialBackend b(c);b.connect();b.enable();b.disable();
    register_positions[1]=.4f;register_positions[2]=-.25f;
    bool failed=false;try{b.commit_calibration(zeros,cancel,progress);}catch(const std::runtime_error&){failed=true;}
    assert(failed && count(1,0xfe)==0);
    register_positions[2]=-.3f;cancel=true;failed=false;
    try{b.commit_calibration(zeros,cancel,progress);}catch(const std::runtime_error&){failed=true;}
    assert(failed && count(1,0xfe)==0);cancel=false;
    b.disconnect();
  }
  {
    c.device_index=4;prepare_calibration_transport(c);saved.assign(2,"NOT_ATTEMPTED");
    backend::DmSerialBackend b(c);b.connect();b.enable();b.disable();
    register_positions[1]=.4f;register_positions[2]=-.3f;missing_save_ack=2;stale_save_ack_on_fd=true;
    b.commit_calibration(zeros,cancel,progress);
    assert(saved[0]=="SAVED" && saved[1]=="SAVED");
    b.send_zero_mit_all();
  }
  {
    c.device_index=5;prepare_calibration_transport(c);saved.assign(2,"NOT_ATTEMPTED");
    backend::DmSerialBackend b(c);b.connect();b.enable();b.disable();
    register_positions[1]=.4f;register_positions[2]=-.3f;drift_after_save=true;
    bool failed=false;try{b.commit_calibration(zeros,cancel,progress);}catch(const std::runtime_error&){failed=true;}
    assert(failed && saved[0]=="UNKNOWN" && saved[1]=="NOT_ATTEMPTED");
    assert(count(1,0xfe)==1 && count(2,0xfe)==0);
  }
  calibration_transport=false;missing_save_ack=-1;drift_after_save=false;frames.clear();
}

static void test_final_angle_readback_window() {
  using namespace dm_openarm;
  auto c=config();
  for(auto& m:c.motors) m.model=MotorModel::DM4340P;
  for(int scenario=0;scenario<3;++scenario) {
    c.device_index=10+scenario;
    prepare_calibration_transport(c);
    register_positions[1]=.4f;register_positions[2]=-.3f;
    // One axis arrives promptly; the other arrives after 300ms or never.
    drop_final_angle_queries=scenario==0 ? 6 : -1;
    final_angle_outside_zero=scenario==2;
    backend::DmSerialBackend b(c);b.connect();b.enable();b.disable();
    std::atomic<bool> cancel{false};
    std::vector<std::string> saved(2,"NOT_ATTEMPTED");
    std::string error;
    try {
      b.commit_calibration({.4,-.3},cancel,[&](std::size_t i,const std::string& value){saved[i]=value;});
    } catch(const std::runtime_error& e) {error=e.what();}
    if(scenario==0) {
      assert(error.empty() && saved[0]=="SAVED" && saved[1]=="SAVED");
    } else {
      assert(error.find(scenario==1 ? "timed out" : "position mismatch")!=std::string::npos);
      assert(saved[0]=="UNKNOWN" && saved[1]=="UNKNOWN");
    }
    // Query retries must never repeat either motor's zero/Flash writes.
    assert(count(1,0xfe)==1 && count(2,0xfe)==1 && count(0x7ff,0xaa)==2);
    if(scenario==0) b.disconnect();
  }
  calibration_transport=false;frames.clear();
}

static void test_calibration_lease_and_position_mit() {
  using namespace dm_openarm;
  ArmConfig c;c.usb_serial="fake";c.left={0,5};c.right={5,5};
  for(unsigned id=0;id<10;++id)
    c.motors.push_back({"joint",id%5<2 ? MotorModel::DM8009 : MotorModel::DM4340P,
        ControlMode::MIT,static_cast<uint16_t>(id),static_cast<uint16_t>(id+16)});
  prepare_calibration_transport(c);mit_feedback=true;
  DmArm arm(c);MitLoopController loop(arm),competitor(arm);
  std::vector<MitCommand> commands(10,MitCommand{160,5,0,0,0});
  loop.begin_calibration(commands);
  auto rejects=[](const std::function<void()>& f){bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);};
  rejects([&]{loop.set_command(0,{});});rejects([&]{arm.set_zero(0);});
  rejects([&]{arm.send_zero_mit_all();});rejects([&]{competitor.start(250);});
  rejects([&]{loop.hold(true);});rejects([&]{arm.disconnect();});
  commands[0].q=-.01;commands[5].q=-.01;loop.calibration_update(commands);
  await_loop([&]{return loop.sent_sequence()==1;});
  auto sent=loop.sent_commands();assert(sent[0].kp==160 && sent[0].kd==5 && sent[0].tau==0 && sent[0].dq==0);
  assert(position_calibration_frames>0); // Firmware position gains must reach the wire.
  loop.cancel_calibration();rejects([&]{loop.calibration_update(commands);});
  loop.calibration_fail("cancelled");loop.finish_calibration();
  assert(loop.running() && loop.safety_state()=="FAULT_HOLD");
  assert(count(0,0xfe)==0 && count(0x7ff,0xaa)==0);
  rejects([&]{loop.set_command(0,{});});
  const auto enabled_before=count(0,0xfc),disabled_before=count(0,0xfd);
  loop.begin_calibration(commands);
  assert(loop.running() && loop.safety_state()=="HOLD" && !loop.calibration_cancelled());
  assert(count(0,0xfc)==enabled_before && count(0,0xfd)==disabled_before);
  loop.calibration_fail("retry stopped");loop.finish_calibration();loop.stop();
  arm.send_zero_mit_all();arm.disconnect();
  calibration_transport=false;mit_feedback=false;frames.clear();
}

static void test_calibration_commit_clears_old_coordinates() {
  using namespace dm_openarm;
  ArmConfig c;c.usb_serial="fake";c.left={0,5};c.right={5,5};
  for (unsigned id=0;id<10;++id)
    c.motors.push_back({"joint",id%5<2 ? MotorModel::DM8009 : MotorModel::DM4340P,
        ControlMode::MIT,static_cast<uint16_t>(id),static_cast<uint16_t>(id+16)});
  prepare_calibration_transport(c);mit_feedback=true;
  DmArm arm(c);MitLoopController loop(arm),competitor(arm);
  loop.begin_calibration(std::vector<MitCommand>(10,MitCommand{160,5,0,0,0}));
  // Stopping an idle competing controller must not release the live owner's lease.
  competitor.stop();
  bool failed=false;try{competitor.start(250);}catch(const std::runtime_error&){failed=true;}assert(failed);
  loop.calibration_commit(std::vector<double>(10,0));loop.finish_calibration();
  assert(!loop.running() && loop.safety_state()=="DISABLED");
  for(const auto& saved:loop.calibration_saves())assert(saved=="SAVED");
  for(const auto& cmd:loop.commands())assert(cmd.kp==0 && cmd.q==0 && cmd.tau==0);
  for(unsigned id=0;id<10;++id)assert(count(id,0xfe)==1);
  arm.disconnect();calibration_transport=false;mit_feedback=false;frames.clear();
}

static void test_idle_controller_cannot_release_another_loop() {
  using namespace dm_openarm;
  mit_feedback=true;
  DmArm arm(config());MitLoopController owner(arm),idle(arm);
  owner.enable_seeded(std::vector<MitCommand>(2,MitCommand{25,1,0,0,0}),250,.5,.3);
  idle.stop();
  bool failed=false;try{idle.start(250);}catch(const std::runtime_error&){failed=true;}
  assert(failed && owner.running());
  owner.stop();arm.disconnect();mit_feedback=false;frames.clear();
}

int main(){using namespace dm_openarm;
  test_idle_controller_cannot_release_another_loop();
  test_calibration_transport();
  test_final_angle_readback_window();
  test_calibration_lease_and_position_mit();
  test_calibration_commit_clears_old_coordinates();
  test_guarded_updates();
  test_startup_and_hold_boundary();
  test_hold_send_failure(false);
  test_hold_send_failure(true);
  test_startup_send_failure();
  test_pending_updates_and_confirmation_timeout();
  test_single_limb_batch();
  {
    backend::DmSerialBackend b(config());b.connect();
    bool rejected=false;try{b.set_zero(1,true);}catch(const std::runtime_error&){rejected=true;}
    assert(rejected && frames.empty());
    std::atomic<bool> feeding{true};
    std::thread feeder([&]{while(feeding){inject_status(1,0);std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    b.set_zero(1,true);feeding=false;feeder.join();
    assert(count(1,0xfc)==0 && count(2,0xfc)==0);
    b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();
    bool rejected=false;try{b.set_zero_all(true);}catch(const std::runtime_error& e){
      rejected=std::string(e.what()).find("CAN ID=1")!=std::string::npos;}
    assert(rejected && frames.empty());
    std::atomic<bool> feeding{true};
    std::thread feeder([&]{while(feeding){inject_status(1,0);inject_status(2,0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    b.set_zero_all(true);feeding=false;feeder.join();
    assert(count(1,0xfc)==0 && count(2,0xfc)==0);
    b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();
    bool threw=false;try{b.send_mit_all({MitCommand{10,.5,0,0,0},MitCommand{10,.5,99,0,0}});}catch(const std::out_of_range&){threw=true;}
    assert(threw && count(1,0)==0 && count(2,0)==0);
    b.send_mit_all({MitCommand{10,.5,0,0,0},MitCommand{10,.5,0,0,0}});
    assert((last_mit==std::array<std::uint8_t,8>{0x7f,0xff,0x7f,0xf0,0x51,0x19,0x97,0xff}));
    b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();probe_motion=true;
    assert(b.probe_status(.01).empty());
    probe_motion=false;probe_reply=true;
    assert(b.probe_status(.01)==std::vector<std::uint16_t>({1,2}));
    probe_reply=false;b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();failed_op=0xfc;
    bool threw=false;try{b.enable();}catch(const std::runtime_error&){threw=true;}
    assert(threw && count(1,0xfc)==5);
    failed_op=-1;b.disable();b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();b.enable();
    for(const auto& s:b.states())assert(s.raw_status==1 && s.enabled_confirmed);
    failed_op=0xfd;bool threw=false;try{b.disable();}catch(const std::runtime_error&){threw=true;}
    assert(threw && count(1,0xfd)==5 && count(2,0xfd)==5);
    failed_op=-1;b.disable();b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();b.enable();
    emit_status=false;bool threw=false;try{b.disable();}catch(const std::runtime_error& e){threw=std::string(e.what()).find("unconfirmed")!=std::string::npos;}
    assert(threw);emit_status=true;b.disable();b.disconnect();frames.clear();
  }
  {
    backend::DmSerialBackend b(config());b.connect();b.enable();
    preempt_status2=true;bool threw=false;try{b.disable();}catch(const std::runtime_error& e){
      threw=std::string(e.what()).find("unconfirmed")!=std::string::npos;}
    assert(threw);preempt_status2=false;b.disable();b.disconnect();frames.clear();
  }
  {
    DmArm arm(config());arm.connect();MitLoopController loop(arm);
    bool rejected=false;try{loop.start(0.01);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
    rejected=false;try{loop.start(1e12);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
    loop.start(10.0);
    const auto begin=std::chrono::steady_clock::now();
    loop.stop();
    assert(std::chrono::steady_clock::now()-begin<std::chrono::milliseconds(500));
    arm.disconnect();frames.clear();
  }
  {
    DmArm arm(config());arm.enable();MitLoopController loop(arm);
    loop.start(10.0);std::this_thread::sleep_for(std::chrono::milliseconds(10));
    loop.stop();
    assert(count(1,0xfd)>=5 && count(2,0xfd)>=5);
    arm.disconnect();frames.clear();
  }
  {
    DmArm arm(config());arm.enable();MitLoopController loop(arm);
    loop.start(10.0);std::this_thread::sleep_for(std::chrono::milliseconds(10));
    failed_mit_id=1;bool failed=false;try{loop.stop();}catch(const std::runtime_error&){failed=true;}
    failed_mit_id=-1;assert(failed && count(1,0xfd)>=5 && count(2,0xfd)>=5);
    arm.disconnect();frames.clear();
  }
  {
    auto c=config();c.device_index=1;
    backend::DmSerialBackend b(c);b.connect();failed_mit_id=2;
    bool partial=false;try{b.send_mit_all({MitCommand{},MitCommand{}});}catch(const std::runtime_error& e){
      partial=std::string(e.what()).find("prior frames sent=1")!=std::string::npos;}
    assert(partial && count(1,0)==1 && count(2,0)==1);
    failed_mit_id=-1;bool blocked=false;try{b.send_zero_mit_all();}catch(const std::runtime_error&){blocked=true;}
    assert(blocked);
    blocked=false;try{b.disconnect();}catch(const std::runtime_error&){blocked=true;}
    assert(blocked);
  }
  {
    auto c=config();c.device_index=2;
    { backend::DmSerialBackend b(c);b.connect();
      std::atomic<bool> feeding{true};
      std::thread feeder([&]{while(feeding){inject_status(1,0);std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
      failed_op=0xaa;bool unknown=false;try{b.set_zero(1,true);}catch(const std::runtime_error& e){
        unknown=std::string(e.what()).find("coordinate/state unknown")!=std::string::npos;}
      failed_op=-1;feeding=false;feeder.join();assert(unknown);
      bool blocked=false;try{b.send_zero_mit_all();}catch(const std::runtime_error&){blocked=true;}
      assert(blocked);
    }
    backend::DmSerialBackend replacement(c);bool blocked=false;
    try{replacement.connect();}catch(const std::runtime_error&){blocked=true;}
    assert(blocked);
  }
  {
    auto c=config();c.device_index=3;
    frames.clear();
    backend::DmSerialBackend b(c);b.connect();flip_on_save1=true;
    std::atomic<bool> feeding{true};
    std::thread feeder([&]{while(feeding){inject_status(1,0);
      inject_status(2,flip_status2_after_save1.load() ? 1 : 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    bool partial=false;try{b.set_zero_all(true);}catch(const std::runtime_error& e){
      partial=std::string(e.what()).find("CAN ID=2")!=std::string::npos;}
    feeding=false;feeder.join();flip_on_save1=false;
    assert(partial && count(1,0xfe)==1 && count(2,0xfe)==0);
    bool blocked=false;try{b.send_zero_mit_all();}catch(const std::runtime_error&){blocked=true;}
    assert(blocked);
  }
}
