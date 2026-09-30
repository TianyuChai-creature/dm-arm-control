#include "dm_openarm/backend/dm_serial_backend.hpp"
#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/mit_loop_controller.hpp"
#include <cassert>
#include <atomic>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <set>
#include <stdexcept>
#include <vector>
struct dmcan_context {};
struct dmcan_device_handle {};
static dmcan_context context;
static dmcan_device_handle device;
static dmcan_frame_callback callback=nullptr;
static std::vector<std::pair<unsigned,unsigned>> frames;
static int failed_op=-1;
static bool probe_motion=false, probe_reply=false, emit_status=true;
static bool preempt_status2=false;
static std::atomic<int> failed_mit_id{-1};
static std::atomic<bool> flip_status2_after_save1{false};
static bool flip_on_save1=false;
static std::array<std::uint8_t,8> last_mit{};
extern "C" {
void dmcan_context_create(dmcan_context** p){*p=&context;}
void dmcan_context_destroy(dmcan_context*){}
int dmcan_find_devices_with_type(dmcan_context*,int){return 4;}
bool dmcan_device_get(dmcan_context*,dmcan_device_handle** p,int){*p=&device;return true;}
bool dmcan_device_open(dmcan_device_handle*){return true;}
void dmcan_device_enable_channel(dmcan_device_handle*,uint8_t){}
void dmcan_device_disable_channel(dmcan_device_handle*,uint8_t){}
bool dmcan_device_set_channel_baudrate(dmcan_device_handle*,uint8_t,dmcan_channel_can_info){return true;}
void dmcan_device_hook_recv_callback(dmcan_device_handle*,dmcan_frame_callback f){callback=f;}
bool dmcan_device_send_can(dmcan_device_handle*,uint8_t,uint32_t id,bool,bool,bool,bool,uint8_t n,const uint8_t* d){
  const unsigned op=(id==0x7ff ? d[2] : n==8 && d[0]==255 ? d[7] : 0);
  frames.emplace_back(id,op);
  if(op==0 && id==1 && n==8) std::memcpy(last_mit.data(),d,8);
  if(op==0 && static_cast<int>(id)==failed_mit_id.load())return false;
  if(static_cast<int>(op)==failed_op)return false;
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
  return true;
}
}
static dm_openarm::ArmConfig config(){
  dm_openarm::ArmConfig c;c.usb_serial="fake";
  dm_openarm::MotorConfig a,b;a.can_id=1;a.mst_id=11;b.can_id=2;b.mst_id=12;
  c.motors={a,b};return c;
}
static unsigned count(unsigned id,unsigned op){unsigned n=0;for(auto f:frames)n+=f.first==id&&f.second==op;return n;}
static void inject_status(unsigned id,unsigned status){
  assert(callback);usb_rx_frame f{};f.head.can_id=id;f.head.dlc=8;
  f.payload[0]=static_cast<uint8_t>((status<<4)|id);
  f.payload[1]=0x80;f.payload[3]=0x80;f.payload[4]=0x08;callback(&device,&f);
}
int main(){using namespace dm_openarm;
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
