#include "dm_openarm/backend/dm_serial_backend.hpp"
#include <cassert>
#include <set>
#include <vector>
#include <stdexcept>

struct dmcan_context {};
struct dmcan_device_handle {int index;};
static dmcan_context context;
static dmcan_device_handle devices[2]{{0},{1}};
static const char* serials[2]{"fake","other"};
static int opens=0, closes=0, channels=0;
static std::set<unsigned> enabled, disabled, commands;
static dmcan_frame_callback recv_callback=nullptr;
extern "C" {
void dmcan_context_create(dmcan_context** out) { *out=&context; }
void dmcan_context_destroy(dmcan_context*) { ++closes; }
int dmcan_find_devices_with_type(dmcan_context*,int) { return 2; }
bool dmcan_device_get(dmcan_context*,dmcan_device_handle** out,int index) { *out=&devices[index];return true; }
void dmcan_device_get_info(dmcan_device_handle* handle,int* index,int*,int*,char** serial) {
  *index=handle->index;*serial=const_cast<char*>(serials[handle->index]);
}
bool dmcan_device_open(dmcan_device_handle*) { ++opens;return true; }
void dmcan_device_enable_channel(dmcan_device_handle*,uint8_t) {++channels;}
void dmcan_device_disable_channel(dmcan_device_handle*,uint8_t) {}
bool dmcan_device_set_channel_baudrate(dmcan_device_handle*,uint8_t,dmcan_channel_can_info) { return true; }
void dmcan_device_hook_recv_callback(dmcan_device_handle*,dmcan_frame_callback cb) { recv_callback=cb; }
bool dmcan_device_send_can(dmcan_device_handle* handle,uint8_t,uint32_t id,bool,bool,bool,bool,uint8_t n,const uint8_t* data) {
  if(n==8 && data[0]==255 && (data[7]==0xFC || data[7]==0xFD)) {
    if(data[7]==0xFC) enabled.insert(id); else disabled.insert(id);
    if(recv_callback) {
      usb_rx_frame f{}; f.head.can_id=id; f.head.dlc=8;
      f.payload[0]=static_cast<uint8_t>(((data[7]==0xFC ? 1 : 0)<<4) | (id & 0x0f));
      f.payload[1]=0x80; f.payload[3]=0x80; f.payload[4]=0x08;
      recv_callback(handle,&f);
    }
  }
  else if(id==0x7FF && n==4 && data[2]==0xCC && data[0]==1 && recv_callback) {
    usb_rx_frame reply{};
    reply.head.can_id=11; reply.head.dlc=8;
    reply.payload[0]=1;reply.payload[1]=0x80;reply.payload[3]=0x80;reply.payload[4]=0x08;
    recv_callback(handle,&reply);
  }
  else if(id!=0x7FF) commands.insert(id);
  return true;
}
}
int main() {
  using namespace dm_openarm;
  ArmConfig a;
  a.usb_serial="fake";
  MotorConfig l,r;l.can_id=1;l.mst_id=11;r.can_id=2;r.mst_id=12;
  a.bus_motors={l,r};a.motors={l};
  for(const char* actual : {"other", "", static_cast<const char*>(nullptr)}) {
    serials[0]=actual;
    backend::DmSerialBackend wrong(a);
    bool rejected=false;try{wrong.connect();}catch(const std::runtime_error& e){
      const auto message=std::string(e.what());
      rejected=message.find("device_index=0")!=std::string::npos &&
        message.find("expected SN='fake'")!=std::string::npos && message.find("actual SN=")!=std::string::npos;
    }
    assert(rejected && !wrong.connected() && opens==0 && channels==0 &&
      enabled.empty() && disabled.empty() && commands.empty());
  }
  // Enumeration order changed: the configured index must be corrected explicitly.
  serials[0]="other";serials[1]="fake";
  auto reordered=a;reordered.device_index=1;
  {backend::DmSerialBackend matching(reordered);matching.connect();assert(opens==1);matching.disconnect();}
  serials[0]="fake";serials[1]="other";
  auto empty_sn=a;empty_sn.usb_serial="";
  {backend::DmSerialBackend empty(empty_sn);bool rejected=false;
   try{empty.connect();}catch(const std::runtime_error&){rejected=true;}assert(rejected);}
  opens=closes=channels=0;
  auto b=a;b.motors={r};
  backend::DmSerialBackend left(a),right(b),duplicate(a);
  left.connect();left.enable();
  assert(opens==1 && enabled==std::set<unsigned>{1});
  right.connect();right.enable();
  assert(opens==1 && enabled==std::set<unsigned>({1,2}));
  bool rejected=false;try { duplicate.connect(); } catch(const std::runtime_error&) { rejected=true; }
  assert(rejected);
  left.disable();left.disconnect();
  assert(disabled==std::set<unsigned>{1} && closes==0 && right.connected());
  right.send_mit_all(std::vector<MitCommand>(1));assert(commands==std::set<unsigned>{2});
  left.connect();left.enable();assert(opens==1);
  right.disable();right.disconnect();assert(left.connected() && closes==0);
  left.disconnect();assert(closes==1);
  // A left-only lifetime must never enable or disable right motors, even on destruction.
  enabled.clear();disabled.clear();
  { backend::DmSerialBackend only_left(a);only_left.connect();only_left.enable(); }
  assert(enabled==std::set<unsigned>{1} && disabled==std::set<unsigned>{1});
  enabled.clear();disabled.clear();commands.clear();opens=closes=0;
  { backend::DmSerialBackend passive(a);passive.connect();
    assert(passive.probe_status(.02)==std::vector<std::uint16_t>{1});
    const auto state=passive.states()[0];
    assert(state.rx_sequence==1 && state.raw_status==0 && !state.enabled_confirmed);
    passive.disconnect(); }
  assert(opens==1 && closes==1 && enabled.empty() && disabled.empty() && commands.empty());
}
