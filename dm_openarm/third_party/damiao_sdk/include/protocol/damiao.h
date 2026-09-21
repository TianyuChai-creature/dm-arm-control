#ifndef DAMIAO_H
#define DAMIAO_H

#include "dmcan/dmcan.h"
#include "unit/sliding_rate.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace damiao
{

// Replies carry the full motor ID, unlike status + low-ID feedback headers.
inline bool is_param_reply(const uint8_t* data, uint8_t len, uint16_t motor_id)
{
  return len == 8 && data[0] == (motor_id & 0xff) && data[1] == (motor_id >> 8) &&
         (data[2] == 0x33 || data[2] == 0x55 || data[2] == 0xAA);
}


#pragma pack(1)

enum DM_Motor_Type
{
  DM3507,
  DM4310,
  DM4310_48V,
  DM4340,
  DM4340_48V,
  DM6006,
  DM6248,
  DM8006,
  DM8009,
  DM10010L,
  DM10010,
  DMH3510,
  DMH6215,
  DMS3519,
  DMG6220,
  Num_Of_Motor
};

enum Control_Mode
{
  MIT_MODE = 0x000,
  POS_VEL_MODE = 0x100,
  VEL_MODE = 0x200,
  POS_FORCE_MODE = 0x300,
};

enum Control_Mode_Code
{
  MIT = 1,
  POS_VEL = 2,
  VEL = 3,
  POS_FORCE = 4,
};

enum DM_REG
{
  UV_Value = 0,
  KT_Value = 1,
  OT_Value = 2,
  OC_Value = 3,
  ACC = 4,
  DEC = 5,
  MAX_SPD = 6,
  MST_ID = 7,
  ESC_ID = 8,
  TIMEOUT = 9,
  CTRL_MODE = 10,
  Damp = 11,
  Inertia = 12,
  hw_ver = 13,
  sw_ver = 14,
  SN = 15,
  NPP = 16,
  Rs = 17,
  LS = 18,
  Flux = 19,
  Gr = 20,
  PMAX = 21,
  VMAX = 22,
  TMAX = 23,
  I_BW = 24,
  KP_ASR = 25,
  KI_ASR = 26,
  KP_APR = 27,
  KI_APR = 28,
  OV_Value = 29,
  GREF = 30,
  Deta = 31,
  V_BW = 32,
  IQ_c1 = 33,
  VL_c1 = 34,
  can_br = 35,
  sub_ver = 36,
  u_off = 50,
  v_off = 51,
  k1 = 52,
  k2 = 53,
  m_off = 54,
  dir = 55,
  p_m = 80,
  xout = 81,
};

#pragma pack()

typedef struct
{
  float Q_MAX;
  float DQ_MAX;
  float TAU_MAX;
} Limit_param;

extern Limit_param limit_param[Num_Of_Motor];

uint16_t encode_mit_field(float value, float min, float max, uint8_t bits);

struct DmActData
{
  DM_Motor_Type motorType;
  Control_Mode mode;
  uint16_t can_id;
  uint16_t mst_id;
  uint8_t channel{0};
};

struct MotorFeedback
{
  float position{0.0f};
  float velocity{0.0f};
  float torque{0.0f};
  double feedback_interval_s{0.0};
  double last_rx_age_s{std::numeric_limits<double>::infinity()};
  uint64_t rx_sequence{0};
  uint8_t error_code{0};
  double feedback_hz{0.0};
};

class Motor
{
private:
  uint16_t Can_id;
  uint16_t Master_id;
  float state_q = 0.0f;
  float state_dq = 0.0f;
  float state_tau = 0.0f;
  Limit_param limit_param{};
  DM_Motor_Type Motor_Type;
  Control_Mode mode;
  uint8_t channel_{0};

  union ValueUnion {
    float floatValue;
    uint32_t uint32Value;
  };

  struct ValueType {
    ValueUnion value;
    bool isFloat;
  };

  std::unordered_map<uint32_t, ValueType> param_map;
  std::chrono::steady_clock::time_point last_time_;
  double delta_time_{0.0};
  uint64_t rx_sequence_{0};
  uint64_t status_probe_sequence_{0};
  uint8_t error_code_{0};
  detail::SlidingRate rx_rate_;

  friend class Motor_Control;

public:
  Motor(DM_Motor_Type motor_type, Control_Mode ctrl_mode, uint16_t can_id, uint16_t master_id,
        uint8_t channel = 0);

  void updateTimeInterval();
  double getTimeInterval();

  void receive_data(float q, float dq, float tau, uint8_t error_code);

  DM_Motor_Type GetMotorType() const { return this->Motor_Type; }
  Control_Mode GetMotorMode() const { return this->mode; }
  Limit_param get_limit_param() { return limit_param; }
  uint16_t GetMasterId() const { return this->Master_id; }
  uint16_t GetCanId() const { return this->Can_id; }
  uint8_t GetChannel() const { return this->channel_; }
  float Get_Position() const { return this->state_q; }
  float Get_Velocity() const { return this->state_dq; }
  float Get_tau() const { return this->state_tau; }
  void set_mode(Control_Mode value) { this->mode = value; }
  void set_param(int key, float value);
  void set_param(int key, uint32_t value);
  float get_param_as_float(int key) const;
  uint32_t get_param_as_uint32(int key) const;

  bool is_have_param(int key) const;
};

/**
 * Motor_Control over the verified u2canfd stack (libdm_device / dmcan).
 * Defaults match station hardware: classic CAN 1M (canfd=false, brs=false).
 */
class Motor_Control
{
public:
  Motor_Control(uint32_t nom_baud, uint32_t dat_baud, std::string sn,
                std::vector<DmActData>* data_ptr, bool canfd = false, bool brs = false,
                int device_index = 0, bool auto_enable = true,
                dmcan_device_type device_type = DMCAN_USB2CANFD, bool auto_disable = true);
  ~Motor_Control();

  Motor_Control(const Motor_Control&) = delete;
  Motor_Control& operator=(const Motor_Control&) = delete;

  void addMotor(std::shared_ptr<Motor> DM_Motor);
  void enable_motor(Motor& motor);
  void disable_motor(Motor& motor);
  void enable_all();
  void disable_all();
  float read_motor_param(Motor& DM_Motor, uint8_t RID);
  void save_motor_param(Motor& DM_Motor);
  void refresh_motor_status(Motor& motor);
  uint64_t response_sequence(uint16_t can_id) const;

  void control_cmd(uint16_t id, uint8_t cmd, uint8_t channel = 0);
  void write_motor_param(Motor& DM_Motor, uint8_t RID, const uint8_t data[4]);
  void set_zero_position(Motor& DM_Motor);

  void control_mit(Motor& DM_Motor, float kp, float kd, float q, float dq, float tau);
  void control_pos_vel(Motor& DM_Motor, float pos, float vel);
  void control_vel(Motor& DM_Motor, float vel);
  void receive_param(uint8_t* data);

  bool switchControlMode(Motor& DM_Motor, Control_Mode_Code mode);
  bool change_motor_param(Motor& DM_Motor, uint8_t RID, float data);
  void changeMotorLimit(Motor& DM_Motor, float P_MAX, float Q_MAX, float T_MAX);
  MotorFeedback getMotorFeedback(uint16_t id) const;

  std::shared_ptr<Motor> getMotor(uint16_t id) const
  {
    auto it = motors.find(id);
    if(it != motors.end())
    {
      return it->second;
    }
    std::cerr << "[Error] In getMotor, no motor with id " << id << " is registered." << std::endl;
    return nullptr;
  }

private:
  static bool is_in_ranges(int number)
  {
    return (7 <= number && number <= 10) || (13 <= number && number <= 16) ||
           (35 <= number && number <= 36);
  }

  static uint32_t float_to_uint32(float value) { return static_cast<uint32_t>(value); }

  static float uint8_to_float(const uint8_t data[4])
  {
    uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                        (static_cast<uint32_t>(data[2]) << 16) |
                        (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
    float result;
    std::memcpy(&result, &combined, sizeof(result));
    return result;
  }

  static void recv_callback_thunk(dmcan_device_handle* handle, usb_rx_frame* frame);
  void on_rx_frame(const usb_rx_frame& frame);
  bool send_can(uint8_t channel, uint32_t can_id, const uint8_t* data, uint8_t len);
  std::vector<std::shared_ptr<Motor>> unique_motors() const;
  void close_device();

  bool auto_disable_;
  std::mutex send_mutex_;
  std::unordered_map<uint16_t, std::shared_ptr<Motor>> motors;
  std::vector<DmActData>* data_ptr_{nullptr};

  dmcan_context* ctx_{nullptr};
  dmcan_device_handle* device_{nullptr};
  bool canfd_{false};
  bool brs_{false};
  uint32_t nom_baud_{1000000};
  uint32_t dat_baud_{1000000};
  std::string sn_;
  bool closed_{false};

  mutable std::mutex mutex_;
};

}  // namespace damiao

#endif
