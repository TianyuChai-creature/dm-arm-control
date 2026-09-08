#include "protocol/damiao.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <unordered_set>

namespace damiao
{

namespace
{
Control_Mode_Code toControlModeCode(Control_Mode mode)
{
  switch(mode)
  {
  case MIT_MODE:
    return MIT;
  case POS_VEL_MODE:
    return POS_VEL;
  case VEL_MODE:
    return VEL;
  case POS_FORCE_MODE:
    return POS_FORCE;
  default:
    return MIT;
  }
}

// Map device handle -> owner for C recv callback (single-process multi-instance safe).
std::mutex g_owner_mu;
std::unordered_map<dmcan_device_handle*, Motor_Control*> g_owners;

uint8_t dlc_to_len(uint8_t dlc)
{
  switch(dlc)
  {
  case 9:
    return 12;
  case 10:
    return 16;
  case 11:
    return 20;
  case 12:
    return 24;
  case 13:
    return 32;
  case 14:
    return 48;
  case 15:
    return 64;
  default:
    return dlc > 8 ? 8 : dlc;
  }
}
}  // namespace

Limit_param limit_param[Num_Of_Motor] = {
  {12.566f, 50, 5},     // DM3507
  {12.5f, 30, 10},      // DM4310
  {12.5f, 50, 10},      // DM4310_48V
  {12.5f, 10, 28},      // DM4340
  {12.5f, 20, 28},      // DM4340_48V
  {12.5f, 45, 12},      // DM6006
  {12.566f, 20, 120},   // DM6248
  {12.5f, 45, 20},      // DM8006
  {12.5f, 45, 54},      // DM8009
  {12.5f, 25, 200},     // DM10010L
  {12.5f, 20, 200},     // DM10010
  {12.5f, 280, 1},      // DMH3510
  {12.5f, 45, 10},      // DMH6215
  {12.5f, 2000, 2},     // DMS3519
  {12.5f, 45, 10}       // DMG6220
};

uint16_t encode_mit_field(float value, float min, float max, uint8_t bits)
{
  if(!std::isfinite(value) || !std::isfinite(min) || !std::isfinite(max) || min >= max ||
     bits == 0 || bits > 16)
  {
    throw std::invalid_argument("invalid MIT field value or range");
  }
  if(value < min || value > max)
  {
    throw std::out_of_range("MIT field value is outside motor limits");
  }
  const uint32_t encoded_max = (uint32_t{1} << bits) - 1;
  return static_cast<uint16_t>((value - min) / (max - min) * encoded_max);
}

Motor::Motor(DM_Motor_Type motor_type, Control_Mode ctrl_mode, uint16_t can_id, uint16_t master_id,
             uint8_t channel)
  : Can_id(can_id)
  , Master_id(master_id)
  , Motor_Type(motor_type)
  , mode(ctrl_mode)
  , channel_(channel)
{
  this->limit_param = damiao::limit_param[motor_type];
  this->last_time_ = std::chrono::steady_clock::now();
}

void Motor::updateTimeInterval()
{
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - last_time_;
  last_time_ = now;
  delta_time_ = dt.count();
}

double Motor::getTimeInterval()
{
  return delta_time_;
}

void Motor::receive_data(float q, float dq, float tau, uint8_t error_code)
{
  this->state_q = q;
  this->state_dq = dq;
  this->state_tau = tau;
  this->error_code_ = error_code;
  ++this->rx_sequence_;
}

void Motor::set_param(int key, float value)
{
  ValueType v{};
  v.value.floatValue = value;
  v.isFloat = true;
  param_map[key] = v;
}

void Motor::set_param(int key, uint32_t value)
{
  ValueType v{};
  v.value.uint32Value = value;
  v.isFloat = false;
  param_map[key] = v;
}

float Motor::get_param_as_float(int key) const
{
  auto it = param_map.find(key);
  if(it != param_map.end() && it->second.isFloat)
  {
    return it->second.value.floatValue;
  }
  return 0;
}

uint32_t Motor::get_param_as_uint32(int key) const
{
  auto it = param_map.find(key);
  if(it != param_map.end() && !it->second.isFloat)
  {
    return it->second.value.uint32Value;
  }
  return 0;
}

bool Motor::is_have_param(int key) const
{
  return param_map.find(key) != param_map.end();
}

Motor_Control::Motor_Control(uint32_t nom_baud, uint32_t dat_baud, std::string sn,
                             std::vector<DmActData>* data_ptr, bool canfd, bool brs,
                             int device_index, bool auto_enable, dmcan_device_type device_type)
  : data_ptr_(data_ptr)
  , canfd_(canfd)
  , brs_(brs)
  , nom_baud_(nom_baud)
  , dat_baud_(dat_baud)
  , sn_(std::move(sn))
{
  if(data_ptr_ == nullptr)
  {
    throw std::invalid_argument("Motor_Control: data_ptr is null");
  }

  for(const auto& act : *data_ptr_)
  {
    addMotor(std::make_shared<Motor>(act.motorType, act.mode, act.can_id, act.mst_id, act.channel));
  }

  dmcan_context_create(&ctx_);
  if(ctx_ == nullptr)
  {
    throw std::runtime_error("Motor_Control: dmcan_context_create failed");
  }

  const int dev_count = dmcan_find_devices_with_type(ctx_, static_cast<int>(device_type));
  if(dev_count <= 0)
  {
    close_device();
    throw std::runtime_error("Motor_Control: no dmcan device found");
  }
  if(device_index < 0 || device_index >= dev_count)
  {
    close_device();
    throw std::runtime_error("Motor_Control: device_index out of range");
  }

  if(!dmcan_device_get(ctx_, &device_, device_index) || device_ == nullptr)
  {
    close_device();
    throw std::runtime_error("Motor_Control: dmcan_device_get failed");
  }
  if(!dmcan_device_open(device_))
  {
    close_device();
    throw std::runtime_error("Motor_Control: failed to open dmcan device");
  }

  {
    std::lock_guard<std::mutex> lock(g_owner_mu);
    g_owners[device_] = this;
  }
  dmcan_device_hook_recv_callback(device_, &Motor_Control::recv_callback_thunk);

  std::unordered_set<uint8_t> channels;
  for(const auto& m : unique_motors())
  {
    channels.insert(m->GetChannel());
  }
  if(channels.empty())
  {
    channels.insert(0);
  }

  for(uint8_t ch : channels)
  {
    dmcan_channel_can_info info{};
    info.channel = ch;
    info.canfd = canfd_;
    info.can_baudrate = nom_baud_;
    info.canfd_baudrate = dat_baud_;
    info.can_sp = 0.75f;
    info.canfd_sp = 0.75f;
    if(!dmcan_device_set_channel_baudrate(device_, ch, info))
    {
      close_device();
      throw std::runtime_error("Motor_Control: set_channel_baudrate failed");
    }
    dmcan_device_enable_channel(device_, ch);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if(auto_enable)
  {
    enable_all();
  }

}

Motor_Control::~Motor_Control()
{
  try
  {
    disable_all();
  }
  catch(...)
  {
  }
  close_device();
}

void Motor_Control::close_device()
{
  if(closed_)
  {
    return;
  }
  closed_ = true;

  // Drop RX ownership first so in-flight callbacks become no-ops.
  if(device_ != nullptr)
  {
    {
      std::lock_guard<std::mutex> lock(g_owner_mu);
      g_owners.erase(device_);
    }

    std::unordered_set<uint8_t> channels;
    for(const auto& m : unique_motors())
    {
      channels.insert(m->GetChannel());
    }
    if(channels.empty())
    {
      channels.insert(0);
    }
    for(uint8_t ch : channels)
    {
      dmcan_device_disable_channel(device_, ch);
    }
    device_ = nullptr;
  }

  // On this host, calling both dmcan_device_close and dmcan_context_destroy triggers a
  // libusb mutex assert (same class of issue as u2canfd/test_link.py skipping full close).
  // Destroying the context alone tears down devices safely.
  if(ctx_ != nullptr)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    dmcan_context_destroy(ctx_);
    ctx_ = nullptr;
  }
}

void Motor_Control::recv_callback_thunk(dmcan_device_handle* handle, usb_rx_frame* frame)
{
  if(handle == nullptr || frame == nullptr)
  {
    return;
  }
  Motor_Control* owner = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_owner_mu);
    auto it = g_owners.find(handle);
    if(it != g_owners.end())
    {
      owner = it->second;
    }
  }
  if(owner != nullptr)
  {
    owner->on_rx_frame(*frame);
  }
}

void Motor_Control::addMotor(std::shared_ptr<Motor> DM_Motor)
{
  motors.insert({DM_Motor->GetCanId(), DM_Motor});
  motors.insert({DM_Motor->GetMasterId(), DM_Motor});
}

MotorFeedback Motor_Control::getMotorFeedback(uint16_t id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = motors.find(id);
  if(it == motors.end() || it->second == nullptr)
  {
    throw std::runtime_error("motor is not registered");
  }

  const auto& motor = *it->second;
  const double age = motor.rx_sequence_ == 0
    ? std::numeric_limits<double>::infinity()
    : std::chrono::duration<double>(std::chrono::steady_clock::now() - motor.last_time_).count();
  return MotorFeedback{
    motor.state_q,
    motor.state_dq,
    motor.state_tau,
    motor.delta_time_,
    age,
    motor.rx_sequence_,
    motor.error_code_};
}

std::vector<std::shared_ptr<Motor>> Motor_Control::unique_motors() const
{
  std::vector<std::shared_ptr<Motor>> out;
  std::unordered_set<uint16_t> seen;
  for(const auto& kv : motors)
  {
    const auto& motor = kv.second;
    if(motor == nullptr)
    {
      continue;
    }
    if(seen.insert(motor->GetCanId()).second)
    {
      out.push_back(motor);
    }
  }
  return out;
}

bool Motor_Control::send_can(uint8_t channel, uint32_t can_id, const uint8_t* data, uint8_t len)
{
  if(device_ == nullptr || closed_)
  {
    return false;
  }
  return dmcan_device_send_can(device_, channel, can_id, canfd_, false, false, brs_, len, data);
}

void Motor_Control::enable_all()
{
  for(const auto& motor : unique_motors())
  {
    switchControlMode(*motor, toControlModeCode(motor->GetMotorMode()));
    usleep(2000);
  }
  for(const auto& motor : unique_motors())
  {
    for(int j = 0; j < 5; ++j)
    {
      control_cmd(static_cast<uint16_t>(motor->GetCanId() + motor->GetMotorMode()), 0xFC,
                  motor->GetChannel());
      usleep(2000);
    }
  }
}

void Motor_Control::disable_all()
{
  for(const auto& motor : unique_motors())
  {
    for(int j = 0; j < 5; ++j)
    {
      control_cmd(static_cast<uint16_t>(motor->GetCanId() + motor->GetMotorMode()), 0xFD,
                  motor->GetChannel());
      usleep(2000);
    }
  }
}

float Motor_Control::read_motor_param(Motor& DM_Motor, uint8_t RID)
{
  const uint16_t id = DM_Motor.GetCanId();
  const uint8_t payload[8] = {static_cast<uint8_t>(id & 0xff), static_cast<uint8_t>((id >> 8) & 0xff),
                              0x33, RID, 0x00, 0x00, 0x00, 0x00};
  send_can(DM_Motor.GetChannel(), 0x7FF, payload, 8);
  usleep(2000);
  return 0;
}

void Motor_Control::save_motor_param(Motor& DM_Motor)
{
  const uint16_t id = DM_Motor.GetCanId();
  const uint16_t mode = DM_Motor.GetMotorMode();
  control_cmd(static_cast<uint16_t>(id + mode), 0xFD, DM_Motor.GetChannel());
  usleep(10000);
  const uint8_t payload[8] = {static_cast<uint8_t>(id & 0xff), static_cast<uint8_t>((id >> 8) & 0xff),
                              0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
  send_can(DM_Motor.GetChannel(), 0x7FF, payload, 8);
  usleep(100000);
}

void Motor_Control::refresh_motor_status(Motor& motor)
{
  const uint8_t payload[4] = {static_cast<uint8_t>(motor.GetCanId() & 0xff),
                              static_cast<uint8_t>((motor.GetCanId() >> 8) & 0xff), 0xCC, 0x00};
  send_can(motor.GetChannel(), 0x7FF, payload, 4);
}

void Motor_Control::control_cmd(uint16_t id, uint8_t cmd, uint8_t channel)
{
  const uint8_t payload[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
  send_can(channel, id, payload, 8);
}

void Motor_Control::write_motor_param(Motor& DM_Motor, uint8_t RID, const uint8_t data[4])
{
  const uint16_t id = DM_Motor.GetCanId();
  const uint8_t payload[8] = {static_cast<uint8_t>(id & 0xff), static_cast<uint8_t>((id >> 8) & 0xff),
                              0x55, RID, data[0], data[1], data[2], data[3]};
  send_can(DM_Motor.GetChannel(), 0x7FF, payload, 8);
}

void Motor_Control::set_zero_position(Motor& DM_Motor)
{
  control_cmd(static_cast<uint16_t>(DM_Motor.GetCanId() + DM_Motor.GetMotorMode()), 0xFE,
              DM_Motor.GetChannel());
}

void Motor_Control::control_mit(Motor& DM_Motor, float kp, float kd, float q, float dq, float tau)
{
  const uint16_t id = DM_Motor.GetCanId();
  if(motors.find(id) == motors.end())
  {
    throw std::runtime_error("control_mit: motor not registered");
  }
  auto& m = motors[id];
  const uint16_t kp_uint = encode_mit_field(kp, 0, 500, 12);
  const uint16_t kd_uint = encode_mit_field(kd, 0, 5, 12);
  const Limit_param limit_param_cmd = m->get_limit_param();
  const uint16_t q_uint =
    encode_mit_field(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
  const uint16_t dq_uint =
    encode_mit_field(dq, -limit_param_cmd.DQ_MAX, limit_param_cmd.DQ_MAX, 12);
  const uint16_t tau_uint =
    encode_mit_field(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

  const uint16_t can_id = static_cast<uint16_t>(id + MIT_MODE);
  uint8_t data[8];
  data[0] = (q_uint >> 8) & 0xff;
  data[1] = q_uint & 0xff;
  data[2] = static_cast<uint8_t>(dq_uint >> 4);
  data[3] = static_cast<uint8_t>(((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf));
  data[4] = kp_uint & 0xff;
  data[5] = static_cast<uint8_t>(kd_uint >> 4);
  data[6] = static_cast<uint8_t>(((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf));
  data[7] = tau_uint & 0xff;
  if(!send_can(DM_Motor.GetChannel(), can_id, data, 8))
  {
    throw std::runtime_error("control_mit: CAN send failed");
  }
}

void Motor_Control::control_pos_vel(Motor& DM_Motor, float pos, float vel)
{
  const uint16_t id = DM_Motor.GetCanId();
  if(motors.find(id) == motors.end())
  {
    throw std::runtime_error("control_pos_vel: motor not registered");
  }
  const uint16_t can_id = static_cast<uint16_t>(id + POS_VEL_MODE);
  uint8_t data[8];
  std::memcpy(data, &pos, 4);
  std::memcpy(data + 4, &vel, 4);
  send_can(DM_Motor.GetChannel(), can_id, data, 8);
}

void Motor_Control::control_vel(Motor& DM_Motor, float vel)
{
  const uint16_t id = DM_Motor.GetCanId();
  if(motors.find(id) == motors.end())
  {
    throw std::runtime_error("control_vel: motor not registered");
  }
  const uint16_t can_id = static_cast<uint16_t>(id + VEL_MODE);
  uint8_t data[4];
  std::memcpy(data, &vel, 4);
  send_can(DM_Motor.GetChannel(), can_id, data, 4);
}

void Motor_Control::receive_param(uint8_t* data)
{
  const uint16_t canID = (uint16_t(data[1]) << 8) | data[0];
  const uint8_t RID = data[3];
  if(motors.find(canID) == motors.end())
  {
    return;
  }
  if(is_in_ranges(RID))
  {
    const uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) |
                                 (uint32_t(data[5]) << 8) | data[4];
    motors[canID]->set_param(RID, data_uint32);
    if(RID == 10)
    {
      if(data_uint32 == 1)
      {
        motors[canID]->set_mode(MIT_MODE);
      }
      else if(data_uint32 == 2)
      {
        motors[canID]->set_mode(POS_VEL_MODE);
      }
      else if(data_uint32 == 3)
      {
        motors[canID]->set_mode(VEL_MODE);
      }
      else if(data_uint32 == 4)
      {
        motors[canID]->set_mode(POS_FORCE_MODE);
      }
    }
  }
  else
  {
    motors[canID]->set_param(RID, uint8_to_float(data + 4));
  }
}

bool Motor_Control::switchControlMode(Motor& DM_Motor, Control_Mode_Code mode)
{
  const uint8_t write_data[4] = {static_cast<uint8_t>(mode), 0x00, 0x00, 0x00};
  write_motor_param(DM_Motor, 10, write_data);
  if(motors.find(DM_Motor.GetCanId()) == motors.end())
  {
    return false;
  }
  return true;
}

bool Motor_Control::change_motor_param(Motor& DM_Motor, uint8_t RID, float data)
{
  if(is_in_ranges(RID))
  {
    uint32_t data_uint32 = float_to_uint32(data);
    write_motor_param(DM_Motor, RID, reinterpret_cast<uint8_t*>(&data_uint32));
  }
  else
  {
    write_motor_param(DM_Motor, RID, reinterpret_cast<uint8_t*>(&data));
  }
  return motors.find(DM_Motor.GetCanId()) != motors.end();
}

void Motor_Control::changeMotorLimit(Motor& DM_Motor, float P_MAX, float Q_MAX, float T_MAX)
{
  limit_param[DM_Motor.GetMotorType()] = {P_MAX, Q_MAX, T_MAX};
}

void Motor_Control::on_rx_frame(const usb_rx_frame& frame)
{
  static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
    const float span = xmax - xmin;
    const float data_norm = float(x) / float((1 << bits) - 1);
    return data_norm * span + xmin;
  };

  std::lock_guard<std::mutex> lock(mutex_);

  const uint32_t canID = frame.head.can_id;
  const uint8_t len = dlc_to_len(static_cast<uint8_t>(frame.head.dlc));
  if(len < 6)
  {
    return;
  }

  const auto it = motors.find(static_cast<uint16_t>(canID));
  if(it == motors.end())
  {
    return;
  }
  auto m = it->second;
  // Every delayed/interleaved register reply must bypass motion feedback.
  if(is_param_reply(frame.payload, len, m->GetCanId()))
  {
    if(frame.payload[2] == 0x33 || frame.payload[2] == 0x55)
    {
      uint8_t buf[8];
      std::memcpy(buf, frame.payload, 8);
      receive_param(buf);
    }
    return;
  }

  const uint16_t q_uint = (uint16_t(frame.payload[1]) << 8) | frame.payload[2];
  const uint16_t dq_uint = (uint16_t(frame.payload[3]) << 4) | (frame.payload[4] >> 4);
  const uint16_t tau_uint = (uint16_t(frame.payload[4] & 0xf) << 8) | frame.payload[5];
  const Limit_param limit_param_receive = m->get_limit_param();
  const float receive_q =
    uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
  const float receive_dq =
    uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
  const float receive_tau =
    uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
  const uint8_t error_code = static_cast<uint8_t>((frame.payload[0] >> 4) & 0x0f);
  m->receive_data(receive_q, receive_dq, receive_tau, error_code);
  m->updateTimeInterval();
}

}  // namespace damiao
