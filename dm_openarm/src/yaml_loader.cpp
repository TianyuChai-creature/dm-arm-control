#include "dm_openarm/yaml_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dm_openarm {
namespace {

YAML::Node require_node(const YAML::Node& node, const std::string& key, const std::string& path)
{
  const auto child = node[key];
  if(!child)
  {
    throw std::runtime_error("missing required config field: " + path + "." + key);
  }
  return child;
}

std::string scalar_string(const YAML::Node& node, const std::string& path)
{
  if(!node.IsScalar())
  {
    throw std::runtime_error("config field must be scalar: " + path);
  }
  return node.Scalar();
}

std::uint32_t parse_uint32(const YAML::Node& node, const std::string& path)
{
  const std::string text = scalar_string(node, path);
  std::size_t parsed = 0;
  unsigned long value = 0;
  try
  {
    value = std::stoul(text, &parsed, 0);
  }
  catch(const std::exception&)
  {
    throw std::runtime_error("config field must be an unsigned integer: " + path);
  }

  if(parsed != text.size() || value > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::runtime_error("config field is outside uint32 range: " + path);
  }

  return static_cast<std::uint32_t>(value);
}

std::uint16_t parse_uint16(const YAML::Node& node, const std::string& path)
{
  const auto value = parse_uint32(node, path);
  if(value > std::numeric_limits<std::uint16_t>::max())
  {
    throw std::runtime_error("config field is outside uint16 range: " + path);
  }
  return static_cast<std::uint16_t>(value);
}

MotorModel parse_model(const YAML::Node& node, const std::string& path)
{
  const std::string value = scalar_string(node, path);
  if(value == "DM4310")
  {
    return MotorModel::DM4310;
  }
  if(value == "DM8009")
  {
    return MotorModel::DM8009;
  }
  if(value == "DM8009P")
  {
    throw std::runtime_error("unknown motor model DM8009P in " + path + "; use DM8009");
  }
  throw std::runtime_error("unknown motor model in " + path + ": " + value);
}

ControlMode parse_mode(const YAML::Node& node, const std::string& path)
{
  const std::string value = scalar_string(node, path);
  if(value == "MIT")
  {
    return ControlMode::MIT;
  }
  throw std::runtime_error("only MIT mode is supported in " + path + ": " + value);
}

}  // namespace

ArmConfig load_arm_config(const std::filesystem::path& path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path.string());
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error("failed to load arm config " + path.string() + ": " + e.what());
  }

  const auto usb = require_node(root, "usb", "root");
  const auto control = require_node(root, "control", "root");
  const auto motors = require_node(root, "motors", "root");
  if(!motors.IsSequence())
  {
    throw std::runtime_error("config field must be a sequence: motors");
  }

  ArmConfig config;
  config.usb_serial = scalar_string(require_node(usb, "serial", "usb"), "usb.serial");
  if(config.usb_serial.empty())
  {
    throw std::runtime_error("usb.serial must not be empty");
  }

  config.nom_baud = parse_uint32(require_node(usb, "nominal_baud", "usb"), "usb.nominal_baud");
  config.dat_baud = parse_uint32(require_node(usb, "data_baud", "usb"), "usb.data_baud");

  // Optional fields; default classic CAN 1M (canfd=false, brs=false) to match u2canfd station config.
  if(usb["canfd"])
  {
    config.canfd = usb["canfd"].as<bool>();
  }
  if(usb["brs"])
  {
    config.brs = usb["brs"].as<bool>();
  }
  if(usb["device_index"])
  {
    config.device_index = static_cast<int>(parse_uint32(usb["device_index"], "usb.device_index"));
  }

  config.loop_period = std::chrono::milliseconds(
    parse_uint32(require_node(control, "loop_period_ms", "control"), "control.loop_period_ms"));

  std::set<std::uint16_t> can_ids;
  std::set<std::uint16_t> mst_ids;
  for(std::size_t i = 0; i < motors.size(); ++i)
  {
    const auto motor = motors[i];
    const std::string prefix = "motors[" + std::to_string(i) + "]";

    MotorConfig motor_config;
    motor_config.name = scalar_string(require_node(motor, "name", prefix), prefix + ".name");
    motor_config.model = parse_model(require_node(motor, "model", prefix), prefix + ".model");
    motor_config.mode = parse_mode(require_node(motor, "mode", prefix), prefix + ".mode");
    motor_config.can_id = parse_uint16(require_node(motor, "can_id", prefix), prefix + ".can_id");
    motor_config.mst_id = parse_uint16(require_node(motor, "mst_id", prefix), prefix + ".mst_id");

    if(!can_ids.insert(motor_config.can_id).second)
    {
      std::ostringstream oss;
      oss << "duplicate CAN ID: 0x" << std::hex << motor_config.can_id;
      throw std::runtime_error(oss.str());
    }
    if(!mst_ids.insert(motor_config.mst_id).second)
    {
      std::ostringstream oss;
      oss << "duplicate MST ID: 0x" << std::hex << motor_config.mst_id;
      throw std::runtime_error(oss.str());
    }

    config.motors.push_back(motor_config);
  }

  if(config.motors.empty())
  {
    throw std::runtime_error("motors must contain at least one motor");
  }

  return config;
}

}  // namespace dm_openarm
