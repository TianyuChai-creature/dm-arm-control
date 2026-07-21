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

void parse_motors_sequence(
  const YAML::Node& motors,
  const std::string& path,
  std::vector<MotorConfig>& out,
  std::set<std::uint16_t>& can_ids,
  std::set<std::uint16_t>& mst_ids)
{
  if(!motors.IsSequence())
  {
    throw std::runtime_error("config field must be a sequence: " + path);
  }
  for(std::size_t i = 0; i < motors.size(); ++i)
  {
    const auto motor = motors[i];
    const std::string prefix = path + "[" + std::to_string(i) + "]";

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

    out.push_back(motor_config);
  }
}

GravityConfig parse_gravity(const YAML::Node& gravity, const std::string& path, std::size_t n_motors)
{
  GravityConfig gcfg;
  if(gravity["enabled"])
  {
    gcfg.enabled = gravity["enabled"].as<bool>();
  }
  if(gravity["scale"])
  {
    gcfg.scale = gravity["scale"].as<double>();
  }
  if(gravity["use_measured_q"])
  {
    gcfg.use_measured_q = gravity["use_measured_q"].as<bool>();
  }
  if(gravity["mode"])
  {
    const std::string mode = gravity["mode"].as<std::string>();
    if(mode != "coupled")
    {
      throw std::runtime_error(
        path + ".mode must be 'coupled' (decoupled mode has been removed)");
    }
  }
  if(gravity["joints"])
  {
    throw std::runtime_error(
      path + ".joints is no longer supported; use gravity.coupled only");
  }
  if(gravity["coupled"])
  {
    const auto coupled = gravity["coupled"];
    if(!coupled["basis"] || !coupled["basis"].IsSequence())
    {
      throw std::runtime_error(path + ".coupled.basis must be a sequence");
    }
    if(!coupled["weights"] || !coupled["weights"].IsSequence())
    {
      throw std::runtime_error(path + ".coupled.weights must be a sequence");
    }
    CoupledGravityParam cp;
    for(std::size_t k = 0; k < coupled["basis"].size(); ++k)
    {
      cp.basis.push_back(coupled["basis"][k].as<std::string>());
    }
    if(cp.basis.empty())
    {
      throw std::runtime_error(path + ".coupled.basis must not be empty");
    }
    if(coupled["weights"].size() != n_motors)
    {
      throw std::runtime_error(
        path + ".coupled.weights length must match limb motors length (" +
        std::to_string(n_motors) + ")");
    }
    for(std::size_t j = 0; j < coupled["weights"].size(); ++j)
    {
      const auto row = coupled["weights"][j];
      if(!row.IsSequence() || row.size() != cp.basis.size())
      {
        throw std::runtime_error(
          path + ".coupled.weights[" + std::to_string(j) +
          "] must be a sequence of length basis");
      }
      std::vector<double> w;
      w.reserve(cp.basis.size());
      for(std::size_t k = 0; k < row.size(); ++k)
      {
        w.push_back(row[k].as<double>());
      }
      cp.weights.push_back(std::move(w));
    }
    gcfg.coupled = std::move(cp);
  }
  if(gcfg.enabled &&
     (gcfg.coupled.basis.empty() || gcfg.coupled.weights.empty()))
  {
    throw std::runtime_error(
      path + ".enabled is true but gravity.coupled basis/weights are missing");
  }
  return gcfg;
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

  ArmConfig config;
  config.usb_serial = scalar_string(require_node(usb, "serial", "usb"), "usb.serial");
  if(config.usb_serial.empty())
  {
    throw std::runtime_error("usb.serial must not be empty");
  }

  config.nom_baud = parse_uint32(require_node(usb, "nominal_baud", "usb"), "usb.nominal_baud");
  config.dat_baud = parse_uint32(require_node(usb, "data_baud", "usb"), "usb.data_baud");

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

  // Dual-arm only: require left: and right: (no root motors:/gravity:).
  if(root["motors"])
  {
    throw std::runtime_error(
      "root motors: is no longer supported; put motors under left: and right:");
  }
  if(root["gravity"])
  {
    throw std::runtime_error(
      "root gravity: is no longer supported; put gravity under left: and right:");
  }
  if(!root["left"] || !root["right"])
  {
    throw std::runtime_error(
      "config must define both left: and right: limbs (dual-arm only)");
  }

  {
    const auto left = root["left"];
    config.left.begin = config.motors.size();
    parse_motors_sequence(
      require_node(left, "motors", "left"), "left.motors", config.motors, can_ids, mst_ids);
    config.left.count = config.motors.size() - config.left.begin;
    if(config.left.count == 0)
    {
      throw std::runtime_error("left.motors must not be empty");
    }
    if(left["gravity"])
    {
      config.left.gravity =
        parse_gravity(left["gravity"], "left.gravity", config.left.count);
    }
  }
  {
    const auto right = root["right"];
    config.right.begin = config.motors.size();
    parse_motors_sequence(
      require_node(right, "motors", "right"), "right.motors", config.motors, can_ids, mst_ids);
    config.right.count = config.motors.size() - config.right.begin;
    if(config.right.count == 0)
    {
      throw std::runtime_error("right.motors must not be empty");
    }
    if(right["gravity"])
    {
      config.right.gravity =
        parse_gravity(right["gravity"], "right.gravity", config.right.count);
    }
  }

  if(config.motors.empty())
  {
    throw std::runtime_error("motors must contain at least one motor");
  }

  return config;
}

}  // namespace dm_openarm
