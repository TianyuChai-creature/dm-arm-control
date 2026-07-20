#include "dm_openarm/yaml_loader.hpp"

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

std::filesystem::path project_root()
{
  auto cwd = std::filesystem::current_path();
  for(auto path = cwd; !path.empty(); path = path.parent_path())
  {
    if(std::filesystem::exists(path / "config" / "arm_5dof.yaml"))
    {
      return path;
    }
    if(std::filesystem::exists(path / "dm_openarm" / "config" / "arm_5dof.yaml"))
    {
      return path / "dm_openarm";
    }
    if(path == path.root_path())
    {
      break;
    }
  }
  throw std::runtime_error("could not locate dm_openarm project root");
}

std::filesystem::path write_case(const std::string& name, const std::string& yaml)
{
  const auto dir = std::filesystem::temp_directory_path() / "dm_openarm_yaml_loader_tests";
  std::filesystem::create_directories(dir);
  const auto path = dir / name;
  std::ofstream out(path);
  out << yaml;
  return path;
}

void expect_throw_contains(const std::function<void()>& fn, const std::string& needle)
{
  try
  {
    fn();
  }
  catch(const std::exception& e)
  {
    const std::string message = e.what();
    if(message.find(needle) == std::string::npos)
    {
      std::cerr << "expected exception containing '" << needle << "', got '" << message << "'\n";
      std::abort();
    }
    return;
  }
  std::cerr << "expected exception containing '" << needle << "'\n";
  std::abort();
}

const char* base_yaml()
{
  return R"yaml(
usb:
  serial: "14AA044B241402B10DDBDAFE448040BB"
  nominal_baud: 1000000
  data_baud: 1000000

control:
  loop_period_ms: 1

motors:
  - name: end_effector
    model: DM4310
    mode: MIT
    can_id: 0x01
    mst_id: 0x11
  - name: wrist_2
    model: DM4310
    mode: MIT
    can_id: 0x02
    mst_id: 0x12
  - name: wrist_3
    model: DM4310
    mode: MIT
    can_id: 0x03
    mst_id: 0x13
  - name: elbow
    model: DM8009
    mode: MIT
    can_id: 0x04
    mst_id: 0x14
  - name: shoulder
    model: DM8009
    mode: MIT
    can_id: 0x05
    mst_id: 0x15
)yaml";
}

void test_default_config_loads()
{
  const auto config = dm_openarm::load_arm_config(project_root() / "config" / "arm_5dof.yaml");

  assert(config.usb_serial == "52A871B1AA5EF4E239371A5083463F26");
  assert(config.nom_baud == 1000000);
  assert(config.dat_baud == 1000000);
  assert(config.canfd == false);
  assert(config.brs == false);
  assert(config.device_index == 0);
  assert(config.loop_period == std::chrono::milliseconds(1));
  assert(config.motors.size() == 5);
  assert(config.gravity.enabled == false);
  assert(config.gravity.joints.size() == 5);

  assert(config.motors[0].name == "end_effector");
  assert(config.motors[0].model == dm_openarm::MotorModel::DM4310);
  assert(config.motors[0].mode == dm_openarm::ControlMode::MIT);
  assert(config.motors[0].can_id == 0x01);
  assert(config.motors[0].mst_id == 0x11);

  assert(config.motors[3].name == "elbow");
  assert(config.motors[3].model == dm_openarm::MotorModel::DM8009);
  assert(config.motors[3].can_id == 0x04);
  assert(config.motors[3].mst_id == 0x14);

  assert(config.motors[4].name == "shoulder");
  assert(config.motors[4].model == dm_openarm::MotorModel::DM8009);
  assert(config.motors[4].can_id == 0x05);
  assert(config.motors[4].mst_id == 0x15);
}

void test_hex_ids_are_parsed()
{
  const auto config = dm_openarm::load_arm_config(write_case("hex_ids.yaml", base_yaml()));
  assert(config.motors[0].can_id == static_cast<std::uint16_t>(0x01));
  assert(config.motors[0].mst_id == static_cast<std::uint16_t>(0x11));
}

void test_duplicate_can_id_fails()
{
  std::string yaml = base_yaml();
  yaml.replace(yaml.find("can_id: 0x02"), std::string("can_id: 0x02").size(), "can_id: 0x01");
  expect_throw_contains(
    [&]() { dm_openarm::load_arm_config(write_case("duplicate_can.yaml", yaml)); },
    "duplicate CAN ID");
}

void test_duplicate_mst_id_fails()
{
  std::string yaml = base_yaml();
  yaml.replace(yaml.find("mst_id: 0x12"), std::string("mst_id: 0x12").size(), "mst_id: 0x11");
  expect_throw_contains(
    [&]() { dm_openarm::load_arm_config(write_case("duplicate_mst.yaml", yaml)); },
    "duplicate MST ID");
}

void test_empty_usb_serial_fails()
{
  std::string yaml = base_yaml();
  yaml.replace(
    yaml.find("serial: \"14AA044B241402B10DDBDAFE448040BB\""),
    std::string("serial: \"14AA044B241402B10DDBDAFE448040BB\"").size(),
    "serial: \"\"");
  expect_throw_contains(
    [&]() { dm_openarm::load_arm_config(write_case("empty_serial.yaml", yaml)); },
    "usb.serial");
}

void test_non_mit_mode_fails()
{
  std::string yaml = base_yaml();
  yaml.replace(yaml.find("mode: MIT"), std::string("mode: MIT").size(), "mode: VEL");
  expect_throw_contains(
    [&]() { dm_openarm::load_arm_config(write_case("non_mit.yaml", yaml)); },
    "only MIT mode");
}

void test_unknown_model_fails_with_dm8009p_hint()
{
  std::string yaml = base_yaml();
  yaml.replace(yaml.find("model: DM8009"), std::string("model: DM8009").size(), "model: DM8009P");
  expect_throw_contains(
    [&]() { dm_openarm::load_arm_config(write_case("unknown_model.yaml", yaml)); },
    "use DM8009");
}

}  // namespace

int main()
{
  test_default_config_loads();
  test_hex_ids_are_parsed();
  test_duplicate_can_id_fails();
  test_duplicate_mst_id_fails();
  test_empty_usb_serial_fails();
  test_non_mit_mode_fails();
  test_unknown_model_fails_with_dm8009p_hint();

  return 0;
}
