#include "dm_openarm/yaml_loader.hpp"

#include <cassert>
#include <cmath>
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
    if(std::filesystem::exists(path / "config" / "arm.yaml"))
    {
      return path;
    }
    if(std::filesystem::exists(path / "dm_openarm" / "config" / "arm.yaml"))
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

left:
  motors:
    - name: L_end_effector
      model: DM4310
      mode: MIT
      can_id: 0x01
      mst_id: 0x11
    - name: L_wrist_2
      model: DM4310
      mode: MIT
      can_id: 0x02
      mst_id: 0x12
    - name: L_wrist_3
      model: DM4310
      mode: MIT
      can_id: 0x03
      mst_id: 0x13
    - name: L_elbow
      model: DM8009
      mode: MIT
      can_id: 0x04
      mst_id: 0x14
    - name: L_shoulder
      model: DM8009
      mode: MIT
      can_id: 0x05
      mst_id: 0x15
right:
  motors:
    - name: R_end_effector
      model: DM4310
      mode: MIT
      can_id: 0x21
      mst_id: 0x31
    - name: R_wrist_2
      model: DM4310
      mode: MIT
      can_id: 0x22
      mst_id: 0x32
    - name: R_wrist_3
      model: DM4310
      mode: MIT
      can_id: 0x23
      mst_id: 0x33
    - name: R_elbow
      model: DM8009
      mode: MIT
      can_id: 0x24
      mst_id: 0x34
    - name: R_shoulder
      model: DM8009
      mode: MIT
      can_id: 0x25
      mst_id: 0x35
)yaml";
}

void test_default_config_loads()
{
  const auto config = dm_openarm::load_arm_config(project_root() / "config" / "arm.yaml");

  assert(config.usb_serial == "52A871B1AA5EF4E239371A5083463F26");
  assert(config.nom_baud == 1000000);
  assert(config.dat_baud == 1000000);
  assert(config.canfd == false);
  assert(config.brs == false);
  assert(config.device_index == 0);
  assert(config.loop_period == std::chrono::milliseconds(1));
  assert(config.motors.size() == 10);
  assert(config.is_dual());
  assert(config.left.present() && config.left.begin == 0 && config.left.count == 5);
  assert(config.right.present() && config.right.begin == 5 && config.right.count == 5);
  assert(config.left.gravity.enabled == true);
  assert(std::abs(config.left.gravity.scale - 0.9) < 1e-9);
  assert(config.left.gravity.coupled.basis.size() == 13);
  assert(config.left.gravity.coupled.weights.size() == 5);
  assert(config.right.gravity.enabled == false);

  assert(config.motors[0].can_id == 0x01);
  assert(config.motors[3].model == dm_openarm::MotorModel::DM8009);
  assert(config.motors[5].can_id == 0x21);
  assert(config.motors[9].can_id == 0x25);
}

void test_hex_ids_are_parsed()
{
  const auto config = dm_openarm::load_arm_config(write_case("hex_ids.yaml", base_yaml()));
  assert(config.motors[0].can_id == static_cast<std::uint16_t>(0x01));
  assert(config.motors[5].can_id == static_cast<std::uint16_t>(0x21));
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

void test_legacy_root_motors_rejected()
{
  const char* yaml = R"yaml(
usb:
  serial: "X"
  nominal_baud: 1000000
  data_baud: 1000000
control:
  loop_period_ms: 1
motors:
  - name: a
    model: DM4310
    mode: MIT
    can_id: 0x01
    mst_id: 0x11
)yaml";
  expect_throw_contains(
    [&]() { dm_openarm::load_arm_config(write_case("legacy.yaml", yaml)); },
    "left:");
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
  test_legacy_root_motors_rejected();

  return 0;
}
