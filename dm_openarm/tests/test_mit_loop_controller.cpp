#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/mit_loop_controller.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

dm_openarm::ArmConfig test_config()
{
  return dm_openarm::ArmConfig{
    "TEST_SERIAL",
    1000000,
    1000000,
    false,
    false,
    0,
    std::chrono::milliseconds(1),
    {
      dm_openarm::MotorConfig{"joint_1", dm_openarm::MotorModel::DM4310, dm_openarm::ControlMode::MIT, 0x01, 0x11},
      dm_openarm::MotorConfig{"joint_2", dm_openarm::MotorModel::DM8009, dm_openarm::ControlMode::MIT, 0x02, 0x12},
    }};
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

void assert_close(double actual, double expected)
{
  assert(std::abs(actual - expected) < 1e-9);
}

void test_initial_commands_are_zero()
{
  dm_openarm::DmArm arm(test_config());
  dm_openarm::MitLoopController loop(arm);

  assert(!loop.running());
  const auto commands = loop.commands();
  assert(commands.size() == 2);
  assert_close(commands[0].kp, 0.0);
  assert_close(commands[0].q, 0.0);
  assert_close(commands[1].tau, 0.0);
}

void test_set_command_by_can_id()
{
  dm_openarm::DmArm arm(test_config());
  dm_openarm::MitLoopController loop(arm);

  loop.set_command(0x02, dm_openarm::MitCommand{10.0, 0.8, 0.2, 0.0, 0.0});

  const auto commands = loop.commands();
  assert_close(commands[0].kp, 0.0);
  assert_close(commands[1].kp, 10.0);
  assert_close(commands[1].kd, 0.8);
  assert_close(commands[1].q, 0.2);
}

void test_set_all_commands_checks_size()
{
  dm_openarm::DmArm arm(test_config());
  dm_openarm::MitLoopController loop(arm);

  loop.set_all_commands({
    dm_openarm::MitCommand{12.0, 0.6, 0.1, 0.0, 0.0},
    dm_openarm::MitCommand{10.0, 0.8, -0.1, 0.0, 0.0},
  });

  const auto commands = loop.commands();
  assert_close(commands[0].kp, 12.0);
  assert_close(commands[0].q, 0.1);
  assert_close(commands[1].kp, 10.0);
  assert_close(commands[1].q, -0.1);

  expect_throw_contains(
    [&]() { loop.set_all_commands({dm_openarm::MitCommand{}}); },
    "command count");
}

void test_unknown_can_id_rejected()
{
  dm_openarm::DmArm arm(test_config());
  dm_openarm::MitLoopController loop(arm);

  expect_throw_contains(
    [&]() { loop.set_command(0x09, dm_openarm::MitCommand{1.0, 0.1, 0.0, 0.0, 0.0}); },
    "unknown motor CAN ID");
}

}  // namespace

int main()
{
  test_initial_commands_are_zero();
  test_set_command_by_can_id();
  test_set_all_commands_checks_size();
  test_unknown_can_id_rejected();

  return 0;
}
