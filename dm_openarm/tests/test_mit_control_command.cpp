#include "dm_openarm/mit_control_command.hpp"
#include "dm_openarm/types.hpp"
#include "protocol/damiao.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

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

void test_status()
{
  const auto command = dm_openarm::parse_mit_control_command("status");
  assert(command.kind == dm_openarm::MitControlCommandKind::Status);
}

void test_move_hex_id()
{
  const auto command = dm_openarm::parse_mit_control_command("move 0x01 0.03");
  assert(command.kind == dm_openarm::MitControlCommandKind::Move);
  assert(command.can_id == static_cast<std::uint16_t>(0x01));
  assert_close(command.delta_rad, 0.03);
}

void test_move_decimal_id()
{
  const auto command = dm_openarm::parse_mit_control_command("move 1 -0.03");
  assert(command.kind == dm_openarm::MitControlCommandKind::Move);
  assert(command.can_id == static_cast<std::uint16_t>(1));
  assert_close(command.delta_rad, -0.03);
}

void test_move_rejects_large_delta()
{
  expect_throw_contains(
    []() { dm_openarm::parse_mit_control_command("move 0x01 0.2"); },
    "exceeds limit");
}

void test_move_accepts_larger_tuning_delta()
{
  const auto command = dm_openarm::parse_mit_control_command("move 0x01 0.08");
  assert(command.kind == dm_openarm::MitControlCommandKind::Move);
  assert(command.can_id == static_cast<std::uint16_t>(0x01));
  assert_close(command.delta_rad, 0.08);
}

void test_home_all()
{
  const auto command = dm_openarm::parse_mit_control_command("home all");
  assert(command.kind == dm_openarm::MitControlCommandKind::HomeAll);
}

void test_home_one()
{
  const auto command = dm_openarm::parse_mit_control_command("home 0x01");
  assert(command.kind == dm_openarm::MitControlCommandKind::HomeOne);
  assert(command.can_id == static_cast<std::uint16_t>(0x01));
}

void test_stop()
{
  const auto command = dm_openarm::parse_mit_control_command("stop");
  assert(command.kind == dm_openarm::MitControlCommandKind::Stop);
}

void test_unknown_command_rejected()
{
  expect_throw_contains(
    []() { dm_openarm::parse_mit_control_command("dance 0x01"); },
    "unknown command");
}

void test_mit_encoding_rejects_invalid_values()
{
  assert(damiao::encode_mit_field(0.0f, 0.0f, 500.0f, 12) == 0);
  assert(damiao::encode_mit_field(500.0f, 0.0f, 500.0f, 12) == 4095);
  expect_throw_contains(
    []() { damiao::encode_mit_field(501.0f, 0.0f, 500.0f, 12); },
    "outside motor limits");
  expect_throw_contains(
    []() {
      damiao::encode_mit_field(
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 500.0f, 12);
    },
    "invalid MIT field");
}

void test_feedback_freshness()
{
  dm_openarm::MotorState state;
  assert(!state.feedback_fresh());
  state.rx_sequence = 1;
  state.last_rx_age_s = 0.05;
  assert(state.feedback_fresh(0.1));
  state.last_rx_age_s = 0.2;
  assert(!state.feedback_fresh(0.1));
}

}  // namespace

int main()
{
  test_status();
  test_move_hex_id();
  test_move_decimal_id();
  test_move_rejects_large_delta();
  test_move_accepts_larger_tuning_delta();
  test_home_all();
  test_home_one();
  test_stop();
  test_unknown_command_rejected();
  test_mit_encoding_rejects_invalid_values();
  test_feedback_freshness();

  return 0;
}
