#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

void print_usage(const char* program)
{
  std::cout << "Usage:\n"
            << "  " << program << " <config.yaml> all [--runtime-only]\n"
            << "  " << program << " <config.yaml> <can_id> [--runtime-only]\n";
}

std::uint16_t parse_can_id(const std::string& text)
{
  std::size_t parsed = 0;
  unsigned long value = 0;
  try
  {
    value = std::stoul(text, &parsed, 0);
  }
  catch(const std::exception&)
  {
    throw std::invalid_argument("invalid CAN ID: " + text);
  }

  if(parsed != text.size() || value > std::numeric_limits<std::uint16_t>::max())
  {
    throw std::invalid_argument("invalid CAN ID: " + text);
  }

  return static_cast<std::uint16_t>(value);
}

const dm_openarm::MotorState* find_state(
  const std::vector<dm_openarm::MotorState>& states,
  std::uint16_t can_id)
{
  for(const auto& state : states)
  {
    if(state.can_id == can_id)
    {
      return &state;
    }
  }
  return nullptr;
}

void print_state_line(const std::string& prefix, const dm_openarm::MotorState& state)
{
  std::cout << prefix
            << " can_id=" << state.can_id
            << " position=" << state.position
            << " velocity=" << state.velocity
            << " torque=" << state.torque
            << " feedback_interval_s=" << state.feedback_interval_s << '\n';
}

void print_target_state(
  const std::string& prefix,
  const std::vector<dm_openarm::MotorState>& states,
  std::uint16_t can_id)
{
  const auto* state = find_state(states, can_id);
  if(state)
  {
    print_state_line(prefix, *state);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  if(argc < 3 || argc > 4)
  {
    print_usage(argv[0]);
    return 1;
  }

  const std::filesystem::path config_path = argv[1];
  const std::string target = argv[2];
  bool persist = true;

  if(argc == 4)
  {
    const std::string option = argv[3];
    if(option == "--runtime-only")
    {
      persist = false;
    }
    else
    {
      print_usage(argv[0]);
      return 1;
    }
  }

  try
  {
    const auto config = dm_openarm::load_arm_config(config_path);
    dm_openarm::DmArm arm(config);
    arm.enable();

    arm.send_zero_mit_all();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / dm_openarm::kDefaultControlHz));
    const auto before_states = arm.states();

    std::vector<std::uint16_t> target_ids;
    if(target == "all")
    {
      for(const auto& motor : config.motors)
      {
        target_ids.push_back(motor.can_id);
      }
    }
    else
    {
      target_ids.push_back(parse_can_id(target));
    }

    for(const auto can_id : target_ids)
    {
      std::cout << "Target motor CAN ID: " << can_id << '\n';
      print_target_state("Before set zero:", before_states, can_id);
      std::cout << "Sending set zero command\n";
      arm.set_zero(can_id, persist);
      std::cout << "Sent set zero command\n";
      if(persist)
      {
        std::cout << "Saved zero position to flash\n";
      }
      else
      {
        std::cout << "Runtime-only zero position; not saved to flash\n";
      }
    }

    arm.send_zero_mit_all();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / dm_openarm::kDefaultControlHz));
    const auto after_states = arm.states();
    for(const auto can_id : target_ids)
    {
      print_target_state("After set zero:", after_states, can_id);
    }

    arm.disable();
  }
  catch(const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
