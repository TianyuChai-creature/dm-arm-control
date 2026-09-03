#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> running{true};

void signal_handler(int signum)
{
  running = false;
  std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

std::filesystem::path config_path(int argc, char** argv)
{
  if(argc > 1)
  {
    return argv[1];
  }
  return "../config/arm.yaml";
}

std::vector<dm_openarm::MitCommand> hold_commands(const std::vector<dm_openarm::MotorState>& states)
{
  std::vector<dm_openarm::MitCommand> commands;
  commands.reserve(states.size());
  for(const auto& state : states)
  {
    commands.push_back(dm_openarm::MitCommand{2.0, 0.5, state.position, 0.0, 0.0});
  }
  return commands;
}

}  // namespace

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;

  std::signal(SIGINT, signal_handler);

  try
  {
    const auto config = dm_openarm::load_arm_config(config_path(argc, argv));
    dm_openarm::DmArm arm(config);
    arm.enable();

    arm.send_zero_mit_all();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / dm_openarm::kDefaultControlHz));
    auto commands = hold_commands(arm.states());

    while(running)
    {
      const auto current_time = clock::now();
      arm.send_mit_all(commands);

      for(const auto& state : arm.states())
      {
        std::cout << "canid is: " << state.can_id
                  << " pos: " << state.position
                  << " vel: " << state.velocity
                  << " effort: " << state.torque
                  << " time(s): " << state.feedback_interval_s << '\n';
      }

      const auto sleep_till =
        current_time + std::chrono::duration_cast<clock::duration>(
                          std::chrono::duration<double>(1.0 / dm_openarm::kDefaultControlHz));
      std::this_thread::sleep_until(sleep_till);
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
