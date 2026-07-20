#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>

namespace {

std::filesystem::path config_path(int argc, char** argv)
{
  if(argc > 1)
  {
    return argv[1];
  }
  return "../config/arm_5dof.yaml";
}

void print_states(const std::vector<dm_openarm::MotorState>& states)
{
  for(const auto& state : states)
  {
    std::cout << "canid is: " << state.can_id
              << " pos: " << state.position
              << " vel: " << state.velocity
              << " effort: " << state.torque
              << " time(s): " << state.feedback_interval_s << '\n';
  }
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    const auto config = dm_openarm::load_arm_config(config_path(argc, argv));
    std::cout << "=== dm_openarm check_comm ===\n"
              << "SN=" << config.usb_serial << '\n'
              << "nom_baud=" << config.nom_baud << " dat_baud=" << config.dat_baud
              << " canfd=" << (config.canfd ? "true" : "false")
              << " brs=" << (config.brs ? "true" : "false")
              << " device_index=" << config.device_index << '\n';

    dm_openarm::DmArm arm(config);
    arm.connect();

    int rx_hits = 0;
    for(int i = 0; i < 200; ++i)
    {
      arm.send_zero_mit_all();
      const auto states = arm.states();
      if(i % 20 == 0)
      {
        print_states(states);
      }
      for(const auto& state : states)
      {
        if(state.feedback_interval_s > 0.0 || std::abs(state.position) > 1e-9 ||
           std::abs(state.velocity) > 1e-9 || std::abs(state.torque) > 1e-9)
        {
          ++rx_hits;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if(rx_hits == 0)
    {
      std::cerr << "FAIL: no motor feedback (check power, wiring, baud, canfd/brs, IDs)\n";
      try
      {
        arm.disable();
      }
      catch(...)
      {
      }
      return 3;
    }
    std::cout << "PASS: motor feedback received — link OK (hits=" << rx_hits << ")\n"
              << std::flush;
    try
    {
      arm.disable();
    }
    catch(...)
    {
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
