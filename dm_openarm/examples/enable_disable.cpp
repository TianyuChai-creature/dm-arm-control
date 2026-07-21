#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <chrono>
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
  return "../config/arm.yaml";
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    const auto config = dm_openarm::load_arm_config(config_path(argc, argv));
    dm_openarm::DmArm arm(config);

    std::cout << "Connecting and enabling motors\n";
    arm.connect();
    arm.send_zero_mit_all();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Disabling motors\n";
    arm.disable();
  }
  catch(const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
