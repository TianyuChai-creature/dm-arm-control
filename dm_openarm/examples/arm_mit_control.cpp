#include "dm_openarm/dm_arm.hpp"
#include "dm_openarm/mit_control_command.hpp"
#include "dm_openarm/yaml_loader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char* kCommandFifo = "/tmp/dm_openarm_mit_control.cmd";
constexpr double kDefaultKp = 8.0;
constexpr double kDefaultKd = 0.4;
constexpr double kMaxTargetVelocityRadPerSec = 0.6;

std::atomic<bool> running{true};

void signal_handler(int signum)
{
  running = false;
  std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

class FifoCommandReader {
public:
  explicit FifoCommandReader(const char* path)
    : path_(path)
  {
    ::unlink(path_.c_str());
    if(::mkfifo(path_.c_str(), 0666) != 0)
    {
      throw std::runtime_error("failed to create FIFO: " + path_);
    }

    read_fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    if(read_fd_ < 0)
    {
      throw std::runtime_error("failed to open FIFO for reading: " + path_);
    }

    keepalive_fd_ = ::open(path_.c_str(), O_WRONLY | O_NONBLOCK);
  }

  ~FifoCommandReader()
  {
    if(keepalive_fd_ >= 0)
    {
      ::close(keepalive_fd_);
    }
    if(read_fd_ >= 0)
    {
      ::close(read_fd_);
    }
    ::unlink(path_.c_str());
  }

  std::vector<std::string> read_lines()
  {
    std::vector<std::string> lines;
    char buffer[256];

    while(true)
    {
      const auto count = ::read(read_fd_, buffer, sizeof(buffer));
      if(count > 0)
      {
        pending_.append(buffer, static_cast<std::size_t>(count));
        continue;
      }
      break;
    }

    std::size_t newline = std::string::npos;
    while((newline = pending_.find('\n')) != std::string::npos)
    {
      lines.push_back(pending_.substr(0, newline));
      pending_.erase(0, newline + 1);
    }

    return lines;
  }

private:
  std::string path_;
  int read_fd_{-1};
  int keepalive_fd_{-1};
  std::string pending_;
};

std::uint16_t require_known_motor(
  std::uint16_t can_id,
  const std::vector<dm_openarm::MotorConfig>& motors)
{
  for(const auto& motor : motors)
  {
    if(motor.can_id == can_id)
    {
      return can_id;
    }
  }
  throw std::invalid_argument("unknown motor CAN ID");
}

std::size_t motor_index(
  std::uint16_t can_id,
  const std::vector<dm_openarm::MotorConfig>& motors)
{
  for(std::size_t i = 0; i < motors.size(); ++i)
  {
    if(motors[i].can_id == can_id)
    {
      return i;
    }
  }
  throw std::invalid_argument("unknown motor CAN ID");
}

double approach_target(double current, double target, double step)
{
  if(current < target - step)
  {
    return current + step;
  }
  if(current > target + step)
  {
    return current - step;
  }
  return target;
}

void print_status(
  const std::vector<dm_openarm::MotorState>& states,
  const std::vector<double>& targets)
{
  for(std::size_t i = 0; i < states.size(); ++i)
  {
    const auto& state = states[i];
    std::cout << "can_id=" << state.can_id
              << " position=" << state.position
              << " velocity=" << state.velocity
              << " torque=" << state.torque
              << " target=" << targets[i]
              << " feedback_interval_s=" << state.feedback_interval_s << '\n';
  }
  std::cout.flush();
}

}  // namespace

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;

  if(argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
    return 1;
  }

  std::signal(SIGINT, signal_handler);

  try
  {
    const auto config = dm_openarm::load_arm_config(std::filesystem::path(argv[1]));
    dm_openarm::DmArm arm(config);
    arm.enable();
    arm.send_zero_mit_all();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / dm_openarm::kDefaultControlHz));

    const auto initial_states = arm.states();
    std::vector<double> home_targets;
    std::vector<double> requested_targets;
    std::vector<double> active_targets;
    home_targets.reserve(initial_states.size());
    requested_targets.reserve(initial_states.size());
    active_targets.reserve(initial_states.size());
    for(const auto& state : initial_states)
    {
      home_targets.push_back(state.position);
      requested_targets.push_back(state.position);
      active_targets.push_back(state.position);
    }

    FifoCommandReader fifo(kCommandFifo);
    std::cout << "Listening for MIT commands on " << kCommandFifo << '\n';
    std::cout << "MIT gains: kp=" << kDefaultKp
              << " kd=" << kDefaultKd
              << " max_target_velocity_rad_per_s=" << kMaxTargetVelocityRadPerSec << '\n';
    std::cout << "Examples:\n"
              << "  echo \"status\" > " << kCommandFifo << '\n'
              << "  echo \"move 0x01 0.08\" > " << kCommandFifo << '\n'
              << "  echo \"home 0x01\" > " << kCommandFifo << '\n'
              << "  echo \"stop\" > " << kCommandFifo << '\n';
    std::cout.flush();

    while(running)
    {
      const auto current_time = clock::now();

      for(const auto& line : fifo.read_lines())
      {
        try
        {
          const auto command = dm_openarm::parse_mit_control_command(line);
          std::cout << "received command: " << line << '\n';
          switch(command.kind)
          {
          case dm_openarm::MitControlCommandKind::Status:
            print_status(arm.states(), requested_targets);
            break;
          case dm_openarm::MitControlCommandKind::Move: {
            require_known_motor(command.can_id, config.motors);
            const auto index = motor_index(command.can_id, config.motors);
            requested_targets[index] += command.delta_rad;
            std::cout << "updated target can_id=" << command.can_id
                      << " target=" << requested_targets[index] << '\n';
            std::cout.flush();
            break;
          }
          case dm_openarm::MitControlCommandKind::HomeOne: {
            require_known_motor(command.can_id, config.motors);
            const auto index = motor_index(command.can_id, config.motors);
            requested_targets[index] = home_targets[index];
            std::cout << "homing can_id=" << command.can_id
                      << " target=" << requested_targets[index] << '\n';
            std::cout.flush();
            break;
          }
          case dm_openarm::MitControlCommandKind::HomeAll:
            requested_targets = home_targets;
            std::cout << "homing all motors\n";
            std::cout.flush();
            break;
          case dm_openarm::MitControlCommandKind::Stop:
            running = false;
            std::cout << "stopping MIT control loop\n";
            std::cout.flush();
            break;
          }
        }
        catch(const std::exception& e)
        {
          std::cerr << "Command rejected: " << e.what() << '\n';
          std::cerr.flush();
        }
      }

      const double step =
        kMaxTargetVelocityRadPerSec *
        1.0 / dm_openarm::kDefaultControlHz;
      std::vector<dm_openarm::MitCommand> commands;
      commands.reserve(config.motors.size());
      for(std::size_t i = 0; i < config.motors.size(); ++i)
      {
        active_targets[i] = approach_target(active_targets[i], requested_targets[i], step);
        commands.push_back(dm_openarm::MitCommand{kDefaultKp, kDefaultKd, active_targets[i], 0.0, 0.0});
      }
      arm.send_mit_all(commands);

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
