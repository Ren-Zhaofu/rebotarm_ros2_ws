#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

namespace {

constexpr std::size_t kMotorCount = 6;
std::atomic_bool g_running{true};

void stop(int) { g_running = false; }

std::string status_name(std::uint8_t status) {
  switch (status) {
  case 0x0:
    return "Disabled";
  case 0x1:
    return "Enabled";
  case 0x3:
    return "Output cal";
  case 0x4:
    return "Output sensor";
  case 0x5:
    return "Encoder cal";
  case 0x8:
    return "Overvoltage";
  case 0x9:
    return "Undervoltage";
  case 0xA:
    return "Overcurrent";
  case 0xB:
    return "MOS overtemp";
  case 0xC:
    return "Motor overtemp";
  case 0xD:
    return "Comm lost";
  case 0xE:
    return "Overload";
  default:
    return "Unknown(" + std::to_string(status) + ")";
  }
}

std::string center_cell(const std::string &text, std::size_t width) {
  const std::string value = text.size() <= width ? text : text.substr(0, width);
  const auto padding = width - value.size();
  const auto left = padding / 2;
  return std::string(left, ' ') + value + std::string(padding - left, ' ');
}

void print_states(
    const std::array<rebotarm_dm_motor_sdk::MotorState, kMotorCount> &states,
    const std::array<bool, kMotorCount> &seen, bool clear_screen) {
  if (clear_screen) {
    std::cout << "\033[H\033[J";
  }
  constexpr int joint_width = 8;
  constexpr int state_width = 14;
  constexpr int position_width = 15;
  constexpr int velocity_width = 18;
  constexpr int effort_width = 12;
  constexpr int temperature_width = 12;
  const std::string divider = "+--------+--------------+---------------+-------"
                              "-----------+------------+"
                              "------------+------------+\n";
  std::cout << divider << "|" << center_cell("Joint", joint_width) << "|"
            << center_cell("State", state_width) << "|"
            << center_cell("Position (rad)", position_width) << "|"
            << center_cell("Velocity (rad/s)", velocity_width) << "|"
            << center_cell("Effort (Nm)", effort_width) << "|"
            << center_cell("MOS Temp (C)", temperature_width) << "|"
            << center_cell("Rotor Temp", temperature_width) << "|\n"
            << divider;

  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const std::string joint = "joint" + std::to_string(i + 1);
    if (!seen[i]) {
      std::cout << "|" << center_cell(joint, joint_width) << "|"
                << center_cell("unavailable", state_width) << "|"
                << center_cell("no feedback", position_width) << "|"
                << center_cell("", velocity_width) << "|"
                << center_cell("", effort_width) << "|"
                << center_cell("", temperature_width) << "|"
                << center_cell("", temperature_width) << "|\n";
      continue;
    }

    const auto &state = states[i];
    std::ostringstream position;
    std::ostringstream velocity;
    std::ostringstream effort;
    position << std::fixed << std::setprecision(6) << state.position;
    velocity << std::fixed << std::setprecision(6) << state.velocity;
    effort << std::fixed << std::setprecision(3) << state.effort;
    std::cout << "|" << center_cell(joint, joint_width) << "|"
              << center_cell(status_name(state.status), state_width) << "|"
              << center_cell(position.str(), position_width) << "|"
              << center_cell(velocity.str(), velocity_width) << "|"
              << center_cell(effort.str(), effort_width) << "|"
              << center_cell(std::to_string(state.mos_temperature),
                             temperature_width)
              << "|"
              << center_cell(std::to_string(state.rotor_temperature),
                             temperature_width)
              << "|\n";
  }
  std::cout << divider << std::flush;
}

bool read_states(
    rebotarm_dm_motor_sdk::MotorBus &bus,
    std::array<rebotarm_dm_motor_sdk::MotorState, kMotorCount> &states,
    std::array<bool, kMotorCount> &seen) {
  bool success = true;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.request_state(i)) {
      std::cerr << "joint" << i + 1 << " state request: " << bus.last_error()
                << '\n';
      success = false;
      continue;
    }
    rebotarm_dm_motor_sdk::MotorState state;
    std::size_t response_index = 0;
    if (!bus.receive_state(state, response_index,
                           std::chrono::milliseconds(100)) ||
        response_index != i) {
      seen[i] = false;
      success = false;
      continue;
    }
    states[i] = state;
    seen[i] = true;
  }
  return success;
}

} // namespace

int main(int argc, char **argv) {
  const std::string interface_name = argc > 1 ? argv[1] : "can0";
  const bool watch = argc > 2 && std::string(argv[2]) == "--watch";
  if (argc > 2 && !watch) {
    std::cerr << "usage: " << argv[0] << " [can0] [--watch [refresh-ms]]\n";
    return 2;
  }

  int refresh_ms = 100;
  try {
    if (argc > 3) {
      refresh_ms = std::stoi(argv[3]);
    }
    if (refresh_ms < 20 || refresh_ms > 60000 || argc > 4) {
      throw std::invalid_argument("range");
    }
  } catch (const std::exception &) {
    std::cerr << "refresh-ms must be an integer from 20 to 60000\n";
    return 2;
  }

  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  rebotarm_dm_motor_sdk::MotorBus bus;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const auto model = i < 3 ? rebotarm_dm_motor_sdk::MotorModel::kDm4340_48V
                             : rebotarm_dm_motor_sdk::MotorModel::kDm4310;
    bus.add_motor({"joint" + std::to_string(i + 1),
                   static_cast<std::uint16_t>(i + 1),
                   static_cast<std::uint16_t>(i + 0x11), model});
  }
  if (!bus.connect(interface_name)) {
    std::cerr << "Failed to open " << interface_name << ": " << bus.last_error()
              << '\n';
    return 1;
  }

  std::array<rebotarm_dm_motor_sdk::MotorState, kMotorCount> states{};
  std::array<bool, kMotorCount> seen{};
  bool success = true;
  bool printed = false;
  do {
    const auto cycle_start = std::chrono::steady_clock::now();
    success = read_states(bus, states, seen) && success;
    print_states(states, seen, printed);
    printed = true;
    if (watch && g_running) {
      const auto next_cycle =
          cycle_start + std::chrono::milliseconds(refresh_ms);
      while (g_running && std::chrono::steady_clock::now() < next_cycle) {
        const auto remaining = next_cycle - std::chrono::steady_clock::now();
        std::this_thread::sleep_for(
            std::min(remaining, std::chrono::steady_clock::duration(
                                    std::chrono::milliseconds(20))));
      }
    }
  } while (watch && g_running);

  return success ? 0 : 1;
}
