#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rs_motor_sdk/rs_motor.hpp"

namespace {

constexpr std::size_t kMotorCount = 6;
std::atomic_bool g_running{true};

void stop(int) { g_running = false; }

void print_states(
    const std::array<rs_motor_sdk::MotorState, kMotorCount> &states,
    const std::array<bool, kMotorCount> &seen, bool clear_screen) {
  if (clear_screen) {
    std::cout << "\033[H\033[J";
  }
  constexpr int joint_width = 8;
  constexpr int position_width = 15;
  constexpr int velocity_width = 18;
  constexpr int effort_width = 10;
  constexpr int temperature_width = 10;
  constexpr int fault_width = 8;
  const std::string divider =
      "+--------+---------------+------------------+----------+----------+--------+\n";
  std::cout << divider
            << "| " << std::left << std::setw(joint_width - 1) << "Joint"
            << "| " << std::setw(position_width - 1) << "Position (rad)"
            << "| " << std::setw(velocity_width - 1) << "Velocity (rad/s)"
            << "| " << std::setw(effort_width - 1) << "Effort"
            << "| " << std::setw(temperature_width - 1) << "Temp (C)"
            << "| " << std::setw(fault_width - 1) << "Fault" << "|\n"
            << divider;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const std::string joint = "joint" + std::to_string(i + 1);
    if (!seen[i]) {
      std::cout << "| " << std::left << std::setw(joint_width - 1) << joint
                << "| " << std::setw(position_width - 1) << "no feedback"
                << "| " << std::setw(velocity_width - 1) << ""
                << "| " << std::setw(effort_width - 1) << ""
                << "| " << std::setw(temperature_width - 1) << ""
                << "| " << std::setw(fault_width - 1) << "" << "|\n";
      continue;
    }
    const auto &state = states[i];
    std::ostringstream position, velocity, effort, temperature, fault;
    position << std::fixed << std::setprecision(6) << state.position;
    velocity << std::fixed << std::setprecision(6) << state.velocity;
    effort << std::fixed << std::setprecision(3) << state.effort;
    temperature << std::fixed << std::setprecision(1) << state.temperature;
    fault << "0x" << std::hex << static_cast<unsigned int>(state.fault);
    std::cout << "| " << std::left << std::setw(joint_width - 1) << joint
              << "| " << std::right << std::setw(position_width - 1) << position.str()
              << "| " << std::setw(velocity_width - 1) << velocity.str()
              << "| " << std::setw(effort_width - 1) << effort.str()
              << "| " << std::setw(temperature_width - 1) << temperature.str()
              << "| " << std::left << std::setw(fault_width - 1) << fault.str()
              << "|\n";
  }
  std::cout << divider;
  std::cout << std::flush;
}

} // namespace

int main(int argc, char **argv) {
  const std::string interface_name = argc > 1 ? argv[1] : "can0";
  const bool watch = argc > 2 && std::string(argv[2]) == "--watch";
  if (argc > 2 && !watch) {
    std::cerr << "usage: " << argv[0]
              << " [can0] [--watch [refresh-ms]]\n";
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
  const std::array<rs_motor_sdk::MotorModel, kMotorCount> models{
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs06,
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs00,
      rs_motor_sdk::MotorModel::kRs00, rs_motor_sdk::MotorModel::kRs00};

  rs_motor_sdk::MotorBus bus;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    bus.add_motor({"joint" + std::to_string(i + 1),
                   static_cast<std::uint8_t>(i + 1), 0xFD, models[i]});
  }
  if (!bus.connect(interface_name)) {
    std::cerr << bus.last_error() << '\n';
    return 1;
  }

  std::array<rs_motor_sdk::MotorState, kMotorCount> states{};
  std::array<bool, kMotorCount> seen{};
  bool success = true;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.set_active_report(i, true)) {
      std::cerr << "joint" << i + 1 << ": " << bus.last_error() << '\n';
      success = false;
    }
  }

  const auto feedback_deadline = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(500);
  auto next_print = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(refresh_ms);
  bool printed = false;
  while (success && g_running &&
         (watch || std::chrono::steady_clock::now() < feedback_deadline)) {
    rs_motor_sdk::MotorState state;
    std::size_t index = 0;
    if (bus.receive_state(state, index, std::chrono::milliseconds(20))) {
      states[index] = state;
      seen[index] = true;
    }
    bool complete = true;
    for (const bool value : seen) {
      complete = complete && value;
    }
    const auto now = std::chrono::steady_clock::now();
    if (watch && complete && now >= next_print) {
      print_states(states, seen, printed);
      printed = true;
      next_print = now + std::chrono::milliseconds(refresh_ms);
    } else if (!watch && complete) {
      break;
    }
  }

  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.set_active_report(i, false)) {
      std::cerr << "joint" << i + 1 << " stop report: " << bus.last_error()
                << '\n';
      success = false;
    }
  }
  bus.disconnect();

  if (!watch || !printed) {
    print_states(states, seen, false);
  }
  for (const bool value : seen) {
    success = success && value;
  }
  return success ? 0 : 1;
}
