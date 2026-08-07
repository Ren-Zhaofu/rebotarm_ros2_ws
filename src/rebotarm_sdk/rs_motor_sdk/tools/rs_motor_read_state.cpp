#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rs_motor_sdk/rs_motor.hpp"

namespace {

constexpr std::size_t kMotorCount = 6;
constexpr std::uint16_t kRunMode = 0x7005;
constexpr std::uint16_t kCurrentLimit = 0x7018;
std::atomic_bool g_running{true};

void stop(int) { g_running = false; }

std::string mode_name(std::uint8_t mode) {
  constexpr std::array<const char *, 4> names{"MIT", "position", "velocity",
                                              "current"};
  const std::string name = mode < names.size() ? names[mode] : "unknown";
  return name + "(" + std::to_string(mode) + ")";
}

std::string center_cell(const std::string &text, std::size_t width) {
  const std::string value = text.size() <= width ? text : text.substr(0, width);
  const auto padding = width - value.size();
  const auto left = padding / 2;
  return std::string(left, ' ') + value + std::string(padding - left, ' ');
}

void print_states(
    const std::array<rs_motor_sdk::MotorState, kMotorCount> &states,
    const std::array<bool, kMotorCount> &seen,
    const std::array<std::uint8_t, kMotorCount> &run_modes,
    const std::array<bool, kMotorCount> &mode_seen,
    const std::array<float, kMotorCount> &current_limits,
    const std::array<bool, kMotorCount> &current_limit_seen,
    bool clear_screen) {
  if (clear_screen) {
    std::cout << "\033[H\033[J";
  }
  constexpr int joint_width = 8;
  constexpr int mode_width = 12;
  constexpr int current_limit_width = 12;
  constexpr int position_width = 15;
  constexpr int velocity_width = 18;
  constexpr int effort_width = 10;
  constexpr int temperature_width = 10;
  constexpr int fault_width = 8;
  const std::string divider =
      "+--------+------------+------------+---------------+"
      "------------------+----------+----------+--------+\n";
  std::cout << divider << "|" << center_cell("Joint", joint_width) << "|"
            << center_cell("Mode", mode_width) << "|"
            << center_cell("Limit Cur", current_limit_width) << "|"
            << center_cell("Position (rad)", position_width) << "|"
            << center_cell("Velocity (rad/s)", velocity_width) << "|"
            << center_cell("Effort", effort_width) << "|"
            << center_cell("Temp (C)", temperature_width) << "|"
            << center_cell("Fault", fault_width) << "|\n"
            << divider;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const std::string joint = "joint" + std::to_string(i + 1);
    const std::string mode =
        mode_seen[i] ? mode_name(run_modes[i]) : "unavailable";
    std::ostringstream current_limit;
    if (current_limit_seen[i])
      current_limit << std::fixed << std::setprecision(3) << current_limits[i];
    else
      current_limit << "unavailable";
    if (!seen[i]) {
      std::cout << "|" << center_cell(joint, joint_width) << "|"
                << center_cell(mode, mode_width) << "|"
                << center_cell(current_limit.str(), current_limit_width) << "|"
                << center_cell("no feedback", position_width) << "|"
                << center_cell("", velocity_width) << "|"
                << center_cell("", effort_width) << "|"
                << center_cell("", temperature_width) << "|"
                << center_cell("", fault_width) << "|\n";
      continue;
    }
    const auto &state = states[i];
    std::ostringstream position, velocity, effort, temperature, fault;
    position << std::fixed << std::setprecision(6) << state.position;
    velocity << std::fixed << std::setprecision(6) << state.velocity;
    effort << std::fixed << std::setprecision(3) << state.effort;
    temperature << std::fixed << std::setprecision(1) << state.temperature;
    fault << "0x" << std::hex << static_cast<unsigned int>(state.fault);
    std::cout << "|" << center_cell(joint, joint_width) << "|"
              << center_cell(mode, mode_width) << "|"
              << center_cell(current_limit.str(), current_limit_width) << "|"
              << center_cell(position.str(), position_width) << "|"
              << center_cell(velocity.str(), velocity_width) << "|"
              << center_cell(effort.str(), effort_width) << "|"
              << center_cell(temperature.str(), temperature_width) << "|"
              << center_cell(fault.str(), fault_width) << "|\n";
  }
  std::cout << divider;
  std::cout << std::flush;
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
  std::array<std::uint8_t, kMotorCount> run_modes{};
  std::array<bool, kMotorCount> mode_seen{};
  std::array<float, kMotorCount> current_limits{};
  std::array<bool, kMotorCount> current_limit_seen{};
  bool success = true;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.read_parameter(i, kRunMode)) {
      std::cerr << "joint" << i + 1 << " run_mode request: " << bus.last_error()
                << '\n';
      success = false;
      continue;
    }
    rs_motor_sdk::ParameterResponse response;
    std::size_t response_index = 0;
    if (!bus.receive_parameter(response, response_index,
                               std::chrono::milliseconds(200)) ||
        response_index != i || response.index != kRunMode) {
      std::cerr << "joint" << i + 1
                << " run_mode response: " << bus.last_error() << '\n';
      success = false;
      continue;
    }
    run_modes[i] = response.value[0];
    mode_seen[i] = true;
    if (!bus.read_parameter(i, kCurrentLimit)) {
      std::cerr << "joint" << i + 1
                << " limit_cur request: " << bus.last_error() << '\n';
      success = false;
      continue;
    }
    if (!bus.receive_parameter(response, response_index,
                               std::chrono::milliseconds(200)) ||
        response_index != i || response.index != kCurrentLimit) {
      std::cerr << "joint" << i + 1
                << " limit_cur response: " << bus.last_error() << '\n';
      success = false;
      continue;
    }
    std::memcpy(&current_limits[i], response.value.data(), sizeof(float));
    current_limit_seen[i] = true;
  }
  bool reporting_ok = true;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.set_active_report(i, true)) {
      std::cerr << "joint" << i + 1 << ": " << bus.last_error() << '\n';
      reporting_ok = false;
    }
  }

  const auto feedback_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  auto next_print =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(refresh_ms);
  bool printed = false;
  while (reporting_ok && g_running &&
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
      print_states(states, seen, run_modes, mode_seen, current_limits,
                   current_limit_seen, printed);
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
    print_states(states, seen, run_modes, mode_seen, current_limits,
                 current_limit_seen, false);
  }
  for (const bool value : seen) {
    success = success && value;
  }
  success = success && reporting_ok;
  return success ? 0 : 1;
}
