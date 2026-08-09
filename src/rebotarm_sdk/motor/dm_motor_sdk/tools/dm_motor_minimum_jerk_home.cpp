#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <variant>

#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

namespace {

using rebotarm_dm_motor_sdk::ControlMode;
using rebotarm_dm_motor_sdk::minimum_jerk;
using rebotarm_dm_motor_sdk::MotorBus;
using rebotarm_dm_motor_sdk::MotorModel;
using rebotarm_dm_motor_sdk::MotorState;
using rebotarm_dm_motor_sdk::ParameterResponse;
using rebotarm_dm_motor_sdk::PositionVelocityCommand;
using rebotarm_dm_motor_sdk::Protocol;
using rebotarm_dm_motor_sdk::Register;

constexpr std::size_t kMotorCount = 6;
constexpr double kCommandVelocityLimit = 0.05;
constexpr std::array<double, kMotorCount> kMaximumStartMagnitudes{
    2.80, 3.14, 3.14, 1.87, 1.57, 3.14};
constexpr double kMaximumTrackingError = 0.08;
constexpr double kFinalTolerance = 0.02;
constexpr auto kCycle = std::chrono::milliseconds(10);
constexpr auto kFeedbackTimeout = std::chrono::milliseconds(100);
std::atomic_bool stop_requested{false};

void signal_handler(int) { stop_requested.store(true); }

bool receive_state_set(MotorBus &bus, bool require_enabled,
                       std::array<MotorState, kMotorCount> &states) {
  std::array<bool, kMotorCount> seen{};
  const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - std::chrono::steady_clock::now());
    MotorState state;
    std::size_t index = 0;
    if (!bus.receive_state(state, index, remaining)) {
      break;
    }
    if (index >= kMotorCount || state.motor_id != index + 1 ||
        state.status >= 8 || state.status != (require_enabled ? 1 : 0)) {
      std::cerr << "Unsafe feedback from joint" << index + 1
                << ": status=" << static_cast<unsigned int>(state.status)
                << '\n';
      return false;
    }
    states[index] = state;
    seen[index] = true;
    if (std::all_of(seen.begin(), seen.end(),
                    [](bool value) { return value; })) {
      return true;
    }
  }
  std::cerr << "Incomplete six-axis feedback set: " << bus.last_error() << '\n';
  return false;
}

bool request_state_set(MotorBus &bus, bool require_enabled,
                       std::array<MotorState, kMotorCount> &states) {
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.request_state(i)) {
      std::cerr << "State request failed for joint" << i + 1 << ": "
                << bus.last_error() << '\n';
      return false;
    }
  }
  return receive_state_set(bus, require_enabled, states);
}

bool verify_parameter(MotorBus &bus, std::size_t index, Register reg,
                      double expected) {
  if (!bus.read_parameter(index, reg)) {
    return false;
  }
  ParameterResponse response;
  std::size_t response_index = 0;
  if (!bus.receive_parameter(response, response_index, kFeedbackTimeout) ||
      response_index != index || response.reg != reg) {
    return false;
  }
  if (reg == Register::kControlMode) {
    const auto *value = std::get_if<std::uint32_t>(&response.value);
    return value != nullptr && *value == static_cast<std::uint32_t>(expected);
  }
  const auto *value = std::get_if<float>(&response.value);
  return value != nullptr &&
         std::abs(static_cast<double>(*value) - expected) < 1e-4;
}

bool verify_parameters(MotorBus &bus) {
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const auto limits = Protocol::limits(bus.motor(i).model);
    if (!verify_parameter(
            bus, i, Register::kControlMode,
            static_cast<double>(ControlMode::kPositionVelocity)) ||
        !verify_parameter(bus, i, Register::kPositionMaximum,
                          limits.position) ||
        !verify_parameter(bus, i, Register::kVelocityMaximum,
                          limits.velocity) ||
        !verify_parameter(bus, i, Register::kTorqueMaximum, limits.effort)) {
      std::cerr << "Mode or protocol-limit verification failed for joint"
                << i + 1 << ": " << bus.last_error() << '\n';
      return false;
    }
  }
  return true;
}

void disable_all(MotorBus &bus) {
  if (!bus.connected()) {
    return;
  }
  for (int attempt = 0; attempt < 2; ++attempt) {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!bus.disable(i)) {
        std::cerr << "Disable failed for joint" << i + 1 << ": "
                  << bus.last_error() << '\n';
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

class DisableGuard {
public:
  explicit DisableGuard(MotorBus &bus) : bus_(bus) {}

  ~DisableGuard() {
    if (armed_) {
      disable_all(bus_);
    }
  }

  void arm() { armed_ = true; }
  void release() { armed_ = false; }

private:
  MotorBus &bus_;
  bool armed_{false};
};

} // namespace

int main(int argc, char **argv) {
  if (argc < 2 || std::string(argv[1]) != "--execute") {
    std::cerr << "Refusing motion. Usage: dm_motor_minimum_jerk_home --execute "
                 "[can0]\n";
    return 2;
  }
  const std::string interface_name = argc > 2 ? argv[2] : "can0";
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  MotorBus bus;
  for (std::uint16_t id = 1; id <= kMotorCount; ++id) {
    const auto model = id <= 3 ? MotorModel::kDm4340_48V : MotorModel::kDm4310;
    bus.add_motor({"joint" + std::to_string(id), id,
                   static_cast<std::uint16_t>(id + 0x10), model});
  }
  if (!bus.connect(interface_name)) {
    std::cerr << "Cannot open " << interface_name << ": " << bus.last_error()
              << '\n';
    return 3;
  }
  DisableGuard guard(bus);

  if (!verify_parameters(bus)) {
    return 4;
  }
  std::array<MotorState, kMotorCount> states{};
  if (!request_state_set(bus, false, states)) {
    return 5;
  }

  std::array<double, kMotorCount> starts{};
  double maximum_displacement = 0.0;
  std::cout << std::fixed << std::setprecision(6) << "Initial positions:";
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    starts[i] = states[i].position;
    maximum_displacement = std::max(maximum_displacement, std::abs(starts[i]));
    std::cout << " j" << i + 1 << '=' << starts[i];
  }
  std::cout << '\n';
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (std::abs(starts[i]) > kMaximumStartMagnitudes[i]) {
      std::cerr << "joint" << i + 1 << " start magnitude "
                << std::abs(starts[i]) << " rad exceeds its homing envelope "
                << kMaximumStartMagnitudes[i]
                << " rad; refusing automatic homing\n";
      return 6;
    }
  }

  const double duration_seconds = std::max(
      8.0, 1.875 * maximum_displacement / (0.8 * kCommandVelocityLimit));
  const auto duration = std::chrono::duration<double>(duration_seconds);
  std::cout << "Minimum-jerk duration: " << duration_seconds
            << " s, command velocity limit: " << kCommandVelocityLimit
            << " rad/s\n";

  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.send_position_velocity(
            i, PositionVelocityCommand{
                   static_cast<float>(starts[i]),
                   static_cast<float>(kCommandVelocityLimit)})) {
      std::cerr << "Preload failed for joint" << i + 1 << ": "
                << bus.last_error() << '\n';
      return 7;
    }
  }
  if (!receive_state_set(bus, false, states)) {
    return 8;
  }

  guard.arm();
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.enable(i)) {
      std::cerr << "Enable failed for joint" << i + 1 << ": "
                << bus.last_error() << '\n';
      return 9;
    }
  }
  if (!receive_state_set(bus, true, states)) {
    return 10;
  }
  std::cout << "All six arm motors enabled; executing minimum-jerk homing\n";

  const auto start_time = std::chrono::steady_clock::now();
  auto next_cycle = start_time;
  std::size_t cycle = 0;
  while (!stop_requested.load()) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - start_time);
    const auto trajectory_time = std::min(elapsed, duration);
    std::array<double, kMotorCount> targets{};
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      const auto sample =
          minimum_jerk(starts[i], 0.0, trajectory_time, duration);
      targets[i] = sample.position;
      if (!bus.send_position_velocity(
              i, PositionVelocityCommand{
                     static_cast<float>(sample.position),
                     static_cast<float>(kCommandVelocityLimit)})) {
        std::cerr << "Command failed for joint" << i + 1 << ": "
                  << bus.last_error() << '\n';
        return 11;
      }
    }
    if (!receive_state_set(bus, true, states)) {
      return 12;
    }

    double maximum_error = 0.0;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      maximum_error =
          std::max(maximum_error, std::abs(states[i].position - targets[i]));
    }
    if (maximum_error > kMaximumTrackingError) {
      std::cerr << "Tracking error " << maximum_error
                << " rad exceeds safety limit\n";
      return 13;
    }
    if (cycle % 50 == 0) {
      std::cout << "t=" << std::min(elapsed.count(), duration_seconds)
                << " max_error=" << maximum_error << '\n';
    }
    if (elapsed >= duration) {
      break;
    }
    ++cycle;
    next_cycle += kCycle;
    std::this_thread::sleep_until(next_cycle);
  }
  if (stop_requested.load()) {
    std::cerr << "Interrupted; disabling all motors\n";
    return 130;
  }

  for (int hold_cycle = 0; hold_cycle < 100; ++hold_cycle) {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!bus.send_position_velocity(
              i, PositionVelocityCommand{
                     0.0F, static_cast<float>(kCommandVelocityLimit)})) {
        return 14;
      }
    }
    if (!receive_state_set(bus, true, states)) {
      return 15;
    }
    std::this_thread::sleep_for(kCycle);
  }

  double final_maximum = 0.0;
  std::cout << "Final enabled positions:";
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    final_maximum = std::max(final_maximum, std::abs(states[i].position));
    std::cout << " j" << i + 1 << '=' << states[i].position;
  }
  std::cout << '\n';
  if (final_maximum > kFinalTolerance) {
    std::cerr << "Final position tolerance failed: " << final_maximum
              << " rad\n";
    return 16;
  }

  disable_all(bus);
  guard.release();
  if (!request_state_set(bus, false, states)) {
    std::cerr << "Could not confirm final disabled state\n";
    return 17;
  }
  std::cout << "Homing complete; all six arm motors confirmed disabled\n";
  return 0;
}
