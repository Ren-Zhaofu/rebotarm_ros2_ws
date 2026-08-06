#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>

#include "rs_motor_sdk/rs_motor.hpp"

namespace {

constexpr std::size_t kMotorCount = 6;

} // namespace

int main(int argc, char **argv) {
  const std::string interface_name = argc > 1 ? argv[1] : "can0";
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

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(500);
  while (success && std::chrono::steady_clock::now() < deadline) {
    rs_motor_sdk::MotorState state;
    std::size_t index = 0;
    if (!bus.receive_state(state, index, std::chrono::milliseconds(50))) {
      continue;
    }
    states[index] = state;
    seen[index] = true;
    bool complete = true;
    for (const bool value : seen) {
      complete = complete && value;
    }
    if (complete) {
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

  std::cout << "joint  position(rad)  velocity(rad/s)  effort  temp(C)  fault\n";
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!seen[i]) {
      std::cout << "joint" << i + 1 << "  no feedback\n";
      success = false;
      continue;
    }
    const auto &state = states[i];
    std::cout << "joint" << i + 1 << "  " << std::fixed
              << std::setprecision(6) << std::setw(13) << state.position << "  "
              << std::setw(15) << state.velocity << "  " << std::setw(7)
              << std::setprecision(3) << state.effort << "  " << std::setw(7)
              << std::setprecision(1) << state.temperature << "  0x" << std::hex
              << static_cast<unsigned int>(state.fault) << std::dec << '\n';
  }
  return success ? 0 : 1;
}
