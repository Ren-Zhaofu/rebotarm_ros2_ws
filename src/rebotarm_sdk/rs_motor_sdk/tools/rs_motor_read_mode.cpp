#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "rs_motor_sdk/rs_motor.hpp"

namespace {
constexpr std::uint16_t kRunMode = 0x7005;
constexpr std::size_t kMotorCount = 6;

const char *mode_name(std::uint8_t mode) {
  switch (mode) {
  case 0:
    return "MIT";
  case 1:
    return "position";
  case 2:
    return "velocity";
  case 3:
    return "current";
  default:
    return "unknown";
  }
}
} // namespace

int main(int argc, char **argv) {
  if (argc > 2) {
    std::cerr << "usage: " << argv[0] << " [can0]\n";
    return 2;
  }
  const std::string interface_name = argc == 2 ? argv[1] : "can0";
  const std::array<rs_motor_sdk::MotorModel, kMotorCount> models{
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs06,
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs00,
      rs_motor_sdk::MotorModel::kRs00, rs_motor_sdk::MotorModel::kRs00};
  rs_motor_sdk::MotorBus bus;
  for (std::size_t i = 0; i < kMotorCount; ++i)
    bus.add_motor({"joint" + std::to_string(i + 1),
                   static_cast<std::uint8_t>(i + 1), 0xFD, models[i]});
  if (!bus.connect(interface_name)) {
    std::cerr << bus.last_error() << '\n';
    return 1;
  }

  bool ok = true;
  std::cout << "Joint   Parameter  Raw  Mode\n";
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!bus.read_parameter(i, kRunMode)) {
      std::cerr << "joint" << i + 1 << " request: " << bus.last_error() << '\n';
      ok = false;
      continue;
    }
    rs_motor_sdk::ParameterResponse response;
    std::size_t response_index = 0;
    if (!bus.receive_parameter(response, response_index,
                               std::chrono::milliseconds(200)) ||
        response_index != i || response.index != kRunMode) {
      std::cerr << "joint" << i + 1
                << " run_mode response: " << bus.last_error() << '\n';
      ok = false;
      continue;
    }
    const auto mode = response.value[0];
    std::cout << std::left << std::setw(8) << ("joint" + std::to_string(i + 1))
              << "0x7005     " << std::setw(5) << unsigned(mode)
              << mode_name(mode) << '\n';
  }
  bus.disconnect();
  return ok ? 0 : 1;
}
