#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rs_motor_sdk/rs_motor.hpp"

namespace {

constexpr std::size_t kMotorCount = 6;

std::vector<std::size_t> parse_indices(int argc, char **argv) {
  std::vector<std::size_t> indices;
  for (int i = 3; i < argc; ++i) {
    std::size_t consumed = 0;
    const auto id = std::stoul(argv[i], &consumed, 10);
    if (consumed != std::string(argv[i]).size() || id < 1 || id > kMotorCount) {
      throw std::invalid_argument("joint ID must be from 1 to 6");
    }
    const auto index = id - 1;
    if (std::find(indices.begin(), indices.end(), index) == indices.end()) {
      indices.push_back(index);
    }
  }
  if (indices.empty()) {
    throw std::invalid_argument("at least one joint ID is required");
  }
  return indices;
}

bool collect_selected(rs_motor_sdk::MotorBus &bus,
                      const std::vector<std::size_t> &selected,
                      std::array<rs_motor_sdk::MotorState, kMotorCount> &states,
                      std::chrono::milliseconds timeout) {
  std::array<bool, kMotorCount> seen{};
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    rs_motor_sdk::MotorState state;
    std::size_t index = 0;
    if (bus.receive_state(state, index, std::chrono::milliseconds(20))) {
      states[index] = state;
      seen[index] = true;
    }
    if (std::all_of(selected.begin(), selected.end(),
                    [&seen, &states](std::size_t index) {
                      return seen[index] && states[index].mode == 0;
                    })) {
      return true;
    }
  }
  return false;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 4 || std::string(argv[1]) != "--execute") {
    std::cerr << "usage: " << argv[0]
              << " --execute <can-interface> <joint-id> [joint-id ...]\n";
    return 2;
  }

  std::vector<std::size_t> selected;
  try {
    selected = parse_indices(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }

  const std::array<rs_motor_sdk::MotorModel, kMotorCount> models{
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs06,
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs00,
      rs_motor_sdk::MotorModel::kRs00, rs_motor_sdk::MotorModel::kRs00};
  rs_motor_sdk::MotorBus bus;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    bus.add_motor({"joint" + std::to_string(i + 1),
                   static_cast<std::uint8_t>(i + 1), 0xFD, models[i]});
  }
  if (!bus.connect(argv[2])) {
    std::cerr << bus.last_error() << '\n';
    return 1;
  }

  for (const auto index : selected) {
    if (!bus.set_active_report(index, true)) {
      std::cerr << "joint" << index + 1 << " start report: "
                << bus.last_error() << '\n';
      return 1;
    }
  }
  for (int attempt = 0; attempt < 2; ++attempt) {
    for (const auto index : selected) {
      if (!bus.disable(index)) {
        std::cerr << "joint" << index + 1 << " disable: " << bus.last_error()
                  << '\n';
        return 1;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::array<rs_motor_sdk::MotorState, kMotorCount> states{};
  const bool feedback_ok =
      collect_selected(bus, selected, states, std::chrono::milliseconds(800));
  bool success = feedback_ok;
  for (const auto index : selected) {
    const bool disabled = feedback_ok && states[index].mode == 0;
    std::cout << "joint" << index + 1 << ": "
              << (disabled ? "Disabled" : "disable verification failed")
              << '\n';
    success = success && disabled;
    bus.set_active_report(index, false);
  }
  bus.disconnect();
  return success ? 0 : 1;
}
