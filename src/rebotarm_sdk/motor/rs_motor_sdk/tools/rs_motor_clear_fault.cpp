#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
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
                    [&seen](std::size_t index) { return seen[index]; })) {
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

  std::array<bool, kMotorCount> reporting{};
  const auto cleanup = [&]() {
    for (const auto index : selected) {
      bus.disable(index);
      if (reporting[index]) {
        bus.set_active_report(index, false);
      }
    }
    bus.disconnect();
  };

  for (const auto index : selected) {
    if (!bus.set_active_report(index, true)) {
      std::cerr << "joint" << index + 1 << " feedback: " << bus.last_error()
                << '\n';
      cleanup();
      return 1;
    }
    reporting[index] = true;
  }

  std::array<rs_motor_sdk::MotorState, kMotorCount> before{};
  if (!collect_selected(bus, selected, before,
                        std::chrono::milliseconds(800))) {
    std::cerr << "Not all selected motors returned feedback; faults were not "
                 "cleared\n";
    cleanup();
    return 1;
  }

  for (const auto index : selected) {
    if (!bus.disable(index, true)) {
      std::cerr << "joint" << index + 1 << " clear fault: " << bus.last_error()
                << '\n';
      cleanup();
      return 1;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  std::array<rs_motor_sdk::MotorState, kMotorCount> after{};
  if (!collect_selected(bus, selected, after, std::chrono::milliseconds(800))) {
    std::cerr << "Clear command was sent, but verification feedback is "
                 "incomplete\n";
    cleanup();
    return 1;
  }

  bool success = true;
  for (const auto index : selected) {
    std::cout << "joint" << index + 1 << ": fault 0x" << std::hex
              << static_cast<unsigned int>(before[index].fault) << " -> 0x"
              << static_cast<unsigned int>(after[index].fault) << std::dec;
    if (after[index].fault == 0) {
      std::cout << "  OK\n";
    } else {
      std::cout << "  STILL ACTIVE\n";
      success = false;
    }
  }
  cleanup();
  return success ? 0 : 1;
}
