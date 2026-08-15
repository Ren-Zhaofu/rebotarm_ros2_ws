#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

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

bool request_selected(
    rebotarm_dm_motor_sdk::MotorBus &bus,
    const std::vector<std::size_t> &selected,
    std::array<rebotarm_dm_motor_sdk::MotorState, kMotorCount> &states) {
  std::array<bool, kMotorCount> wanted{};
  std::array<bool, kMotorCount> seen{};
  for (const auto index : selected) {
    wanted[index] = true;
    if (!bus.request_state(index)) {
      std::cerr << "joint" << index + 1
                << " state request: " << bus.last_error() << '\n';
      return false;
    }
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    rebotarm_dm_motor_sdk::MotorState state;
    std::size_t index = 0;
    if (bus.receive_state(state, index, std::chrono::milliseconds(20)) &&
        wanted[index]) {
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

  rebotarm_dm_motor_sdk::MotorBus bus;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const auto model = i < 3 ? rebotarm_dm_motor_sdk::MotorModel::kDm4340_48V
                             : rebotarm_dm_motor_sdk::MotorModel::kDm4310;
    bus.add_motor({"joint" + std::to_string(i + 1),
                   static_cast<std::uint16_t>(i + 1),
                   static_cast<std::uint16_t>(i + 0x11), model});
  }
  if (!bus.connect(argv[2])) {
    std::cerr << "Failed to open " << argv[2] << ": " << bus.last_error()
              << '\n';
    return 1;
  }

  std::array<rebotarm_dm_motor_sdk::MotorState, kMotorCount> before{};
  if (!request_selected(bus, selected, before)) {
    std::cerr << "Not all selected motors returned feedback; faults were not "
                 "cleared\n";
    return 1;
  }

  for (const auto index : selected) {
    if (!bus.disable(index)) {
      std::cerr << "joint" << index + 1 << " disable: " << bus.last_error()
                << '\n';
      return 1;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (const auto index : selected) {
    if (!bus.clear_error(index)) {
      std::cerr << "joint" << index + 1
                << " clear fault: " << bus.last_error() << '\n';
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::array<rebotarm_dm_motor_sdk::MotorState, kMotorCount> after{};
  if (!request_selected(bus, selected, after)) {
    std::cerr << "Clear commands were sent, but verification feedback is "
                 "incomplete\n";
    return 1;
  }

  bool success = true;
  for (const auto index : selected) {
    std::cout << "joint" << index + 1 << ": status 0x" << std::hex
              << static_cast<unsigned int>(before[index].status) << " -> 0x"
              << static_cast<unsigned int>(after[index].status) << std::dec;
    if (after[index].status == 0) {
      std::cout << "  OK\n";
    } else {
      std::cout << "  STILL ACTIVE\n";
      success = false;
    }
  }
  return success ? 0 : 1;
}
