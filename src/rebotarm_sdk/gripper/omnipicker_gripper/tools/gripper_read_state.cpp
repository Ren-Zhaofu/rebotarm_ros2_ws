#include "rebotarm_gripper_sdk/gripper.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc > 4) {
    std::cerr << "Usage: " << argv[0]
              << " [can_interface] [feedback_id] [timeout_ms]\n";
    return 2;
  }
  try {
    const std::string interface_name = argc > 1 ? argv[1] : "can0";
    const auto feedback_id = argc > 2
                                 ? static_cast<std::uint16_t>(std::stoul(argv[2], nullptr, 0))
                                 : rebotarm_gripper_sdk::kDefaultCanId;
    const int timeout_ms = argc > 3 ? std::stoi(argv[3]) : 1000;
    if (timeout_ms < 0) throw std::invalid_argument("timeout_ms must not be negative");

    rebotarm_gripper_sdk::Gripper gripper;
    if (!gripper.connect(interface_name, feedback_id, feedback_id)) {
      std::cerr << gripper.last_error() << '\n';
      return 1;
    }
    rebotarm_gripper_sdk::State state;
    if (!gripper.receive(state, std::chrono::milliseconds(timeout_ms))) {
      std::cerr << gripper.last_error() << '\n';
      return 1;
    }
    std::cout << "id=0x" << std::hex << state.can_id
              << " fault=0x" << static_cast<unsigned int>(state.fault_code)
              << " status=0x" << static_cast<unsigned int>(state.status)
              << std::dec << std::fixed << std::setprecision(3)
              << " position=" << state.position
              << " velocity=" << state.velocity
              << " force=" << state.force << '\n';
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 2;
  }
  return 0;
}
