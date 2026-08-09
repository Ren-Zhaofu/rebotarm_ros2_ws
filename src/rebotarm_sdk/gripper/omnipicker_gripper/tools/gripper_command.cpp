#include "rebotarm_gripper_sdk/gripper.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void usage(const char *program) {
  std::cerr << "Usage: " << program
            << " <position 0..1> [velocity 0..1] [force 0..1]"
               " [can_interface] [can_id] [duration_ms]\n"
            << "Defaults: velocity=1 force=1 can_interface=can0"
               " can_id=0x07 duration_ms=1000\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 7) {
    usage(argv[0]);
    return 2;
  }

  try {
    rebotarm_gripper_sdk::Command command;
    command.position = std::stod(argv[1]);
    if (argc > 2) command.velocity = std::stod(argv[2]);
    if (argc > 3) command.force = std::stod(argv[3]);
    const std::string interface_name = argc > 4 ? argv[4] : "can0";
    const auto parsed_can_id = argc > 5
                                   ? std::stoul(argv[5], nullptr, 0)
                                   : rebotarm_gripper_sdk::kDefaultCanId;
    if (parsed_can_id > 0x7FFUL) {
      throw std::invalid_argument("can_id must be within [0, 0x7FF]");
    }
    const auto can_id = static_cast<std::uint16_t>(parsed_can_id);
    const auto duration_ms = argc > 6 ? std::stoul(argv[6]) : 1000UL;
    if (duration_ms < 100UL || duration_ms > 60000UL) {
      throw std::invalid_argument("duration_ms must be within [100, 60000]");
    }

    // Validate all arguments before opening or writing to hardware.
    (void)rebotarm_gripper_sdk::Protocol::command(command, can_id);
    rebotarm_gripper_sdk::Gripper gripper;
    if (!gripper.connect(interface_name, can_id, can_id)) {
      std::cerr << gripper.last_error() << '\n';
      return 1;
    }
    const auto interval = std::chrono::milliseconds(10);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(duration_ms);
    bool feedback_received = false;
    rebotarm_gripper_sdk::State last_state;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto next_send = std::chrono::steady_clock::now() + interval;
      if (!gripper.send(command)) {
        std::cerr << gripper.last_error() << '\n';
        return 1;
      }
      rebotarm_gripper_sdk::State state;
      if (gripper.receive(state, std::chrono::milliseconds(2))) {
        last_state = state;
        feedback_received = true;
      }
      std::this_thread::sleep_until(next_send);
    }
    if (!feedback_received) {
      std::cerr << "Command frames were queued for " << duration_ms
                << " ms, but no feedback was received from CAN ID 0x"
                << std::hex << can_id << std::dec
                << ". Check gripper power, CAN_H/CAN_L, common ground,"
                   " termination, bitrate, and feedback ID.\n";
      return 1;
    }
    if (last_state.fault_code != 0) {
      std::cerr << "Gripper feedback reports fault 0x" << std::hex
                << static_cast<unsigned int>(last_state.fault_code) << std::dec
                << ".\n";
      return 1;
    }
    std::cout << "gripper 0x" << std::hex << can_id << std::dec
              << " confirmed target=" << command.position
              << " feedback_position=" << last_state.position << '\n';
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    usage(argv[0]);
    return 2;
  }
  return 0;
}
