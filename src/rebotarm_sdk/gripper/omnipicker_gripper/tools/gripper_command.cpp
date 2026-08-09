#include "rebotarm_gripper_sdk/gripper.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace {

void usage(const char *program) {
  std::cerr << "Usage: " << program
            << " <position 0..1> [velocity 0..1] [force 0..1]"
               " [can_interface] [can_id]\n"
            << "Defaults: velocity=1 force=1 can_interface=can0 can_id=0x07\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 6) {
    usage(argv[0]);
    return 2;
  }

  try {
    rebotarm_gripper_sdk::Command command;
    command.position = std::stod(argv[1]);
    if (argc > 2) command.velocity = std::stod(argv[2]);
    if (argc > 3) command.force = std::stod(argv[3]);
    const std::string interface_name = argc > 4 ? argv[4] : "can0";
    const auto can_id = argc > 5
                            ? static_cast<std::uint16_t>(std::stoul(argv[5], nullptr, 0))
                            : rebotarm_gripper_sdk::kDefaultCanId;

    // Validate all arguments before opening or writing to hardware.
    (void)rebotarm_gripper_sdk::Protocol::command(command, can_id);
    rebotarm_gripper_sdk::Gripper gripper;
    if (!gripper.connect(interface_name, can_id, can_id)) {
      std::cerr << gripper.last_error() << '\n';
      return 1;
    }
    // Repeated frames make the one-shot tool robust to devices expecting a
    // short command stream, while keeping the target after the process exits.
    for (int index = 0; index < 5; ++index) {
      if (!gripper.send(command)) {
        std::cerr << gripper.last_error() << '\n';
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "gripper 0x" << std::hex << can_id << std::dec
              << " target position=" << command.position << '\n';
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    usage(argv[0]);
    return 2;
  }
  return 0;
}
