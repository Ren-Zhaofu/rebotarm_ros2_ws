#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <variant>

#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

namespace
{

using rebotarm_dm_motor_sdk::MotorBus;
using rebotarm_dm_motor_sdk::MotorModel;
using rebotarm_dm_motor_sdk::ParameterResponse;
using rebotarm_dm_motor_sdk::Register;

const char * register_name(Register reg)
{
  switch (reg) {
    case Register::kControlMode:
      return "control_mode";
    case Register::kPositionMaximum:
      return "pmax";
    case Register::kVelocityMaximum:
      return "vmax";
    case Register::kTorqueMaximum:
      return "tmax";
    default:
      return "unknown";
  }
}

void print_value(const ParameterResponse & response)
{
  if (const auto * integer = std::get_if<std::uint32_t>(&response.value)) {
    std::cout << *integer;
  } else {
    std::cout << std::get<float>(response.value);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string interface_name = argc > 1 ? argv[1] : "can0";
  MotorBus bus;

  // Verified reBotArm limits: joints 1-3 use +/-10 rad/s and +/-28 Nm,
  // while joints 4-7 use +/-30 rad/s and +/-10 Nm.
  for (std::uint16_t id = 1; id <= 7; ++id) {
    const auto model = id <= 3 ? MotorModel::kDm4340_48V : MotorModel::kDm4310;
    bus.add_motor(
      {"joint" + std::to_string(id), id, static_cast<std::uint16_t>(id + 0x10), model});
  }

  if (!bus.connect(interface_name)) {
    std::cerr << "Failed to open " << interface_name << ": " << bus.last_error() << '\n';
    return 1;
  }

  constexpr std::array<Register, 4> registers{
    Register::kControlMode,
    Register::kPositionMaximum,
    Register::kVelocityMaximum,
    Register::kTorqueMaximum,
  };
  for (std::size_t motor_index = 0; motor_index < bus.size(); ++motor_index) {
    std::cout << bus.motor(motor_index).name;
    for (const auto reg : registers) {
      if (!bus.read_parameter(motor_index, reg)) {
        std::cerr << "\nCAN send failed: " << bus.last_error() << '\n';
        return 2;
      }
      ParameterResponse response;
      std::size_t response_index = 0;
      if (!bus.receive_parameter(
          response, response_index, std::chrono::milliseconds(100)) ||
        response_index != motor_index)
      {
        std::cerr << "\nNo matching response for " << bus.motor(motor_index).name
                  << ' ' << register_name(reg) << ": " << bus.last_error() << '\n';
        return 3;
      }
      std::cout << ' ' << register_name(reg) << '=';
      print_value(response);
    }
    std::cout << '\n';
  }

  return 0;
}
