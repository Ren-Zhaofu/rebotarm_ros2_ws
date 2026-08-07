#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "rs_motor_sdk/rs_motor.hpp"

namespace {

constexpr std::uint16_t kCurrentLimit = 0x7018;
constexpr std::uint8_t kHostId = 0xFD;
constexpr auto kTimeout = std::chrono::milliseconds(500);

float decode_f32(const std::array<std::uint8_t, 4> &bytes) {
  float value = 0.0F;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(&value, bytes.data(), sizeof(value));
#else
  std::array<std::uint8_t, 4> reversed{bytes[3], bytes[2], bytes[1], bytes[0]};
  std::memcpy(&value, reversed.data(), sizeof(value));
#endif
  return value;
}

const char *model_name(rs_motor_sdk::MotorModel model) {
  return model == rs_motor_sdk::MotorModel::kRs06 ? "RS-06" : "RS-00";
}

double maximum_limit(rs_motor_sdk::MotorModel model) {
  return model == rs_motor_sdk::MotorModel::kRs06 ? 57.0 : 16.0;
}

bool read_current_limit(rs_motor_sdk::MotorBus &bus, float &value) {
  if (!bus.read_parameter(0, kCurrentLimit)) {
    std::cerr << "limit_cur request failed: " << bus.last_error() << '\n';
    return false;
  }
  rs_motor_sdk::ParameterResponse response;
  std::size_t motor_index = 0;
  if (!bus.receive_parameter(response, motor_index, kTimeout) ||
      motor_index != 0 || response.index != kCurrentLimit) {
    std::cerr << "limit_cur response failed: " << bus.last_error() << '\n';
    return false;
  }
  value = decode_f32(response.value);
  return std::isfinite(value);
}

bool wait_until_disabled(rs_motor_sdk::MotorBus &bus) {
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - std::chrono::steady_clock::now());
    rs_motor_sdk::MotorState state;
    std::size_t motor_index = 0;
    if (!bus.receive_state(state, motor_index, remaining)) {
      break;
    }
    if (motor_index == 0 && state.fault == 0 && state.mode == 0) {
      return true;
    }
    if (motor_index == 0 && state.fault != 0) {
      std::cerr << "motor fault=0x" << std::hex
                << static_cast<unsigned>(state.fault) << std::dec << '\n';
      return false;
    }
  }
  std::cerr << "motor did not confirm disabled mode: " << bus.last_error()
            << '\n';
  return false;
}

bool approximately_equal(float left, float right) {
  return std::abs(static_cast<double>(left) - static_cast<double>(right)) <=
         1e-4;
}

void usage(const char *program) {
  std::cerr << "Usage: " << program
            << " --execute --store <interface> <joint-id 1..6> <limit>\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 6 || std::string(argv[1]) != "--execute" ||
      std::string(argv[2]) != "--store") {
    usage(argv[0]);
    return 2;
  }

  try {
    const std::string interface_name = argv[3];
    std::size_t joint_end = 0;
    std::size_t limit_end = 0;
    const std::string joint_text = argv[4];
    const std::string limit_text = argv[5];
    const int joint_id = std::stoi(joint_text, &joint_end);
    const double requested = std::stod(limit_text, &limit_end);
    if (joint_end != joint_text.size() || limit_end != limit_text.size() ||
        joint_id < 1 || joint_id > 6 || !std::isfinite(requested)) {
      throw std::invalid_argument("invalid joint or current limit");
    }
    const auto model = joint_id <= 3 ? rs_motor_sdk::MotorModel::kRs06
                                     : rs_motor_sdk::MotorModel::kRs00;
    if (requested <= 0.0 || requested > maximum_limit(model)) {
      std::cerr << "joint" << joint_id << ' ' << model_name(model)
                << " requires 0 < limit <= " << maximum_limit(model) << '\n';
      return 2;
    }

    rs_motor_sdk::MotorBus bus;
    bus.add_motor({"joint" + std::to_string(joint_id),
                   static_cast<std::uint8_t>(joint_id), kHostId, model});
    if (!bus.connect(interface_name)) {
      std::cerr << bus.last_error() << '\n';
      return 1;
    }
    const auto cleanup = [&bus]() {
      bus.disable(0);
      bus.set_active_report(0, false);
      bus.disconnect();
    };

    if (!bus.disable(0) || !bus.set_active_report(0, true) ||
        !wait_until_disabled(bus)) {
      cleanup();
      return 1;
    }

    float old_value = 0.0F;
    if (!read_current_limit(bus, old_value)) {
      cleanup();
      return 1;
    }
    std::cout << std::fixed << std::setprecision(6) << "joint=joint" << joint_id
              << " model=" << model_name(model) << " old_limit=" << old_value
              << " new_limit=" << requested << '\n';

    const float requested_f32 = static_cast<float>(requested);
    if (!bus.write_parameter_f32(0, kCurrentLimit, requested_f32)) {
      std::cerr << "limit_cur write failed: " << bus.last_error() << '\n';
      cleanup();
      return 1;
    }
    float written_value = 0.0F;
    if (!read_current_limit(bus, written_value) ||
        !approximately_equal(written_value, requested_f32)) {
      std::cerr << "write verification failed; parameters were not stored\n";
      bus.write_parameter_f32(0, kCurrentLimit, old_value);
      cleanup();
      return 1;
    }
    std::cout << "write_verified=true\n";

    if (!bus.store_parameters(0)) {
      std::cerr << "store_parameters failed: " << bus.last_error() << '\n';
      cleanup();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    float stored_value = 0.0F;
    if (!read_current_limit(bus, stored_value) ||
        !approximately_equal(stored_value, requested_f32)) {
      std::cerr << "stored value verification failed\n";
      cleanup();
      return 1;
    }
    std::cout << "store_sent=true stored_value=" << stored_value
              << " verified=true\n";
    cleanup();
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    usage(argv[0]);
    return 2;
  }
}
