#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace rebotarm_gripper_sdk {

inline constexpr std::uint16_t kDefaultCanId = 0x07;

struct CanFrame {
  std::uint16_t id{0};
  std::uint8_t size{0};
  std::array<std::uint8_t, 8> data{};
};

struct Command {
  double position{0.0};
  double velocity{1.0};
  double force{1.0};
  double acceleration{1.0};
  double deceleration{1.0};
};

struct State {
  std::uint16_t can_id{0};
  std::uint8_t fault_code{0};
  std::uint8_t status{0};
  double position{0.0};
  double velocity{0.0};
  double force{0.0};
  std::array<std::uint8_t, 8> raw_data{};
};

class Protocol {
public:
  static CanFrame command(const Command &command,
                          std::uint16_t can_id = kDefaultCanId);
  static CanFrame stop(std::uint16_t can_id = kDefaultCanId);
  static std::optional<State> decode_state(
      const CanFrame &frame, std::uint16_t feedback_id = kDefaultCanId);
};

class Gripper {
public:
  Gripper() = default;
  ~Gripper();

  Gripper(const Gripper &) = delete;
  Gripper &operator=(const Gripper &) = delete;
  Gripper(Gripper &&other) noexcept;
  Gripper &operator=(Gripper &&other) noexcept;

  bool connect(const std::string &interface_name,
               std::uint16_t command_id = kDefaultCanId,
               std::uint16_t feedback_id = kDefaultCanId);
  void disconnect();
  bool connected() const;

  bool send(const Command &command);
  bool stop();
  bool receive(State &state, std::chrono::microseconds timeout);

  std::uint16_t command_id() const;
  std::uint16_t feedback_id() const;
  const std::string &last_error() const;

private:
  void set_error(const std::string &operation);

  int fd_{-1};
  std::uint16_t command_id_{kDefaultCanId};
  std::uint16_t feedback_id_{kDefaultCanId};
  std::string last_error_;
};

} // namespace rebotarm_gripper_sdk
