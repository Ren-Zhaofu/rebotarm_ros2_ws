#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace rebotarm_dm_motor_sdk {

enum class MotorModel : std::uint8_t {
  kDm3507,
  kDm4310,
  kDm4310_48V,
  kDm4340,
  kDm4340_48V,
  kDm6006,
  kDm8006,
  kDm8009,
  kDm10010L,
  kDm10010,
  kDmh3510,
  kDmh6215,
  kDmg6220,
};

enum class ControlMode : std::uint8_t {
  kMit = 1,
  kPositionVelocity = 2,
  kVelocity = 3,
  kPositionForce = 4,
};

enum class Register : std::uint8_t {
  kUnderVoltage = 0,
  kTorqueConstant = 1,
  kOverTemperature = 2,
  kOverCurrent = 3,
  kAcceleration = 4,
  kDeceleration = 5,
  kMaximumSpeed = 6,
  kMasterId = 7,
  kEscId = 8,
  kTimeout = 9,
  kControlMode = 10,
  kDamping = 11,
  kInertia = 12,
  kHardwareVersion = 13,
  kSoftwareVersion = 14,
  kSerialNumber = 15,
  kPolePairs = 16,
  kResistance = 17,
  kInductance = 18,
  kFlux = 19,
  kGearRatio = 20,
  kPositionMaximum = 21,
  kVelocityMaximum = 22,
  kTorqueMaximum = 23,
  kCurrentBandwidth = 24,
  kVelocityKp = 25,
  kVelocityKi = 26,
  kPositionKp = 27,
  kPositionKi = 28,
  kOverVoltage = 29,
  kGearReference = 30,
  kDelta = 31,
  kVelocityBandwidth = 32,
  kIqC1 = 33,
  kVelocityLimitC1 = 34,
  kCanBitrate = 35,
  kSubVersion = 36,
  kBootVersion = 37,
  kDirection = 55,
  kMotorOffset = 56,
  kMaximumCurrent = 59,
  kBusVoltage = 60,
  kPcbTemperature = 61,
  kMotorTemperature = 62,
  kUCurrentOffset = 63,
  kVCurrentOffset = 64,
  kWCurrentOffset = 65,
  kPositionMeasured = 80,
  kOutput = 81,
};

enum class StatusCommand : std::uint8_t {
  kClearError = 0xFB,
  kEnable = 0xFC,
  kDisable = 0xFD,
  kSetZero = 0xFE,
};

struct Limits {
  double position;
  double velocity;
  double effort;
};

struct MotorConfig {
  std::string name;
  std::uint16_t command_id{0};
  std::uint16_t feedback_id{0};
  MotorModel model{MotorModel::kDm4310};
};

struct CanFrame {
  std::uint16_t id{0};
  std::uint8_t size{0};
  std::array<std::uint8_t, 8> data{};
};

struct MitCommand {
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  double feedforward_effort{0.0};
};

struct PositionVelocityCommand {
  float position{0.0F};
  float velocity{0.0F};
};

struct PositionForceCommand {
  float position{0.0F};
  double velocity{0.0};
  double normalized_current{0.0};
};

struct MotionSample {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

MotionSample minimum_jerk(double start, double goal,
                          std::chrono::duration<double> elapsed,
                          std::chrono::duration<double> duration);

struct MotorState {
  std::uint8_t motor_id{0};
  std::uint8_t status{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint8_t mos_temperature{0};
  std::uint8_t rotor_temperature{0};
};

using ParameterValue = std::variant<std::uint32_t, float>;

struct ParameterResponse {
  std::uint16_t motor_id{0};
  std::uint8_t operation{0};
  Register reg{Register::kUnderVoltage};
  ParameterValue value{std::uint32_t{0}};
};

class Protocol {
public:
  static constexpr std::uint16_t kManagementCanId = 0x7FF;

  static Limits limits(MotorModel model);
  static CanFrame status(const MotorConfig &motor, StatusCommand command);
  static CanFrame mit(const MotorConfig &motor, const MitCommand &command);
  static CanFrame position_velocity(const MotorConfig &motor,
                                    const PositionVelocityCommand &command);
  static CanFrame velocity(const MotorConfig &motor, float velocity);
  static CanFrame position_force(const MotorConfig &motor,
                                 const PositionForceCommand &command);
  static CanFrame refresh(const MotorConfig &motor);
  static CanFrame read_parameter(const MotorConfig &motor, Register reg);
  static CanFrame write_parameter(const MotorConfig &motor, Register reg,
                                  const ParameterValue &value);
  static CanFrame store_parameters(const MotorConfig &motor);

  static std::optional<MotorState> decode_state(const MotorConfig &motor,
                                                const CanFrame &frame);
  static std::optional<ParameterResponse>
  decode_parameter(const MotorConfig &motor, const CanFrame &frame);
  static bool decode_store_ack(const MotorConfig &motor, const CanFrame &frame);
  static bool register_uses_integer_value(Register reg);
  static bool register_is_writable(Register reg);
};

class SocketCan {
public:
  SocketCan() = default;
  explicit SocketCan(const std::string &interface_name);
  ~SocketCan();

  SocketCan(const SocketCan &) = delete;
  SocketCan &operator=(const SocketCan &) = delete;
  SocketCan(SocketCan &&other) noexcept;
  SocketCan &operator=(SocketCan &&other) noexcept;

  bool open(const std::string &interface_name);
  void close();
  bool is_open() const;
  bool set_receive_own_messages(bool enabled);
  bool set_filters(const std::vector<std::uint16_t> &ids);
  bool send(const CanFrame &frame);
  // Returns true only when a complete classic CAN data frame was received.
  bool receive(CanFrame &frame, std::chrono::microseconds timeout);
  const std::string &last_error() const;

private:
  void set_error(const std::string &operation);

  int fd_{-1};
  std::string last_error_;
};

class MotorBus {
public:
  bool connect(const std::string &interface_name);
  void disconnect();
  bool connected() const;
  const std::string &last_error() const;

  std::size_t add_motor(const MotorConfig &motor);
  const MotorConfig &motor(std::size_t index) const;
  std::size_t size() const;

  bool clear_error(std::size_t index);
  bool enable(std::size_t index);
  bool disable(std::size_t index);
  bool set_zero(std::size_t index);
  bool send_mit(std::size_t index, const MitCommand &command);
  bool send_position_velocity(std::size_t index,
                              const PositionVelocityCommand &command);
  bool send_velocity(std::size_t index, float velocity);
  bool send_position_force(std::size_t index,
                           const PositionForceCommand &command);
  bool request_state(std::size_t index);
  bool read_parameter(std::size_t index, Register reg);
  bool write_parameter(std::size_t index, Register reg,
                       const ParameterValue &value);
  bool store_parameters(std::size_t index);

  bool receive_state(MotorState &state, std::size_t &motor_index,
                     std::chrono::microseconds timeout);
  bool receive_parameter(ParameterResponse &response, std::size_t &motor_index,
                         std::chrono::microseconds timeout);
  bool receive_store_ack(std::size_t &motor_index,
                         std::chrono::microseconds timeout);

private:
  bool send(const CanFrame &frame);

  SocketCan socket_;
  std::vector<MotorConfig> motors_;
  std::string last_error_;
};

} // namespace rebotarm_dm_motor_sdk
