#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rs_motor_sdk {

enum class MotorModel : std::uint8_t { kRs00, kRs06 };

enum class CommunicationType : std::uint8_t {
  kGetDeviceId = 0,
  kMitControl = 1,
  kFeedback = 2,
  kEnable = 3,
  kDisable = 4,
  kSetZero = 6,
  kReadParameter = 17,
  kWriteParameter = 18,
  kActiveReport = 24,
};

struct Limits {
  double position;
  double velocity;
  double effort;
  double kp;
  double kd;
};

struct MotorConfig {
  std::string name;
  std::uint8_t motor_id{0};
  std::uint8_t host_id{0xFD};
  MotorModel model{MotorModel::kRs00};
};

struct CanFrame {
  std::uint32_t id{0};
  std::uint8_t size{0};
  bool extended{true};
  std::array<std::uint8_t, 8> data{};
};

struct MitCommand {
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  double feedforward_effort{0.0};
};

struct TrajectorySample {
  double position{0.0};
  double velocity{0.0};
};

TrajectorySample minimum_jerk(double start, double goal, double duration,
                              double elapsed);

struct MotorState {
  std::uint8_t motor_id{0};
  std::uint8_t fault{0};
  std::uint8_t mode{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double temperature{0.0};
  CommunicationType source{CommunicationType::kFeedback};
  std::uint32_t raw_can_id{0};
};

struct ParameterResponse {
  std::uint8_t motor_id{0};
  std::uint16_t index{0};
  std::array<std::uint8_t, 4> value{};
};

class Protocol {
public:
  static Limits limits(MotorModel model);
  static CanFrame ping(const MotorConfig &motor);
  static CanFrame enable(const MotorConfig &motor);
  static CanFrame disable(const MotorConfig &motor, bool clear_fault = false);
  static CanFrame set_zero(const MotorConfig &motor);
  static CanFrame active_report(const MotorConfig &motor, bool enabled);
  static CanFrame mit(const MotorConfig &motor, const MitCommand &command);
  static CanFrame read_parameter(const MotorConfig &motor, std::uint16_t index);
  static CanFrame write_parameter_f32(const MotorConfig &motor,
                                      std::uint16_t index, float value);
  static std::optional<MotorState> decode_feedback(const MotorConfig &motor,
                                                   const CanFrame &frame);
  static std::optional<ParameterResponse>
  decode_parameter(const MotorConfig &motor, const CanFrame &frame);
  static CommunicationType communication_type(const CanFrame &frame);
};

class SocketCan {
public:
  SocketCan() = default;
  ~SocketCan();
  SocketCan(const SocketCan &) = delete;
  SocketCan &operator=(const SocketCan &) = delete;

  bool open(const std::string &interface_name);
  void close();
  bool is_open() const;
  bool send(const CanFrame &frame);
  bool receive(CanFrame &frame, std::chrono::microseconds timeout);
  const std::string &last_error() const;

private:
  void set_error(const std::string &operation);
  int fd_{-1};
  std::string last_error_;
};

class MotorBus {
public:
  std::size_t add_motor(const MotorConfig &motor);
  bool connect(const std::string &interface_name);
  void disconnect();
  bool connected() const;
  const std::string &last_error() const;

  bool ping(std::size_t index);
  bool enable(std::size_t index);
  bool disable(std::size_t index, bool clear_fault = false);
  bool set_zero(std::size_t index);
  bool set_active_report(std::size_t index, bool enabled);
  bool send_mit(std::size_t index, const MitCommand &command);
  bool read_parameter(std::size_t index, std::uint16_t parameter);
  bool write_parameter_f32(std::size_t index, std::uint16_t parameter,
                           float value);
  bool receive_state(MotorState &state, std::size_t &motor_index,
                     std::chrono::microseconds timeout);
  bool receive_parameter(ParameterResponse &response, std::size_t &motor_index,
                         std::chrono::microseconds timeout);

private:
  bool send(const CanFrame &frame);
  const MotorConfig &motor(std::size_t index) const;

  SocketCan socket_;
  std::vector<MotorConfig> motors_;
  std::string last_error_;
};

} // namespace rs_motor_sdk
