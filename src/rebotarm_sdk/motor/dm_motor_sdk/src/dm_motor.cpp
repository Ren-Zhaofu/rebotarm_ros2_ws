#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <type_traits>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rebotarm_dm_motor_sdk {
namespace {

constexpr std::array<Limits, 13> kModelLimits{{
    {12.5, 50.0, 5.0},
    {12.5, 30.0, 10.0},
    {12.5, 50.0, 10.0},
    {12.5, 8.0, 28.0},
    {12.5, 10.0, 28.0},
    {12.5, 45.0, 20.0},
    {12.5, 45.0, 40.0},
    {12.5, 45.0, 54.0},
    {12.5, 25.0, 200.0},
    {12.5, 20.0, 200.0},
    {12.5, 280.0, 1.0},
    {12.5, 45.0, 10.0},
    {12.5, 45.0, 10.0},
}};

void validate_id(std::uint16_t id) {
  if (id > CAN_SFF_MASK) {
    throw std::invalid_argument(
        "DM protocol requires an 11-bit standard CAN identifier");
  }
}

void validate_finite(double value, const char *label) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(label) + " must be finite");
  }
}

double clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(value, maximum));
}

std::uint32_t encode_linear(double value, double minimum, double maximum,
                            unsigned int bits) {
  validate_finite(value, "linear protocol value");
  const double normalized =
      (clamp(value, minimum, maximum) - minimum) / (maximum - minimum);
  return static_cast<std::uint32_t>(normalized *
                                    static_cast<double>((1U << bits) - 1U));
}

double decode_linear(std::uint32_t value, double minimum, double maximum,
                     unsigned int bits) {
  return static_cast<double>(value) * (maximum - minimum) /
             static_cast<double>((1U << bits) - 1U) +
         minimum;
}

template <typename T>
std::array<std::uint8_t, sizeof(T)> to_little_endian_bytes(T value) {
  static_assert(std::is_trivially_copyable<T>::value,
                "wire type must be trivial");
  std::array<std::uint8_t, sizeof(T)> native{};
  std::memcpy(native.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return native;
#else
  std::reverse(native.begin(), native.end());
  return native;
#endif
}

template <typename T> T from_little_endian_bytes(const std::uint8_t *bytes) {
  std::array<std::uint8_t, sizeof(T)> native{};
  std::copy(bytes, bytes + sizeof(T), native.begin());
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
  std::reverse(native.begin(), native.end());
#endif
  T result{};
  std::memcpy(&result, native.data(), sizeof(T));
  return result;
}

CanFrame management_prefix(const MotorConfig &motor, std::uint8_t operation,
                           std::uint8_t fourth_byte, std::uint8_t size) {
  validate_id(motor.command_id);
  CanFrame frame;
  frame.id = Protocol::kManagementCanId;
  frame.size = size;
  frame.data[0] = static_cast<std::uint8_t>(motor.command_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((motor.command_id >> 8) & 0xFF);
  frame.data[2] = operation;
  frame.data[3] = fourth_byte;
  return frame;
}

} // namespace

MotionSample minimum_jerk(double start, double goal,
                          std::chrono::duration<double> elapsed,
                          std::chrono::duration<double> duration) {
  validate_finite(start, "minimum-jerk start");
  validate_finite(goal, "minimum-jerk goal");
  validate_finite(elapsed.count(), "minimum-jerk elapsed time");
  validate_finite(duration.count(), "minimum-jerk duration");
  if (duration.count() <= 0.0) {
    throw std::invalid_argument("minimum-jerk duration must be positive");
  }
  const double time = clamp(elapsed.count(), 0.0, duration.count());
  const double tau = time / duration.count();
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;
  const double blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
  const double blend_velocity =
      (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / duration.count();
  const double blend_acceleration = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) /
                                    (duration.count() * duration.count());
  const double displacement = goal - start;
  return MotionSample{start + displacement * blend,
                      displacement * blend_velocity,
                      displacement * blend_acceleration};
}

Limits Protocol::limits(MotorModel model) {
  const auto index = static_cast<std::size_t>(model);
  if (index >= kModelLimits.size()) {
    throw std::invalid_argument("unknown DM motor model");
  }
  return kModelLimits[index];
}

CanFrame Protocol::status(const MotorConfig &motor, StatusCommand command) {
  validate_id(motor.command_id);
  CanFrame frame;
  frame.id = motor.command_id;
  frame.size = 8;
  frame.data.fill(0xFF);
  frame.data[7] = static_cast<std::uint8_t>(command);
  return frame;
}

CanFrame Protocol::mit(const MotorConfig &motor, const MitCommand &command) {
  validate_id(motor.command_id);
  validate_finite(command.kp, "MIT kp");
  validate_finite(command.kd, "MIT kd");
  const auto range = limits(motor.model);
  const auto p =
      encode_linear(command.position, -range.position, range.position, 16);
  const auto v =
      encode_linear(command.velocity, -range.velocity, range.velocity, 12);
  const auto kp = encode_linear(command.kp, 0.0, 500.0, 12);
  const auto kd = encode_linear(command.kd, 0.0, 5.0, 12);
  const auto t = encode_linear(command.feedforward_effort, -range.effort,
                               range.effort, 12);

  CanFrame frame;
  frame.id = motor.command_id;
  frame.size = 8;
  frame.data = {static_cast<std::uint8_t>(p >> 8),
                static_cast<std::uint8_t>(p),
                static_cast<std::uint8_t>(v >> 4),
                static_cast<std::uint8_t>((v & 0x0F) << 4 | kp >> 8),
                static_cast<std::uint8_t>(kp),
                static_cast<std::uint8_t>(kd >> 4),
                static_cast<std::uint8_t>((kd & 0x0F) << 4 | t >> 8),
                static_cast<std::uint8_t>(t)};
  return frame;
}

CanFrame Protocol::position_velocity(const MotorConfig &motor,
                                     const PositionVelocityCommand &command) {
  validate_id(motor.command_id);
  validate_finite(command.position, "position-velocity position");
  validate_finite(command.velocity, "position-velocity velocity");
  if (static_cast<unsigned int>(motor.command_id) + 0x100U > CAN_SFF_MASK) {
    throw std::invalid_argument(
        "position-velocity CAN identifier overflows 11 bits");
  }
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(motor.command_id + 0x100);
  frame.size = 8;
  const auto position = to_little_endian_bytes(command.position);
  const auto velocity_bytes = to_little_endian_bytes(command.velocity);
  std::copy(position.begin(), position.end(), frame.data.begin());
  std::copy(velocity_bytes.begin(), velocity_bytes.end(),
            frame.data.begin() + 4);
  return frame;
}

CanFrame Protocol::velocity(const MotorConfig &motor, float velocity_value) {
  validate_id(motor.command_id);
  validate_finite(velocity_value, "velocity");
  if (static_cast<unsigned int>(motor.command_id) + 0x200U > CAN_SFF_MASK) {
    throw std::invalid_argument("velocity CAN identifier overflows 11 bits");
  }
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(motor.command_id + 0x200);
  frame.size = 4;
  const auto bytes = to_little_endian_bytes(velocity_value);
  std::copy(bytes.begin(), bytes.end(), frame.data.begin());
  return frame;
}

CanFrame Protocol::position_force(const MotorConfig &motor,
                                  const PositionForceCommand &command) {
  validate_id(motor.command_id);
  validate_finite(command.position, "position-force position");
  validate_finite(command.velocity, "position-force velocity");
  validate_finite(command.normalized_current,
                  "position-force normalized current");
  if (static_cast<unsigned int>(motor.command_id) + 0x300U > CAN_SFF_MASK) {
    throw std::invalid_argument(
        "position-force CAN identifier overflows 11 bits");
  }
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(motor.command_id + 0x300);
  frame.size = 8;
  const auto position = to_little_endian_bytes(command.position);
  std::copy(position.begin(), position.end(), frame.data.begin());
  const auto velocity_scaled =
      static_cast<std::uint16_t>(clamp(command.velocity * 100.0, 0.0, 10000.0));
  const auto current_scaled = static_cast<std::uint16_t>(
      clamp(command.normalized_current, 0.0, 1.0) * 10000.0);
  frame.data[4] = static_cast<std::uint8_t>(velocity_scaled);
  frame.data[5] = static_cast<std::uint8_t>(velocity_scaled >> 8);
  frame.data[6] = static_cast<std::uint8_t>(current_scaled);
  frame.data[7] = static_cast<std::uint8_t>(current_scaled >> 8);
  return frame;
}

CanFrame Protocol::refresh(const MotorConfig &motor) {
  return management_prefix(motor, 0xCC, 0x00, 8);
}

CanFrame Protocol::read_parameter(const MotorConfig &motor, Register reg) {
  return management_prefix(motor, 0x33, static_cast<std::uint8_t>(reg), 4);
}

CanFrame Protocol::write_parameter(const MotorConfig &motor, Register reg,
                                   const ParameterValue &value) {
  if (!register_is_writable(reg)) {
    throw std::invalid_argument("attempted to write a read-only DM register");
  }
  const bool integer_register = register_uses_integer_value(reg);
  if (integer_register != std::holds_alternative<std::uint32_t>(value)) {
    throw std::invalid_argument(
        "parameter value type does not match register type");
  }
  auto frame =
      management_prefix(motor, 0x55, static_cast<std::uint8_t>(reg), 8);
  if (integer_register) {
    const auto bytes = to_little_endian_bytes(std::get<std::uint32_t>(value));
    std::copy(bytes.begin(), bytes.end(), frame.data.begin() + 4);
  } else {
    const float number = std::get<float>(value);
    validate_finite(number, "parameter value");
    const auto bytes = to_little_endian_bytes(number);
    std::copy(bytes.begin(), bytes.end(), frame.data.begin() + 4);
  }
  return frame;
}

CanFrame Protocol::store_parameters(const MotorConfig &motor) {
  return management_prefix(motor, 0xAA, 0x01, 4);
}

std::optional<MotorState> Protocol::decode_state(const MotorConfig &motor,
                                                 const CanFrame &frame) {
  if (frame.id != motor.feedback_id || frame.size != 8) {
    return std::nullopt;
  }
  // Parameter responses share the feedback CAN ID but carry the command ID in
  // D0/D1 and an operation marker in D2.
  const auto response_motor_id = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(frame.data[0]) |
      (static_cast<std::uint16_t>(frame.data[1]) << 8));
  if (response_motor_id == motor.command_id &&
      (frame.data[2] == 0x33 || frame.data[2] == 0x55)) {
    return std::nullopt;
  }
  const auto range = limits(motor.model);
  const std::uint16_t p =
      (static_cast<std::uint16_t>(frame.data[1]) << 8) | frame.data[2];
  const std::uint16_t v =
      (static_cast<std::uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const std::uint16_t t =
      (static_cast<std::uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];
  return MotorState{static_cast<std::uint8_t>(frame.data[0] & 0x0F),
                    static_cast<std::uint8_t>(frame.data[0] >> 4),
                    decode_linear(p, -range.position, range.position, 16),
                    decode_linear(v, -range.velocity, range.velocity, 12),
                    decode_linear(t, -range.effort, range.effort, 12),
                    frame.data[6],
                    frame.data[7]};
}

std::optional<ParameterResponse>
Protocol::decode_parameter(const MotorConfig &motor, const CanFrame &frame) {
  if (frame.id != motor.feedback_id || frame.size != 8 ||
      (frame.data[2] != 0x33 && frame.data[2] != 0x55)) {
    return std::nullopt;
  }
  const auto response_motor_id = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(frame.data[0]) |
      (static_cast<std::uint16_t>(frame.data[1]) << 8));
  if (response_motor_id != motor.command_id) {
    return std::nullopt;
  }
  const auto reg = static_cast<Register>(frame.data[3]);
  ParameterValue value;
  if (register_uses_integer_value(reg)) {
    value = from_little_endian_bytes<std::uint32_t>(&frame.data[4]);
  } else {
    value = from_little_endian_bytes<float>(&frame.data[4]);
    if (!std::isfinite(std::get<float>(value))) {
      return std::nullopt;
    }
  }
  return ParameterResponse{response_motor_id, frame.data[2], reg, value};
}

bool Protocol::decode_store_ack(const MotorConfig &motor,
                                const CanFrame &frame) {
  if (frame.id != motor.feedback_id || frame.size < 4 ||
      frame.data[2] != 0xAA || frame.data[3] != 0x01) {
    return false;
  }
  const auto response_motor_id = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(frame.data[0]) |
      (static_cast<std::uint16_t>(frame.data[1]) << 8));
  return response_motor_id == motor.command_id;
}

bool Protocol::register_uses_integer_value(Register reg) {
  const auto value = static_cast<std::uint8_t>(reg);
  return (value >= 7 && value <= 10) || (value >= 13 && value <= 16) ||
         (value >= 35 && value <= 37);
}

bool Protocol::register_is_writable(Register reg) {
  const auto value = static_cast<std::uint8_t>(reg);
  return value <= 10 || (value >= 21 && value <= 35);
}

SocketCan::SocketCan(const std::string &interface_name) {
  open(interface_name);
}

SocketCan::~SocketCan() { close(); }

SocketCan::SocketCan(SocketCan &&other) noexcept
    : fd_(std::exchange(other.fd_, -1)),
      last_error_(std::move(other.last_error_)) {}

SocketCan &SocketCan::operator=(SocketCan &&other) noexcept {
  if (this != &other) {
    close();
    fd_ = std::exchange(other.fd_, -1);
    last_error_ = std::move(other.last_error_);
  }
  return *this;
}

void SocketCan::set_error(const std::string &operation) {
  last_error_ = operation + ": " + std::strerror(errno);
}

bool SocketCan::open(const std::string &interface_name) {
  close();
  last_error_.clear();
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    last_error_ = "invalid SocketCAN interface name";
    return false;
  }
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (fd_ < 0) {
    set_error("socket(PF_CAN)");
    return false;
  }
  struct ifreq request {};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
    set_error("SIOCGIFINDEX " + interface_name);
    close();
    return false;
  }
  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&address),
             sizeof(address)) < 0) {
    set_error("bind " + interface_name);
    close();
    return false;
  }
  return set_receive_own_messages(false);
}

void SocketCan::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCan::is_open() const { return fd_ >= 0; }

bool SocketCan::set_receive_own_messages(bool enabled) {
  if (!is_open()) {
    last_error_ = "SocketCAN is not open";
    return false;
  }
  const int value = enabled ? 1 : 0;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &value,
                   sizeof(value)) < 0) {
    set_error("setsockopt CAN_RAW_RECV_OWN_MSGS");
    return false;
  }
  return true;
}

bool SocketCan::set_filters(const std::vector<std::uint16_t> &ids) {
  if (!is_open()) {
    last_error_ = "SocketCAN is not open";
    return false;
  }
  std::vector<struct can_filter> filters;
  filters.reserve(ids.size());
  for (const auto id : ids) {
    if (id > CAN_SFF_MASK) {
      last_error_ = "CAN filter identifier exceeds 11 bits";
      return false;
    }
    filters.push_back({id, CAN_SFF_MASK});
  }
  const void *data = filters.empty() ? nullptr : filters.data();
  const auto size =
      static_cast<socklen_t>(filters.size() * sizeof(struct can_filter));
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, data, size) < 0) {
    set_error("setsockopt CAN_RAW_FILTER");
    return false;
  }
  return true;
}

bool SocketCan::send(const CanFrame &frame) {
  if (!is_open()) {
    last_error_ = "SocketCAN is not open";
    return false;
  }
  if (frame.id > CAN_SFF_MASK || frame.size > CAN_MAX_DLEN) {
    last_error_ = "invalid classic CAN frame";
    return false;
  }
  struct can_frame native {};
  native.can_id = frame.id;
  native.can_dlc = frame.size;
  std::copy(frame.data.begin(), frame.data.begin() + frame.size, native.data);
  if (::write(fd_, &native, sizeof(native)) !=
      static_cast<ssize_t>(sizeof(native))) {
    set_error("CAN write");
    return false;
  }
  return true;
}

bool SocketCan::receive(CanFrame &frame, std::chrono::microseconds timeout) {
  if (!is_open()) {
    last_error_ = "SocketCAN is not open";
    return false;
  }
  if (timeout.count() < 0) {
    last_error_ = "CAN receive timeout must not be negative";
    return false;
  }
  struct pollfd descriptor {};
  descriptor.fd = fd_;
  descriptor.events = POLLIN;
  const auto timeout_ms = static_cast<int>(std::min<std::int64_t>(
      (timeout.count() + 999) / 1000, std::numeric_limits<int>::max()));
  const int ready = ::poll(&descriptor, 1, timeout_ms);
  if (ready == 0) {
    last_error_ = "CAN receive timeout";
    return false;
  }
  if (ready < 0) {
    set_error("CAN poll");
    return false;
  }
  struct can_frame native {};
  if (::read(fd_, &native, sizeof(native)) !=
      static_cast<ssize_t>(sizeof(native))) {
    set_error("CAN read");
    return false;
  }
  if ((native.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
      native.can_dlc > CAN_MAX_DLEN) {
    last_error_ = "received unsupported CAN frame";
    return false;
  }
  frame = CanFrame{};
  frame.id = static_cast<std::uint16_t>(native.can_id & CAN_SFF_MASK);
  frame.size = native.can_dlc;
  std::copy(native.data, native.data + native.can_dlc, frame.data.begin());
  return true;
}

const std::string &SocketCan::last_error() const { return last_error_; }

bool MotorBus::connect(const std::string &interface_name) {
  if (!socket_.open(interface_name)) {
    last_error_ = socket_.last_error();
    return false;
  }
  std::vector<std::uint16_t> ids;
  ids.reserve(motors_.size());
  for (const auto &item : motors_) {
    ids.push_back(item.feedback_id);
  }
  if (!ids.empty() && !socket_.set_filters(ids)) {
    last_error_ = socket_.last_error();
    socket_.close();
    return false;
  }
  last_error_.clear();
  return true;
}

void MotorBus::disconnect() { socket_.close(); }

bool MotorBus::connected() const { return socket_.is_open(); }

const std::string &MotorBus::last_error() const { return last_error_; }

std::size_t MotorBus::add_motor(const MotorConfig &config) {
  validate_id(config.command_id);
  validate_id(config.feedback_id);
  if (connected()) {
    throw std::logic_error(
        "motors must be configured before connecting SocketCAN");
  }
  if (config.command_id == Protocol::kManagementCanId) {
    throw std::invalid_argument(
        "motor command ID conflicts with management CAN ID");
  }
  for (const auto &existing : motors_) {
    if (existing.command_id == config.command_id ||
        existing.feedback_id == config.feedback_id) {
      throw std::invalid_argument(
          "motor command and feedback IDs must be unique");
    }
  }
  motors_.push_back(config);
  return motors_.size() - 1;
}

const MotorConfig &MotorBus::motor(std::size_t index) const {
  return motors_.at(index);
}

std::size_t MotorBus::size() const { return motors_.size(); }

bool MotorBus::send(const CanFrame &frame) {
  if (!socket_.send(frame)) {
    last_error_ = socket_.last_error();
    return false;
  }
  return true;
}

bool MotorBus::clear_error(std::size_t index) {
  return send(Protocol::status(motor(index), StatusCommand::kClearError));
}

bool MotorBus::enable(std::size_t index) {
  return send(Protocol::status(motor(index), StatusCommand::kEnable));
}

bool MotorBus::disable(std::size_t index) {
  return send(Protocol::status(motor(index), StatusCommand::kDisable));
}

bool MotorBus::set_zero(std::size_t index) {
  return send(Protocol::status(motor(index), StatusCommand::kSetZero));
}

bool MotorBus::send_mit(std::size_t index, const MitCommand &command) {
  return send(Protocol::mit(motor(index), command));
}

bool MotorBus::send_position_velocity(std::size_t index,
                                      const PositionVelocityCommand &command) {
  return send(Protocol::position_velocity(motor(index), command));
}

bool MotorBus::send_velocity(std::size_t index, float velocity_value) {
  return send(Protocol::velocity(motor(index), velocity_value));
}

bool MotorBus::send_position_force(std::size_t index,
                                   const PositionForceCommand &command) {
  return send(Protocol::position_force(motor(index), command));
}

bool MotorBus::request_state(std::size_t index) {
  return send(Protocol::refresh(motor(index)));
}

bool MotorBus::read_parameter(std::size_t index, Register reg) {
  return send(Protocol::read_parameter(motor(index), reg));
}

bool MotorBus::write_parameter(std::size_t index, Register reg,
                               const ParameterValue &value) {
  return send(Protocol::write_parameter(motor(index), reg, value));
}

bool MotorBus::store_parameters(std::size_t index) {
  return send(Protocol::store_parameters(motor(index)));
}

bool MotorBus::receive_state(MotorState &state, std::size_t &motor_index,
                             std::chrono::microseconds timeout) {
  CanFrame frame;
  if (!socket_.receive(frame, timeout)) {
    last_error_ = socket_.last_error();
    return false;
  }
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto decoded = Protocol::decode_state(motors_[i], frame);
    if (decoded) {
      state = *decoded;
      motor_index = i;
      return true;
    }
  }
  last_error_ = "CAN frame is not a configured DM state response";
  return false;
}

bool MotorBus::receive_parameter(ParameterResponse &response,
                                 std::size_t &motor_index,
                                 std::chrono::microseconds timeout) {
  CanFrame frame;
  if (!socket_.receive(frame, timeout)) {
    last_error_ = socket_.last_error();
    return false;
  }
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto decoded = Protocol::decode_parameter(motors_[i], frame);
    if (decoded) {
      response = *decoded;
      motor_index = i;
      return true;
    }
  }
  last_error_ = "CAN frame is not a configured DM parameter response";
  return false;
}

bool MotorBus::receive_store_ack(std::size_t &motor_index,
                                 std::chrono::microseconds timeout) {
  CanFrame frame;
  if (!socket_.receive(frame, timeout)) {
    last_error_ = socket_.last_error();
    return false;
  }
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (Protocol::decode_store_ack(motors_[i], frame)) {
      motor_index = i;
      return true;
    }
  }
  last_error_ = "CAN frame is not a configured DM store acknowledgement";
  return false;
}

} // namespace rebotarm_dm_motor_sdk
