#include "rs_motor_sdk/rs_motor.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rs_motor_sdk {
namespace {

constexpr std::uint32_t kExtendedIdMask = 0x1FFFFFFF;
constexpr int kSendAttempts = 6;
constexpr auto kSendRetryDelay = std::chrono::milliseconds(1);

void validate_motor(const MotorConfig &motor) {
  if (motor.motor_id == 0) {
    throw std::invalid_argument("RobStride motor ID must be in 1..255");
  }
}

void validate_finite(double value, const char *name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

std::uint16_t encode(double value, double minimum, double maximum) {
  validate_finite(value, "protocol value");
  const auto clamped = std::max(minimum, std::min(value, maximum));
  return static_cast<std::uint16_t>((clamped - minimum) * 65535.0 /
                                    (maximum - minimum));
}

double decode(std::uint16_t value, double minimum, double maximum) {
  return static_cast<double>(value) * (maximum - minimum) / 65535.0 + minimum;
}

std::uint32_t make_id(CommunicationType type, std::uint16_t data,
                      std::uint8_t target) {
  return (static_cast<std::uint32_t>(type) << 24) |
         (static_cast<std::uint32_t>(data) << 8) | target;
}

CanFrame command(const MotorConfig &motor, CommunicationType type,
                 std::uint16_t data = 0) {
  validate_motor(motor);
  CanFrame frame;
  frame.id = make_id(type, data, motor.motor_id);
  frame.size = 8;
  return frame;
}

template <typename T> void write_little(T value, std::uint8_t *output) {
  static_assert(std::is_trivially_copyable<T>::value, "wire value is trivial");
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(output, &value, sizeof(T));
#else
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  std::reverse(bytes.begin(), bytes.end());
  std::copy(bytes.begin(), bytes.end(), output);
#endif
}

} // namespace

TrajectorySample minimum_jerk(double start, double goal, double duration,
                              double elapsed) {
  validate_finite(start, "trajectory start");
  validate_finite(goal, "trajectory goal");
  validate_finite(duration, "trajectory duration");
  validate_finite(elapsed, "trajectory elapsed time");
  if (duration <= 0.0) {
    throw std::invalid_argument("trajectory duration must be positive");
  }

  const double s = std::clamp(elapsed / duration, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  const double blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  const double blend_rate = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / duration;
  const double distance = goal - start;
  return {start + distance * blend, distance * blend_rate};
}

Limits Protocol::limits(MotorModel model) {
  // RobStride MIT protocol ranges. Mechanical joint limits remain enforced by
  // ros2_control and are intentionally narrower than these wire ranges.
  switch (model) {
  case MotorModel::kRs00:
    return {12.5, 44.0, 17.0, 500.0, 5.0};
  case MotorModel::kRs06:
    return {12.5, 15.0, 120.0, 500.0, 5.0};
  }
  throw std::invalid_argument("unknown RobStride motor model");
}

CanFrame Protocol::ping(const MotorConfig &motor) {
  return command(motor, CommunicationType::kGetDeviceId,
                 static_cast<std::uint16_t>(motor.host_id));
}

CanFrame Protocol::enable(const MotorConfig &motor) {
  return command(motor, CommunicationType::kEnable,
                 static_cast<std::uint16_t>(motor.host_id));
}

CanFrame Protocol::disable(const MotorConfig &motor, bool clear_fault) {
  auto frame = command(motor, CommunicationType::kDisable,
                       static_cast<std::uint16_t>(motor.host_id));
  frame.data[0] = clear_fault ? 1 : 0;
  return frame;
}

CanFrame Protocol::set_zero(const MotorConfig &motor) {
  auto frame = command(motor, CommunicationType::kSetZero,
                       static_cast<std::uint16_t>(motor.host_id));
  frame.data[0] = 1;
  return frame;
}

CanFrame Protocol::active_report(const MotorConfig &motor, bool enabled) {
  auto frame = command(motor, CommunicationType::kActiveReport,
                       static_cast<std::uint16_t>(motor.host_id));
  frame.data = {1, 2, 3, 4, 5, 6, static_cast<std::uint8_t>(enabled ? 1 : 0),
                0};
  return frame;
}

CanFrame Protocol::mit(const MotorConfig &motor, const MitCommand &value) {
  const auto range = limits(motor.model);
  const auto effort =
      encode(value.feedforward_effort, -range.effort, range.effort);
  auto frame = command(motor, CommunicationType::kMitControl, effort);
  const std::array<std::uint16_t, 4> fields{
      encode(value.position, -range.position, range.position),
      encode(value.velocity, -range.velocity, range.velocity),
      encode(value.kp, 0.0, range.kp), encode(value.kd, 0.0, range.kd)};
  for (std::size_t i = 0; i < fields.size(); ++i) {
    frame.data[i * 2] = static_cast<std::uint8_t>(fields[i] >> 8);
    frame.data[i * 2 + 1] = static_cast<std::uint8_t>(fields[i]);
  }
  return frame;
}

CanFrame Protocol::read_parameter(const MotorConfig &motor,
                                  std::uint16_t index) {
  auto frame = command(motor, CommunicationType::kReadParameter,
                       static_cast<std::uint16_t>(motor.host_id));
  write_little(index, frame.data.data());
  return frame;
}

CanFrame Protocol::write_parameter_f32(const MotorConfig &motor,
                                       std::uint16_t index, float value) {
  validate_finite(value, "parameter value");
  auto frame = command(motor, CommunicationType::kWriteParameter,
                       static_cast<std::uint16_t>(motor.host_id));
  write_little(index, frame.data.data());
  write_little(value, frame.data.data() + 4);
  return frame;
}

CanFrame Protocol::store_parameters(const MotorConfig &motor) {
  auto frame = command(motor, CommunicationType::kStoreParameters,
                       static_cast<std::uint16_t>(motor.host_id));
  frame.data = {1, 2, 3, 4, 5, 6, 7, 8};
  return frame;
}

CommunicationType Protocol::communication_type(const CanFrame &frame) {
  return static_cast<CommunicationType>((frame.id >> 24) & 0x1F);
}

std::optional<MotorState> Protocol::decode_feedback(const MotorConfig &motor,
                                                    const CanFrame &frame) {
  const auto type = communication_type(frame);
  if (!frame.extended || frame.size != 8 ||
      (type != CommunicationType::kFeedback &&
       type != CommunicationType::kActiveReport) ||
      static_cast<std::uint8_t>(frame.id) != motor.host_id ||
      static_cast<std::uint8_t>(frame.id >> 8) != motor.motor_id) {
    return std::nullopt;
  }
  const auto range = limits(motor.model);
  const auto word = [&frame](std::size_t offset) {
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(frame.data[offset]) << 8) |
        frame.data[offset + 1]);
  };
  const auto status = static_cast<std::uint8_t>((frame.id >> 16) & 0xFF);
  return MotorState{motor.motor_id,
                    static_cast<std::uint8_t>(status & 0x3F),
                    static_cast<std::uint8_t>(status >> 6),
                    decode(word(0), -range.position, range.position),
                    decode(word(2), -range.velocity, range.velocity),
                    decode(word(4), -range.effort, range.effort),
                    static_cast<double>(word(6)) / 10.0,
                    type,
                    frame.id};
}

std::optional<ParameterResponse>
Protocol::decode_parameter(const MotorConfig &motor, const CanFrame &frame) {
  if (!frame.extended || frame.size != 8 ||
      communication_type(frame) != CommunicationType::kReadParameter ||
      static_cast<std::uint8_t>(frame.id) != motor.host_id ||
      static_cast<std::uint8_t>(frame.id >> 8) != motor.motor_id) {
    return std::nullopt;
  }
  const auto index = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(frame.data[0]) |
      (static_cast<std::uint16_t>(frame.data[1]) << 8));
  return ParameterResponse{
      motor.motor_id,
      index,
      {frame.data[4], frame.data[5], frame.data[6], frame.data[7]}};
}

SocketCan::~SocketCan() { close(); }

void SocketCan::set_error(const std::string &operation) {
  last_error_ = operation + ": " + std::strerror(errno);
}

bool SocketCan::open(const std::string &interface_name) {
  close();
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
  return true;
}

void SocketCan::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCan::is_open() const { return fd_ >= 0; }

bool SocketCan::send(const CanFrame &frame) {
  if (fd_ < 0) {
    last_error_ = "SocketCAN interface is not open";
    return false;
  }
  struct can_frame raw {};
  raw.can_id = frame.id & kExtendedIdMask;
  if (frame.extended) {
    raw.can_id |= CAN_EFF_FLAG;
  }
  raw.can_dlc = frame.size;
  std::copy_n(frame.data.begin(), frame.size, raw.data);
  for (int attempt = 0; attempt < kSendAttempts; ++attempt) {
    if (::write(fd_, &raw, sizeof(raw)) == static_cast<ssize_t>(sizeof(raw))) {
      return true;
    }
    const int write_error = errno;
    const bool retryable = write_error == ENOBUFS || write_error == EAGAIN ||
                           write_error == EWOULDBLOCK || write_error == EINTR;
    if (!retryable || attempt + 1 == kSendAttempts) {
      errno = write_error;
      set_error("write CAN frame");
      return false;
    }
    std::this_thread::sleep_for(kSendRetryDelay);
  }
  return false;
}

bool SocketCan::receive(CanFrame &frame, std::chrono::microseconds timeout) {
  if (fd_ < 0) {
    last_error_ = "SocketCAN interface is not open";
    return false;
  }
  struct pollfd descriptor {
    fd_, POLLIN, 0
  };
  const auto milliseconds =
      std::max<long long>(1, (timeout.count() + 999) / 1000);
  const int result = ::poll(&descriptor, 1, static_cast<int>(milliseconds));
  if (result <= 0) {
    if (result < 0) {
      set_error("poll CAN frame");
    } else {
      last_error_ = "CAN receive timeout";
    }
    return false;
  }
  struct can_frame raw {};
  if (::read(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    set_error("read CAN frame");
    return false;
  }
  frame.id =
      raw.can_id & (raw.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame.size = raw.can_dlc;
  std::copy_n(raw.data, raw.can_dlc, frame.data.begin());
  return true;
}

const std::string &SocketCan::last_error() const { return last_error_; }

std::size_t MotorBus::add_motor(const MotorConfig &config) {
  validate_motor(config);
  const auto duplicate = std::find_if(
      motors_.begin(), motors_.end(), [&config](const MotorConfig &motor) {
        return motor.motor_id == config.motor_id;
      });
  if (duplicate != motors_.end()) {
    throw std::invalid_argument("duplicate RobStride motor ID");
  }
  motors_.push_back(config);
  return motors_.size() - 1;
}

bool MotorBus::connect(const std::string &name) {
  if (!socket_.open(name)) {
    last_error_ = socket_.last_error();
    return false;
  }
  return true;
}

void MotorBus::disconnect() { socket_.close(); }
bool MotorBus::connected() const { return socket_.is_open(); }
const std::string &MotorBus::last_error() const { return last_error_; }

const MotorConfig &MotorBus::motor(std::size_t index) const {
  if (index >= motors_.size()) {
    throw std::out_of_range("RobStride motor index");
  }
  return motors_[index];
}

bool MotorBus::send(const CanFrame &frame) {
  if (!socket_.send(frame)) {
    last_error_ = socket_.last_error();
    return false;
  }
  return true;
}

bool MotorBus::ping(std::size_t i) { return send(Protocol::ping(motor(i))); }
bool MotorBus::enable(std::size_t i) {
  return send(Protocol::enable(motor(i)));
}
bool MotorBus::disable(std::size_t i, bool clear) {
  return send(Protocol::disable(motor(i), clear));
}
bool MotorBus::set_zero(std::size_t i) {
  return send(Protocol::set_zero(motor(i)));
}
bool MotorBus::set_active_report(std::size_t i, bool enabled) {
  return send(Protocol::active_report(motor(i), enabled));
}
bool MotorBus::send_mit(std::size_t i, const MitCommand &value) {
  return send(Protocol::mit(motor(i), value));
}
bool MotorBus::read_parameter(std::size_t i, std::uint16_t parameter) {
  return send(Protocol::read_parameter(motor(i), parameter));
}
bool MotorBus::write_parameter_f32(std::size_t i, std::uint16_t parameter,
                                   float value) {
  return send(Protocol::write_parameter_f32(motor(i), parameter, value));
}
bool MotorBus::store_parameters(std::size_t i) {
  return send(Protocol::store_parameters(motor(i)));
}

bool MotorBus::receive_state(MotorState &state, std::size_t &motor_index,
                             std::chrono::microseconds timeout) {
  CanFrame frame;
  if (!socket_.receive(frame, timeout)) {
    last_error_ = socket_.last_error();
    return false;
  }
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto decoded = Protocol::decode_feedback(motors_[i], frame);
    if (decoded) {
      state = *decoded;
      motor_index = i;
      return true;
    }
  }
  last_error_ = "CAN frame is not RobStride feedback for a configured motor";
  return false;
}

bool MotorBus::receive_parameter(ParameterResponse &response,
                                 std::size_t &motor_index,
                                 std::chrono::microseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - std::chrono::steady_clock::now());
    CanFrame frame;
    if (!socket_.receive(frame, remaining)) {
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
  }
  last_error_ = "CAN parameter response timeout";
  return false;
}

} // namespace rs_motor_sdk
