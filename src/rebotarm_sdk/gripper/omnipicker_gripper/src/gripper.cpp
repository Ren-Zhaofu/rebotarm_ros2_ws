#include "rebotarm_gripper_sdk/gripper.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rebotarm_gripper_sdk {
namespace {

void validate_id(std::uint16_t id) {
  if (id > CAN_SFF_MASK) {
    throw std::invalid_argument(
        "gripper protocol requires an 11-bit standard CAN identifier");
  }
}

std::uint8_t encode_unit(double value, const char *name) {
  if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
    throw std::invalid_argument(std::string(name) + " must be within [0, 1]");
  }
  return static_cast<std::uint8_t>(std::lround(value * 255.0));
}

double decode_unit(std::uint8_t value) {
  return static_cast<double>(value) / 255.0;
}

} // namespace

CanFrame Protocol::command(const Command &command_value, std::uint16_t can_id) {
  validate_id(can_id);
  CanFrame frame;
  frame.id = can_id;
  frame.size = 8;
  frame.data = {0x00,
                encode_unit(command_value.position, "position"),
                encode_unit(command_value.velocity, "velocity"),
                encode_unit(command_value.force, "force"),
                encode_unit(command_value.acceleration, "acceleration"),
                encode_unit(command_value.deceleration, "deceleration"),
                0x00,
                0x00};
  return frame;
}

CanFrame Protocol::stop(std::uint16_t can_id) {
  return command(Command{0.0, 0.0, 0.0, 0.0, 0.0}, can_id);
}

std::optional<State> Protocol::decode_state(const CanFrame &frame,
                                            std::uint16_t feedback_id) {
  validate_id(feedback_id);
  if (frame.id != feedback_id || frame.size != 8) {
    return std::nullopt;
  }
  State state;
  state.can_id = frame.id;
  state.fault_code = frame.data[0];
  state.status = frame.data[1];
  state.position = decode_unit(frame.data[2]);
  state.velocity = decode_unit(frame.data[3]);
  state.force = decode_unit(frame.data[4]);
  state.raw_data = frame.data;
  return state;
}

Gripper::~Gripper() { disconnect(); }

Gripper::Gripper(Gripper &&other) noexcept
    : fd_(std::exchange(other.fd_, -1)), command_id_(other.command_id_),
      feedback_id_(other.feedback_id_), last_error_(std::move(other.last_error_)) {}

Gripper &Gripper::operator=(Gripper &&other) noexcept {
  if (this != &other) {
    disconnect();
    fd_ = std::exchange(other.fd_, -1);
    command_id_ = other.command_id_;
    feedback_id_ = other.feedback_id_;
    last_error_ = std::move(other.last_error_);
  }
  return *this;
}

bool Gripper::connect(const std::string &interface_name,
                      std::uint16_t command_id,
                      std::uint16_t feedback_id) {
  disconnect();
  try {
    validate_id(command_id);
    validate_id(feedback_id);
  } catch (const std::exception &exception) {
    last_error_ = exception.what();
    return false;
  }
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    last_error_ = "invalid SocketCAN interface name";
    return false;
  }

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    set_error("socket");
    return false;
  }

  struct ifreq request {};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
    set_error("resolve SocketCAN interface");
    disconnect();
    return false;
  }

  const struct can_filter filter {static_cast<canid_t>(feedback_id), CAN_SFF_MASK};
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    set_error("configure CAN receive filter");
    disconnect();
    return false;
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    set_error("bind SocketCAN interface");
    disconnect();
    return false;
  }

  command_id_ = command_id;
  feedback_id_ = feedback_id;
  last_error_.clear();
  return true;
}

void Gripper::disconnect() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Gripper::connected() const { return fd_ >= 0; }

bool Gripper::send(const Command &command_value) {
  if (!connected()) {
    last_error_ = "gripper is not connected";
    return false;
  }
  const auto value = Protocol::command(command_value, command_id_);
  struct can_frame frame {};
  frame.can_id = value.id;
  frame.can_dlc = value.size;
  std::copy(value.data.begin(), value.data.end(), frame.data);
  if (::write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    set_error("send CAN frame");
    return false;
  }
  last_error_.clear();
  return true;
}

bool Gripper::stop() {
  if (!connected()) {
    last_error_ = "gripper is not connected";
    return false;
  }
  const auto value = Protocol::stop(command_id_);
  struct can_frame frame {};
  frame.can_id = value.id;
  frame.can_dlc = value.size;
  std::copy(value.data.begin(), value.data.end(), frame.data);
  if (::write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    set_error("send gripper stop frame");
    return false;
  }
  last_error_.clear();
  return true;
}

bool Gripper::receive(State &state, std::chrono::microseconds timeout) {
  if (!connected()) {
    last_error_ = "gripper is not connected";
    return false;
  }
  const int timeout_ms = static_cast<int>(
      std::max<std::int64_t>(0, (timeout.count() + 999) / 1000));
  struct pollfd descriptor {fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, timeout_ms);
  if (ready == 0) {
    last_error_ = "receive timeout";
    return false;
  }
  if (ready < 0) {
    set_error("poll CAN frame");
    return false;
  }
  struct can_frame raw {};
  if (::read(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    set_error("receive CAN frame");
    return false;
  }
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(raw.can_id & CAN_SFF_MASK);
  frame.size = raw.can_dlc;
  std::copy(raw.data, raw.data + std::min<std::size_t>(raw.can_dlc, 8),
            frame.data.begin());
  const auto decoded = Protocol::decode_state(frame, feedback_id_);
  if (!decoded) {
    last_error_ = "received malformed gripper feedback";
    return false;
  }
  state = *decoded;
  last_error_.clear();
  return true;
}

std::uint16_t Gripper::command_id() const { return command_id_; }
std::uint16_t Gripper::feedback_id() const { return feedback_id_; }
const std::string &Gripper::last_error() const { return last_error_; }

void Gripper::set_error(const std::string &operation) {
  last_error_ = operation + ": " + std::strerror(errno);
}

} // namespace rebotarm_gripper_sdk
