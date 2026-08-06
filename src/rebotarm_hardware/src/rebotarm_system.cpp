#include "rebotarm_hardware/rebotarm_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rebotarm_hardware {

namespace {

using rebotarm_dm_motor_sdk::ControlMode;
using rebotarm_dm_motor_sdk::MotorModel;
using rebotarm_dm_motor_sdk::MotorState;
using rebotarm_dm_motor_sdk::ParameterResponse;
using rebotarm_dm_motor_sdk::PositionVelocityCommand;
using rebotarm_dm_motor_sdk::Protocol;
using rebotarm_dm_motor_sdk::Register;

const auto kLogger = rclcpp::get_logger("rebotarm_hardware.RebotArmSystem");

bool has_exact_interfaces(
    const std::vector<hardware_interface::InterfaceInfo> &actual,
    const std::vector<std::string> &expected) {
  if (actual.size() != expected.size()) {
    return false;
  }
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (actual[i].name != expected[i]) {
      return false;
    }
  }
  return true;
}

template <typename T, typename Converter>
bool parse_list(const std::string &text,
                std::array<T, RebotArmSystem::kJointCount> &output,
                Converter converter) {
  std::stringstream stream(text);
  std::string item;
  std::size_t index = 0;
  try {
    while (std::getline(stream, item, ',')) {
      if (index >= output.size() || item.empty()) {
        return false;
      }
      output[index++] = converter(item);
    }
  } catch (const std::exception &) {
    return false;
  }
  return index == output.size();
}

bool parameter_is_true(const std::string &value) {
  return value == "true" || value == "True" || value == "TRUE" || value == "1";
}

MotorModel parse_motor_model(const std::string &value) {
  if (value == "dm4340_48v") {
    return MotorModel::kDm4340_48V;
  }
  if (value == "dm4310") {
    return MotorModel::kDm4310;
  }
  throw std::invalid_argument("unsupported reBotArm DM motor model: " + value);
}

const char *register_name(Register reg) {
  switch (reg) {
  case Register::kControlMode:
    return "CTRL_MODE";
  case Register::kPositionMaximum:
    return "PMAX";
  case Register::kVelocityMaximum:
    return "VMAX";
  case Register::kTorqueMaximum:
    return "TMAX";
  default:
    return "unknown";
  }
}

} // namespace

RebotArmSystem::~RebotArmSystem() { disable_motors(); }

RebotArmSystem::CallbackReturn
RebotArmSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto transport = info_.hardware_parameters.find("transport");
  if (transport == info_.hardware_parameters.end() ||
      (transport->second != "mock" && transport->second != "socketcan")) {
    RCLCPP_ERROR(kLogger, "transport must be 'mock' or 'socketcan'");
    return CallbackReturn::ERROR;
  }
  transport_ = transport->second;
  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(kLogger, "Expected %zu joints, got %zu", kJointCount,
                 info_.joints.size());
    return CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!validate_joint(info_.joints[i], i)) {
      return CallbackReturn::ERROR;
    }
  }

  position_states_.fill(0.0);
  velocity_states_.fill(0.0);
  effort_states_.fill(0.0);
  position_commands_.fill(0.0);
  hold_position_commands_.fill(0.0);
  last_sent_commands_.fill(0.0);
  direction_.fill(1.0);
  offset_.fill(0.0);
  feedback_seen_.fill(false);
  enable_mask_.fill(false);
  motor_enabled_.fill(false);
  feedback_states_.fill(0);
  active_ = false;

  if (transport_ == "socketcan" && !configure_socketcan()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool RebotArmSystem::configure_socketcan() {
  const auto model = info_.hardware_parameters.find("model");
  if (model == info_.hardware_parameters.end() || model->second != "dm") {
    RCLCPP_ERROR(kLogger,
                 "SocketCAN hardware currently supports only model=dm");
    return false;
  }

  const auto get = [this](const std::string &name) -> const std::string * {
    const auto found = info_.hardware_parameters.find(name);
    return found == info_.hardware_parameters.end() ? nullptr : &found->second;
  };
  const auto interface = get("can_interface");
  if (interface == nullptr || interface->empty()) {
    RCLCPP_ERROR(kLogger, "socketcan requires a non-empty can_interface");
    return false;
  }
  can_interface_ = *interface;
  const auto enable = get("allow_motor_enable");
  allow_motor_enable_ = enable != nullptr && parameter_is_true(*enable);
  const auto hold_only = get("hold_only");
  hold_only_ = hold_only != nullptr && parameter_is_true(*hold_only);

  const auto parse_double_array = [&](const char *name, auto &target) {
    const auto value = get(name);
    return value != nullptr &&
           parse_list(*value, target,
                      [](const std::string &item) { return std::stod(item); });
  };
  const auto parse_id_array = [&](const char *name, auto &target) {
    const auto value = get(name);
    return value != nullptr &&
           parse_list(*value, target, [](const std::string &item) {
             const auto parsed = std::stoul(item, nullptr, 0);
             if (parsed > 0x7FF) {
               throw std::out_of_range("CAN identifier");
             }
             return static_cast<std::uint16_t>(parsed);
           });
  };
  const auto models = get("motor_models");
  const auto enable_mask = get("motor_enable_mask");
  std::array<std::uint8_t, kJointCount> enable_mask_values{};
  if (!parse_id_array("motor_ids", motor_ids_) ||
      !parse_id_array("feedback_ids", feedback_ids_) || models == nullptr ||
      !parse_list(*models, motor_models_, parse_motor_model) ||
      !parse_double_array("joint_directions", direction_) ||
      !parse_double_array("joint_offsets", offset_) ||
      !parse_double_array("position_velocity_limits",
                          position_velocity_limits_) ||
      enable_mask == nullptr ||
      !parse_list(*enable_mask, enable_mask_values,
                  [](const std::string &item) {
                    const auto value = std::stoul(item);
                    if (value > 1) {
                      throw std::out_of_range("motor enable mask");
                    }
                    return static_cast<std::uint8_t>(value);
                  })) {
    RCLCPP_ERROR(
        kLogger,
        "Invalid or incomplete six-value SocketCAN hardware parameters");
    return false;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    enable_mask_[i] = allow_motor_enable_ && enable_mask_values[i] == 1;
    motor_limits_[i] = Protocol::limits(motor_models_[i]);
    if ((direction_[i] != 1.0 && direction_[i] != -1.0) ||
        !std::isfinite(offset_[i]) ||
        !std::isfinite(position_velocity_limits_[i]) ||
        position_velocity_limits_[i] <= 0.0 ||
        position_velocity_limits_[i] > motor_limits_[i].velocity) {
      RCLCPP_ERROR(kLogger, "Unsafe SocketCAN parameter for joint%zu", i + 1);
      return false;
    }
  }
  if (allow_motor_enable_ &&
      std::none_of(enable_mask_.begin(), enable_mask_.end(),
                   [](bool enabled) { return enabled; })) {
    RCLCPP_ERROR(kLogger,
                 "allow_motor_enable=true requires at least one enabled joint "
                 "in motor_enable_mask");
    return false;
  }

  try {
    const auto timeout = get("feedback_timeout_ms");
    const auto step = get("max_command_step");
    if (timeout == nullptr || step == nullptr) {
      throw std::invalid_argument("missing safety parameter");
    }
    feedback_timeout_ = std::chrono::milliseconds(std::stoul(*timeout));
    max_command_step_ = std::stod(*step);
    if (feedback_timeout_.count() < 20 || feedback_timeout_.count() > 1000 ||
        !std::isfinite(max_command_step_) || max_command_step_ <= 0.0 ||
        max_command_step_ > 0.5) {
      throw std::invalid_argument("safety parameter outside accepted range");
    }
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(kLogger, "Invalid SocketCAN safety parameter: %s",
                 exception.what());
    return false;
  }

  try {
    bus_ = std::make_unique<rebotarm_dm_motor_sdk::MotorBus>();
    for (std::size_t i = 0; i < kJointCount; ++i) {
      bus_->add_motor({info_.joints[i].name, motor_ids_[i], feedback_ids_[i],
                       motor_models_[i]});
    }
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(kLogger, "Invalid DM motor configuration: %s",
                 exception.what());
    bus_.reset();
    return false;
  }

  if (!allow_motor_enable_) {
    RCLCPP_INFO(kLogger, "SocketCAN configured in read-only mode; no enable or "
                         "position command will be sent");
  } else if (hold_only_) {
    RCLCPP_INFO(kLogger,
                "SocketCAN configured for frozen-position hold_only validation");
  }
  return true;
}

bool RebotArmSystem::validate_joint(
    const hardware_interface::ComponentInfo &joint, std::size_t index) {
  const std::string expected_name = "joint" + std::to_string(index + 1);
  if (joint.name != expected_name) {
    RCLCPP_ERROR(kLogger, "Joint %zu must be named '%s', got '%s'", index + 1,
                 expected_name.c_str(), joint.name.c_str());
    return false;
  }

  const std::vector<std::string> command_names{
      hardware_interface::HW_IF_POSITION};
  const std::vector<std::string> state_names{hardware_interface::HW_IF_POSITION,
                                             hardware_interface::HW_IF_VELOCITY,
                                             hardware_interface::HW_IF_EFFORT};
  if (!has_exact_interfaces(joint.command_interfaces, command_names) ||
      !has_exact_interfaces(joint.state_interfaces, state_names)) {
    RCLCPP_ERROR(kLogger,
                 "%s must have one position command interface and "
                 "position/velocity/effort state interfaces",
                 joint.name.c_str());
    return false;
  }

  const auto &position_interface = joint.command_interfaces.front();
  if (position_interface.min.empty() || position_interface.max.empty()) {
    RCLCPP_ERROR(kLogger, "%s position limits must define min and max",
                 joint.name.c_str());
    return false;
  }

  try {
    std::size_t min_end = 0;
    std::size_t max_end = 0;
    lower_limits_[index] = std::stod(position_interface.min, &min_end);
    upper_limits_[index] = std::stod(position_interface.max, &max_end);
    if (min_end != position_interface.min.size() ||
        max_end != position_interface.max.size() ||
        !std::isfinite(lower_limits_[index]) ||
        !std::isfinite(upper_limits_[index]) ||
        lower_limits_[index] > upper_limits_[index]) {
      throw std::invalid_argument("invalid position limits");
    }
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(kLogger, "Invalid position limits for %s: %s",
                 joint.name.c_str(), exception.what());
    return false;
  }
  return true;
}

std::vector<hardware_interface::StateInterface>
RebotArmSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 3);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &position_states_[i]);
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY,
                            &velocity_states_[i]);
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_EFFORT,
                            &effort_states_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RebotArmSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &position_commands_[i]);
  }
  return interfaces;
}

bool RebotArmSystem::verify_motor_parameters() {
  const auto read = [this](std::size_t index, Register reg,
                           ParameterResponse &response) {
    if (!bus_->read_parameter(index, reg)) {
      RCLCPP_ERROR(kLogger, "Failed to request %s from joint%zu: %s",
                   register_name(reg), index + 1, bus_->last_error().c_str());
      return false;
    }
    std::size_t response_index = 0;
    if (!bus_->receive_parameter(response, response_index, feedback_timeout_) ||
        response_index != index || response.reg != reg ||
        response.operation != 0x33) {
      RCLCPP_ERROR(kLogger, "Missing or invalid %s response from joint%zu: %s",
                   register_name(reg), index + 1, bus_->last_error().c_str());
      return false;
    }
    return true;
  };

  for (std::size_t i = 0; i < kJointCount; ++i) {
    ParameterResponse response;
    if (!read(i, Register::kControlMode, response)) {
      return false;
    }
    const auto *mode = std::get_if<std::uint32_t>(&response.value);
    if (mode == nullptr ||
        *mode != static_cast<std::uint32_t>(ControlMode::kPositionVelocity)) {
      RCLCPP_ERROR(kLogger,
                   "joint%zu CTRL_MODE is not position-velocity mode 2", i + 1);
      return false;
    }

    const std::array<std::pair<Register, double>, 3> expected{{
        {Register::kPositionMaximum, motor_limits_[i].position},
        {Register::kVelocityMaximum, motor_limits_[i].velocity},
        {Register::kTorqueMaximum, motor_limits_[i].effort},
    }};
    for (const auto &item : expected) {
      if (!read(i, item.first, response)) {
        return false;
      }
      const auto *value = std::get_if<float>(&response.value);
      if (value == nullptr ||
          std::abs(static_cast<double>(*value) - item.second) > 1e-4) {
        RCLCPP_ERROR(kLogger,
                     "joint%zu %s does not match configured protocol limits",
                     i + 1, register_name(item.first));
        return false;
      }
    }
  }
  return true;
}

bool RebotArmSystem::request_feedback() {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!bus_->request_state(i)) {
      RCLCPP_ERROR(kLogger, "Feedback request failed for joint%zu: %s", i + 1,
                   bus_->last_error().c_str());
      return false;
    }
  }
  return true;
}

bool RebotArmSystem::receive_one_feedback(std::chrono::microseconds timeout) {
  MotorState state;
  std::size_t index = 0;
  if (!bus_->receive_state(state, index, timeout)) {
    return false;
  }
  if (index >= kJointCount || state.motor_id != (motor_ids_[index] & 0x0F)) {
    RCLCPP_ERROR(kLogger,
                 "DM feedback motor ID does not match the configured joint");
    return false;
  }
  if (state.status >= 8) {
    RCLCPP_ERROR(kLogger,
                 "DM fault state 0x%X on joint%zu (MOS %u C, rotor %u C)",
                 state.status, index + 1, state.mos_temperature,
                 state.rotor_temperature);
    return false;
  }
  const std::uint8_t expected_status = motor_enabled_[index] ? 1 : 0;
  if (state.status != expected_status) {
    RCLCPP_ERROR(kLogger,
                 "joint%zu state is %u but staged enable policy requires %u",
                 index + 1, state.status, expected_status);
    return false;
  }

  position_states_[index] = direction_[index] * state.position + offset_[index];
  velocity_states_[index] = direction_[index] * state.velocity;
  effort_states_[index] = direction_[index] * state.effort;
  feedback_seen_[index] = true;
  feedback_states_[index] = state.status;
  last_feedback_[index] = std::chrono::steady_clock::now();
  return true;
}

bool RebotArmSystem::await_feedback(bool require_enabled) {
  const auto deadline = std::chrono::steady_clock::now() + feedback_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - std::chrono::steady_clock::now());
    if (!receive_one_feedback(remaining)) {
      break;
    }
    bool complete = std::all_of(feedback_seen_.begin(), feedback_seen_.end(),
                                [](bool seen) { return seen; });
    if (complete && require_enabled) {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const std::uint8_t expected = motor_enabled_[i] ? 1 : 0;
        complete = complete && feedback_states_[i] == expected;
      }
    }
    if (complete) {
      return true;
    }
  }
  RCLCPP_ERROR(kLogger,
               "Timed out waiting for a complete six-axis DM feedback set");
  return false;
}

bool RebotArmSystem::send_position_velocity_commands() {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!enable_mask_[i]) {
      continue;
    }
    const double joint_target =
        hold_only_ ? hold_position_commands_[i] : position_commands_[i];
    const double motor_position =
        direction_[i] * (joint_target - offset_[i]);
    if (!std::isfinite(motor_position) ||
        std::abs(motor_position) > motor_limits_[i].position) {
      RCLCPP_ERROR(kLogger,
                   "Mapped motor command exceeds protocol range for joint%zu",
                   i + 1);
      return false;
    }
    if (!bus_->send_position_velocity(
            i, PositionVelocityCommand{
                   static_cast<float>(motor_position),
                   static_cast<float>(position_velocity_limits_[i])})) {
      RCLCPP_ERROR(kLogger, "Position-velocity command failed for joint%zu: %s",
                   i + 1, bus_->last_error().c_str());
      return false;
    }
    last_sent_commands_[i] = joint_target;
  }
  return true;
}

RebotArmSystem::CallbackReturn
RebotArmSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (transport_ == "socketcan") {
    if (!bus_->connect(can_interface_)) {
      RCLCPP_ERROR(kLogger, "%s", bus_->last_error().c_str());
      return CallbackReturn::ERROR;
    }
    if (!verify_motor_parameters()) {
      bus_->disconnect();
      return CallbackReturn::ERROR;
    }

    feedback_seen_.fill(false);
    if (!request_feedback() || !await_feedback(false)) {
      RCLCPP_ERROR(kLogger,
                   "Not all six DM motors responded; motors remain disabled");
      bus_->disconnect();
      return CallbackReturn::ERROR;
    }
    position_commands_ = position_states_;
    hold_position_commands_ = position_states_;
    last_sent_commands_ = position_states_;

    if (!allow_motor_enable_) {
      active_ = true;
      RCLCPP_INFO(
          kLogger,
          "Six DM motors connected read-only on %s; motors remain disabled",
          can_interface_.c_str());
      return CallbackReturn::SUCCESS;
    }

    if (!hold_only_) {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        if (!enable_mask_[i]) {
          continue;
        }
        if (!std::isfinite(position_states_[i]) ||
            position_states_[i] < lower_limits_[i] ||
            position_states_[i] > upper_limits_[i]) {
          RCLCPP_ERROR(
              kLogger,
              "Measured position %.6f for joint%zu is outside URDF limits "
              "[%.6f, %.6f]; verify motor zero, direction, and offset before "
              "enabling trajectory control",
              position_states_[i], i + 1, lower_limits_[i], upper_limits_[i]);
          bus_->disconnect();
          return CallbackReturn::ERROR;
        }
      }
    }

    if (!send_position_velocity_commands()) {
      bus_->disconnect();
      return CallbackReturn::ERROR;
    }
    feedback_seen_.fill(false);
    if (!request_feedback() || !await_feedback(false)) {
      bus_->disconnect();
      return CallbackReturn::ERROR;
    }

    feedback_seen_.fill(false);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (!enable_mask_[i]) {
        continue;
      }
      if (!bus_->enable(i)) {
        RCLCPP_ERROR(kLogger, "Motor enable failed for joint%zu: %s", i + 1,
                     bus_->last_error().c_str());
        motor_enabled_[i] = true;
        disable_motors();
        return CallbackReturn::ERROR;
      }
      motor_enabled_[i] = true;
    }
    if (!request_feedback() || !await_feedback(true) ||
        !send_position_velocity_commands()) {
      RCLCPP_ERROR(kLogger,
                   "DM motor enable or initial hold acknowledgement failed");
      disable_motors();
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
        kLogger,
        "Requested DM motor subset enabled in position-velocity mode on %s at "
        "its measured positions",
        can_interface_.c_str());
  }

  position_commands_ = position_states_;
  last_sent_commands_ = position_states_;
  active_ = true;
  return CallbackReturn::SUCCESS;
}

RebotArmSystem::CallbackReturn
RebotArmSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  disable_motors();
  velocity_states_.fill(0.0);
  effort_states_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

void RebotArmSystem::disable_motors() {
  if (!bus_ || !bus_->connected()) {
    motor_enabled_.fill(false);
    return;
  }
  if (std::any_of(motor_enabled_.begin(), motor_enabled_.end(),
                  [](bool enabled) { return enabled; })) {
    for (int attempt = 0; attempt < 2; ++attempt) {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        if (!motor_enabled_[i]) {
          continue;
        }
        if (!bus_->disable(i)) {
          RCLCPP_ERROR(kLogger, "Motor disable command failed for joint%zu: %s",
                       i + 1, bus_->last_error().c_str());
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  motor_enabled_.fill(false);
  bus_->disconnect();
}

hardware_interface::return_type RebotArmSystem::read(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (transport_ == "mock") {
    position_states_ = position_commands_;
    velocity_states_.fill(0.0);
    effort_states_.fill(0.0);
    return hardware_interface::return_type::OK;
  }

  feedback_seen_.fill(false);
  if (!request_feedback() || !await_feedback(true)) {
    disable_motors();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
RebotArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (transport_ == "socketcan" && !allow_motor_enable_) {
    return hardware_interface::return_type::OK;
  }
  if (transport_ == "socketcan" && hold_only_) {
    if (!active_ ||
        std::none_of(motor_enabled_.begin(), motor_enabled_.end(),
                     [](bool enabled) { return enabled; }) ||
        !bus_ || !bus_->connected()) {
      RCLCPP_ERROR(kLogger,
                   "Rejecting frozen hold while selected motors are not enabled");
      disable_motors();
      return hardware_interface::return_type::ERROR;
    }
    if (!send_position_velocity_commands()) {
      disable_motors();
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (transport_ == "socketcan" && !enable_mask_[i]) {
      continue;
    }
    const double command = position_commands_[i];
    if (!std::isfinite(command)) {
      RCLCPP_ERROR(kLogger, "Non-finite position command for %s",
                   info_.joints[i].name.c_str());
      if (transport_ == "socketcan") {
        disable_motors();
      }
      return hardware_interface::return_type::ERROR;
    }
    if (command < lower_limits_[i] || command > upper_limits_[i]) {
      RCLCPP_ERROR(kLogger,
                   "Position command %.6f for %s is outside [%.6f, %.6f]",
                   command, info_.joints[i].name.c_str(), lower_limits_[i],
                   upper_limits_[i]);
      if (transport_ == "socketcan") {
        disable_motors();
      }
      return hardware_interface::return_type::ERROR;
    }
    if (transport_ == "socketcan" &&
        std::abs(command - last_sent_commands_[i]) > max_command_step_) {
      RCLCPP_ERROR(kLogger, "Command step %.6f rad for %s exceeds %.6f rad",
                   std::abs(command - last_sent_commands_[i]),
                   info_.joints[i].name.c_str(), max_command_step_);
      disable_motors();
      return hardware_interface::return_type::ERROR;
    }
  }

  if (transport_ == "socketcan") {
    if (!active_ ||
        std::none_of(motor_enabled_.begin(), motor_enabled_.end(),
                     [](bool enabled) { return enabled; }) ||
        !bus_ || !bus_->connected()) {
      RCLCPP_ERROR(kLogger,
                   "Rejecting command while real motors are not enabled");
      disable_motors();
      return hardware_interface::return_type::ERROR;
    }
    if (!send_position_velocity_commands()) {
      disable_motors();
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

} // namespace rebotarm_hardware

PLUGINLIB_EXPORT_CLASS(rebotarm_hardware::RebotArmSystem,
                       hardware_interface::SystemInterface)
