#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rebotarm_dm_motor_sdk/dm_motor.hpp"

namespace rebotarm_hardware {

class RebotArmSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RebotArmSystem)
  ~RebotArmSystem() override;

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  static constexpr std::size_t kJointCount = 6;

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  bool validate_joint(const hardware_interface::ComponentInfo &joint,
                      std::size_t index);
  bool configure_socketcan();
  bool verify_motor_parameters();
  bool request_feedback();
  bool receive_one_feedback(std::chrono::microseconds timeout);
  bool await_feedback(bool require_enabled);
  bool send_position_velocity_commands();
  void disable_motors();

  std::array<double, kJointCount> position_states_{};
  std::array<double, kJointCount> velocity_states_{};
  std::array<double, kJointCount> effort_states_{};
  std::array<double, kJointCount> position_commands_{};
  std::array<double, kJointCount> hold_position_commands_{};
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> direction_{};
  std::array<double, kJointCount> offset_{};
  std::array<double, kJointCount> position_velocity_limits_{};
  std::array<double, kJointCount> last_sent_commands_{};
  std::array<std::uint16_t, kJointCount> motor_ids_{};
  std::array<std::uint16_t, kJointCount> feedback_ids_{};
  std::array<rebotarm_dm_motor_sdk::MotorModel, kJointCount> motor_models_{};
  std::array<rebotarm_dm_motor_sdk::Limits, kJointCount> motor_limits_{};
  std::array<std::chrono::steady_clock::time_point, kJointCount>
      last_feedback_{};
  std::array<bool, kJointCount> feedback_seen_{};
  std::array<bool, kJointCount> enable_mask_{};
  std::array<bool, kJointCount> motor_enabled_{};
  std::array<std::uint8_t, kJointCount> feedback_states_{};
  std::unique_ptr<rebotarm_dm_motor_sdk::MotorBus> bus_;
  std::string transport_{"mock"};
  std::string can_interface_{"can0"};
  std::chrono::milliseconds feedback_timeout_{100};
  double max_command_step_{0.05};
  bool allow_motor_enable_{false};
  bool hold_only_{false};
  bool active_{false};
};

} // namespace rebotarm_hardware
