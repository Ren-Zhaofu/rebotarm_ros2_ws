#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rebotarm_hardware/rebotarm_system.hpp"
#include "gtest/gtest.h"

namespace {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

hardware_interface::HardwareInfo make_hardware_info() {
  hardware_interface::HardwareInfo info;
  info.name = "RebotArmSystem";
  info.type = "system";
  info.hardware_class_type = "rebotarm_hardware/RebotArmSystem";
  info.hardware_parameters["transport"] = "mock";

  const std::vector<double> lower{-2.8, -3.14, -3.14, -1.87, -1.57, -3.14};
  const std::vector<double> upper{2.8, 0.0, 0.0, 1.57, 1.57, 3.14};
  for (std::size_t i = 0; i < rebotarm_hardware::RebotArmSystem::kJointCount;
       ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(i + 1);
    joint.type = "joint";

    hardware_interface::InterfaceInfo command;
    command.name = hardware_interface::HW_IF_POSITION;
    command.min = std::to_string(lower[i]);
    command.max = std::to_string(upper[i]);
    joint.command_interfaces.push_back(command);

    for (const auto *name : {hardware_interface::HW_IF_POSITION,
                             hardware_interface::HW_IF_VELOCITY,
                             hardware_interface::HW_IF_EFFORT}) {
      hardware_interface::InterfaceInfo state;
      state.name = name;
      joint.state_interfaces.push_back(state);
    }
    info.joints.push_back(joint);
  }
  return info;
}

void add_socketcan_parameters(hardware_interface::HardwareInfo &info,
                              bool allow_enable) {
  info.hardware_parameters["transport"] = "socketcan";
  info.hardware_parameters["model"] = "dm";
  info.hardware_parameters["can_interface"] = "vcan0";
  info.hardware_parameters["allow_motor_enable"] =
      allow_enable ? "true" : "false";
  info.hardware_parameters["motor_enable_mask"] =
      allow_enable ? "1,0,0,0,0,0" : "0,0,0,0,0,0";
  info.hardware_parameters["motor_ids"] = "0x01,0x02,0x03,0x04,0x05,0x06";
  info.hardware_parameters["feedback_ids"] = "0x11,0x12,0x13,0x14,0x15,0x16";
  info.hardware_parameters["motor_models"] =
      "dm4340_48v,dm4340_48v,dm4340_48v,dm4310,dm4310,dm4310";
  info.hardware_parameters["joint_directions"] = "1,1,1,1,1,1";
  info.hardware_parameters["joint_offsets"] = "0,0,0,0,0,0";
  info.hardware_parameters["position_velocity_limits"] =
      "0.05,0.05,0.05,0.05,0.05,0.05";
  info.hardware_parameters["feedback_timeout_ms"] = "100";
  info.hardware_parameters["max_command_step"] = "0.05";
}

void add_rs_socketcan_parameters(hardware_interface::HardwareInfo &info) {
  info.hardware_parameters["transport"] = "socketcan";
  info.hardware_parameters["model"] = "rs";
  info.hardware_parameters["can_interface"] = "vcan0";
  info.hardware_parameters["allow_motor_enable"] = "false";
  info.hardware_parameters["motor_enable_mask"] = "0,0,0,0,0,0";
  info.hardware_parameters["motor_ids"] = "0x01,0x02,0x03,0x04,0x05,0x06";
  info.hardware_parameters["feedback_ids"] = "0xFD,0xFD,0xFD,0xFD,0xFD,0xFD";
  info.hardware_parameters["motor_models"] =
      "rs-06,rs-06,rs-06,rs-00,rs-00,rs-00";
  info.hardware_parameters["mit_kp"] = "50,150,150,50,50,50";
  info.hardware_parameters["mit_kd"] = "3,10,10,5,4,4";
  info.hardware_parameters["joint_directions"] = "1,1,1,1,1,1";
  info.hardware_parameters["joint_offsets"] = "0,0,0,0,0,0";
  info.hardware_parameters["position_velocity_limits"] = "1,1,1,1,1,1";
  info.hardware_parameters["feedback_timeout_ms"] = "100";
  info.hardware_parameters["max_command_step"] = "0.05";
}

TEST(RebotArmSystemTest, InitializesAndExportsExpectedInterfaces) {
  rebotarm_hardware::RebotArmSystem system;
  EXPECT_EQ(system.on_init(make_hardware_info()), CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_command_interfaces().size(), 6U);
  EXPECT_EQ(system.export_state_interfaces().size(), 18U);
}

TEST(RebotArmSystemTest, RejectsInvalidConfiguration) {
  auto wrong_transport = make_hardware_info();
  wrong_transport.hardware_parameters["transport"] = "can";
  rebotarm_hardware::RebotArmSystem transport_system;
  EXPECT_EQ(transport_system.on_init(wrong_transport), CallbackReturn::ERROR);

  auto wrong_name = make_hardware_info();
  wrong_name.joints[2].name = "elbow";
  rebotarm_hardware::RebotArmSystem name_system;
  EXPECT_EQ(name_system.on_init(wrong_name), CallbackReturn::ERROR);

  auto wrong_interfaces = make_hardware_info();
  wrong_interfaces.joints[0].state_interfaces.pop_back();
  rebotarm_hardware::RebotArmSystem interface_system;
  EXPECT_EQ(interface_system.on_init(wrong_interfaces), CallbackReturn::ERROR);
}

TEST(RebotArmSystemTest, SupportsReadOnlySocketCanWithoutEnableGate) {
  auto info = make_hardware_info();
  add_socketcan_parameters(info, false);
  rebotarm_hardware::RebotArmSystem system;
  EXPECT_EQ(system.on_init(info), CallbackReturn::SUCCESS);
}

TEST(RebotArmSystemTest, SupportsRobStrideSocketCanConfiguration) {
  auto info = make_hardware_info();
  add_rs_socketcan_parameters(info);
  rebotarm_hardware::RebotArmSystem system;
  EXPECT_EQ(system.on_init(info), CallbackReturn::SUCCESS);
}

TEST(RebotArmSystemTest, RejectsInvalidSocketCanModelMapping) {
  auto info = make_hardware_info();
  add_socketcan_parameters(info, false);
  info.hardware_parameters["motor_models"] =
      "dm4310,dm4310,dm4310,dm4310,dm4310,unknown";
  rebotarm_hardware::RebotArmSystem system;
  EXPECT_EQ(system.on_init(info), CallbackReturn::ERROR);
}

TEST(RebotArmSystemTest, RejectsEnableWithoutSelectedJoint) {
  auto info = make_hardware_info();
  add_socketcan_parameters(info, true);
  info.hardware_parameters["motor_enable_mask"] = "0,0,0,0,0,0";
  rebotarm_hardware::RebotArmSystem system;
  EXPECT_EQ(system.on_init(info), CallbackReturn::ERROR);
}

TEST(RebotArmSystemTest, FollowsValidCommands) {
  rebotarm_hardware::RebotArmSystem system;
  ASSERT_EQ(system.on_init(make_hardware_info()), CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  commands[0].set_value(1.25);
  commands[1].set_value(-0.5);

  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  // Activation seeds commands from the current state, as required by
  // ros2_control.
  commands[0].set_value(1.25);
  commands[1].set_value(-0.5);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 1.25);
  EXPECT_DOUBLE_EQ(states[3].get_value(), -0.5);
}

TEST(RebotArmSystemTest, RejectsOutOfRangeAndNonFiniteCommands) {
  rebotarm_hardware::RebotArmSystem system;
  ASSERT_EQ(system.on_init(make_hardware_info()), CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();

  commands[0].set_value(2.81);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);

  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
}

TEST(RebotArmSystemTest, LoadsThroughPluginlib) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto system = loader.createSharedInstance("rebotarm_hardware/RebotArmSystem");
  ASSERT_NE(system, nullptr);
  EXPECT_EQ(system->on_init(make_hardware_info()), CallbackReturn::SUCCESS);
}

} // namespace
