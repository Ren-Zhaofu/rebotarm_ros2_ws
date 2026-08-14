#include <chrono>
#include <memory>
#include <thread>
#include <algorithm>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "rebotarm_planning_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  const bool execute = node->declare_parameter("execute", false);
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(5);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);

  arm.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  const bool home_ok = arm.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  RCLCPP_INFO(node->get_logger(), "Home joint-target planning: %s", home_ok ? "success" : "failed");
  if (execute && home_ok) {
    arm.execute(home_plan);
  }

  const auto current = arm.getCurrentPose("gripper_tcp").pose;
  geometry_msgs::msg::Pose approach = current;
  approach.position.z += 0.03;
  arm.setPoseTarget(approach, "gripper_tcp");
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
  const bool pose_ok = arm.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  RCLCPP_INFO(node->get_logger(), "TCP pose-target planning: %s", pose_ok ? "success" : "failed");
  if (execute && pose_ok) {
    arm.execute(pose_plan);
  }

  std::vector<geometry_msgs::msg::Pose> waypoints{current, approach};
  moveit_msgs::msg::RobotTrajectory cartesian;
  const double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, cartesian);
  RCLCPP_INFO(node->get_logger(), "TCP Cartesian approach coverage: %.1f%%", fraction * 100.0);
  if (execute && fraction > 0.95) {
    arm.execute(cartesian);
    auto gripper = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      node, "/gripper_controller/gripper_cmd");
    if (gripper->wait_for_action_server(3s)) {
      control_msgs::action::GripperCommand::Goal close_goal;
      close_goal.command.position = 0.0;
      close_goal.command.max_effort = 20.0;
      gripper->async_send_goal(close_goal);
      std::this_thread::sleep_for(1500ms);
      std::reverse(waypoints.begin(), waypoints.end());
      moveit_msgs::msg::RobotTrajectory lift;
      if (arm.computeCartesianPath(waypoints, 0.005, 0.0, lift) > 0.95) {
        arm.execute(lift);
      }
      control_msgs::action::GripperCommand::Goal open_goal;
      open_goal.command.position = 0.05;
      open_goal.command.max_effort = 20.0;
      gripper->async_send_goal(open_goal);
    } else {
      RCLCPP_WARN(node->get_logger(), "Gripper action unavailable; skipping close/lift/release sequence");
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return home_ok && pose_ok && fraction > 0.95 ? 0 : 1;
}
