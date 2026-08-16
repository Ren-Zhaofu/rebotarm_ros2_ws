from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from rebotarm_digital_twin.can_preflight import (
    check_motor_feedback,
    check_socketcan_interface,
)


def start_after_can_preflight(context, actions):
    interface = LaunchConfiguration("can_interface").perform(context)
    model = LaunchConfiguration("model").perform(context)
    ready, message = check_socketcan_interface(interface)
    if not ready:
        get_logger("feedback_twin").error(f"CAN preflight failed: {message}")
        return []
    feedback_ready, feedback_message = check_motor_feedback(interface, model)
    if not feedback_ready:
        get_logger("feedback_twin").error(
            f"CAN preflight failed: {feedback_message}"
        )
        return []
    return [LogInfo(msg=f"{message} {feedback_message}"), *actions]


def generate_launch_description():
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    description = Path(get_package_share_directory("rebotarm_description"))
    model = LaunchConfiguration("model")
    can_interface = LaunchConfiguration("can_interface")
    allow_motor_enable = LaunchConfiguration("allow_motor_enable")
    motor_enable_mask = LaunchConfiguration("motor_enable_mask")
    robot_description = ParameterValue(
        Command(["xacro ", str(description / "urdf" / "rebotarm.urdf.xacro"), " model:=", model]),
        value_type=str,
    )
    real_bringup = GroupAction([
        PushRosNamespace("real"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "rebotarm_hardware.launch.py")),
            launch_arguments={
                "model": model,
                "can_interface": can_interface,
                "allow_motor_enable": allow_motor_enable,
                "hold_only": "false",
                "enable_on_controller_start": "true",
                "motor_enable_mask": motor_enable_mask,
                "start_arm_controller": allow_motor_enable,
                "arm_controller_start_stopped": "true",
                "publish_robot_state": "false",
            }.items(),
        ),
    ])
    runtime_actions = [
        real_bringup,
        Node(
            package="rebotarm_digital_twin",
            executable="state_mirror.py",
            parameters=[{"model": model}],
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="mode_arbiter.py",
            parameters=[{"model": model}],
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="joint_control_gui.py",
            parameters=[{"model": model}],
            condition=IfCondition(allow_motor_enable),
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="target_limiter.py",
            parameters=[{"model": model}],
            condition=IfCondition(allow_motor_enable),
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="target_to_trajectory.py",
            parameters=[{
                "input_topic": "/real/command_target",
                "output_topic": "/real/arm_controller/joint_trajectory",
            }],
            condition=IfCondition(allow_motor_enable),
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/twin/joint_states")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(description / "rviz" / "display.rviz")],
            output="screen",
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="dm", choices=["dm", "rs"]),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument(
            "allow_motor_enable", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "motor_enable_mask", default_value="0,0,0,0,0,0"
        ),
        OpaqueFunction(
            function=start_after_can_preflight,
            args=[runtime_actions],
        ),
    ])
