from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("rebotarm_bringup"))
    model = LaunchConfiguration("model")
    transport = LaunchConfiguration("transport")
    can_interface = LaunchConfiguration("can_interface")
    allow_motor_enable = LaunchConfiguration("allow_motor_enable")
    hold_only = LaunchConfiguration("hold_only")
    enable_on_controller_start = LaunchConfiguration("enable_on_controller_start")
    motor_enable_mask = LaunchConfiguration("motor_enable_mask")
    start_arm_controller = LaunchConfiguration("start_arm_controller")
    arm_controller_start_stopped = LaunchConfiguration("arm_controller_start_stopped")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                str(bringup_share / "urdf" / "rebotarm_controlled.urdf.xacro"),
                " model:=",
                model,
                " transport:=",
                transport,
                " can_interface:=",
                can_interface,
                " allow_motor_enable:=",
                allow_motor_enable,
                " hold_only:=",
                hold_only,
                " enable_on_controller_start:=",
                enable_on_controller_start,
                " motor_enable_mask:=",
                motor_enable_mask,
            ]
        ),
        value_type=str,
    )
    controllers = str(bringup_share / "config" / "controllers.yaml")
    arm_controller_allowed = PythonExpression(
        [
            "'",
            start_arm_controller,
            "' == 'true' and ('",
            transport,
            "' == 'mock' or ('",
            allow_motor_enable,
            "' == 'true' and '",
            hold_only,
            "' == 'false'))",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="rs",
                choices=["rs", "dm"],
                description="reBotArm mechanical and motor model",
            ),
            DeclareLaunchArgument(
                "transport",
                default_value="mock",
                choices=["mock", "socketcan"],
                description="Mock control or SocketCAN hardware transport",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="Preconfigured SocketCAN interface",
            ),
            DeclareLaunchArgument(
                "allow_motor_enable",
                default_value="false",
                choices=["true", "false"],
                description="Explicit real-motor enable safety gate",
            ),
            DeclareLaunchArgument(
                "hold_only",
                default_value="true",
                choices=["true", "false"],
                description="Hold activation positions and ignore trajectories",
            ),
            DeclareLaunchArgument(
                "enable_on_controller_start",
                default_value="false",
                choices=["true", "false"],
                description="Defer real motor enable until a position controller claims interfaces",
            ),
            DeclareLaunchArgument(
                "motor_enable_mask",
                default_value="0,0,0,0,0,0",
                description="Six-value staged motor enable mask",
            ),
            DeclareLaunchArgument(
                "start_arm_controller",
                default_value="true",
                choices=["true", "false"],
                description="Start the trajectory controller when safety gates allow",
            ),
            DeclareLaunchArgument(
                "arm_controller_start_stopped",
                default_value="false",
                choices=["true", "false"],
                description="Load and configure arm_controller without activating it",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers],
                remappings=[("~/robot_description", "robot_description")],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "controller_manager",
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "controller_manager",
                    "--controller-manager-timeout",
                    "30",
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            arm_controller_allowed,
                            " and '",
                            arm_controller_start_stopped,
                            "' == 'false'",
                        ]
                    )
                ),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "controller_manager",
                    "--controller-manager-timeout",
                    "30",
                    "--stopped",
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            arm_controller_allowed,
                            " and '",
                            arm_controller_start_stopped,
                            "' == 'true'",
                        ]
                    )
                ),
                output="screen",
            ),
        ]
    )
