from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


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
            }.items(),
        ),
    ])
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="dm", choices=["dm", "rs"]),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument(
            "allow_motor_enable", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "motor_enable_mask", default_value="0,0,0,0,0,0"
        ),
        real_bringup,
        Node(package="rebotarm_digital_twin", executable="state_mirror.py", output="screen"),
        Node(package="rebotarm_digital_twin", executable="mode_arbiter.py", output="screen"),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            remappings=[("joint_states", "/host/raw_joint_states")],
            condition=IfCondition(allow_motor_enable),
            output="screen",
        ),
        Node(
            package="rebotarm_digital_twin",
            executable="target_limiter.py",
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
    ])
