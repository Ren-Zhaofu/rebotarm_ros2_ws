import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    model = context.launch_configurations["model"]
    world = context.launch_configurations["world"]
    control_mode = context.launch_configurations["control_mode"]
    gui = context.launch_configurations["gui"]
    start_gripper_controller = context.launch_configurations["start_gripper_controller"] == "true"

    if model not in {"rs", "dm"}:
        raise RuntimeError("model must be 'rs' or 'dm'")
    if world not in {"empty", "workcell"}:
        raise RuntimeError("world must be 'empty' or 'workcell'")
    if control_mode not in {"position", "effort"}:
        raise RuntimeError("control_mode must be 'position' or 'effort'")

    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    simulation = Path(get_package_share_directory("rebotarm_mujoco"))
    headless = "false" if gui == "true" else "true"

    xacro_file = bringup / "urdf" / "rebotarm_controlled.urdf.xacro"
    robot_description_xml = subprocess.check_output(
        ["xacro", str(xacro_file), f"model:={model}", "transport:=mujoco", f"mujoco_headless:={headless}", f"control_mode:={control_mode}"],
        text=True,
    )
    robot_description = {"robot_description": ParameterValue(robot_description_xml, value_type=str)}

    controllers = str(simulation / "config" / f"controllers_{control_mode}.yaml")
    converter_args = [
        "--robot_description", robot_description_xml,
        "--m", str(simulation / "mjcf" / f"{control_mode}_actuators.xml"),
        "--scene", str(simulation / "mjcf" / "worlds" / f"{world}.xml"),
        "--publish_topic", "/mujoco_robot_description",
    ]

    nodes = [
        Node(
            package="mujoco_ros2_control",
            executable="robot_description_to_mjcf.sh",
            arguments=converter_args,
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            parameters=[controllers, {"use_sim_time": True}],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
            emulate_tty=True,
            on_exit=Shutdown(),
        ),
    ]
    if control_mode == "effort":
        print("WARNING: effort mode uses provisional, uncalibrated dynamics and gains; do not infer real-hardware safety limits.")
    controllers_to_start = ["joint_state_broadcaster", "arm_controller"]
    if start_gripper_controller:
        controllers_to_start.append("gripper_controller")
    for controller in controllers_to_start:
        nodes.append(TimerAction(
            period=5.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
                output="screen",
            )],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rs", choices=["rs", "dm"]),
        DeclareLaunchArgument("world", default_value="empty", choices=["empty", "workcell"]),
        DeclareLaunchArgument("control_mode", default_value="position", choices=["position", "effort"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_gripper_controller", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("random_seed", default_value="0", description="Reserved deterministic environment seed"),
        OpaqueFunction(function=_launch_setup),
    ])
