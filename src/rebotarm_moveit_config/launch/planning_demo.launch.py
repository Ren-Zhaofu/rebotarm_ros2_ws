from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    model = context.launch_configurations["model"]
    execute = context.launch_configurations["execute"] == "true"
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path=str(bringup / "urdf" / "rebotarm_controlled.urdf.xacro"),
                           mappings={"model": model, "transport": "mujoco", "control_mode": "position"})
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    return [Node(
        package="rebotarm_moveit_config",
        executable="planning_demo",
        parameters=[config.to_dict(), {"use_sim_time": True, "execute": execute}],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rs", choices=["rs", "dm"]),
        DeclareLaunchArgument("execute", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_launch_setup),
    ])
