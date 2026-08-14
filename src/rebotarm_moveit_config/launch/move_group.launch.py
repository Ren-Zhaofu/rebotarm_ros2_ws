from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    model = context.launch_configurations["model"]
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(
            file_path=str(bringup / "urdf" / "rebotarm_controlled.urdf.xacro"),
            mappings={"model": model, "transport": "mujoco", "control_mode": "position"},
        )
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    return [Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config.to_dict(), {
            "use_sim_time": True,
            "allow_trajectory_execution": True,
            "moveit_manage_controllers": False,
            "planning_scene_monitor.publish_planning_scene": True,
            "planning_scene_monitor.publish_geometry_updates": True,
            "planning_scene_monitor.publish_state_updates": True,
            "planning_scene_monitor.publish_transforms_updates": True,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rs", choices=["rs", "dm"]),
        OpaqueFunction(function=_launch_setup),
    ])
