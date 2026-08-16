from pathlib import Path

from rebotarm_digital_twin.twin_utils import (
    JOINT_NAMES,
    clamp_vector,
    joint_limits,
    ordered_joint_values,
    rate_limit_vector,
    within_limits,
)
from rebotarm_digital_twin.target_to_trajectory import positions_changed
from rebotarm_digital_twin.target_limiter import advance_motion
from rebotarm_digital_twin.state_mirror import (
    DISPLAY_JOINT_NAMES,
    with_fixed_gripper,
)
from rebotarm_digital_twin import can_preflight
from rebotarm_digital_twin.can_preflight import check_socketcan_interface


PACKAGE = Path(__file__).parents[1]


def test_joint_contract_and_limits():
    assert JOINT_NAMES == ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    dm_lower, dm_upper = joint_limits("dm")
    rs_lower, rs_upper = joint_limits("rs")
    assert clamp_vector([99] * 6, "dm") == dm_upper
    assert clamp_vector([-99] * 6, "dm") == dm_lower
    assert clamp_vector([99] * 6, "rs") == rs_upper
    assert clamp_vector([-99] * 6, "rs") == rs_lower
    assert within_limits([0.0, -1.0, -1.0, 0.0, 0.0, 0.0], "dm")
    assert not within_limits([0.0, 0.1, -1.0, 0.0, 0.0, 0.0], "dm")
    assert within_limits([0.0, 1.0, 1.0, 0.0, 0.0, 0.0], "rs")
    assert not within_limits([0.0, -0.1, 1.0, 0.0, 0.0, 0.0], "rs")
    assert not within_limits([0.0, -1.0, 0.001, 0.0, 0.0, 0.0], "dm")
    assert within_limits(
        [0.0, -1.0, 0.001, 0.0, 0.0, 0.0], "dm", tolerance=0.005
    )


def test_rate_limiter_bounds_every_step():
    assert rate_limit_vector((0,) * 6, (1,) * 6, 0.02) == (0.02,) * 6


def test_joint_values_follow_names_instead_of_transport_order():
    names = ("joint2", "joint3", "joint1", "joint4", "joint5", "joint6")
    values = (2.0, 3.0, 1.0, 4.0, 5.0, 6.0)
    assert ordered_joint_values(names, values) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def test_feedback_display_includes_a_fixed_closed_gripper():
    assert DISPLAY_JOINT_NAMES == JOINT_NAMES + ("gripper_joint1", "gripper_joint2")
    assert with_fixed_gripper((1.0,) * 6) == (1.0,) * 6 + (0.0, 0.0)


def test_rate_limiter_reaches_target_after_single_target_update():
    current = (0.0,) * 6
    target = (0.1, -0.1, -0.1, 0.1, -0.1, 0.1)
    for _ in range(5):
        current = rate_limit_vector(current, target, 0.02)
    assert current == target


def test_smooth_motion_respects_velocity_and_acceleration_limits():
    current = (0.0,) * 6
    velocity = (0.0,) * 6
    target = (0.1, -0.1, -0.1, 0.1, -0.1, 0.1)
    previous_velocity = velocity
    for _ in range(200):
        current, velocity = advance_motion(
            current, velocity, target, 0.02, 0.004, 0.6
        )
        assert all(abs(value) <= 0.2 + 1e-12 for value in velocity)
        assert all(
            abs(new - old) <= 0.012 + 1e-12
            for old, new in zip(previous_velocity, velocity)
        )
        previous_velocity = velocity
    assert current == target
    assert velocity == (0.0,) * 6


def test_trajectory_bridge_suppresses_unchanged_targets():
    target = (0.0, -1.0, -1.0, 0.0, 0.0, 0.0)
    assert positions_changed(None, target)
    assert not positions_changed(target, target)
    assert not positions_changed(target, (*target[:-1], 1e-10))
    assert positions_changed(target, (*target[:-1], 1e-3))


def test_launch_files_exist():
    assert (PACKAGE / "launch" / "feedback_twin.launch.py").exists()
    assert (PACKAGE / "launch" / "host_sim.launch.py").exists()
    assert (
        PACKAGE / "rebotarm_digital_twin" / "joint_control_gui.py"
    ).exists()


def test_ros_python_nodes_have_executable_entrypoints():
    for filename in (
        "target_limiter.py",
        "target_to_trajectory.py",
        "mode_arbiter.py",
        "state_mirror.py",
    ):
        source = (PACKAGE / "rebotarm_digital_twin" / filename).read_text()
        assert "if __name__ == \"__main__\":" in source


def test_rviz_robot_model_uses_latched_description_topic():
    rviz_config = (
        PACKAGE.parents[0] / "rebotarm_description" / "rviz" / "display.rviz"
    ).read_text()
    assert "Durability Policy: Transient Local" in rviz_config


def test_feedback_twin_has_one_tf_publisher_for_arm_and_gripper():
    launch_source = (PACKAGE / "launch" / "feedback_twin.launch.py").read_text()
    assert '"publish_robot_state": "false"' in launch_source
    assert '"robot_description_topic"' not in launch_source


def test_can_preflight_rejects_unavailable_interface(monkeypatch):
    import subprocess

    result = subprocess.CompletedProcess(
        args=[], returncode=1, stdout="", stderr="Cannot find device"
    )
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: result)
    ready, message = check_socketcan_interface("can9")
    assert not ready
    assert "does not exist" in message


def test_can_preflight_accepts_active_socketcan(monkeypatch):
    import subprocess

    result = subprocess.CompletedProcess(
        args=[],
        returncode=0,
        stdout="can0: <NOARP,UP,LOWER_UP>\nlink/can\ncan state ERROR-ACTIVE\n",
        stderr="",
    )
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: result)
    ready, message = check_socketcan_interface("can0")
    assert ready
    assert "ERROR-ACTIVE" in message


def test_motor_preflight_rejects_incomplete_feedback(monkeypatch):
    import subprocess

    monkeypatch.setattr(can_preflight, "get_package_prefix", lambda package: "/opt/test")
    result = subprocess.CompletedProcess(args=[], returncode=1, stdout="", stderr="")
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: result)
    ready, message = can_preflight.check_motor_feedback("can0", "dm")
    assert not ready
    assert "No complete DM motor feedback" in message


def test_motor_preflight_accepts_complete_feedback(monkeypatch):
    import subprocess

    monkeypatch.setattr(can_preflight, "get_package_prefix", lambda package: "/opt/test")
    result = subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: result)
    ready, message = can_preflight.check_motor_feedback("can0", "dm")
    assert ready
    assert "Complete DM motor feedback" in message
