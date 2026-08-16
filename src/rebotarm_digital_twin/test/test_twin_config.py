from pathlib import Path

from rebotarm_digital_twin.twin_utils import (
    JOINT_NAMES,
    clamp_vector,
    joint_limits,
    rate_limit_vector,
    within_limits,
)
from rebotarm_digital_twin.target_to_trajectory import positions_changed
from rebotarm_digital_twin.target_limiter import advance_motion


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


def test_rate_limiter_bounds_every_step():
    assert rate_limit_vector((0,) * 6, (1,) * 6, 0.02) == (0.02,) * 6


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
