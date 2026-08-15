from pathlib import Path

from rebotarm_digital_twin.twin_utils import (
    JOINT_NAMES,
    LOWER_LIMITS,
    UPPER_LIMITS,
    clamp_vector,
    rate_limit_vector,
    within_limits,
)


PACKAGE = Path(__file__).parents[1]


def test_joint_contract_and_limits():
    assert JOINT_NAMES == ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    assert clamp_vector([99] * 6) == UPPER_LIMITS
    assert clamp_vector([-99] * 6) == LOWER_LIMITS
    assert within_limits([0.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    assert not within_limits([0.0, 0.1, -1.0, 0.0, 0.0, 0.0])


def test_rate_limiter_bounds_every_step():
    assert rate_limit_vector((0,) * 6, (1,) * 6, 0.02) == (0.02,) * 6


def test_launch_files_exist():
    assert (PACKAGE / "launch" / "feedback_twin.launch.py").exists()
    assert (PACKAGE / "launch" / "host_sim.launch.py").exists()
    assert (
        PACKAGE / "rebotarm_digital_twin" / "joint_control_gui.py"
    ).exists()
