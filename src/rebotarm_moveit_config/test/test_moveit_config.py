from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).parents[1]


def test_srdf_has_arm_tcp_and_gripper():
    root = ET.parse(PACKAGE / "config" / "rebotarm.srdf").getroot()
    groups = {group.attrib["name"]: group for group in root.findall("group")}
    chain = groups["arm"].find("chain")
    assert chain.attrib == {"base_link": "base_footprint", "tip_link": "gripper_tcp"}
    assert "gripper" in groups


def test_moveit_controller_maps_existing_actions():
    config = yaml.safe_load((PACKAGE / "config" / "moveit_controllers.yaml").read_text())
    manager = config["moveit_simple_controller_manager"]
    assert manager["arm_controller"]["action_ns"] == "follow_joint_trajectory"
    assert manager["gripper_controller"]["type"] == "GripperCommand"


def test_all_arm_joints_have_limits():
    limits = yaml.safe_load((PACKAGE / "config" / "joint_limits.yaml").read_text())["joint_limits"]
    assert all(f"joint{i}" in limits for i in range(1, 7))
