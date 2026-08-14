import contextlib
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import pytest


PACKAGE_ROOT = Path(__file__).parents[1]
XACRO_FILE = PACKAGE_ROOT / "urdf" / "rebotarm.urdf.xacro"
ARM_JOINTS = [f"joint{index}" for index in range(1, 7)]
MOVING_JOINTS = ARM_JOINTS + ["gripper_joint1", "gripper_joint2"]
PACKAGE_URI = "package://rebotarm_description/"

# Keep the runtime tests isolated from ROS nodes that may be active on the host.
os.environ["ROS_DOMAIN_ID"] = str(80 + os.getpid() % 100)


def expand_model(model):
    result = subprocess.run(
        ["xacro", str(XACRO_FILE), f"model:={model}"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def test_unified_xacro_defaults_to_dm():
    result = subprocess.run(
        ["xacro", str(XACRO_FILE)],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    assert root.attrib["name"] == "rebotarm"
    assert root.find("joint[@name='gripper_joint1']/limit").attrib["upper"] == "0.0715"


def assert_valid_tree(root):
    links = [link.attrib["name"] for link in root.findall("link")]
    joints = [joint.attrib["name"] for joint in root.findall("joint")]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))

    children = {}
    child_links = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        assert parent in links
        assert child in links
        assert child not in child_links
        child_links.add(child)
        children.setdefault(parent, []).append(child)

    roots = set(links) - child_links
    assert roots == {"base_footprint"}

    visited = set()
    stack = ["base_footprint"]
    while stack:
        link = stack.pop()
        assert link not in visited
        visited.add(link)
        stack.extend(children.get(link, []))
    assert visited == set(links)
    assert "gripper_tcp" in visited


@pytest.mark.parametrize("model", ["dm", "rs"])
def test_model_structure_and_resources(model):
    description = expand_model(model)
    root = ET.fromstring(description)
    assert root.attrib["name"] == "rebotarm"
    assert_valid_tree(root)

    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    for name in ARM_JOINTS:
        joint = joints[name]
        assert joint.attrib["type"] == "revolute"
        axis = [float(value) for value in joint.find("axis").attrib["xyz"].split()]
        assert math.isclose(sum(value * value for value in axis), 1.0, abs_tol=1e-6)
        limit = joint.find("limit").attrib
        assert float(limit["lower"]) < float(limit["upper"])
        assert float(limit["effort"]) > 0
        assert float(limit["velocity"]) > 0

    assert joints["gripper_joint1"].attrib["type"] == "prismatic"
    assert joints["gripper_joint2"].attrib["type"] == "prismatic"
    mimic = joints["gripper_joint2"].find("mimic").attrib
    assert mimic == {
        "joint": "gripper_joint1",
        "multiplier": "1.0",
        "offset": "0.0",
    }

    meshes = root.findall(".//mesh")
    assert meshes
    for mesh in meshes:
        filename = mesh.attrib["filename"]
        assert filename.startswith(PACKAGE_URI)
        assert (PACKAGE_ROOT / filename.removeprefix(PACKAGE_URI)).is_file()

    collision_meshes = root.findall(".//collision/geometry/mesh")
    collision_boxes = root.findall(".//collision/geometry/box")
    assert not collision_meshes
    assert len(collision_boxes) == 10
    for box in collision_boxes:
        assert all(float(value) > 0 for value in box.attrib["size"].split())

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf_file:
        urdf_file.write(description)
        urdf_file.flush()
        subprocess.run(
            ["check_urdf", urdf_file.name],
            check=True,
            capture_output=True,
            text=True,
        )


@contextlib.contextmanager
def running_display(model, joint_state_source):
    command = [
        "ros2",
        "launch",
        "rebotarm_description",
        "display.launch.py",
        f"model:={model}",
        f"joint_state_source:={joint_state_source}",
        "use_rviz:=false",
    ]
    process = subprocess.Popen(
        command,
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        time.sleep(0.25)
        if process.poll() is not None:
            output = process.communicate(timeout=1)[0]
            pytest.fail(f"display launch exited early:\n{output}")
        yield process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate(timeout=2)


@pytest.fixture
def ros_node():
    import rclpy

    rclpy.init()
    node = rclpy.create_node(f"description_test_{time.time_ns()}")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def spin_until(node, predicate, timeout, publish=None):
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if publish is not None:
            publish()
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def transient_local_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


@pytest.mark.parametrize("model", ["dm", "rs"])
def test_headless_launch_publishes_complete_state(model, ros_node):
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    received = {
        "description": None,
        "joint_state": None,
        "tf": set(),
        "tf_static": set(),
    }
    ros_node.create_subscription(
        String,
        "/robot_description",
        lambda message: received.update(description=message.data),
        transient_local_qos(),
    )
    ros_node.create_subscription(
        JointState,
        "/joint_states",
        lambda message: received.update(joint_state=set(message.name)),
        10,
    )
    ros_node.create_subscription(
        TFMessage,
        "/tf",
        lambda message: received["tf"].update(
            transform.child_frame_id for transform in message.transforms
        ),
        100,
    )
    ros_node.create_subscription(
        TFMessage,
        "/tf_static",
        lambda message: received["tf_static"].update(
            transform.child_frame_id for transform in message.transforms
        ),
        transient_local_qos(),
    )

    def complete():
        return (
            received["description"] is not None
            and received["joint_state"] is not None
            and set(MOVING_JOINTS).issubset(received["joint_state"])
            and "link1" in received["tf"]
            and "gripper_tcp" in received["tf_static"]
        )

    with running_display(model, "headless"):
        assert spin_until(ros_node, complete, timeout=10)
    assert '<robot name="rebotarm">' in received["description"]


def transform_signature(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )


@pytest.mark.parametrize("model", ["dm", "rs"])
def test_external_joint_states_move_arm_and_gripper(model, ros_node):
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    transforms = {}
    ros_node.create_subscription(
        TFMessage,
        "/tf",
        lambda message: transforms.update(
            {
                transform.child_frame_id: transform
                for transform in message.transforms
            }
        ),
        100,
    )
    publisher = ros_node.create_publisher(JointState, "/joint_states", 10)
    if model == "dm":
        first_arm_pose = [0.0, -1.2, -1.2, 0.0, 0.0, 0.0]
        second_arm_pose = [0.4, -0.6, -0.7, 0.3, 0.3, 0.4]
    else:
        first_arm_pose = [0.0, 0.8, 0.8, 0.0, 0.0, 0.0]
        second_arm_pose = [0.4, 1.2, 1.3, 0.3, 0.3, 0.4]

    def publish_pose(arm_pose, gripper):
        message = JointState()
        message.header.stamp = ros_node.get_clock().now().to_msg()
        message.name = MOVING_JOINTS
        message.position = arm_pose + [gripper, gripper]
        publisher.publish(message)

    arm_frames = {f"link{index}" for index in range(1, 7)}
    expected_frames = arm_frames | {"gripper_left", "gripper_right"}
    with running_display(model, "none"):
        assert spin_until(
            ros_node,
            lambda: expected_frames.issubset(transforms),
            timeout=10,
            publish=lambda: publish_pose(first_arm_pose, 0.01),
        )
        first_pose = {
            frame: transform_signature(transforms[frame])
            for frame in expected_frames
        }

        transforms.clear()
        assert spin_until(
            ros_node,
            lambda: expected_frames.issubset(transforms),
            timeout=5,
            publish=lambda: publish_pose(second_arm_pose, 0.03),
        )
        second_pose = {
            frame: transform_signature(transforms[frame])
            for frame in expected_frames
        }

    for frame in arm_frames:
        assert first_pose[frame] != second_pose[frame]
    assert first_pose["gripper_left"] != second_pose["gripper_left"]
    assert first_pose["gripper_right"] != second_pose["gripper_right"]
