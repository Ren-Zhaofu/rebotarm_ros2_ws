#!/usr/bin/env python3
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarm_digital_twin.twin_utils import JOINT_NAMES


class TargetToTrajectory(Node):
    def __init__(self):
        super().__init__("target_to_trajectory")
        self.declare_parameter("input_topic", "/host/target_joint_states")
        self.declare_parameter("output_topic", "/arm_controller/joint_trajectory")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(JointTrajectory, output_topic, 10)
        self.create_subscription(JointState, input_topic, self.on_target, 10)

    def on_target(self, message):
        if tuple(message.name) != JOINT_NAMES or len(message.position) != 6:
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(message.position)
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        trajectory.points = [point]
        self.publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = TargetToTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
