#!/usr/bin/env python3
"""Publish a latched robot description without publishing TF."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_description_topic", "robot_description")
        description = str(self.get_parameter("robot_description").value)
        topic = str(self.get_parameter("robot_description_topic").value)
        if not description:
            raise RuntimeError("robot_description must not be empty")
        if not topic:
            raise RuntimeError("robot_description_topic must not be empty")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(String, topic, qos)
        self.publisher.publish(String(data=description))


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
