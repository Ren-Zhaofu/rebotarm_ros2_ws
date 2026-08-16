#!/usr/bin/env python3
import json
import time

from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rebotarm_digital_twin.twin_utils import (
    JOINT_NAMES,
    joint_limits,
    rate_limit_vector,
    within_limits,
)


MODES = {
    "FEEDBACK_ONLY",
    "SIM_ONLY",
    "REAL_ARMED",
    "REAL_EXECUTING",
    "FAULT",
}


class ModeArbiter(Node):
    def __init__(self):
        super().__init__("mode_arbiter")
        self.declare_parameter("model", "dm")
        self.declare_parameter("initial_mode", "FEEDBACK_ONLY")
        self.declare_parameter("feedback_timeout_sec", 0.1)
        self.declare_parameter("max_command_step_rad", 0.02)
        self.declare_parameter(
            "controller_switch_service",
            "/real/controller_manager/switch_controller",
        )
        self.declare_parameter("controller_service_timeout_sec", 2.0)
        self.mode = str(self.get_parameter("initial_mode").value)
        self.model = str(self.get_parameter("model").value)
        joint_limits(self.model)
        if self.mode not in MODES:
            raise ValueError(f"initial_mode must be one of {sorted(MODES)}")
        self.timeout = float(
            self.get_parameter("feedback_timeout_sec").value
        )
        self.max_command_step = float(
            self.get_parameter("max_command_step_rad").value
        )
        if not 0.0 < self.max_command_step <= 0.05:
            raise ValueError("max_command_step_rad must be in (0, 0.05]")
        self.service_timeout = float(
            self.get_parameter("controller_service_timeout_sec").value
        )
        self.last_feedback = 0.0
        self.feedback_positions = None
        self.last_command = None
        self.switch_pending = False
        self.status_publisher = self.create_publisher(
            String, "/real/command_status", 10
        )
        self.command_publisher = self.create_publisher(
            JointState, "/real/command_target", 10
        )
        switch_service = str(
            self.get_parameter("controller_switch_service").value
        )
        self.switch_client = self.create_client(SwitchController, switch_service)
        self.create_subscription(
            JointState, "/real/joint_states", self.on_feedback, 10
        )
        self.create_subscription(
            JointState, "/host/target_joint_states", self.on_target, 10
        )
        self.create_service(SetBool, "/real/arm", self.set_arm)
        self.create_service(
            SetBool, "/real/set_execution", self.set_execution
        )
        self.timer = self.create_timer(0.02, self.on_timer)
        self.publish_status()

    def feedback_is_fresh(self):
        return (
            self.feedback_positions is not None
            and time.monotonic() - self.last_feedback <= self.timeout
        )

    def publish_status(self, reason=""):
        message = String()
        message.data = json.dumps(
            {
                "mode": self.mode,
                "enabled": self.mode == "REAL_EXECUTING",
                "reason": reason,
                "feedback_age_sec": (
                    time.monotonic() - self.last_feedback
                    if self.last_feedback
                    else None
                ),
                "timestamp": time.time(),
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)

    def publish_target(self, positions, header=None):
        output = JointState()
        if header is not None:
            output.header = header
        else:
            output.header.stamp = self.get_clock().now().to_msg()
        output.name = list(JOINT_NAMES)
        output.position = list(positions)
        self.command_publisher.publish(output)

    def on_feedback(self, message):
        if tuple(message.name) != JOINT_NAMES or not within_limits(
            message.position, self.model
        ):
            if self.mode == "REAL_EXECUTING":
                self.enter_fault("invalid_joint_feedback")
            return
        self.feedback_positions = tuple(
            float(value) for value in message.position
        )
        self.last_feedback = time.monotonic()

    def on_target(self, message):
        if self.mode != "REAL_EXECUTING":
            return
        if tuple(message.name) != JOINT_NAMES or not within_limits(
            message.position, self.model
        ):
            self.enter_fault("invalid_host_target")
            return
        if not self.feedback_is_fresh():
            self.enter_fault("feedback_timeout")
            return
        target = tuple(float(value) for value in message.position)
        if self.last_command is None:
            self.last_command = self.feedback_positions
        target = rate_limit_vector(
            self.last_command, target, self.max_command_step
        )
        self.last_command = target
        self.publish_target(target, message.header)

    def on_timer(self):
        if self.mode == "REAL_EXECUTING" and not self.feedback_is_fresh():
            self.enter_fault("feedback_timeout")

    def make_switch_request(self, activate):
        request = SwitchController.Request()
        if activate:
            request.activate_controllers = ["arm_controller"]
        else:
            request.deactivate_controllers = ["arm_controller"]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 1
        return request

    async def switch_controller(self, activate):
        if not self.switch_client.wait_for_service(
            timeout_sec=self.service_timeout
        ):
            return False, "controller_switch_service_unavailable"
        result = await self.switch_client.call_async(
            self.make_switch_request(activate)
        )
        if result is None or not result.ok:
            return False, "controller_switch_failed"
        return True, ""

    def request_controller_stop(self, reason):
        if self.switch_pending or not self.switch_client.service_is_ready():
            self.get_logger().error(
                "Controller switch service unavailable during fault stop"
            )
            return
        self.switch_pending = True
        future = self.switch_client.call_async(self.make_switch_request(False))

        def complete(switch_future):
            self.switch_pending = False
            try:
                result = switch_future.result()
                if result is None or not result.ok:
                    self.publish_status(reason + "_controller_stop_failed")
            except Exception as exception:  # pragma: no cover
                self.get_logger().error(
                    f"Controller stop request failed: {exception}"
                )

        future.add_done_callback(complete)

    def enter_fault(self, reason):
        if self.mode == "FAULT":
            return
        self.mode = "FAULT"
        self.publish_status(reason)
        self.request_controller_stop(reason)

    async def set_execution(self, request, response):
        if request.data:
            if self.mode != "REAL_ARMED":
                response.success = False
                response.message = "REAL_ARMED is required before REAL_EXECUTING"
                self.publish_status("execution_rejected_not_armed")
                return response
            if not self.feedback_is_fresh():
                response.success = False
                response.message = "recent real feedback is required"
                self.publish_status("enable_rejected_no_feedback")
                return response
            self.publish_target(self.feedback_positions)
            self.last_command = self.feedback_positions
            success, reason = await self.switch_controller(True)
            if not success:
                self.mode = "FAULT"
                response.success = False
                response.message = reason
                self.publish_status(reason)
                return response
            self.mode = "REAL_EXECUTING"
            response.success = True
            response.message = "real execution enabled"
        else:
            if self.mode != "REAL_EXECUTING":
                self.mode = "FEEDBACK_ONLY"
                response.success = True
                response.message = "real execution already disabled"
                self.publish_status()
                return response
            success, reason = await self.switch_controller(False)
            if not success:
                self.mode = "FAULT"
                response.success = False
                response.message = reason
                self.publish_status(reason)
                return response
            self.mode = "FEEDBACK_ONLY"
            response.success = True
            response.message = "real execution disabled and controller stopped"
        self.publish_status()
        return response

    async def set_arm(self, request, response):
        if request.data:
            if self.mode not in {"FEEDBACK_ONLY", "REAL_ARMED"}:
                response.success = False
                response.message = "FEEDBACK_ONLY is required before arming"
                self.publish_status("arm_rejected_invalid_mode")
                return response
            if not self.feedback_is_fresh():
                response.success = False
                response.message = "recent real feedback is required"
                self.publish_status("arm_rejected_no_feedback")
                return response
            self.mode = "REAL_ARMED"
            response.success = True
            response.message = "armed; execution remains disabled"
        else:
            if self.mode == "REAL_EXECUTING":
                success, reason = await self.switch_controller(False)
                if not success:
                    self.mode = "FAULT"
                    response.success = False
                    response.message = reason
                    self.publish_status(reason)
                    return response
            self.mode = "FEEDBACK_ONLY"
            response.success = True
            response.message = "real execution disarmed"
        self.publish_status()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModeArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
