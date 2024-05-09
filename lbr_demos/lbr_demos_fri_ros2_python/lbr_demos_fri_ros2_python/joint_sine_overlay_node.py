import math

import rclpy
from rclpy.node import Node

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRPositionCommand, LBRState


class JointSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self._amplitude = 0.04  # rad
        self._frequency = 0.25  # Hz
        self._phase = 0.0
        self._lbr_position_command = LBRPositionCommand()

        # create publisher to /lbr/command/joint_position
        self._lbr_position_command_pub = self.create_publisher(
            LBRPositionCommand,
            "/lbr/command/joint_position",
            1,
        )

        # create subscription to /lbr_state
        self._lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr/state", self._on_lbr_state, 1
        )

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._lbr_position_command.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay sine wave on 4th joint
            self._lbr_position_command.joint_position[3] += self._amplitude * math.sin(
                self._phase
            )
            self._phase += 2 * math.pi * self._frequency * lbr_state.sample_time

            self._lbr_position_command_pub.publish(self._lbr_position_command)
        else:
            # reset phase
            self._phase = 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(JointSineOverlayNode("joint_sine_overlay_node"))
    rclpy.shutdown()
