import math

import rclpy
from rclpy.node import Node

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRWrenchCommand, LBRState


class WrenchSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self._amplitude_x, self._amplitude_y = 5.0, 5.0  # N
        self._frequency_x, self._frequency_y = 0.25, 0.25  # Hz
        self._phase_x, self._phase_y = 0.0, 0.0
        self._lbr_wrench_command = LBRWrenchCommand()

        # create publisher to /lbr/command/wrench
        self._lbr_wrench_command_pub = self.create_publisher(
            LBRWrenchCommand, "/lbr/command/wrench", 1
        )

        # create subscription to /lbr_state
        self._lbr_state_sub = self.create_subscription(
            LBRState, "/lbr/state", self._on_lbr_state, 1
        )

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._lbr_wrench_command.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay wrench sine wave on x / y direction
            self._lbr_wrench_command.wrench[0] = self._amplitude_x * math.sin(
                self._phase_x
            )
            self._lbr_wrench_command.wrench[1] = self._amplitude_y * math.sin(
                self._phase_y
            )
            self._phase_x += 2 * math.pi * self._frequency_x * lbr_state.sample_time
            self._phase_y += 2 * math.pi * self._frequency_y * lbr_state.sample_time

            self._lbr_wrench_command_pub.publish(self._lbr_wrench_command)
        else:
            # reset phase
            self._phase_x, self._phase_y = 0.0, 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(WrenchSineOverlayNode("wrench_sine_overlay_node"))
    rclpy.shutdown()
