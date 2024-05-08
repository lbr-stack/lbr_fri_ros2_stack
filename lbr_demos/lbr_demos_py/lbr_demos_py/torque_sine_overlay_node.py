import math

import rclpy
from rclpy.node import Node

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRState, LBRTorqueCommand


class TorqueSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self._amplitude = 15.0  # Nm
        self._frequency = 0.25  # Hz
        self._phase = 0.0
        self._lbr_torque_command = LBRTorqueCommand()

        # create publisher to /lbr/command/torque
        self._lbr_torque_command_pub = self.create_publisher(
            LBRTorqueCommand, "/lbr/command/torque", 1
        )

        # create subscription to /lbr_state
        self._lbr_state_sub = self.create_subscription(
            LBRState, "/lbr/state", self._on_lbr_state, 1
        )

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._lbr_torque_command.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay torque sine wave on 4th joint
            self._lbr_torque_command.torque[3] = self._amplitude * math.sin(self._phase)
            self._phase += 2 * math.pi * self._frequency * lbr_state.sample_time

            self._lbr_torque_command_pub.publish(self._lbr_torque_command)
        else:
            # reset phase
            self._phase = 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(TorqueSineOverlayNode("torque_sine_overlay_node"))
    rclpy.shutdown()
