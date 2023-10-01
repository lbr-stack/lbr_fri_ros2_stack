import math

import rclpy
from rclpy.node import Node

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRWrenchCommand, LBRState


class WrenchSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.amplitude_x_, self.amplitude_y_ = 5.0, 5.0  # N
        self.frequency_x_, self.frequency_y_ = 0.25, 0.25  # Hz
        self.phase_x_, self.phase_y_ = 0.0, 0.0
        self.lbr_wrench_command_ = LBRWrenchCommand()

        # create publisher to /lbr/command/wrench
        self.lbr_wrench_command_pub_ = self.create_publisher(
            LBRWrenchCommand, "/lbr/command/wrench", 1
        )

        # create subscription to /lbr_state
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr/state", self.on_lbr_state_, 1
        )

    def on_lbr_state_(self, lbr_state: LBRState) -> None:
        self.lbr_wrench_command_.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay wrench sine wave on x / y direction
            self.lbr_wrench_command_.wrench[0] = self.amplitude_x_ * math.sin(
                self.phase_x_
            )
            self.lbr_wrench_command_.wrench[1] = self.amplitude_y_ * math.sin(
                self.phase_y_
            )
            self.phase_x_ += 2 * math.pi * self.frequency_x_ * lbr_state.sample_time
            self.phase_y_ += 2 * math.pi * self.frequency_y_ * lbr_state.sample_time

            self.lbr_wrench_command_pub_.publish(self.lbr_wrench_command_)
        else:
            # reset phase
            self.phase_x_, self.phase_y_ = 0.0, 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(WrenchSineOverlayNode("wrench_sine_overlay_node"))
    rclpy.shutdown()
