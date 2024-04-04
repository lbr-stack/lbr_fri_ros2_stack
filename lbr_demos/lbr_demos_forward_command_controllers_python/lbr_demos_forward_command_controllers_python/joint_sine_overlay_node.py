import math

import rclpy
from rclpy.node import Node

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRPositionCommand, LBRState


class JointSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.amplitude_ = 0.04  # rad
        self.frequency_ = 0.25  # Hz
        self.phase_ = 0.0
        self.lbr_position_command_ = LBRPositionCommand()

        # create publisher to /lbr/command/joint_position
        self.lbr_position_command_pub_ = self.create_publisher(
            LBRPositionCommand,
            "/lbr/command/joint_position",
            1,
        )

        # create subscription to /lbr_state
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr/state", self.on_lbr_state_, 1
        )

    def on_lbr_state_(self, lbr_state: LBRState) -> None:
        self.lbr_position_command_.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay sine wave on 4th joint
            self.lbr_position_command_.joint_position[3] += self.amplitude_ * math.sin(
                self.phase_
            )
            self.phase_ += 2 * math.pi * self.frequency_ * lbr_state.sample_time

            self.lbr_position_command_pub_.publish(self.lbr_position_command_)
        else:
            # reset phase
            self.phase_ = 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(JointSineOverlayNode("joint_sine_overlay_node"))
    rclpy.shutdown()
