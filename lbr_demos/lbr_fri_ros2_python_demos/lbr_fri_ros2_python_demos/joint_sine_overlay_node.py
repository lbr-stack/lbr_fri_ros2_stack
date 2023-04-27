import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRCommand, LBRState


class JointSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.amplitude_ = 0.04  # rad
        self.frequency_ = 0.25  # Hz
        self.phase_ = 0.0
        self.lbr_command_ = LBRCommand()

        # create publisher to /lbr_command
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand, "/lbr_command", qos_profile_sensor_data
        )

        # create subscription to /lbr_state
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr_state", self.lbr_state_cb_, qos_profile_sensor_data
        )

    def lbr_state_cb_(self, lbr_state: LBRState) -> None:
        self.lbr_command_.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay sine wave on 4th joint
            self.lbr_command_.joint_position[3] += self.amplitude_ * math.sin(
                self.phase_
            )
            self.phase_ += 2 * math.pi * self.frequency_ * lbr_state.sample_time

            self.lbr_command_pub_.publish(self.lbr_command_)
        else:
            # reset phase
            self.phase_ = 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(JointSineOverlayNode("joint_sine_overlay_node"))
    rclpy.shutdown()
