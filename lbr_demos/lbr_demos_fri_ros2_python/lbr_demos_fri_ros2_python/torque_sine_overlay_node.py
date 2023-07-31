import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# import lbr_fri_msgs
from lbr_fri_msgs.msg import LBRCommand, LBRState


class TorqueSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.amplitude_ = 15.0  # Nm
        self.frequency_ = 0.25  # Hz
        self.phase_ = 0.0
        self.lbr_command_ = LBRCommand()

        # create publisher to /lbr/command
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand,
            "/lbr/command",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )

        # create subscription to /lbr_state
        self.lbr_state_sub_ = self.create_subscription(
            LBRState,
            "/lbr/state",
            self.on_lbr_state_,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )

    def on_lbr_state_(self, lbr_state: LBRState) -> None:
        self.lbr_command_.joint_position = lbr_state.ipo_joint_position

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay torque sine wave on 4th joint
            self.lbr_command_.torque[3] = self.amplitude_ * math.sin(self.phase_)
            self.phase_ += 2 * math.pi * self.frequency_ * lbr_state.sample_time

            self.lbr_command_pub_.publish(self.lbr_command_)
        else:
            # reset phase
            self.phase_ = 0.0


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(TorqueSineOverlayNode("torque_sine_overlay_node"))
    rclpy.shutdown()
