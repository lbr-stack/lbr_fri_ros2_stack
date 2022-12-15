#!/usr/bin/python3
import math
from copy import deepcopy

import rclpy
from rclpy import qos
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRCommand, LBRState


class LBRSinusoidalNode(Node):
    amplitude_: float
    omega_: float
    t0_: float
    initial_lbr_state_: LBRState

    def __init__(self, node_name: str = "lbr_sinusoidal_node"):
        super().__init__(node_name)

        self.declare_parameter("amplitude", math.pi / 4.0)
        self.declare_parameter("period", 20.0)

        self.amplitude_ = float(self.get_parameter("amplitude").value)
        self.omega_ = 2*math.pi / float(self.get_parameter("period").value)
        self.t0_ = None
        self.initial_lbr_state_ = None

        self.lbr_state_sub_ = self.create_subscription(
            LBRState,
            "/lbr_state",
            self.lbr_state_sub_cb_,
            qos.qos_profile_system_default,
        )
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand, "/lbr_command", qos.qos_profile_system_default
        )
        self.lbr_command_timer_ = self.create_timer(0.01, self.timer_cb_)

    def lbr_state_sub_cb_(self, msg: LBRState) -> None:
        if not self.initial_lbr_state_:
            self.t0_ = float(self.get_clock().now().nanoseconds)
            self.initial_lbr_state_ = msg

    def timer_cb_(self):
        if not self.initial_lbr_state_:
            return
        command = LBRCommand()
        command.joint_position = deepcopy(self.initial_lbr_state_.measured_joint_position)
        t = (float(self.get_clock().now().nanoseconds) - self.t0_) / 1.0e9
        command.joint_position[3] -= self.amplitude_ * math.sin(self.omega_ * t)
        command.joint_position[4] -= self.amplitude_ * math.sin(self.omega_ * t)
        command.joint_position[5] += self.amplitude_ * math.sin(self.omega_ * t)
        command.joint_position[6] += self.amplitude_ * math.sin(self.omega_ * t)
        self.lbr_command_pub_.publish(command)


def main(args=None):
    rclpy.init(args=args)

    lbr_sinusoidal_node = LBRSinusoidalNode()
    rclpy.spin(lbr_sinusoidal_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
