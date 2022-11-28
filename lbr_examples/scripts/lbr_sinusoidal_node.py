#!/usr/bin/python3
import math
from copy import deepcopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from lbr_state_msgs.msg import LBRState


class LBRSinusoidalNode(Node):
    _sim: bool
    _command_rate: int
    _amplitude: float
    _period: float
    _t0: float
    _initial_joint_states: JointState

    def __init__(self, node_name: str="lbr_read_write_node"):
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("sim", False)
        self.declare_parameter("command_rate", 100)
        self.declare_parameter("amplitude", math.pi/4.)
        self.declare_parameter("period", 10.)

        self._sim = self.get_parameter("sim")
        self._command_rate = int(self.get_parameter("command_rate").value)
        self._amplitude = float(self.get_parameter("amplitude").value)
        self._period = float(self.get_parameter("period").value)
        self._t0 = None
        self._initial_joint_states = None

        # create publishers and subscribers
        self._joint_states_subscriber = self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 1
        )

        if not self._sim:
            self._lbr_state_subscriber = self.create_subscription(
                LBRState, "/lbr_state", self._lbr_state_callback, 1
            )

        self._position_command_timer = self.create_timer(
            1./self._command_rate, self._position_command_timer_callback
        )

        self._position_command_publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 1
        )

    def _joint_states_callback(self, msg: JointState) -> None:
        self.get_logger().info("Got position:\n{}".format(msg.position))
        # get the initial joint configuration and time
        if not self._initial_joint_states:
            self._t0 = float(self.get_clock().now().nanoseconds)
            self._initial_joint_states = msg

    def _lbr_state_callback(self, msg: LBRState) -> None:
        self.get_logger().info("Got external torques:\n{}".format(msg.external_torque))

    def _position_command_timer_callback(self):
        command = Float64MultiArray()
        if not self._initial_joint_states:
            return
        command.data = deepcopy(self._initial_joint_states.position)

        omega = 2*math.pi/self._period
        t = (float(self.get_clock().now().nanoseconds) - self._t0)/1.e9
        command.data[6] += self._amplitude*math.sin(omega*t)
        self._position_command_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)

    lbr_sinusoidal_node = LBRSinusoidalNode()
    rclpy.spin(lbr_sinusoidal_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
