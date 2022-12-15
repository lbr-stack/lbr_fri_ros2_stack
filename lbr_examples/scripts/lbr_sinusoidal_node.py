#!/usr/bin/python3
import math
from copy import deepcopy
from typing import Union

import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from lbr_fri_msgs.msg import LBRCommand, LBRState


class LBRSinusoidalNode(Node):
    _sim: bool
    _amplitude: float
    _period: float
    _t0: float
    _initial_state: Union[JointState, LBRState]

    def __init__(self, node_name: str = "lbr_sinusoidal_node"):
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("sim", False)
        self.declare_parameter("amplitude", math.pi / 4.0)
        self.declare_parameter("period", 10.0)

        self._sim = bool(self.get_parameter("sim").value)
        self._amplitude = float(self.get_parameter("amplitude").value)
        self._period = float(self.get_parameter("period").value)
        self._t0 = None
        self._initial_state = None

        # create publishers and subscribers
        if self._sim:
            # careful when using joint states and sending commands, as they
            # are not sorted! https://github.com/ros-controls/ros2_controllers/issues/159
            # using the /joint_states might cause unintended behavior on the robot
            # use the /lbr_state instead
            self._joint_states_subscriber = self.create_subscription(
                JointState,
                "/joint_states",
                self._joint_states_callback,
                qos.qos_profile_system_default,
            )
            self._position_command_publisher = self.create_publisher(
                Float64MultiArray,
                "/forward_position_controller/commands",
                qos.qos_profile_system_default,
            )
        else:
            self._lbr_state_subscriber = self.create_subscription(
                LBRState,
                "/lbr_state",
                self._lbr_state_callback,
                qos.qos_profile_system_default,
            )
            self._lbr_command_publisher = self.create_publisher(
                LBRCommand, "/lbr_command", qos.qos_profile_system_default
            )

    def _joint_states_callback(self, msg: JointState) -> None:
        # get the initial joint configuration and time
        if not self._initial_state:
            self._t0 = float(self.get_clock().now().nanoseconds)
            self._initial_state = msg

        command = Float64MultiArray()
        command.data = deepcopy(self._initial_state.position)

        omega = 2 * math.pi / self._period
        t = (float(self.get_clock().now().nanoseconds) - self._t0) / 1.0e9
        command.data[6] += self._amplitude * math.sin(omega * t)
        self._position_command_publisher.publish(command)

    def _lbr_state_callback(self, msg: LBRState) -> None:
        # get the initial joint configuration and time
        if not self._initial_state:
            self._t0 = float(self.get_clock().now().nanoseconds)
            self._initial_state = msg

        command = LBRCommand()
        command.joint_position = deepcopy(self._initial_state.measured_joint_position)

        omega = 2 * math.pi / self._period
        t = (float(self.get_clock().now().nanoseconds) - self._t0) / 1.0e9
        command.joint_position[6] += self._amplitude * math.sin(omega * t)

        self._lbr_command_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)

    lbr_sinusoidal_node = LBRSinusoidalNode()
    rclpy.spin(lbr_sinusoidal_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
