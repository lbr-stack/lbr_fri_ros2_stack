import math
from copy import deepcopy

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

# import lbr_fri_idl
from lbr_fri_idl.msg import LBRState, LBRTorqueCommand


class TorqueSineOverlayNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self._amplitude = 15.0  # Nm
        self._frequency = 0.25  # Hz
        self._phase = 0.0
        self._lbr_torque_command = LBRTorqueCommand()

        # create publisher to command/torque
        self._lbr_torque_command_pub = self.create_publisher(
            LBRTorqueCommand, "command/torque", 1
        )

        # create subscription to state
        self._lbr_state = None
        self._lbr_state_sub = self.create_subscription(
            LBRState, "state", self._on_lbr_state, 1
        )

        # get control rate from controller_manager
        self._dt = None
        self._retrieve_update_rate()

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        if self._dt is None:
            return
        if self._lbr_state is None:
            self._lbr_state = lbr_state
        self._lbr_torque_command.joint_position = deepcopy(
            self._lbr_state.measured_joint_position
        )

        if lbr_state.session_state == 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            # overlay torque sine wave on 4th joint
            self._lbr_torque_command.torque[3] = self._amplitude * math.sin(self._phase)
            self._phase += 2 * math.pi * self._frequency * self._dt

            self._lbr_torque_command_pub.publish(self._lbr_torque_command)
        else:
            # reset phase
            self._phase = 0.0

    def _retrieve_update_rate(self) -> float:
        paramter_client = self.create_client(
            GetParameters, "controller_manager/get_parameters"
        )
        paramter_name = "update_rate"
        self.get_logger().info(f"Waiting for {paramter_client.srv_name}...")
        while not paramter_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self.get_logger().info(f"Waiting for {paramter_client.srv_name}...")
        future = paramter_client.call_async(
            GetParameters.Request(names=[paramter_name])
        )
        self.get_logger().info(f"Calling service {paramter_client.srv_name}...")
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f"Failed to get parameter '{paramter_name}'.")
        update_rate = future.result().values[0].integer_value
        self.get_logger().info(f"{paramter_name}: {update_rate} Hz")
        self._dt = 1.0 / float(update_rate)


def main(args: list = None) -> None:
    rclpy.init(args=args)
    rclpy.spin(TorqueSineOverlayNode("torque_sine_overlay_node"))
    rclpy.shutdown()
