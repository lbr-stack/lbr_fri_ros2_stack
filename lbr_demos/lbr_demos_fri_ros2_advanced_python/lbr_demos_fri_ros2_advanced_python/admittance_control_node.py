import numpy as np
import rclpy
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRPositionCommand, LBRState

from .admittance_controller import AdmittanceController


class AdmittanceControlNode(Node):
    def __init__(self, node_name: str = "admittance_control_node") -> None:
        super().__init__(node_name=node_name)

        # parameters
        self.declare_parameter("robot_description", "")
        self.declare_parameter("base_link", "link_0")
        self.declare_parameter("end_effector_link", "link_ee")

        self._init = False
        self._lbr_state = LBRState()

        self._controller = AdmittanceController(
            robot_description=str(self.get_parameter("robot_description").value),
            base_link=str(self.get_parameter("base_link").value),
            end_effector_link=str(self.get_parameter("end_effector_link").value),
        )

        # publishers and subscribers
        self._lbr_state_sub = self.create_subscription(
            LBRState, "/lbr/state", self._on_lbr_state, 1
        )
        self._lbr_position_command_pub = self.create_publisher(
            LBRPositionCommand,
            "/lbr/command/joint_position",
            1,
        )

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._smooth_lbr_state(lbr_state, 0.95)

        lbr_command = self._controller(self._lbr_state)
        self._lbr_position_command_pub.publish(lbr_command)

    def _smooth_lbr_state(self, lbr_state: LBRState, alpha: float) -> None:
        if not self._init:
            self._lbr_state = lbr_state
            self._init = True
            return

        self._lbr_state.measured_joint_position = (
            (1 - alpha) * np.array(self._lbr_state.measured_joint_position.tolist())
            + alpha * np.array(lbr_state.measured_joint_position.tolist())
        ).data

        self._lbr_state.external_torque = (
            (1 - alpha) * np.array(self._lbr_state.external_torque.tolist())
            + alpha * np.array(lbr_state.external_torque.tolist())
        ).data


def main(args=None) -> None:
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()
