import numpy as np
import rclpy

from lbr_fri_msgs.msg import LBRPositionCommand, LBRState

from .admittance_rcm_controller import AdmittanceRCMController
from .lbr_base_position_command_node import LBRBasePositionCommandNode


class LBRAdmittanceControlRCMNode(LBRBasePositionCommandNode):
    def __init__(self, node_name: str = "admittance_rcm_control_node") -> None:
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("f_ext_th", [4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
        self.declare_parameter("dq_gain", [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        self.declare_parameter("dx_gain", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        self._f_ext_th = np.array(
            self.get_parameter("f_ext_th").get_parameter_value().double_array_value
        )
        self._dq_gain = np.diag(
            self.get_parameter("dq_gain").get_parameter_value().double_array_value
        )
        self._dx_gain = np.diag(
            self.get_parameter("dx_gain").get_parameter_value().double_array_value
        )

        self._alpha = 0.95

        self._dq = np.zeros(7)
        self._controller = AdmittanceRCMController(self._robot_description)

    def _command(self, q) -> None:
        lbr_command = LBRPositionCommand()
        lbr_command.joint_position = q
        self._lbr_position_command_pub.publish(lbr_command)

    def _admittance(self, tau_ext, qc) -> None:
        J = self._controller.jacobian(qc)
        Jinv = np.linalg.pinv(J, rcond=0.1)
        f_ext = Jinv.T @ tau_ext
        dx = np.where(
            abs(f_ext) > self._f_ext_th,
            self._dx_gain @ np.sign(f_ext) * (abs(f_ext) - self._f_ext_th),
            0.0,
        ).flatten()
        dx = np.clip(dx, -1.0, 1.0)
        dq = Jinv @ dx
        self._dq = self._alpha * self._dq + (1.0 - self._alpha) * dq
        return self._dq

    def _on_lbr_state(self, msg: LBRState) -> None:
        if self._controller._rcm is None:
            self._controller.set_start(msg.measured_joint_position)
            return

        dq_goal = self._admittance(msg.external_torque, msg.measured_joint_position)
        self._controller.reset(msg.measured_joint_position, dq_goal)

        if self._controller.solve():
            qc = np.array(msg.measured_joint_position)
            dq = self._controller.get_qd_target()
            qn = qc + self._dt * self._dq_gain @ dq
            self._command(qn.tolist())
        else:
            self.get_logger().error("Solver failed!")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LBRAdmittanceControlRCMNode()
    rclpy.spin(node)
    rclpy.shutdown()
