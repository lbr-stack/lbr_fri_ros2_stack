import numpy as np
import rclpy
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRPositionCommand, LBRState

from .admittance_rcm_controller import AdmittanceRCMController


class LBRAdmittanceControlRCMNode(Node):
    def __init__(self, node_name: str = "admittance_rcm_control_node") -> None:
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("command_rate", 100)
        self.declare_parameter("robot_description")

        self._command_rate = int(self.get_parameter("command_rate").value)
        self._dt = 1.0 / float(self._command_rate)
        self._robot_description = str(self.get_parameter("robot_description").value)

        self._dq = np.zeros(7)
        self._controller = AdmittanceRCMController(self._robot_description)

        self._lbr_state_subscriber = self.create_subscription(
            LBRState, "/lbr/state", self._on_lbr_state, 1
        )

        self._lbr_command_publisher = self.create_publisher(
            LBRPositionCommand, "/lbr/command/joint_position", 1
        )

    def _command(self, q) -> None:
        lbr_command = LBRPositionCommand()
        lbr_command.joint_position = q
        self._lbr_command_publisher.publish(lbr_command)

    def _admittance(self, tau_ext, qc) -> None:
        J = self._controller.Jac(qc).toarray()

        Jinv = np.linalg.pinv(J, rcond=0.1)
        f_ext = Jinv.T @ tau_ext

        dx = np.zeros(6)

        translational_vel = 0.12
        th_f = 4.0
        th_tau = 1.0
        scale = 2.0

        for i in [0, 1, 2]:
            sign = np.sign(f_ext[i])
            if (abs(f_ext[i]) > th_f) and (abs(f_ext[i]) < scale * th_f):
                dx[i] = (
                    sign
                    * translational_vel
                    * (abs(f_ext[i]) - th_f)
                    / (scale * th_f - th_f)
                )
            elif abs(f_ext[i]) > scale * th_f:
                if f_ext[i] > 0.0:
                    dx[i] = translational_vel
                else:
                    dx[i] = -translational_vel

        for i in [3, 4, 5]:
            sign = np.sign(f_ext[i])
            if (abs(f_ext[i]) > th_tau) and (abs(f_ext[i]) < scale * th_tau):
                dx[i] = (
                    sign
                    * translational_vel
                    * (abs(f_ext[i]) - th_tau)
                    / (scale * th_tau - th_tau)
                )
            elif abs(f_ext[i]) > scale * th_tau:
                if f_ext[i] > 0.0:
                    dx[i] = translational_vel
                else:
                    dx[i] = -translational_vel

        dq = Jinv @ dx

        self._dq = (1 - 0.02) * self._dq + 0.02 * dq

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
            qn = qc + self._dt * dq * 10.0
            self._command(qn.tolist())
        else:
            self.get_logger().error("Solver failed!")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LBRAdmittanceControlRCMNode()
    rclpy.spin(node)
    rclpy.shutdown()
