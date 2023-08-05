import numpy as np
import optas

from lbr_fri_msgs.msg import LBRCommand, LBRState


class AdmittanceController(object):
    def __init__(
        self,
        robot_description: str,
        base_link: str = "lbr_link_0",
        end_effector_link: str = "lbr_link_ee",
        f_ext_th: np.ndarray = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]),
        dq_gain: np.ndarray = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]),
        dx_gain: np.ndarray = np.array([1.0, 1.0, 1.0, 20.0, 40.0, 60.0]),
    ) -> None:
        self.lbr_command_ = LBRCommand()

        robot = optas.RobotModel(urdf_string=robot_description)
        J = robot.get_geometric_jacobian_function(end_effector_link, base_link)
        self.jacobian_ = lambda q: J(q, numpy_output=True)

        self.dof_ = robot.ndof
        self.jacobian_inv_ = np.zeros((self.dof_, 6))
        self.q = np.zeros(self.dof_)
        self.dq_ = np.zeros(self.dof_)
        self.tau_ext_ = np.zeros(6)
        self.dq_gain_ = np.diag(dq_gain)
        self.dx_gain_ = np.diag(dx_gain)
        self.f_ext_ = np.zeros(6)
        self.f_ext_th_ = f_ext_th
        self.alpha_ = 0.99

    def __call__(self, lbr_state: LBRState) -> LBRCommand:
        self.q_ = np.array(lbr_state.measured_joint_position.tolist())
        self.tau_ext_ = np.array(lbr_state.external_torque.tolist())

        self.jacobian_ = self.jacobian_(self.q_)

        self.jacobian_inv_ = np.linalg.pinv(self.jacobian_, rcond=0.05)
        self.f_ext_ = self.jacobian_inv_.T @ self.tau_ext_

        self.f_ext_ = np.where(
            abs(self.f_ext_) > self.f_ext_th_,
            self.dx_gain_ @ np.sign(self.f_ext_) * (abs(self.f_ext_) - self.f_ext_th_),
            0.0,
        )

        # additional smoothing required in python
        self.dq_ = (
            self.alpha_ * self.dq_
            + (1 - self.alpha_) * self.dq_gain_ @ self.jacobian_inv_ @ self.f_ext_
        )

        self.lbr_command_.joint_position = (
            np.array(lbr_state.measured_joint_position.tolist())
            + lbr_state.sample_time * self.dq_
        ).data

        return self.lbr_command_
