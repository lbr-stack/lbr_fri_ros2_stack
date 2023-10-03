import kinpy
import numpy as np

from lbr_fri_msgs.msg import LBRPositionCommand, LBRState


class AdmittanceController(object):
    def __init__(
        self,
        robot_description: str,
        base_link: str = "link_0",
        end_effector_link: str = "link_ee",
        f_ext_th: np.ndarray = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]),
        dq_gain: np.ndarray = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        dx_gain: np.ndarray = np.array([1.0, 1.0, 1.0, 20.0, 40.0, 60.0]),
    ) -> None:
        self.lbr_position_command_ = LBRPositionCommand()

        self.chain_ = kinpy.build_serial_chain_from_urdf(
            data=robot_description,
            root_link_name=base_link,
            end_link_name=end_effector_link,
        )

        self.dof_ = len(self.chain_.get_joint_parameter_names())
        self.jacobian_ = np.zeros((6, self.dof_))
        self.jacobian_inv_ = np.zeros((self.dof_, 6))
        self.q = np.zeros(self.dof_)
        self.dq_ = np.zeros(self.dof_)
        self.tau_ext_ = np.zeros(6)
        self.dq_gain_ = np.diag(dq_gain)
        self.dx_gain_ = np.diag(dx_gain)
        self.f_ext_ = np.zeros(6)
        self.f_ext_th_ = f_ext_th
        self.alpha_ = 0.99

    def __call__(self, lbr_state: LBRState) -> LBRPositionCommand:
        self.q_ = np.array(lbr_state.measured_joint_position.tolist())
        self.tau_ext_ = np.array(lbr_state.external_torque.tolist())

        self.jacobian_ = self.chain_.jacobian(self.q_)

        self.jacobian_inv_ = np.linalg.pinv(self.jacobian_, rcond=0.1)
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

        self.lbr_position_command_.joint_position = (
            np.array(lbr_state.measured_joint_position.tolist())
            + lbr_state.sample_time * self.dq_
        ).data

        return self.lbr_position_command_
