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
        self._lbr_position_command = LBRPositionCommand()

        self._chain = kinpy.build_serial_chain_from_urdf(
            data=robot_description,
            root_link_name=base_link,
            end_link_name=end_effector_link,
        )

        self._dof = len(self._chain.get_joint_parameter_names())
        self._jacobian = np.zeros((6, self._dof))
        self._jacobian_inv = np.zeros((self._dof, 6))
        self._q = np.zeros(self._dof)
        self._dq = np.zeros(self._dof)
        self._tau_ext = np.zeros(6)
        self._dq_gain = np.diag(dq_gain)
        self._dx_gain = np.diag(dx_gain)
        self._f_ext = np.zeros(6)
        self._f_ext_th = f_ext_th
        self._alpha = 0.99

    def __call__(self, lbr_state: LBRState) -> LBRPositionCommand:
        self.q_ = np.array(lbr_state.measured_joint_position.tolist())
        self._tau_ext = np.array(lbr_state.external_torque.tolist())

        self._jacobian = self._chain.jacobian(self.q_)

        self._jacobian_inv = np.linalg.pinv(self._jacobian, rcond=0.1)
        self._f_ext = self._jacobian_inv.T @ self._tau_ext

        self._f_ext = np.where(
            abs(self._f_ext) > self._f_ext_th,
            self._dx_gain @ np.sign(self._f_ext) * (abs(self._f_ext) - self._f_ext_th),
            0.0,
        )

        # additional smoothing required in python
        self._dq = (
            self._alpha * self._dq
            + (1 - self._alpha) * self._dq_gain @ self._jacobian_inv @ self._f_ext
        )

        self._lbr_position_command.joint_position = (
            np.array(lbr_state.measured_joint_position.tolist())
            + lbr_state.sample_time * self._dq
        ).data

        return self._lbr_position_command
