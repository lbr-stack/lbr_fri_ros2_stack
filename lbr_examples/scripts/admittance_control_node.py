#!/usr/bin/python3
import os
from copy import deepcopy

import kinpy
import numpy as np
import rclpy
import xacro
from ament_index_python import get_package_share_directory
from rclpy import qos
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRCommand, LBRState


class Controller(object):
    def __init__(
        self,
        urdf_string: str,
        end_link_name: str = "lbr_link_ee",
        root_link_name: str = "lbr_link_0",
        f_threshold: np.ndarray = np.array([6.0, 6.0, 6.0, 1.0, 1.0, 1.0]),
        dq_gain: np.ndarray = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        dx_gain: np.ndarray = np.array([1.0, 1.0, 1.0, 20.0, 40.0, 60.0]),
        smooth: float = 0.02,
    ) -> None:
        self.chain_ = kinpy.build_serial_chain_from_urdf(
            data=urdf_string, end_link_name=end_link_name, root_link_name=root_link_name
        )

        self.f_threshold_ = f_threshold
        self.dq_gain_ = np.diag(dq_gain)
        self.dx_gain_ = np.diag(dx_gain)
        self.smooth_ = smooth

        self.dof_ = len(self.chain_.get_joint_parameter_names())
        self.dq_ = np.zeros(self.dof_)

    def __call__(self, q: np.ndarray, tau_ext: np.ndarray) -> np.ndarray:
        jacobian = self.chain_.jacobian(q)
        # J^T fext = tau
        if tau_ext.size != self.dof_ or q.size != self.dof_:
            raise BufferError(
                f"Expected joint position and torque with {self.dof_} dof, got {q.size()} amd {tau_ext.size()} dof."
            )

        jacobian_inv = np.linalg.pinv(jacobian, rcond=0.05)

        f_ext = jacobian_inv.T @ tau_ext
        f_ext = np.where(
            abs(f_ext) > self.f_threshold_,
            self.dx_gain_ @ np.sign(f_ext) * (abs(f_ext) - self.f_threshold_),
            0.0,
        )

        dq = jacobian_inv @ self.dq_gain_ @ f_ext
        self.dq_ = (1.0 - self.smooth_) * self.dq_ + self.smooth_ * dq
        return deepcopy(self.dq_), f_ext


class AdmittanceControlNode(Node):
    def __init__(self, node_name="admittance_control_node") -> None:
        super().__init__(node_name=node_name)

        # parameters
        self.declare_parameter("model", "med7")
        self.declare_parameter("end_link_name", "lbr_link_ee")
        self.declare_parameter("root_link_name", "lbr_link_0")
        self.declare_parameter("command_rate", 100.0)
        self.declare_parameter("buffer_len", 20)

        self.model_ = str(self.get_parameter("model").value)
        self.end_link_name_ = str(self.get_parameter("end_link_name").value)
        self.root_link_name_ = str(self.get_parameter("root_link_name").value)
        self.dt_ = 1.0 / float(self.get_parameter("command_rate").value)

        # controller
        path = os.path.join(
            get_package_share_directory("lbr_description"),
            "urdf",
            self.model_,
            f"{self.model_}.urdf.xacro",
        )
        self.urdf_string_ = xacro.process(path)

        self.controller_ = Controller(
            urdf_string=self.urdf_string_,
            end_link_name=self.end_link_name_,
            root_link_name=self.root_link_name_,
        )

        self.lbr_state_ = None

        # publishers and subscribers
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr_state", self.lbr_state_cb_, qos.qos_profile_system_default
        )
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand, "/lbr_command", qos.qos_profile_system_default
        )
        self.lbr_command_timer_ = self.create_timer(self.dt_, self.timer_cb_)

        self.joint_position_buffer_len_ = int(self.get_parameter("buffer_len").value)
        self.joint_position_buffer_ = []

        self.external_torque_buffer_len_ = int(self.get_parameter("buffer_len").value)
        self.external_torque_buffer_ = []

    def lbr_state_cb_(self, msg: LBRState) -> None:
        self.lbr_state_ = msg

    def timer_cb_(self) -> None:
        if not self.lbr_state_:
            return
        # compute control
        q = deepcopy(np.array(self.lbr_state_.measured_joint_position.tolist()))

        if len(self.joint_position_buffer_) > self.joint_position_buffer_len_:
            self.joint_position_buffer_.pop(0)
        self.joint_position_buffer_.append(q)

        q = np.zeros_like(q)
        for qi in self.joint_position_buffer_:
            q += qi / len(self.joint_position_buffer_)

        tau_ext = deepcopy(np.array(self.lbr_state_.external_torque.tolist()))

        if len(self.external_torque_buffer_) > self.external_torque_buffer_len_:
            self.external_torque_buffer_.pop(0)
        self.external_torque_buffer_.append(tau_ext)

        tau_ext = np.zeros_like(tau_ext)
        for tau_ext_i in self.external_torque_buffer_:
            tau_ext += tau_ext_i / len(self.external_torque_buffer_)

        dq, f_ext = self.controller_(q, tau_ext)

        # command
        command = LBRCommand()
        command.joint_position = (q + dq * self.dt_).data
        self.lbr_command_pub_.publish(command)


def main(args=None):
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
