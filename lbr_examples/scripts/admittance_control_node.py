#!/usr/bin/python3
import os
import numpy as np
from copy import deepcopy
import xacro
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
import kinpy

from lbr_fri_msgs.msg import LBRState, LBRCommand

class Controller(object):

    def __init__(self,
                 urdf_string: str,
                 end_link_name: str="lbr_link_ee",
                 root_link_name: str="lbr_link_0",
                 f_threshold: np.ndarray=np.array([6., 6., 6., 1.5, 1.5, 1.5]),
                 gain: np.ndarray=np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
                 dx: np.ndarray=np.array([2., 2., 2., 5., 5., 5.]),
                 smooth: float=0.02
    ) -> None:
        self.chain_ = kinpy.build_serial_chain_from_urdf(
            data=urdf_string,
            end_link_name=end_link_name,
            root_link_name=root_link_name
        )

        self.f_threshold_ = f_threshold
        self.gain_ = np.diag(gain)
        self.dx_ = dx
        self.smooth_ = smooth

        self.dof_ = len(self.chain_.get_joint_parameter_names())
        self.dq_ = np.zeros(self.dof_)

    def __call__(self, q: np.ndarray, tau_ext: np.ndarray) -> np.ndarray:
        jacobian = self.chain_.jacobian(q)
        # J^T fext = tau
        if (tau_ext.size != self.dof_ or q.size != self.dof_):
            raise BufferError(f"Expected joint position and torque with {self.dof_} dof, got {q.size()} amd {tau_ext.size()} dof.")

        jacobian_inv= np.linalg.pinv(jacobian, rcond=0.01)
        
        f_ext = np.matmul(jacobian_inv.transpose(), tau_ext)
        f_ext = np.where(abs(f_ext) > self.f_threshold_, np.sign(f_ext)*self.dx_, 0.)

        dq = np.matmul(jacobian_inv, np.matmul(self.gain_, f_ext))
        self.dq_ = (1.-self.smooth_)*self.dq_ + self.smooth_*dq
        return deepcopy(self.dq_), f_ext


class AdmittanceControlNode(Node):
    def __init__(self, node_name="admittance_control_node"):
        super().__init__(node_name=node_name)

        # parameters
        self.declare_parameter("model", "med7")
        self.declare_parameter("end_link_name", "lbr_link_ee")
        self.declare_parameter("root_link_name", "lbr_link_0")
        
        self.model_ = str(self.get_parameter("model").value)
        self.end_link_name_ = str(self.get_parameter("end_link_name").value)
        self.root_link_name_ = str(self.get_parameter("root_link_name").value)

        # controller
        path = os.path.join(get_package_share_directory("lbr_description"), "urdf", self.model_, f"{self.model_}.urdf.xacro")
        self.urdf_string_ = xacro.process(path)

        self.controller_ = Controller(
            urdf_string=self.urdf_string_,
            end_link_name=self.end_link_name_,
            root_link_name=self.root_link_name_
        )

        # publishers and subscribers
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr_state", self.lbr_state_cb_, 1 
        )
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand, "/lbr_command", 1
        )

        self.joint_position_buffer_len_ = 30
        self.joint_position_buffer_ = []

        self.external_torque_buffer_len_ = 30
        self.external_torque_buffer_ = []

    def lbr_state_cb_(self, msg: LBRState) -> None:
        # compute control
        q = deepcopy(np.array(msg.measured_joint_position.tolist()))

        if len(self.joint_position_buffer_) > self.joint_position_buffer_len_:
            self.joint_position_buffer_.pop(0)
        self.joint_position_buffer_.append(q)

        q = np.zeros_like(q)
        for qi in self.joint_position_buffer_:
            q += qi/len(self.joint_position_buffer_)

        tau_ext = deepcopy(np.array(msg.external_torque.tolist()))

        if len(self.external_torque_buffer_) > self.external_torque_buffer_len_:
            self.external_torque_buffer_.pop(0)
        self.external_torque_buffer_.append(tau_ext)

        tau_ext = np.zeros_like(tau_ext)
        for tau_ext_i in self.external_torque_buffer_:
            tau_ext += tau_ext_i/len(self.external_torque_buffer_)

        dq, f_ext = self.controller_(q, tau_ext)

        # command
        command = LBRCommand()
        command.client_command_mode = 1
        command.joint_position = (q + dq).data
        # command.torque = dq.data
        self.lbr_command_pub_.publish(command)
    

def main(args=None):
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
