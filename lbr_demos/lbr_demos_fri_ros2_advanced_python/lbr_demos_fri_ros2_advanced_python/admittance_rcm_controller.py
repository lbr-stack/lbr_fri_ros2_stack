#!/usr/bin/python3
import time

import casadi as cs
import numpy as np
import optas  # https://github.com/cmower/optas
import rclpy
from lbr_fri_msgs.msg import LBRPositionCommand, LBRState
from rclpy.node import Node


def dampedLeastSquares(J, lam=2e-1):
    U, S, VT = np.linalg.svd(J)

    # print(f"{U.shape=}, {S.shape=}, {VT.shape=}")
    ns = S.shape[0]
    s = np.zeros((J.shape[1], J.shape[0]))
    for i in range(ns):
        s[i, i] = S[i] / (S[i] * S[i] + lam * lam)
    # print("s=", s)
    return VT.T @ s @ U.T


class Controller:

    def __init__(self, robot_description):
        self._solver_dur = None
        ee_link = "link_ee"
        # urdf_filename = replace_package('{kukamed_dev}/configs/v3/med7.urdf')
        robot = optas.RobotModel(urdf_string=robot_description, time_derivs=[0, 1])
        self.robot = robot

        self.Jac = self.robot.get_global_geometric_jacobian_function(ee_link)

        T = 2
        builder = optas.OptimizationBuilder(T, robots=[robot])
        name = robot.get_name()
        self.name = name

        rcm = builder.add_parameter("rcm", 3)

        q0 = builder.get_model_state(name, 0, time_deriv=0)
        qF = builder.get_model_state(name, 1, time_deriv=0)
        qd = builder.get_model_state(name, 0, time_deriv=1)

        qc = builder.add_parameter("qc", robot.ndof)
        qd_goal = builder.add_parameter("qd_goal", robot.ndof)

        _q = optas.SX.sym("q", self.robot.ndof)
        _rcm = optas.SX.sym("rcm", 3)
        Tf = self.robot.get_global_link_transform(ee_link, _q)
        zf = Tf[:3, 2]
        pf = Tf[:3, 3]
        alpha = zf.T @ (_rcm - pf)
        _dist_sqr = optas.sumsqr(pf + alpha * zf - _rcm)
        self.dist_sqr = optas.Function("dist_sqr", [_q, _rcm], [_dist_sqr])

        builder.add_equality_constraint("rcm", self.dist_sqr(qF, rcm))

        xlim = -0.15
        zlim = 0.8

        c1 = cs.sumsqr(qd - qd_goal)
        builder.add_cost_term("match_qd_goal", c1)

        builder.add_equality_constraint("qinit", q0, qc)
        builder.add_equality_constraint("dynamics", q0 + qd, qF)

        self.eff_transform = robot.get_global_link_transform_function(ee_link)
        pF = robot.get_global_link_position(ee_link, qF)

        q = cs.SX.sym("q", robot.ndof)
        p = robot.get_global_link_position(ee_link, q)
        self.xpos = cs.Function("xpos", [q], [p[0]])

        self.solver = optas.ScipyMinimizeSolver(builder.build()).setup("SLSQP")

        self._rcm = None

    def set_start(self, q):
        T = self.eff_transform(q)
        z = T[:3, 2]
        p = T[:3, 3]
        self._rcm = p + 0.2 * z

    def reset(self, qc, qd_goal):
        params = {"qc": qc, "qd_goal": qd_goal, "rcm": self._rcm}
        self.solver.reset_parameters(params)
        self.solver.reset_initial_seed(
            {
                f"{self.name}/q": cs.horzcat(cs.DM(qc), cs.DM(qc)),
                f"{self.name}/dq": qd_goal,
            }
        )

    def solve(self):
        self.sol = self.solver.solve()
        return self.solver.did_solve()

    def get_qd_target(self):
        return self.sol[f"{self.name}/dq"].toarray().flatten()


class LBRAdmittanceControlRCMNode(Node):

    def __init__(self, node_name="admittance_rcm_control_node"):
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("command_rate", 100)
        self.declare_parameter("robot_description")

        self._command_rate = int(self.get_parameter("command_rate").value)
        self.dt = 1.0 / float(self._command_rate)
        self._robot_description = str(self.get_parameter("robot_description").value)

        self.dq = np.zeros(7)
        self._controller = Controller(self._robot_description)

        self._lbr_state_subscriber = self.create_subscription(
            LBRState, "/lbr/state", self._lbr_state_callback, 1
        )

        self._lbr_command_publisher = self.create_publisher(
            LBRPositionCommand, "/lbr/command/joint_position", 1
        )

    def command(self, q):
        lbr_command = LBRPositionCommand()
        lbr_command.joint_position = q
        self._lbr_command_publisher.publish(lbr_command)

    def _admittance(self, tau_ext, qc):

        J = self._controller.Jac(qc).toarray()

        Jinv = dampedLeastSquares(J)
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

        self.dq = (1 - 0.02) * self.dq + 0.02 * dq

        return self.dq

    def _lbr_state_callback(self, msg: LBRState) -> None:

        if self._controller._rcm is None:
            self._controller.set_start(msg.measured_joint_position)
            return

        dq_goal = self._admittance(msg.external_torque, msg.measured_joint_position)
        self._controller.reset(msg.measured_joint_position, dq_goal)

        if self._controller.solve():
            qc = np.array(msg.measured_joint_position)
            dq = self._controller.get_qd_target()
            qn = qc + self.dt * dq * 10.0
            self.command(qn.tolist())
        else:
            self.get_logger().error("Solver failed!")


def main(args=None):
    rclpy.init(args=args)
    node = LBRAdmittanceControlRCMNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
