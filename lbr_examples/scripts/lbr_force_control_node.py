#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from copy import deepcopy

from lbr_fri_msgs.msg import LBRState, LBRCommand

import optas  # https://github.com/cmower/optas

class Controller:

    def __init__(self, robot_description, hz, end_effector):

        # Setup variables
        dt = 1.0/float(hz)

        # Setup robot model
        robot = optas.RobotModel(urdf_string=robot_description, time_derivs=[0, 1])
        self.name = robot.get_name()

        # Setup optimization builder
        builder = optas.OptimizationBuilder(T=2, robots=robot)

        # Get jacobian function
        self.J = robot.get_global_geometric_jacobian_function(end_effector)

        # Setup parameters
        qc = builder.add_parameter('current_robot_configuration', robot.ndof)
        dx_goal = builder.add_parameter('goal_end_effector_velocity', 6)

        # Get states
        q0 = builder.get_model_state(self.name, 0, time_deriv=0)
        qF = builder.get_model_state(self.name, 1, time_deriv=0)
        dq = builder.get_model_state(self.name, 0, time_deriv=1)

        # Get jacobian at current configuration
        J = self.J(qc)

        # Constraint: initial state
        builder.add_equality_constraint('init', q0, qc)

        # Constraint: dynamics
        builder.add_equality_constraint('dynamics', q0 + dt*dq, qF)

        # Cost: match end-effector goal veloity
        pc = robot.get_global_link_position(end_effector, qc)
        dx = J @ dq
        pF = pc + dt * dx[:3]
        pG = pc + dt * dx_goal[:3]
        # diff = dx - dx_goal
        Wx = optas.diag([1, 1, 1, 1, 1, 1])
        # builder.add_cost_term('match_eff_goal_vel', optas.sumsqr(dx - dx_goal))#rho*(diff.T @ Wx @ diff))
        builder.add_cost_term('match_eff_goal_pos', optas.sumsqr(pF - pG))

        # Constraint: joint position/velocity limits
        # builder.enforce_model_limits(self.name, time_deriv=0)
        # builder.enforce_model_limits(self.name, time_deriv=1)

        # Cost: minimize joint motion
        # Wq = optas.diag(range(robot.ndof, 0, -1))
        # builder.add_cost_term('min_dq', dq.T @ Wq @ dq)

        # Setup solver
        self.dx = optas.DM.zeros(6)
        self.solution = None
        self.solver = optas.CasADiSolver(builder.build()).setup('ipopt')

    def get_jacobian_inverse(self, q):
        return optas.pinv(self.J(q))

    # def get_jacobian_inverse(self, q):
    #     return optas.DM(optas.np.linalg.pinv(self.J(q).toarray(), rcond=0.01))

    def smooth(self, nxt, prv, alpha=0.02):
        return (1.-alpha)*prv + alpha*nxt

    def admittance_control(self, f_ext):

        translational_vel = 0.2
        th_f = 4.0
        th_tau = 0.5
        scale = 2.0

        f_ext = f_ext.toarray().flatten()

        dx = optas.np.zeros(6)

        for i in [0, 1, 2]:
            sign = optas.np.sign(f_ext[i])
            if (abs(f_ext[i]) > th_f) and (abs(f_ext[i]) < scale*th_f):
                dx[i] = sign * translational_vel * (abs(f_ext[i]) - th_f)/(scale*th_f - th_f)
            elif abs(f_ext[i]) > scale*th_f:
                if f_ext[i] > 0.0:
                    dx[i] = translational_vel
                else:
                    dx[i] = -translational_vel

        for i in [3, 4, 5]:
            sign = optas.np.sign(f_ext[i])
            if (abs(f_ext[i]) > th_tau) and (abs(f_ext[i]) < scale*th_tau):
                dx[i] = sign*translational_vel*(abs(f_ext[i]) - th_tau)/(scale*th_tau - th_tau)
            elif (abs(f_ext[i]) > scale*th_tau):
                if f_ext[i] > 0.:
                    dx[i] = translational_vel
                else:
                    dx[i] = -translational_vel

        return optas.DM(dx)

    def compute_next_state(self, qc, tau_ext):

        # Compute external wrench on end-effector
        f_ext = self.get_jacobian_inverse(qc).T @ tau_ext

        # Admittance control
        self.dx = self.smooth(self.admittance_control(f_ext), self.dx)

        # Reset solver
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed({f'{self.name}/q': optas.horzcat(qc, qc)})

        self.solver.reset_parameters({
            'current_robot_configuration': qc,
            'goal_end_effector_velocity': self.dx,
        })

        # Solve problem
        # if self.solution is None:
        self.solution = self.solver.solve()
        return self.solver.did_solve()
        # return True

    def get_next_state(self):
        return self.solution[f'{self.name}/q'].toarray()[:,-1].flatten().tolist()

    def get_goal_dq(self):
        return self.solution[f'{self.name}/dq'].toarray().flatten().tolist()

class LBRForceControlNode(Node):

    def __init__(self):


        super().__init__('lbr_force_control_node')

        # Declare and get parameters
        self.declare_parameter("robot_description")
        robot_description = str(self.get_parameter("robot_description").value)

        self.declare_parameter("command_rate", 100)
        command_rate = int(self.get_parameter("command_rate").value)

        self.declare_parameter("end_effector", "lbr_link_ee")
        end_effector = str(self.get_parameter("end_effector").value)

        # Setup controller
        self._controller = Controller(robot_description, command_rate, end_effector)

        # Setup publishers/subscribers
        self._lbr_command_publisher = self.create_publisher(LBRCommand, "/lbr_command", 1)
        self._lbr_state_smooth_subscriber = self.create_subscription(
            LBRState, "/lbr_state/smooth", self._lbr_state_smooth_callback, 1,
        )

    def _command(self, q):
        command = LBRCommand(client_command_mode=1, joint_position=q)
        self._lbr_command_publisher.publish(command)

    def _lbr_state_smooth_callback(self, msg):
        if self._controller.compute_next_state(msg.measured_joint_position, msg.external_torque):
            self.get_logger().info('dx: '+str(self._controller.dx))
            self.get_logger().info('dq: '+str(self._controller.get_goal_dq()))
            self._command(self._controller.get_next_state())
        else:
            self.get_logger().error("Solver failed!")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LBRForceControlNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
