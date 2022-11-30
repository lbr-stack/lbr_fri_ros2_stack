#!/usr/bin/python3
import optas  # https://github.com/cmower/optas
import rclpy
from rclpy.node import Node
from typing import List
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from lbr_state_msgs.msg import LBRState

optas.np.set_printoptions(precision=2, suppress=True, linewidth=1000)


class Controller:

    def __init__(self,
                 urdf_string,
                 end_effector='lbr_link_ee',
                 hz=100,
                 wdq=[1, 1, 1, 1, 1, 1, 1],
                 f_zero=[4.5, 4.5, 4.5, 4.5, 4.5, 4.5],
                 f_max=[5, 5, 5, 5, 5, 5],
                 admittance=[0.0025, 0.0025, 0.0025, 0.0025, 0.0025, 0.0025],
                 dx_max=[0.03, 0.03, 0.03, 0.01, 0.01, 0.01],
                 smooth=0.02,
    ):

        dt = 1.0/float(hz)

        # Setup robot
        robot = optas.RobotModel(urdf_string=urdf_string, time_derivs=[0, 1])
        self._name = robot.get_name()

        # Setup optimization builder
        builder = optas.OptimizationBuilder(T=2, robots=robot)

        # Add parameters
        dq_prev = builder.add_parameter('dq_prev', robot.ndof)
        qc = builder.add_parameter('qc', robot.ndof)
        tau_ext = builder.add_parameter('tau_ext', robot.ndof) # external torque

        # Get joint states
        dq = builder.get_model_state(self._name, 0, time_deriv=1)
        q0 = builder.get_model_state(self._name, 0)
        qF = builder.get_model_state(self._name, 1)

        # Constraint: initial configuration
        builder.add_equality_constraint('init_config', qc, q0)

        # Constraint: dynamics
        builder.add_equality_constraint('dynamics', qF, q0 + dt*dq)

        # Get jacobian array
        J = robot.get_global_geometric_jacobian(end_effector, qc)
        Jinv = optas.pinv(J)

        # Compute wrench at end-effector
        f_ext = Jinv.T @ tau_ext

        # Zero the forces if within threshold
        for i in range(6):
            f_ext[i] = optas.if_else(optas.fabs(f_ext[i]) <= f_zero[i], 0., f_ext[i])

        # Admittance control (convert wrench at end-effector to goal velocity)
        dx_goal = optas.diag(admittance) @ f_ext

        # Threshold dx_goal
        dx_max = optas.vec(dx_max)
        dx_goal = optas.clip(dx_goal, -dx_max, dx_max)

        # Compute joint velocity goal
        dq_goal = Jinv @ dx_goal

        # Smooth joint velocity goal
        dq_goal = (1.0-smooth)*dq_prev + smooth*dq_goal

        # Cost: match end-effector velocity
        builder.add_cost_term('match_vel_goal', 1e3*optas.sumsqr(dq_goal - dq))

        # Cost: minimize joint velocity
        Wdq = optas.diag(wdq)
        builder.add_cost_term('min_joint_vel', dq.T @ Wdq @ dq)

        # Create solver
        self._solver = optas.CasADiSolver(builder.build()).setup('ipopt')
        self._solution = None
        self._dq_prev = optas.DM.zeros(robot.ndof)

    def compute_next_state(self, qc, tau_ext) -> bool:

        # Reset initial seed and parameters
        if self._solution is not None:
            self._solver.reset_initial_seed(self._solution)
        else:
            self._solver.reset_initial_seed({f'{self._name}/q': optas.horzcat(qc, qc)})

        self._solver.reset_parameters({'qc': qc, 'tau_ext': tau_ext, 'dq_prev': self._dq_prev})

        # Solve and return if solved
        self._solution = self._solver.solve()
        return self._solver.did_solve()

    def get_next_state(self) -> List:
        self._dq_prev = self._solution[f'{self._name}/dq']
        return self._solution[f'{self._name}/q'][:, 1].toarray().flatten().tolist()



class LBRForceControlNode(Node):

    def __init__(self, node_name='lbr_force_control_node'):
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("sim", False)
        self.declare_parameter("command_rate", 100)
        self.declare_parameter("robot_description")

        self._sim = self.get_parameter("sim")
        self._command_rate = int(self.get_parameter("command_rate").value)
        self._robot_description = str(self.get_parameter("robot_description").value)
        self._lbr_state = None
        self._controller = Controller(self._robot_description, hz=self._command_rate)

        self._position_command_publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 1
        )

        self._lbr_state_subscriber = self.create_subscription(
            LBRState, "/lbr_state", self._lbr_state_callback, 1
        )

    def command(self, q):
        command = Float64MultiArray()
        command.data = q
        # self.get_logger().info(str(q))
        self._position_command_publisher.publish(command)

    def _lbr_state_callback(self, msg: LBRState) -> None:

        if self._controller.compute_next_state(msg.position, msg.external_torque):

            e = optas.np.array(msg.external_torque)
            c = optas.np.array(msg.position)
            n = optas.np.array(self._controller.get_next_state())

            dq = (n - c)*float(self._command_rate)

            self.get_logger().error('exter:\n'+str(e))

            self.get_logger().error('dq:\n'+str(dq))
            self.command(self._controller.get_next_state())
        else:
            self.get_logger().error("Solver failed!")

def main(args=None):
    rclpy.init(args=args)
    node = LBRForceControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
