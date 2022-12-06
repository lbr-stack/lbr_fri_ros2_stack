#!/usr/bin/python3
import time
import math
from rclpy.action import ActionClient
import optas  # https://github.com/cmower/optas
import time
import numpy as np
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from typing import List
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.action import FollowJointTrajectory

from lbr_fri_msgs.msg import LBRState, LBRCommand

optas.np.set_printoptions(precision=2, suppress=True, linewidth=1000)

action_server = '/position_trajectory_controller/follow_joint_trajectory'
joint_names = [f'lbr_joint_{i}' for i in range(7)]


class Controller:

    def __init__(self,
                 urdf_string,
                 end_effector='lbr_link_ee',
                 hz=100,
                 wdq=[1, 1, 1, 1, 1, 1, 1],
                 f_zero=[4.0, 4.0, 4.0, 1.0, 1.0, 1.0],
                 f_max=[7, 7, 7, 2, 2, 2],
                 admittance=[0.0025, 0.0025, 0.0025, 0.0025, 0.0025, 0.0025],
                 dx_max=[0.03, 0.03, 0.03, 0.01, 0.01, 0.01],
                 smooth=0.02,
    ):

        # Setup robot
        robot = optas.RobotModel(urdf_string=urdf_string, time_derivs=[0, 1])
        self._name = robot.get_name()
        # self._J = robot.get_global_linear_jacobian_function(end_effector)
        self._J = robot.get_global_geometric_jacobian_function(end_effector)
        self.dq = optas.np.zeros(7)

    def get_jacobian(self, q):
        return self._J(q).toarray()

    def get_jacobian_inverse(self, q, lam=2e-1):
        J = self.get_jacobian(q)        
        return np.linalg.pinv(J, rcond=0.01), J
        # U, S, VT = optas.np.linalg.svd(J)
        # ns = S.shape[0]
        # s = optas.np.zeros((J.shape[1], J.shape[0]))
        # for i in range(ns):
        #     s[i, i] = S[i]/(S[i]**2 + lam**2)
        # return VT.T @ s @ U.T

    def compute_next_state(self, qc, tau_ext) -> bool:

        Jinv, J = self.get_jacobian_inverse(qc)

        # dx = optas.np.array([0.0,0.0,0.01])
        # dq = Jinv @ dx

        # self.dq = dq
        # dt = 1.0/100.
        # self._goal = qc + dt*dq

        f_ext = Jinv.T @ tau_ext

        exp_smooth = 0.02
        translational_vel = 0.2
        th_f = 4.0
        th_tau = 0.5
        scale = 2.0

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

        dq = Jinv @ dx

        self.dq = (1-exp_smooth)*self.dq + exp_smooth*dq

        dt = 1.0/100.

        self._goal = qc + dt*self.dq

        return True, J

    def get_next_state(self) -> List:
        return self._goal.tolist()



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

        self.get_logger().info("-------here-----------")

        # Setup action client
        # self._client = ActionClient(self, FollowJointTrajectory, action_server)
        # self._client.wait_for_server()

        self.qinit = None
        self.qstart = optas.np.deg2rad([0, 30, 0, -45, 0, 60, 0]).tolist()
        self.at_start = False
        # self.command(optas.np.deg2rad([0, 30, 0, -45, 0, 60, 0]).tolist())

        self._window_length = 20
        self._position_window = []
        self._ext_torque_window = []

        # time.sleep(2)

        # # Start state subscriber
        self._lbr_state_subscriber = self.create_subscription(
            LBRState, "/lbr_state/smooth", self._lbr_state_callback, 100
        )


        self._position_command_publisher = self.create_publisher(LBRCommand, '/lbr_command', 10)

        # self._position_command_publisher = self.create_publisher(
        #     Float64MultiArray, "/forward_position_controller/commands", 1
        # )

        # Start state subscriber
        # self._lbr_state_subscriber = self.create_subscription(
        #     JointState, "/joint_states", self._lbr_state_callback, 1
        # )

    # def command(self, q):
    #     command = Float64MultiArray()
    #     command.data = q
    #     # self.get_logger().info(str(q))
    #     self._position_command_publisher.publish(command)

    def command(self, q):
        cmd = LBRCommand(client_command_mode=1, joint_position=q)
        self._position_command_publisher.publish(cmd)


    # def command(self, q):
    #     dt = (1.0/float(self._command_rate))/0.15
    #     goal = FollowJointTrajectory.Goal()
    #     goal.trajectory.joint_names = joint_names
    #     goal.trajectory.points = [JointTrajectoryPoint(time_from_start=Duration(seconds=dt).to_msg())]
    #     goal.trajectory.points[0].positions = q
    #     self._client.send_goal_async(goal)
        # self._client.wait_for_server()


    def _lbr_state_callback(self, msg: LBRState) -> None:


        # NO SMOOTH
        # [-1.0306387399183016e-05, 0.8454608216638262, 4.4880923548877684e-05, -1.2561714743503292, 0.053008527780070175, 0.8892982774301188, -0.0001024052303653588])
        # [-0.6471269279269534, 1.555126048423324, -0.06054706312318544, 0.2874986168976977, -0.3578914178156747, -0.2563220743772851, 0.007513290569132143])        


        # SMOOTH
        # [-1.0337037251303277e-05, 0.8454607867448243, 4.487106941960401e-05, -1.2561714769476038, 0.05300855708929061, 0.8892981845721579, -0.00010240863369751193])
        # [-0.64821639313649, 1.5492188626869248, -0.05952264896134366, 0.28917341625792925, -0.3620496232703626, -0.2566734043640636, 0.007754301946650115])
        
        

        # self.get_logger().info(str(msg.measured_joint_position))
        # self.get_logger().info(str(msg.external_torque))        

        # p = []
        # for name in joint_names:
        #     p.append(msg.position[msg.name.index(name)])


        # if self.qinit is None:
        #     self.qinit = p
        #     self.t0 = time.time()
        #     return

        # if not self.at_start:

        #     dur = 3.
        #     t = time.time()
        #     alpha = (t - self.t0)/dur

        #     self.get_logger().info('type(qinit)' + str(type(self.qinit)))
        #     self.get_logger().info('type(qstart)' + str(type(self.qstart)))

        #     qn = alpha*optas.np.array(self.qstart) + (1-alpha)*optas.np.array(self.qinit)

        #     self.get_logger().info('goal: '+str(qn))

        #     self.command(qn.tolist())

        #     if alpha > 1.0:
        #         self.at_start = True
        #     return

        # self._position_window.append(msg.measured_joint_position)
        # self._ext_torque_window.append(msg.external_torque)
        # if len(self._position_window) > self._window_length:
        #     self._position_window.pop(0)
        #     self._ext_torque_window.pop(0)

        # P = np.mean(self._position_window, axis=0).flatten().tolist()
        # E = np.mean(self._ext_torque_window, axis=0).flatten().tolist()

        success, J = self._controller.compute_next_state(msg.measured_joint_position, msg.external_torque) #(msg.position, msg.external_torque):
        # success, J = self._controller.compute_next_state(P, E)

        # self.get_logger().info('J='+str(J))
        
        if success:
            # c = optas.np.array(msg.position)
            # n = optas.np.array(self._controller.get_next_state())

            # n# self.command(n)
            n = self._controller.get_next_state()


            # self.get_logger().info('goal: '+str(n))
            # self.get_logger().info('goal: '+str(p))

            self.get_logger().info('-=------hwerwerhwerhwer-----------------')
            self.get_logger().info('dq:\n' + str(self._controller.dq))            
            self.command(n)
        else:
            self.get_logger().error("Solver failed!")

def main(args=None):
    rclpy.init(args=args)
    node = LBRForceControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
