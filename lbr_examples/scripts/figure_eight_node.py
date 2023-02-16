#!/usr/bin/python3
import time
import optas
import rclpy
from visualization_msgs.msg import Marker
from copy import deepcopy
from lbr_python_tools.lbr_node import LBRPositionControlNode


class Controller:
    def __init__(self, xacro_filename, dt, end_effector_name):
        self.robot = optas.RobotModel(xacro_filename=xacro_filename, time_derivs=[0, 1])
        self.eff_pos = self.robot.get_global_link_position_function(end_effector_name)
        self.zaxis = self.robot.get_global_link_axis_function(end_effector_name, "z")
        self.name = self.robot.get_name()
        T = 2
        builder = optas.OptimizationBuilder(T, robots=self.robot)
        qc = builder.add_parameter("qc", self.robot.ndof)
        qn = builder.add_parameter("qn", self.robot.ndof)
        goal_pos = builder.add_parameter("goal_pos", 3)
        goal_zaxis = builder.add_parameter("goal_zaxis", 3)
        goal_zaxis = optas.unit(goal_zaxis)  # ensure normalized
        builder.integrate_model_states(self.name, 1, dt)  # dynamics
        builder.fix_configuration(self.name, config=qc)  # initial configuration
        q = builder.get_model_state(self.name, -1)
        p = self.eff_pos(q)
        builder.add_cost_term("nominal", 10 * optas.sumsqr(q - qn))
        builder.add_cost_term("goal_pos", 1e3 * optas.sumsqr(p - goal_pos))
        builder.add_equality_constraint(
            "goal_zaxis", self.zaxis(q), goal_zaxis
        )  # goal zaxis
        dq = builder.get_model_state(self.name, 0, time_deriv=1)
        builder.add_cost_term("min_dq", 0.1 * optas.sumsqr(dq))
        self.solver = optas.CasADiSolver(
            builder.build(), throw_error_when_solver_failed=True
        ).setup("ipopt")
        self.solution = None

    def __call__(self, qc, goal_pos, goal_zaxis, nominal):
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed({f"{self.name}/q": optas.horzcat(qc, qc)})
        self.solver.reset_parameters(
            {"qc": qc, "goal_pos": goal_pos, "goal_zaxis": goal_zaxis, "qn": nominal}
        )
        self.solution = self.solver.solve()
        return self.solution[f"{self.name}/q"][:, -1].toarray().flatten().tolist()


class FigureEightNode(LBRPositionControlNode):
    def __init__(self):
        super().__init__("figure_eight_node")
        self.declare_parameter("hz", 100)
        self.declare_parameter("end_effector_name", "lbr_link_ee")
        self.declare_parameter("visualize_target", True)
        end_effector_name = str(self.get_parameter("end_effector_name").value)
        hz = int(self.get_parameter("hz").value)
        dt = 1.0 / float(hz)
        self._first = True
        self._goal_zaxis = None
        self._start_time = None
        self._controller = Controller(self.xacro_filename, dt, end_effector_name)
        self._delay = 2.0

        self._vis_pub = None
        self._marker = Marker()
        self._visualize_target = bool(self.get_parameter("visualize_target").value)
        self.visualize_target = self._no_visualize
        if self._visualize_target:
            self._vis_pub = self.create_publisher(Marker, "target", 10)
            self.visualize_target = self._visualize
            self._marker.header.frame_id = "world"
            self._marker.type = Marker.SPHERE
            self._marker.action = Marker.ADD
            self._marker.pose.orientation.w = 1.0
            self._marker.scale.x = 0.025
            self._marker.scale.y = 0.025
            self._marker.scale.z = 0.025
            self._marker.color.a = 0.75
            self._marker.color.g = 1.0

        self.create_timer(dt, self._timer_callback)

    def _no_visualize(self, target):
        pass

    def _visualize(self, target):
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._marker.pose.position.x = target[0]
        self._marker.pose.position.y = target[1]
        self._marker.pose.position.z = target[2]
        self._vis_pub.publish(self._marker)

    def get_goal_pos(self):
        t = time.time() - self._start_time - self._delay
        if t < 0.0:
            t = 0.0
        x = 0
        y = 0.1 * optas.sin(0.05 * optas.np.pi * t)
        z = 0.1 * optas.sin(0.1 * optas.np.pi * t)
        return self._initial_eff_pos + optas.DM([x, y, z])

    def _timer_callback(self):
        if not self.joint_states.recieved():
            return
        qc = self.joint_states.get_position()
        if self._first:
            self._nominal = deepcopy(qc)
            self._start_time = time.time()
            self._goal_zaxis = self._controller.zaxis(qc)
            self._initial_eff_pos = self._controller.eff_pos(qc)
            self._first = False
        goal_pos = self.get_goal_pos()
        self.visualize_target(goal_pos.toarray().flatten().tolist())
        qg = self._controller(qc, goal_pos, self._goal_zaxis, self._nominal)
        self.command(qg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FigureEightNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
