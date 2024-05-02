import casadi as cs
import optas


class AdmittanceRCMController:
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

        c1 = cs.sumsqr(qd - qd_goal)
        builder.add_cost_term("match_qd_goal", c1)

        builder.add_equality_constraint("qinit", q0, qc)
        builder.add_equality_constraint("dynamics", q0 + qd, qF)

        self.eff_transform = robot.get_global_link_transform_function(ee_link)

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
