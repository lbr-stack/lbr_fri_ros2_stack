#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <Eigen/Core>

#include <lbr_msgs/LBRState.h>


# include <iostream>


// damped least squares solutions: http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
dampedLeastSquares(const MatT &mat, typename MatT::Scalar lambda = typename MatT::Scalar{2e-1}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(mat.cols(), mat.rows());
    dampedSingularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        dampedSingularValuesInv(i, i) = singularValues(i) / (singularValues(i)*singularValues(i) + lambda*lambda);
    }
    return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
};


class ForceController {

public:
    ForceController(
        ros::NodeHandle& nh,
        double dt=0.005, double alpha=0.01, double exp_smooth=0.03, double th_f=4., double th_tau=1.5,
        std::string control_client="PositionJointInterface_trajectory_controller/follow_joint_trajectory", 
        std::string state_topic="states", std::string planning_group="arm"
    ) : 
        _nh(nh), 
        _move_group(planning_group),
        _ac(nh, control_client, false),
        _control_client(control_client),
        _dt(dt), _alpha(alpha), _exp_smooth(exp_smooth), _th_f(th_f), _th_tau(th_tau),
        _dq(_move_group.getActiveJoints().size(), 0.),
        _control_timer(_nh.createTimer(ros::Duration(_dt), &ForceController::controlCB_, this)),
        _state_sub(nh.subscribe(state_topic, 1, &ForceController::_stateCB, this)) {
            ROS_INFO("ForceController: Waiting for action server under %s...", _control_client.c_str());
            _ac.waitForServer();
            ROS_INFO("Done.");

            if (_dt == 0.) ROS_ERROR("ForceController: Received invalid argument dt %f", _dt); std::exit(-1);
            if (_alpha == 0.) ROS_ERROR("ForceController: Received invalid argument alpha %f", _alpha); std::exit(-1);
    };

    ~ForceController() {    };

private:
    auto controlCB_(const ros::TimerEvent&) -> void {
        auto robot_state = _move_group.getCurrentState();

        auto J = robot_state->getJacobian(
            robot_state->getJointModelGroup(_move_group.getName())
        );
        auto q = _move_group.getCurrentJointValues();

        auto tau_ext = Eigen::VectorXd::Map(_state.external_torque.data(), _state.external_torque.size());
        Eigen::VectorXd f_ext = dampedLeastSquares(J.transpose())*tau_ext;

        // Velocity in direction of force -> hand-guiding
        Eigen::VectorXd dx = Eigen::VectorXd::Zero(J.rows());

        int j = 0;
        for (int i = 0; i < 3; i++) {
            if (std::abs(f_ext[j]) > _th_f) (f_ext[j] > 0. ? dx[j] = 0.1 : dx[j] = -0.1);
            j++;
        }
        for (int i = 0; i < 3; i++) {
            if (std::abs(f_ext[j]) > _th_tau) (f_ext[j] > 0. ? dx[j] = 1.0 : dx[j] = -1.0);
            j++;
        }

        std::cout << f_ext.transpose() << std::endl;

        // Compute update
        Eigen::VectorXd dq = dampedLeastSquares(J)*dx;

        for (int i = 0; i < q.size(); i++) {
            _dq[i] = (1-_exp_smooth)*_dq[i] + _exp_smooth*dq[i];
            q[i] += _dq[i];
        }

        // Send joint position goal
        // auto status = _executeGoal(q);
    };

    auto _stateCB(const lbr_msgs::LBRStateConstPtr& msg) -> void {
        _state = *msg;
    };

    auto _executeGoal(std::vector<double> q, bool wait_for_result=false) -> actionlib::SimpleClientGoalState {
        // See for example http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
        
        // Execute motion on client
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(_dt/_alpha);
        point.positions = q;

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = _move_group.getJointNames();
        goal.trajectory.points.push_back(point);

        _ac.sendGoal(goal);
        if (wait_for_result) _ac.waitForResult();

        return _ac.getState();
    };



    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface _move_group;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _ac;

    std::string _control_client;
    double _dt, _alpha, _exp_smooth, _th_f, _th_tau;
    std::vector<double> _dq;

    ros::Timer _control_timer;
    ros::Subscriber _state_sub;
    lbr_msgs::LBRState _state;

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "force_control_node");
    ros::NodeHandle nh;

    double dt, alpha, exp_smooth, th_f, th_tau;
    std::string control_client, state_topic, planning_group;

    nh.getParam("dt", dt);
    nh.getParam("alpha", alpha);
    nh.getParam("exp_smooth", exp_smooth);
    nh.getParam("th_f", th_f);
    nh.getParam("th_tau", th_tau);
    nh.getParam("control_client", control_client);
    nh.getParam("state_topic", state_topic);
    nh.getParam("planning_group", planning_group);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ForceController force_controller(
        nh,
        dt, alpha, exp_smooth, th_f, th_tau,
        control_client, state_topic, planning_group
    );

    ros::waitForShutdown();

    return 0;
};
