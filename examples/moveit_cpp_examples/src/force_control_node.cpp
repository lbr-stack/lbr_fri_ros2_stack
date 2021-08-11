#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Core>

#include <lbr_msgs/LBRState.h>


# include <iostream>


// damped least squares solutions: http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
dampedLeastSquares(const MatT &mat, typename MatT::Scalar lambda = typename MatT::Scalar{1e-2}) // choose appropriately
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
    ForceController(ros::NodeHandle& nh, double ctrl_int=0.005, std::string state_topic="states", std::string planning_group="arm") :
        nh_(nh), 
        move_group_(planning_group),
        control_timer_(nh_.createTimer(ros::Duration(ctrl_int), &ForceController::controlCB, this)),
        state_sub_(nh.subscribe(state_topic, 1, &ForceController::stateCB, this)) {

    };

    ~ForceController() {    };

private:
    auto controlCB(const ros::TimerEvent&) -> void {
        auto robot_state = move_group_.getCurrentState();

        // robot_state->getJacobian(

        // );
    };

    auto stateCB(const lbr_msgs::LBRStateConstPtr& msg) -> void {
        state_ = *msg;
    };



    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    ros::Timer control_timer_;
    ros::Subscriber state_sub_;
    lbr_msgs::LBRState state_;

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "force_control_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ForceController force_controller(nh);

    ros::waitForShutdown();

    return 0;
};





// read external torques

// obtain externally applied generalized force through jacobian

// compute velocity command opposite to acting force

// feedback joint position update through pseudo inverse jacobian

// exponential smoothing



