// Standard lib
#include <string>
#include <iostream>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>


class RobotModel {

public:

  RobotModel(std::string urdf) {

    KDL::Tree tree; // KDL tree representation for robot
    if (!kdl_parser::treeFromString(urdf, tree)) {
      return;
    }

  }

  ~RobotModel() {}

};


/*


// Standard lib
#include<string>
#include<vector>
#include <algorithm>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Number of degrees of freedom for robot
#define NDOF=7

// Status of robot
enum
RobotStatus
{
OK=0,
INITIALIZATION_FAILED=1,
INSUFFICIENT_COMMAND=2,
JOINT_LIMIT_VIOLATION=3,
LINK_LIMIT_VIOLATION=4,
};


// Container for a robot command
struct RobotCommand {
double t;
Eigen::VectorXd q;
};

class FK {

public:

FK(chain) {
_solver = ChainFkSolverPos_recursive(chain);
}

bool solve(Eigen::VectorXd q) {

// Convert Eigen vector to KDL joint array
KDL::JntArray jointpositions = JntArray(NDOF);
for (int i=0; i<NDOF; ++i)
jointpositions(i)=q[i];

// Compute position
int kinematics_status = fksolver.JntToCart(jointpositions, _cartpos);
return kinematics_status >= 0;

}

Eigen::Vector3d get_position(Eigen::VectorXd q) {
return Eigen::Vector3d(_cartpos.p.data);
}

private:

KDL::Frame _cartpos;
ChainFkSolverPos_recursive _solver;


};

// Main class for modeling robot
class Robot {

public:

std::vector<std::string> joint_names; // vector containing joint names in required order (set in constructor)

// Constructor
//
//   urdf: URDF as a string (from robot_description parameter)
Robot(std::string urdf) {

// Set status
_status = OK;

// Setup joint names
joint_names.resize(NDOF);
for (int i=0; i<NDOF; ++i)
joint_names.push_back("lbr_joint_"+std::to_string(i));

// Load tree model from URDF
KDL::Tree tree; // KDL tree representation for robot
if (!kdl_parser::treeFromString(urdf, tree)) {
_status = INITIALIZATION_FAILED;
_reason = "KDL failed to load the URDF from the given string.";
return;
}

// Setup the FK solver
const std::string root_name = "world";
std::vector<std::string> link_names(6);
for (int i = 2; i < NDOF; ++i) {
link_names.push_back("lbr_link_"+std::to_string(i));
}
link_names.push_back("lbr_link_ee");

ChainFkSolverPos_recursive fksolver _fksolver = ChainFkSolverPos_recursive(chain);



// Hard code joint/link limits
//
//             =========================================================================
//             == The following should NOT be adjusted, speak with Chris/Martin first ==
//             =========================================================================
//

// joint limits
_joint_position_limit_lower << -2.96705972839, -2.09439510239, -2.96705972839, -2.09439510239, -2.96705972839, -2.09439510239, -3.05432619099;
_joint_position_limit_upper <<  2.96705972839,  2.09439510239,  2.96705972839,  2.09439510239,  2.96705972839,  2.09439510239,  3.05432619099;
_joint_velocity_limit << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0; // >>TODO: tune<<
_joint_acceleration_limit << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0; // >>TODO: tune<<

// link limits
_link_position_limit_lower << -10.0, -10.0, 0.0; // >>TODO: tune<<
_link_position_limit_upper <<  10.0,  10.0, 10.0; // >>TODO: tune<<
_end_effector_velocity_limit << 10.0, 10.0, 10.0; // >>TODO: tune<<

//             =========================================================================
//             ==   The above should NOT be adjusted, speak with Chris/Martin first   ==
//             =========================================================================
//

}

// Destructor
~Robot() {}

// Check if commanded target is safe to execute on robot
bool is_safe(const sensor_msgs::msg::JointState::SharedPtr msg) {

// Setup
bool is_cmd_safe = true;

// Check 0: ensure status is currently ok
if (_status != OK) {
is_cmd_safe = false;
return is_cmd_safe;
}

// Check 1: ensure enough data in msg
if (msg->name.size() < NDOF) {
is_cmd_safe = false;
_status = INSUFFICIENT_COMMAND;
_reason = "There is not enough joint names in the commanded joint state message.";
return is_cmd_safe;
}

if (msg->position.size() < NDOF) {
is_cmd_safe = false;
_status = INSUFFICIENT_COMMAND;
_reason = "There is not enough joint positions in the commanded joint state message.";
return is_cmd_safe;
}

if (msg->name.size() == msg->position.size()) {
is_cmd_safe = false;
_status = INSUFFICIENT_COMMAND;
_reason = "The vectors name and position vectors in the joint state message are different lengths.";
return is_cmd_safe;
}

_cmd.t = msg->header.stamp.seconds();
_cmd.q = Eigen::VectorXd::Zero(NDOF);
for (int i = 0; i < NDOF; ++i) {

// Check 2: joint name in command msg
std::string name = joint_names[i];
auto it = std::find(msg->name.begin(), msg->name.end(), name);
if(it != msg->name.end()) {
// joint name in command, get position and populate _q_cmd
int idx = it - msg->name.begin();
_cmd.q[i] = msg->position[idx];

}
else {
// joint name not in command, return error
is_cmd_safe = false;
_status = INSUFFICIENT_DATA;
_reason = "Joint '" + joint_names[i] + "' is missing from joint state command message.";
return is_cmd_safe;
}

}

// Check 3: joint position limits
if (!_in_joint_position_limit()) {
is_cmd_safe = false;
_status = JOINT_LIMIT_VIOLATION;
_reason = "Joint position limits are violated.";
return is_cmd_safe;
}

// Check 4: joint velocity limits
if (_can_estimate_dq()) {
if (!_in_joint_velocity_limit()) {
is_cmd_safe = false;
_status = JOINT_LIMIT_VIOLATION;
_reason = "Joint velocity limits are violated.";
return is_cmd_safe;
}
}

// Check 5: joint acceleration limits
if (_can_estimate_ddq()) {
if (!_in_joint_acceleration_limit()) {
is_cmd_safe = false;
_status = JOINT_LIMIT_VIOLATION;
_reason = "Joint acceleration limits are violated.";
return is_cmd_safe;
}
}

// Check 6: link position limits
if (!_links_in_position_limits()) {
is_cmd_safe = false;
_status = LINK_LIMIT_VIOLATION;
_reason = "Position limits are violated by at least one link.";
return is_cmd_safe;
}

// Check 7: end-effector velocity
if (_can_estimate_deff()) {
if (!_end_effector_velocity_in_limit()) {
is_cmd_safe = false;
_status = LINK_LIMIT_VIOLATION;
_reason = "End-effector velocity violates limit.";
return is_cmd_safe;
}
}

return is_cmd_safe; // true

}

// Acknowledge that the previous message set in is_safe was executed on robot
void command_executed() {

// Append commanded state to _prev_cmd
_prev_cmd.push_back(_cmd);

// Remove first element when vector larger than 2
if (_prev_cmd.size() > 2)
_prev_cmd.erase(_prev_cmd.begin());

}

double get_safe_cmd(int i) {return _cmd.q[i];}

std::string get_status() {return _status;}

std::string get_reason() {return _reason;}

private:

enum RobotStatus _status; // status of the robot model
std::string _reason; // human readable string containing a reason why the current command can't be executed on the robot
std::vector<RobotCommand> _prev_cmd; // previous commands
RobotCommand _cmd;
Eigen::VectorXd _joint_position_limit_lower(NDOF); // manaully specified in constructor
Eigen::VectorXd _joint_position_limit_upper(NDOF); // manaully specified in constructor
Eigen::VectorXd _joint_velocity_limit(NDOF); // manaully specified in constructor
Eigen::VectorXd _joint_acceleration_limit(NDOF); // manaully specified in constructor
Eigen::Vector3d _link_position_limit_lower; // lower link position limit in global coordinate frame, manaully specified in constructor
Eigen::Vector3d _link_position_limit_upper; // upper link position limit in global coordinate frame, manaully specified in constructor
Eigen::Vector3d _end_effector_velocity_limit; // manaully specified in constructor
std::vector<FK> fks; // forward kinematic solvers

// True when there is enough data to estimate joint velocity
bool _can_estimate_dq() {
return _prev_cmd.size() >= 1;
}

// Esimate numerically the target joint velocity
Eigen::VectorXd _estimate_dq() {
RobotCommand prev = _prev_cmd[_prev_cmd.size()-1];
return (_cmd.q - prev.q)/(_cmd.t - prev.t);
}

// True when there is enough data to estimate joint acceleration
bool _can_estimate_ddq() {
return _prev_cmd.size() >= 2;
}

// Esimate numerically the target joint acceleration
// https://mathformeremortals.wordpress.com/2013/01/12/a-numerical-second-derivative-from-three-points/
Eigen::VectorXd _estimate_ddq() {
RobotCommand prev = _prev_cmd[_prev_cmd.size()-1];
RobotCommand pprev = _prev_cmd[_prev_cmd.size()-2];
Eigen::VectorXd l = (_cmd.q - prev.q)/(_cmd.t - prev.t);
Eigen::VectorXd r = (prev.q - pprev.q)/(prev.t - pprev.t);
double d = 0.5*(_cmd.t - pprev.t);
return (l - r)/d;
}

// True when command is in joint position limits
bool _in_joint_position_limit() {
for (int i = 0; i < NDOF; ++i) {
if (!(_joint_position_limit_lower[i] <= _cmd.q[i] && _cmd.q[i] <= _joint_position_limit_upper[i])) {
return false;
}
}
return true;
}

// True when command is in joint velocity limits
bool _in_joint_velocity_limit() {
Eigen::VectorXd abs_dq = _estimate_dq().abs();
for (int i = 0; i < NDOF; ++i) {
if (abs_dq[i] > _joint_velocity_limit[i]) {
return false;
}
}
return true;
}

// True when command is in joint acceleration limits
bool _in_joint_acceleration_limit() {
Eigen::VectorXd abs_ddq = _estimate_ddq().abs();
for (int i = 0; i < NDOF; ++i) {
if (abs_ddq[i] > _joint_acceleration_limit[i]) {
return false;
}
}
return true;
}

// True when command is in link position limits
bool _links_in_position_limits() {

for (FK fk : fks) {

// Compute FK
if (!fk.solve(_cmd.q)) {
return false;
}

// Check position
Eigen::Vector3d p = fk.get_position();
for (int i = 0; i < 3; ++i) {
if (!(_link_position_limit_lower[i] <= p[i] && p[i] <= _link_position_limit_upper[i]))
return false;
}

}

return true;

}

// True when there is enough data to estimate end-effector velocity
bool _can_estimate_deff() {
return _can_estimate_dq();
}

// True when command is in end-effector velocity limits
bool _end_effector_velocity_in_limit() {
FK eefk = fks.back();
if (!eefk.solve(_cmd.q))
return false;
Eigen::Vector3d p_cmd = eefk.get_position();
if (!eefk.solve(_prev_cmd.back().q))
return false;
Eigen::Vector3d p_prev = eefk.get_position();
Eigen::Vector3d deff = (p_cmd - p_prev) / (_cmd.t - _prev_cmd.back().t);

for (int i = 0; i < 3; ++i) {
if (deff[i] > _end_effector_velocity_limit[i])
return false;
}

return true;

}

};


*/
