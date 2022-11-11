// Standard lib
#include <string>
#include <vector>
#include <iostream>

// // KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Eigen
#include <Eigen/Dense>

class RobotModel {

public:


  // Constructor
  RobotModel () {}

  RobotModel(std::string urdf) {

    // Load URDF
    KDL::Tree _tree; // KDL tree representation for robot
    if (!kdl_parser::treeFromString(urdf, _tree))
      return;

    // Get number of dofs and resize q window
    _ndof = _tree.getNrOfJoints();
    _t_window = Eigen::VectorXd::Zero(_n_window);
    _q_window = Eigen::MatrixXd::Zero(_ndof, _n_window);

    // Create FK solvers
    std::string root_name = _tree.getRootSegment()->second.segment.getName();
    std::vector<std::string> link_names;
    for (auto const& seg : _tree.getSegments()) {

      // Skip root
      std::string link_name = seg.second.segment.getName();
      if (link_name != root_name) {

	// Get chain
	KDL::Chain chain;
	if (!_tree.getChain(root_name, link_name, chain)) {
	  return;
	}

	// Only add solvers for chains with at least 1 joint
	if (chain.getNrOfJoints() > 0) {

	  // Add chain
	  _chains.push_back(chain);

	  // Setup FK solver
	  KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(chain);
	  _fksolvers.push_back(fk);

	  _nfk++;

	}

      }

    }

  }

  // Append joint state
  void append(double t, const sensor_msgs::msg::JointState::SharedPtr msg) {

    for (unsigned int i = 0; i < _n_window-1; ++i) {
      if (i < _n_window-1) {
	_t_window(i) = _t_window(i+1);
	_q_window.col(i) = _q_window.col(i+1);
      } else {
	_t_window(i) = t;
	_q_window.col(i) = _msg2q(msg);
      }
    }
    _nappend++;
  }

  ~RobotModel() {}

private:

  unsigned long long int _nappend = 0;
  const unsigned int _n_window = 2; // number of previously executed joint states to keep in memory
  unsigned int _ndof; // number of degrees of freedom for the robot
  KDL::Tree _tree; // kinematic tree
  int _nfk = 0; // number of chains/fksolvers
  std::vector<KDL::Chain> _chains; // kinematics chains
  std::vector<KDL::ChainFkSolverPos_recursive> _fksolvers; // fk solvers associcated with each kinematic chain
  Eigen::VectorXd _t_window; // window of time stamps for each executed joint state
  Eigen::MatrixXd _q_window; // window of executed joint states

  // Extract q from msg
  Eigen::VectorXd _msg2q(const sensor_msgs::msg::JointState::SharedPtr msg) {
    Eigen::VectorXd q(_ndof);

    for (auto const& seg : _tree.getSegments()) {

      std::string link_name = seg.second.segment.getName();
      


    }

  }


  // Compute the positions of each link
  Eigen::MatrixXd _compute_link_positions(Eigen::VectorXd q) {

    // Setup
    Eigen::MatrixXd P(3, _nfk);

    // Iterate over FK solvers
    for (int i = 0; i < _nfk; ++i) {

      // Extract joint angles
      KDL::JntArray joints;
      for (unsigned int j = 0; j < _chains[i].getNrOfJoints(); ++j) joints(j) = q[j];

      // Compute position
      KDL::Frame pos;
      _fksolvers[i].JntToCart(joints, pos);

      P(0,i) = pos.p.x();
      P(1,i) = pos.p.y();
      P(2,i) = pos.p.z();

    }
    return P;

  }

};
