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
    KDL::Tree tree; // KDL tree representation for robot
    if (!kdl_parser::treeFromString(urdf, tree))
      return;

    // Create FK solvers
    std::string root_name = tree.getRootSegment()->second.segment.getName();
    std::vector<std::string> link_names;
    for (auto const& seg : tree.getSegments()) {

      // Skip root
      std::string link_name = seg.second.segment.getName();
      if (link_name != root_name) {

	// Get chain
	KDL::Chain chain;
	if (!tree.getChain(root_name, link_name, chain)) {
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

  ~RobotModel() {}

private:

  int _nfk = 0;
  std::vector<KDL::Chain> _chains;
  std::vector<KDL::ChainFkSolverPos_recursive> _fksolvers;


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
