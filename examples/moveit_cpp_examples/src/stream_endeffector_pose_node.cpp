#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>

#include <Eigen/Core>


// types
using COL = std::pair<std::string, std::vector<float>>;
using CSV = std::vector<COL>;

// see https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
CSV read_csv(std::string filename);


class PoseExecutor {
    public:
        PoseExecutor(std::string action_server, std::string planning_group, std::vector<double> initial_position);
        bool executeDeltaToInitialPosition(geometry_msgs::Point point, double time_from_start, double timeout=0.1, bool wait_for_result=false);

    private:
        // members
        std::string _planning_group;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _ac;
        moveit::planning_interface::MoveGroupInterface _group;

        geometry_msgs::PoseStamped _initial_pose_stamped;
        geometry_msgs::PoseStamped _previous_pose_stamped;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "stream_endeffector_pose_node");
    auto nh = ros::NodeHandle();

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string filename, action_server, planning_group;
    nh.getParam("data", filename);
    nh.getParam("action_server", action_server);
    nh.getParam("planning_group", planning_group);

    // load data
    ROS_INFO("Loading pose data...");
    auto data = read_csv(filename);
    ROS_INFO("Done.");

    // init executor
    std::vector<double> initial_position = {0., M_PI/3., 0., -M_PI/3., 0., M_PI/3., 0.};
    PoseExecutor pose_executor(action_server, planning_group, initial_position);
    double previous_time = 0.;
    double time_from_start = 0.;

    ros::Rate rate(int(std::get<1>(data[0]).size()/std::get<1>(data[0])[std::get<1>(data[0]).size() - 1]));

    // execute motion
    for (int i=0; i < std::get<1>(data[0]).size(); i++) {
        ROS_INFO("Executing row %d, computing IK...", i);
        geometry_msgs::Point position;
        position.x = std::get<1>(data[1])[i];
        position.y = std::get<1>(data[2])[i];
        position.z = std::get<1>(data[3])[i];

        time_from_start = std::get<1>(data[0])[i] - previous_time;
        previous_time = std::get<1>(data[0])[i];

        bool success = pose_executor.executeDeltaToInitialPosition(position, time_from_start);
        ROS_INFO("Success: %s", (success ? "true" : "false"));
        rate.sleep();
    }

    spinner.stop();
    ros::shutdown();

    return 0;
}


CSV read_csv(std::string filename) {
    // Reads a CSV file into a vector of <string, vector<float>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, float vector> pairs to store the result
    CSV result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){
            
            // Initialize and add <colname, float vector> pairs to result
            result.push_back({colname, std::vector<float> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        int colIdx = 0;
        
        // Extract each integer
        while(ss >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
};

PoseExecutor::PoseExecutor(std::string action_server, std::string planning_group, std::vector<double> initial_position) 
    : _planning_group(planning_group),
      _ac(action_server), 
      _group(planning_group) {
    ROS_INFO("PoseExecutor: Waiting for server %s...", action_server.c_str());
    _ac.waitForServer();
    ROS_INFO("Done.");

    // initialize pose
    _group.setJointValueTarget(initial_position);
    _group.move();

    _initial_pose_stamped = _group.getCurrentPose();
    _previous_pose_stamped = _initial_pose_stamped;
};

bool PoseExecutor::executeDeltaToInitialPosition(geometry_msgs::Point point, double time_from_start, double timeout, bool wait_for_result) {
    bool success = false;
    if (time_from_start == 0.) {
        return success;
    }

    // see https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html#enforce-planning-in-joint-space
    geometry_msgs::Pose pose = _initial_pose_stamped.pose;
    pose.position.x += point.x;
    pose.position.y += point.y;
    pose.position.z += point.z;

    auto robot_state = *_group.getCurrentState();
    success = robot_state.setFromIK(
        robot_state.getJointModelGroup(_planning_group),
        pose,
        timeout
    );

    Eigen::VectorXd joint_states(_group.getActiveJoints().size());

    if (success) {
        robot_state.copyJointGroupPositions(
            robot_state.getJointModelGroup(_planning_group),
            joint_states
        );
    }

    std::vector<double> q(joint_states.data(), joint_states.data() + joint_states.size());


    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.time_from_start = ros::Duration(time_from_start);
    trajectory_point.positions = q;
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = _group.getJointNames();
    goal.trajectory.points.push_back(trajectory_point);

    _ac.sendGoal(goal);
    if (wait_for_result) _ac.waitForResult();

    return success;
};
