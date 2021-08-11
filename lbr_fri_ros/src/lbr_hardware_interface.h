#pragma once

#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <fri/friLBRState.h>
#include <lbr.h>
#include <lbr_msgs/LBRState.h>

// Hardware interface according to https://github.com/ros-controls/ros_control/wiki/hardware_interface
class LBRHardwareInterface : public hardware_interface::RobotHW
{
public:
    LBRHardwareInterface(std::shared_ptr<LBR> lbr) : lbr_(lbr) {   };

    auto init(ros::NodeHandle& nh) -> bool {
        // LBRState publisher
        state_pub_ = nh.advertise<lbr_msgs::LBRState>("states", 1);

        // Get joints names
        if (!nh.getParam("joints", joint_names_)) {
            ROS_ERROR("Could not get joints from parameter server.");
            return false;
        }

        pos_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        vel_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        eff_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        cmd_pos_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS); 
        cmd_eff_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

        if (joint_names_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
            ROS_ERROR("Number of joints has to equal %d, got %d.", KUKA::FRI::LBRState::NUMBER_OF_JOINTS, (int)joint_names_.size());
            return false;
        }

        // Register the state handles
        for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
            hardware_interface::JointStateHandle state_handle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]);
            jsi_.registerHandle(state_handle);
            
            // Position
            hardware_interface::JointHandle pos_handle(jsi_.getHandle(joint_names_[i]), &cmd_pos_[i]);
            pji_.registerHandle(pos_handle);
            
            // Effort
            hardware_interface::JointHandle eff_handle(jsi_.getHandle(joint_names_[i]), &cmd_eff_[i]);
            eji_.registerHandle(eff_handle);
        }

        this->registerInterface(&jsi_);
        this->registerInterface(&pji_);
        this->registerInterface(&eji_);

        return true;
    }

    auto read() -> void {
        auto state = lbr_->get_current_state();
        state_pub_.publish(state);

        auto period = state.stamp.data.toSec() - time_;
        time_ = state.stamp.data.toSec();

        for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
            vel_[i] = (pos_[i] - state.position[i])/period; 
        }
        pos_ = state.position;
        eff_ = state.torque;
    };

    auto write() -> void {
        lbr_msgs::LBRState state;
        state.position = cmd_pos_;
        state.torque = cmd_eff_;
        lbr_->set_commanded_state(state);
    };

private:
    // Interfaces
    hardware_interface::JointStateInterface jsi_;
    hardware_interface::PositionJointInterface pji_;
    hardware_interface::EffortJointInterface eji_;

    std::vector<std::string> joint_names_;

    // Publishers
    ros::Publisher state_pub_;

    // States
    std::vector<double> pos_, vel_, eff_;
    std::vector<double> cmd_pos_, cmd_eff_;
    double time_;

    // Robot
    std::shared_ptr<LBR> lbr_;
};
