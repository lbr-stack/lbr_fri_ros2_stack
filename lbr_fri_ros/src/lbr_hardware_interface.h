#pragma once

#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <fri/friLBRState.h>
#include <lbr.h>

// Hardware interface according to https://github.com/ros-controls/ros_control/wiki/hardware_interface
class LBRHardwareInterface : public hardware_interface::RobotHW
{
public:
    LBRHardwareInterface(std::shared_ptr<LBR> lbr) try :
        lbr_(lbr),
        pos_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.), 
        vel_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.), 
        eff_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.), 
        cmd_pos_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.), 
        cmd_eff_(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.) {

        // Load urdf file from parameter server
        if (!urdf_.initParam("robot_description")) {
            ROS_ERROR("Could not initialize URDF file from robot_description parameter server.");
            throw std::runtime_error("Could not initialize URDF file from robot_description parameter server.");
        };

        // Read joint names from urdf file
        for (auto key : urdf_.joints_) {
            if (urdf_.joints_[key.first]->parent_link_name != "world")
                j_.push_back(key.first);
            else
                continue;
        }

        if (j_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
            ROS_ERROR("Wrong number of joints provided through URDF file from /robot_description.");
            throw std::runtime_error("Wrong number of joints provided through URDF file from robot_description.");
        }
      
        // Register the state handles
        for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
            hardware_interface::JointStateHandle state_handle(j_[i], &pos_[i], &vel_[i], &eff_[i]);
            jsi_.registerHandle(state_handle);
            
            hardware_interface::JointHandle pos_handle(jsi_.getHandle(j_[i]), &cmd_pos_[i]);
            pji_.registerHandle(pos_handle);
            
            hardware_interface::JointHandle eff_handle(jsi_.getHandle(j_[i]), &cmd_eff_[i]);
            eji_.registerHandle(eff_handle);
        }

        registerInterface(&jsi_);
        registerInterface(&pji_);
        registerInterface(&eji_);

        init();
    } catch (std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    };

    auto init() -> void {
        auto state = lbr_->get_current_state();

        time_ = state.stamp.data.toSec();
        pos_ = state.position;
        eff_ = state.torque;
    }

    auto read() -> void {
        auto state = lbr_->get_current_state();

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
        state.position = pos_;
        state.torque = eff_;
        lbr_->set_commanded_state(state);
    };

private:
    std::shared_ptr<LBR> lbr_;
    std::vector<std::string> j_;
    std::vector<double> pos_, vel_, eff_;
    std::vector<double> cmd_pos_, cmd_eff_;
    double time_;

    // Interfaces
    hardware_interface::JointStateInterface jsi_;
    hardware_interface::PositionJointInterface pji_;
    hardware_interface::EffortJointInterface eji_;

    urdf::Model urdf_;
};
