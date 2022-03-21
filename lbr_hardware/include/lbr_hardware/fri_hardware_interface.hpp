#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <fri/friUdpConnection.h>
#include <fri/friLBRClient.h>

#include <fri_hardware_interface_client_application.hpp>
#include <fri_hardware_interface_type_values.hpp>


namespace LBR {

class FRIHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>, KUKA::FRI::LBRClient {

    public:
        FRIHardwareInterface() : app_(connection_, *this) { };
        ~FRIHardwareInterface();

        // hardware interface
        hardware_interface::return_type configure(const hardware_interface::HardwareInfo& system_info) override;  // check ros2 control and set status
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;  // not supported in FRI

        hardware_interface::return_type start() override;
        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;
        hardware_interface::return_type write() override;

        // FRI
        void onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state) override;
        void waitForCommand() override;
        void command() override;

    private:
        const std::string FRI_HW_LOGGER = "FRIHardwareInterface";
        const std::vector<double> JOINT_ZEROS = std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.);
        const std::vector<double> WRENCH_ZEROS = std::vector<double>(6, 0.);

        // exposed states
        std::vector<double> hw_position_;         // accessible through FRI
        std::vector<double> hw_effort_;           // accessible through FRI
        std::vector<double> hw_external_torque_;  // accessible through FRI

        // FRI specific state interfaces, see KUKA::FRI::LBRState
        double hw_sample_time_;                   // accessible through FRI
        double hw_time_stamp_sec_;                // accessible through FRI
        double hw_time_stamp_nano_sec_;           // accessible through FRI

        // commands
        std::vector<double> hw_position_command_;  // supported by FRI
        std::vector<double> hw_effort_command_;    // supported by FRI

        // services to switch controllers (to be replaced by ERROR return in read/write,
        // see https://discourse.ros.org/t/ros2-control-controller-restart/24662, https://github.com/ros-controls/ros2_control/issues/674,
        // and https://github.com/ros-controls/ros2_control/pull/677)
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_clt_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_clt_;

        // FRI and controller synchronization tracker
        bool fri_and_controllers_in_sync_;

        // FRI
        KUKA::FRI::UdpConnection connection_;
        KUKA::FRI::FRIHardwareInterfaceClientApplication app_;

        std::uint16_t hw_port_;
        const char* hw_remote_host_;

        // track command mode as FRI does not support switches
        bool command_mode_init_;

        // utilities
        std::string fri_e_session_state_to_string_(const KUKA::FRI::ESessionState& state);
};

} // end of name space LBR
