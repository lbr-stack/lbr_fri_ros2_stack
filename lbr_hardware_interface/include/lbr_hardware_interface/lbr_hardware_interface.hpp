#pragma once

#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_interface/controller_interface.hpp"

#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_hardware_interface/lbr_hardware_interface_type_values.hpp"

#include "fri/friLBRState.h"

namespace lbr_hardware
{
    class LBRHardwareInterface : public hardware_interface::SystemInterface
    {

    public:
        LBRHardwareInterface() = default;
        ~LBRHardwareInterface();

        // hardware interface
        controller_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override; // check ros2 control and set status
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override; // not supported in FRI

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        template <typename ServiceT>
        bool wait_for_service_(
            const typename rclcpp::Client<ServiceT>::SharedPtr client,
            const uint8_t &attempts = 10, const std::chrono::seconds &timeout = std::chrono::seconds(1));
        bool spawn_clients_();
        bool init_command_interfaces_();
        bool init_state_interfaces_();
        bool verify_number_of_joints_();
        bool verify_joint_command_interfaces_();
        bool verify_joint_state_interfaces_();
        bool verify_sensors_();
        bool connect_();
        bool disconnect_();

        const std::string LBR_HW_LOGGER = "LBRHardwareInterface";

        // node for handling communication
        rclcpp::Node::SharedPtr node_;

        // publisher for sending commands / subscriber to receive goals!

        // clients to switch controllers (to be replaced by ERROR return in read/write,
        // see https://discourse.ros.org/t/ros2-control-controller-restart/24662, https://github.com/ros-controls/ros2_control/issues/674,
        // and https://github.com/ros-controls/ros2_control/pull/677)
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_clt_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_clt_;

        // clients to connect / disconnect from / to LBR
        rclcpp::Client<lbr_fri_msgs::srv::AppConnect>::SharedPtr app_connect_clt_;
        rclcpp::Client<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr app_disconnect_clt_;

        // app connect call request
        std::uint16_t port_id_;
        const char *remote_host_;

        // exposed state interfaces
        double hw_sample_time_;
        int hw_session_state_;
        int hw_connection_quality_;
        int hw_safety_state_;
        int hw_operation_mode_;
        int hw_drive_state_;
        int hw_client_command_mode_;
        int hw_overlay_type_;
        int hw_control_mode_;

        unsigned int hw_time_stamp_sec_;
        unsigned int hw_time_stamp_nano_sec_;

        std::vector<double> hw_position_;
        std::vector<double> hw_commanded_joint_position_;
        std::vector<double> hw_effort_;
        std::vector<double> hw_commanded_torque_;
        std::vector<double> hw_external_torque_;
        std::vector<double> hw_ipo_joint_position_;
        double hw_tracking_performance_;

        // exposed command interfaces
        std::vector<double> hw_position_command_;
        std::vector<double> hw_effort_command_;
        std::vector<double> hw_wrench_command_;
    };
} // end of namespace lbr_hardware
