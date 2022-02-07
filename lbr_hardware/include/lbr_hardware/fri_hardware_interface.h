#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>

#include <fri/friUdpConnection.h>
#include <fri/friClientData.h>
#include <fri/friLBRState.h>
#include <fri/friLBRCommand.h>

#include <controller_manager/controller_manager.hpp>


namespace LBR {

class FRIHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {

    public:
        FRIHardwareInterface();
        ~FRIHardwareInterface() = default;

        // hardware interface
        hardware_interface::return_type configure(const hardware_interface::HardwareInfo& system_info) override;  // check ros2 control and set status
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;  // not supported in FRI

        hardware_interface::return_type start() override;
        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;
        hardware_interface::return_type write() override;

    private:
        std::string FRI_HW_LOGGER = "FRIHardwareInterface";

        // exposed states
        std::vector<double> hw_position_;      // accessible through FRI
        std::vector<double> hw_effort_;        // accessible through FRI

        // commands
        std::vector<double> hw_position_command_;  // supported by FRI
        std::vector<double> hw_effort_command_;    // supported by FRI

        // FRI
        KUKA::FRI::UdpConnection connection_;

        std::unique_ptr<KUKA::FRI::ClientData> lbr_data_;
        std::shared_ptr<KUKA::FRI::LBRState> lbr_state_;      // points to data in lbr_data_
        std::shared_ptr<KUKA::FRI::LBRCommand> lbr_command_;  // points to data in lbr_data_

        int buffer_size_;  // number of bits received from controller

        std::string hw_operation_mode_;
        std::uint16_t hw_port_;
        const char* hw_remote_host_;

        // track command mode as FRI does not support switches
        bool command_mode_init_;

        // utilities
        std::string fri_e_session_state_to_string_(const KUKA::FRI::ESessionState& state);
        std::string fri_e_operation_mode_to_string_(const KUKA::FRI::EOperationMode& mode);

        hardware_interface::return_type receive_and_decode();  // mimics decoding in KUKA::FRI::ClientApplication::step()
        hardware_interface::return_type encode_and_send();     // mimics encoding in KUKA::FRI::ClientApplication::step()
        void fri_callback();
};

} // end of name space LBR
