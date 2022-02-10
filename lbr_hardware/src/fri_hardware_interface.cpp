#include <lbr_hardware/fri_hardware_interface.h>


namespace LBR {

CallbackReturn FRIHardwareInterface::on_init(const hardware_interface::HardwareInfo & system_info) {
    if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    
    // state interface references
    hw_position_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    hw_effort_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

    // command interface references
    hw_position_command_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    hw_effort_command_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

    // other hardware parameters
    hw_port_ = std::stoul(info_.hardware_parameters["port"]);
    info_.hardware_parameters["remote_host"] == "INADDR_ANY" ? hw_remote_host_ = NULL : hw_remote_host_ = info_.hardware_parameters["remote_host"].c_str();

    if (30200 > hw_port_ || 
        30209 <= hw_port_
    ) {
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Expected port in [30200, 30209]. Found %d.", hw_port_);
        return CallbackReturn::ERROR;
    }

    // for each joint check interfaces
    if (info_.joints.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) { 
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Expected %d joints in URDF. Found %ld.", KUKA::FRI::LBRState::NUMBER_OF_JOINTS, info_.joints.size());
        return CallbackReturn::ERROR;
    }

    for (auto& joint: info_.joints) {
        // check state interfaces
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger(FRI_HW_LOGGER),
                "Joint %s received invalid number of state interfaces. Received %ld, expected 2.", joint.name.c_str(), joint.state_interfaces.size()
            );
            return CallbackReturn::ERROR;
        }
        for (auto& si: joint.state_interfaces) {
            if (si.name != "position" &&
                si.name != "effort") {
                RCLCPP_FATAL(
                    rclcpp::get_logger(FRI_HW_LOGGER),
                    "Joint %s received invalid state interface: %s. Expected %s or %s",
                    joint.name.c_str(), si.name.c_str(), "position", "effort"
                );
                return CallbackReturn::ERROR;
            }
        }

        // check command interfaces
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger(FRI_HW_LOGGER), 
                "Joint %s received invalid number of command interfaces. Received %ld, expected 2.", joint.name.c_str(), joint.command_interfaces.size()
            );
            return CallbackReturn::ERROR;
        };
        for (auto& ci: joint.command_interfaces) {
            if (ci.name != "position" &&
                ci.name != "effort") {
                RCLCPP_FATAL(
                    rclcpp::get_logger(FRI_HW_LOGGER),
                    "Joint %s received invalid command interface: %s. Expected %s or %s.",
                    joint.name.c_str(), ci.name.c_str(), "position", "effort"
                );
                return CallbackReturn::ERROR;
            }
        }
    }

    // command mode tracker
    command_mode_init_ = false;

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FRIHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        // position interface
        state_interfaces.emplace_back(
            info_.joints[i].name, "position", &hw_position_[i]
        );

        // effort interface
        state_interfaces.emplace_back(
            info_.joints[i].name, "effort", &hw_effort_[i]
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FRIHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        // position interface
        command_interfaces.emplace_back(
            info_.joints[i].name, "position", &hw_position_command_[i]
        );

        // effort interface
        command_interfaces.emplace_back(
            info_.joints[i].name, "effort", &hw_effort_command_[i]
        );
    }

    return command_interfaces;
}

hardware_interface::return_type FRIHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {

    if (!command_mode_init_) {
        command_mode_init_ = true;
        return hardware_interface::return_type::OK;
    }
    else {
        if (start_interfaces != stop_interfaces) {
            RCLCPP_ERROR(
                rclcpp::get_logger(FRI_HW_LOGGER),
                "FRI does not support command mode switches."
            );
            return hardware_interface::return_type::ERROR;
        }
        else {
            return hardware_interface::return_type::OK;
        }
    }
}

CallbackReturn FRIHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    app_.connect(hw_port_, hw_remote_host_);    
    return CallbackReturn::SUCCESS;
}

CallbackReturn FRIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)  {
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Disconnecting FRI...");
    app_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Done.");

    command_mode_init_ = false;

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FRIHardwareInterface::read() {
    // read incoming data from controller
    if (!app_.receiveAndDecode()) {
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to receive and decode data from controller.");
        return hardware_interface::return_type::ERROR;  
    };

    // get the position and efforts and share them with exposed state interfaces
    const double* position = robotState().getMeasuredJointPosition();
    hw_position_.assign(position, position+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    const double* effort = robotState().getMeasuredTorque();
    hw_effort_.assign(effort, effort+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::write() {
    // callback callst LBRClient's (this) method, e.g. onStateChange(), command()
    app_.callback();
    if (!app_.encodeAndSend()) {
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to encode and send data to controller.");
        return hardware_interface::return_type::ERROR;
    };

    return hardware_interface::return_type::OK;
}

// FRI
void FRIHardwareInterface::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state) {
    RCLCPP_INFO(
        rclcpp::get_logger(FRI_HW_LOGGER), 
        "LBR switched from %s to %s.",
        fri_e_session_state_to_string_(old_state).c_str(),
        fri_e_session_state_to_string_(new_state).c_str()
    );

    if (old_state == KUKA::FRI::ESessionState::COMMANDING_ACTIVE) {
        RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Controller manager does not allow resets. Command interfaces can't be updated to current state. Shutting down.");
        //this->on_deactivate(new_state);
        rclcpp::shutdown();
    }

    if (new_state == KUKA::FRI::ESessionState::IDLE) {
        RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "FRI stoppped. Shutting down.");
      //  this->on_deactivate(new_state);
        rclcpp::shutdown();
    }
}

void FRIHardwareInterface::command() {
    switch (robotState().getClientCommandMode()) {
        case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
            RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "No client command mode available.");
            break;
        case KUKA::FRI::EClientCommandMode::POSITION:
            if (std::isnan(hw_position_command_[0])) { 
                KUKA::FRI::LBRClient::command();
            }
            else {
                robotCommand().setJointPosition(hw_position_command_.data()); // control manager stores internal value, which needs to be updated
            }
            break;
        case KUKA::FRI::EClientCommandMode::TORQUE:
            if (std::isnan(hw_effort_command_[0])) {
                KUKA::FRI::LBRClient::command();
            } 
            else {
                robotCommand().setTorque(hw_effort_command_.data());
            }
            break;
        case KUKA::FRI::EClientCommandMode::WRENCH:
            RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Wrench command mode not supported through hardware interface.");
            KUKA::FRI::LBRClient::command();
            break;
        default:
            KUKA::FRI::LBRClient::command();
            break;
    }
}

std::string FRIHardwareInterface::fri_e_session_state_to_string_(const KUKA::FRI::ESessionState& state) {
    switch (state) {
        case KUKA::FRI::ESessionState::IDLE:
            return "IDLE";
        case KUKA::FRI::ESessionState::MONITORING_WAIT:
            return "MONITORING_WAIT";
        case KUKA::FRI::ESessionState::MONITORING_READY:
            return "MONITORING_READY";
        case KUKA::FRI::ESessionState::COMMANDING_WAIT:
            return "COMMANDING_WAIT";
        case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
            return "COMMANDING_ACTIVE";
        default:
            RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Received unknown state.");
            throw std::runtime_error("Reveived unknown state.");
    }
}

} // end of namespace LBR

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    LBR::FRIHardwareInterface,
    hardware_interface::SystemInterface
)
