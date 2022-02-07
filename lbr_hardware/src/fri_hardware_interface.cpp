#include <lbr_hardware/fri_hardware_interface.h>

namespace LBR {

FRIHardwareInterface::FRIHardwareInterface() {
    // see KUKA::FRI::LBRClient::createData()
    lbr_data_ = std::make_unique<KUKA::FRI::ClientData>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    lbr_state_ = std::make_shared<KUKA::FRI::LBRState>(&lbr_data_->monitoringMsg);
    lbr_command_ = std::make_shared<KUKA::FRI::LBRCommand>(&lbr_data_->commandMsg, &lbr_data_->monitoringMsg);

    lbr_data_->expectedMonitorMsgID = lbr_state_->LBRMONITORMESSAGEID;
    lbr_data_->commandMsg.header.messageIdentifier = lbr_command_->LBRCOMMANDMESSAGEID;

    buffer_size_ = 0;
}

hardware_interface::return_type FRIHardwareInterface::configure(const hardware_interface::HardwareInfo& system_info) {
    if (this->configure_default(system_info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    
    // state interface references
    hw_position_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    hw_effort_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

    // command interface references
    hw_position_command_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    hw_effort_command_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

    // other hardware parameters
    hw_operation_mode_ = info_.hardware_parameters["operation_mode"];
    hw_port_ = std::stoul(info_.hardware_parameters["port"]);
    info_.hardware_parameters["remote_host"] == "INADDR_ANY" ? hw_remote_host_ = NULL : hw_remote_host_ = info_.hardware_parameters["remote_host"].c_str();

    if (hw_operation_mode_ != "TEST_MODE_1" &&
        hw_operation_mode_ != "TEST_MODE_2" &&
        hw_operation_mode_ != "AUTOMATIC_MODE") {
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Received invalid operation mode: %s.", hw_operation_mode_);
        return hardware_interface::return_type::ERROR;
    }

    if (30200 > hw_port_ || 
        30209 <= hw_port_
    ) {
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Expected port in [30200, 30209]. Found %d.", hw_port_);
        return hardware_interface::return_type::ERROR;
    }

    // for each joint check interfaces
    if (info_.joints.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) { 
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Expected %d joints in URDF. Found %d.", KUKA::FRI::LBRState::NUMBER_OF_JOINTS, info_.joints.size());
        return hardware_interface::return_type::ERROR;
    }

    for (auto& joint: info_.joints) {
        // check state interfaces
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger(FRI_HW_LOGGER),
                "Joint %s received invalid number of state interfaces. Received %d, expected 2.", joint.name.c_str(), joint.state_interfaces.size()
            );
            return hardware_interface::return_type::ERROR;
        }
        for (auto& si: joint.state_interfaces) {
            if (si.name != hardware_interface::HW_IF_POSITION &&
                si.name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(FRI_HW_LOGGER),
                    "Joint %s received invalid state interface: %s. Expected %s or %s",
                    joint.name.c_str(), si.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT
                );
                return hardware_interface::return_type::ERROR;
            }
        }

        // check command interfaces
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger(FRI_HW_LOGGER), 
                "Joint %s received invalid number of command interfaces. Received %d, expected 2.", joint.name.c_str(), joint.command_interfaces.size()
            );
            return hardware_interface::return_type::ERROR;
        };
        for (auto& ci: joint.command_interfaces) {
            if (ci.name != hardware_interface::HW_IF_POSITION &&
                ci.name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(FRI_HW_LOGGER),
                    "Joint %s received invalid command interface: %s. Expected %s or %s.",
                    joint.name.c_str(), ci.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT
                );
                return hardware_interface::return_type::ERROR;
            }
        }
    }

    // command mode tracker
    command_mode_init_ = false;

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> FRIHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        // position interface
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_[i]
        );

        // effort interface
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_[i]
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FRIHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        // position interface
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_command_[i]
        );

        // effort interface
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_command_[i]
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

hardware_interface::return_type FRIHardwareInterface::start() {
    if (connection_.isOpen()) {
        RCLCPP_WARN(rclcpp::get_logger(FRI_HW_LOGGER), "Client application already connected.");
    }

    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Trying to connect to controller with IP %s at port %d.", hw_remote_host_ ? hw_remote_host_ : "INADDR_ANY", hw_port_);
    if (!connection_.open(hw_port_, hw_remote_host_)) {
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Could not connect to controller with IP %s at port %d.", hw_remote_host_ ? hw_remote_host_ : "INADDR_ANY", hw_port_);
        return hardware_interface::return_type::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Connection to controller established.");
    
    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::stop() {
    // RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Disconnecting FRI...");
    // app_.disconnect();
    // RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Done.");

    // command_mode_init_ = false;

    // status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::read() { 
    if (!(this->receive_and_decode() == hardware_interface::return_type::OK)) {
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to receive and decode message.");
        return hardware_interface::return_type::ERROR;
    };

    // app->receive_and_decode()
    // app->callback()

    // get the position and efforts and share them with exposed state interfaces
    // (lbr_state_ shares data with lbr_data_)
    const double* position = lbr_state_->getMeasuredJointPosition();
    hw_position_.assign(position, position+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    const double* effort = lbr_state_->getMeasuredTorque();
    hw_effort_.assign(effort, effort+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

//    // **************************************************************************
//    // callbacks
//    // **************************************************************************
//    // reset commmand message before callbacks
//    lbr_data_->resetCommandMessage();
   
//    // callbacks for robot client
//    ESessionState currentState = (ESessionState)lbr_data_->monitoringMsg.connectionInfo.sessionState;
   
//    if (lbr_data_->lastState != currentState) {
//       _robotClient->onStateChange(_data->lastState, currentState);
//       lbr_data_->lastState = currentState;
//    }
   
//    switch (currentState)
//    {
//       case MONITORING_WAIT:
//       case MONITORING_READY:
//          _robotClient->monitor();
//          break;
//       case COMMANDING_WAIT:
//          _robotClient->waitForCommand();
//          break;
//       case COMMANDING_ACTIVE:
//          _robotClient->command();
//          break;
//       case IDLE:
//       default:
//          return true; // nothing to send back
//    }

//    // callback for transformation client
//    if(_trafoClient != NULL)
//    {
//       _trafoClient->provide();
//    }





    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::write() {
    // // get values from command interfaces and set lbr_command_ with them
    // // (lbr_command_ in turn shares data with lbr_data_)
    // switch (lbr_state_->getClientCommandMode()) {
    //     case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    //         RCLCPP_WARN(rclcpp::get_logger(FRI_HW_LOGGER), "No client command mode available.");
    //         return hardware_interface::return_type::ERROR;
    //     case KUKA::FRI::EClientCommandMode::POSITION:
    //         if (std::isnan(hw_position_command_[0])) { 
    //             lbr_command_->setJointPosition(lbr_state_->getIpoJointPosition());
    //         }
    //         else {
    //             // lbr_command_->setJointPosition(hw_position_command_.data());
    //             lbr_command_->setJointPosition(lbr_state_->getIpoJointPosition());
    //         }
    //         break;
    //     case KUKA::FRI::EClientCommandMode::TORQUE:
    //         if (std::isnan(hw_effort_command_[0])) {
    //             lbr_command_->setTorque(lbr_state_->getMeasuredTorque());
    //         } 
    //         else {
    //             // lbr_command_->setTorque(hw_effort_command_.data());
    //             lbr_command_->setTorque(lbr_state_->getMeasuredTorque());
    //         }
    //         break;
    //     case KUKA::FRI::EClientCommandMode::WRENCH:
    //         RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Wrench command mode not supported through hardware interface.");
    //         lbr_command_->setJointPosition(lbr_state_->getIpoJointPosition());
    //         return hardware_interface::return_type::ERROR;
    //     default:
    //         lbr_command_->setJointPosition(lbr_state_->getIpoJointPosition());
    //         break;
    // }
    lbr_command_->setJointPosition(lbr_state_->getCommandedJointPosition());

    if (!(this->encode_and_send() == hardware_interface::return_type::OK)) {
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to encode and send message.");
        return hardware_interface::return_type::ERROR;
    };

    // app->encode_and_send()

    return hardware_interface::return_type::OK;
}

// // FRI
// void FRIHardwareInterface::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state) {
//     RCLCPP_INFO(
//         rclcpp::get_logger(FRI_HW_LOGGER), 
//         "LBR switched from %s to %s.",
//         fri_e_session_state_to_string_(old_state).c_str(),
//         fri_e_session_state_to_string_(new_state).c_str()
//     );

//     if (new_state == KUKA::FRI::ESessionState::IDLE) {
//         this->stop();
//     }
// }

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

std::string FRIHardwareInterface::fri_e_operation_mode_to_string_(const KUKA::FRI::EOperationMode& mode) {
    switch (mode) {
        case KUKA::FRI::EOperationMode::TEST_MODE_1:
            return "TEST_MODE_1";
        case KUKA::FRI::EOperationMode::TEST_MODE_2:
            return "TEST_MODE_2";
        case KUKA::FRI::EOperationMode::AUTOMATIC_MODE:
            return "AUTOMATIC_MODE";
        default:
            RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Received unknown operation mode.");
            throw std::runtime_error("Received unknown mode.");
    }
}

hardware_interface::return_type FRIHardwareInterface::receive_and_decode() {
    // **************************************************************************
    // Receive and decode new monitoring message, see KUKA::FRI::ClientApplication::step
    // **************************************************************************
    if (!connection_.isOpen()) {
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Connection to controller with IP %s at port %d lost.", hw_remote_host_ ? hw_remote_host_ : "INADDR_ANY", hw_port_);
        return hardware_interface::return_type::ERROR;
    }

    buffer_size_ = connection_.receive(lbr_data_->receiveBuffer, KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE);

    if (buffer_size_ <= 0) {  // TODO: buffer_size_ == 0 -> connection closed (maybe go to IDLE instead of stopping?)
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed while trying to receive monitoring message.");
        return hardware_interface::return_type::ERROR;
    }

    if (!lbr_data_->decoder.decode(lbr_data_->receiveBuffer, buffer_size_)) {
        return hardware_interface::return_type::ERROR;
    }

    // check message type (so that our wrappers match)
    if (lbr_data_->expectedMonitorMsgID != lbr_data_->monitoringMsg.header.messageIdentifier) {
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), 
            "Incompatible IDs for received message, got: %d, expected %d.",
            (int)lbr_data_->monitoringMsg.header.messageIdentifier,
            (int)lbr_data_->expectedMonitorMsgID
        );
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::encode_and_send() {
    // **************************************************************************
    // Encode and send command message, see KUKA::FRI::ClientApplication::step
    // **************************************************************************

    lbr_data_->lastSendCounter++;
    // check if its time to send an answer
    if (lbr_data_->lastSendCounter >= lbr_data_->monitoringMsg.connectionInfo.receiveMultiplier) {
        lbr_data_->lastSendCounter = 0;

        // set sequence counters
        lbr_data_->commandMsg.header.sequenceCounter = lbr_data_->sequenceCounter++;
        lbr_data_->commandMsg.header.reflectedSequenceCounter = lbr_data_->monitoringMsg.header.sequenceCounter;

        if (!lbr_data_->encoder.encode(lbr_data_->sendBuffer, buffer_size_)) {
            RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to encode data.");
            return hardware_interface::return_type::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "trying to send data");
        if (!connection_.send(lbr_data_->sendBuffer, buffer_size_)) {
            RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed while trying to send command message.");
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

} // end of namespace LBR

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    LBR::FRIHardwareInterface,
    hardware_interface::SystemInterface
)
