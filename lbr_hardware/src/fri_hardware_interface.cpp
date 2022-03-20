#include <lbr_hardware/fri_hardware_interface.hpp>


namespace LBR {

FRIHardwareInterface::~FRIHardwareInterface() {
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Disconnecting FRI on destruct...");
    app_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Done.");
}

hardware_interface::return_type FRIHardwareInterface::configure(const hardware_interface::HardwareInfo& system_info) {
    if (this->configure_default(system_info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    // services for switching controllers on reset
    node_ = std::make_shared<rclcpp::Node>("fri_hardware_interface_node");
    list_ctrl_clt_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
    switch_ctrl_clt_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    fri_and_controllers_in_sync_ = false;

    // state interface references
    hw_position_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    hw_effort_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    hw_external_torque_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

    // lbr specific states
    hw_sample_time_ = std::numeric_limits<double>::quiet_NaN();
    hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
    hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();

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
        return hardware_interface::return_type::ERROR;
    }

    // check lbr specific state interfaces
    if (info_.sensors.size() > 1) {
        RCLCPP_FATAL(
            rclcpp::get_logger(FRI_HW_LOGGER),
            "Expected 1 sensor, got %d", info_.sensors.size()
        );
    }
    
    auto sensor = info_.sensors[0];
    if (sensor.state_interfaces.size() != 3) {
        RCLCPP_FATAL(
            rclcpp::get_logger(FRI_HW_LOGGER),
            "Sensor %s received invalid state interface. Received %d, expected 3. ", sensor.name.c_str(), sensor.state_interfaces.size()
        );
    }

    for (auto& si: sensor.state_interfaces) {
        if (si.name != LBR::HW_IF_SAMPLE_TIME &&
            si.name != LBR::HW_IF_TIME_STAMP_SEC &&
            si.name != LBR::HW_IF_TIME_STAMP_NANO_SEC) {
            RCLCPP_FATAL(
                rclcpp::get_logger(FRI_HW_LOGGER),
                "Sensor %s received invalid state interface %s. Expected %s, %s, or %s.",
                sensor.name.c_str(), si.name.c_str(), LBR::HW_IF_SAMPLE_TIME, LBR::HW_IF_TIME_STAMP_NANO_SEC, LBR::HW_IF_TIME_STAMP_SEC
            );
        }
    }

    // for each joint check interfaces
    if (info_.joints.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) { 
        RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "Expected %d joints in URDF. Found %d.", KUKA::FRI::LBRState::NUMBER_OF_JOINTS, info_.joints.size());
        return hardware_interface::return_type::ERROR;
    }

    for (auto& joint: info_.joints) {
        // check state interfaces
        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(
                rclcpp::get_logger(FRI_HW_LOGGER),
                "Joint %s received invalid number of state interfaces. Received %d, expected 3.", joint.name.c_str(), joint.state_interfaces.size()
            );
            return hardware_interface::return_type::ERROR;
        }
        for (auto& si: joint.state_interfaces) {
            if (si.name != hardware_interface::HW_IF_POSITION &&
                si.name != hardware_interface::HW_IF_EFFORT &&
                si.name != LBR::HW_IF_EXTERNAL_TORQUE) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(FRI_HW_LOGGER),
                    "Joint %s received invalid state interface: %s. Expected %s, %s, or %s",
                    joint.name.c_str(), si.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT, LBR::HW_IF_EXTERNAL_TORQUE
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

    // lbr specific state interfaces
    auto sensor = info_.sensors[0];
    state_interfaces.emplace_back(
        sensor.name, LBR::HW_IF_SAMPLE_TIME, &hw_sample_time_ 
    );
    state_interfaces.emplace_back(
        sensor.name, LBR::HW_IF_TIME_STAMP_SEC, &hw_time_stamp_sec_
    );
    state_interfaces.emplace_back(
        sensor.name, LBR::HW_IF_TIME_STAMP_NANO_SEC, &hw_time_stamp_nano_sec_
    );

    // other interfaces
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        // position interface
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_[i]
        );

        // effort interface
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_[i]
        );

        // external torque interface (lbr specififc)
        state_interfaces.emplace_back(
            info_.joints[i].name, LBR::HW_IF_EXTERNAL_TORQUE, &hw_external_torque_[i]
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
    app_.connect(hw_port_, hw_remote_host_);    
    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::stop() {
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Disconnecting FRI on stop...");
    app_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Done.");

    command_mode_init_ = false;

    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FRIHardwareInterface::read() {
    // read incoming data from controller
    if (!app_.receiveAndDecode()) {
        RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to receive and decode data from controller.");
        return hardware_interface::return_type::ERROR;  
    };

    // get lbr specific states
    hw_sample_time_ = robotState().getSampleTime();
    hw_time_stamp_sec_ = robotState().getTimestampSec();
    hw_time_stamp_nano_sec_ = robotState().getTimestampNanoSec();
 
    // get the position and efforts and share them with exposed state interfaces
    const double* position = robotState().getMeasuredJointPosition();
    hw_position_.assign(position, position+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    const double* effort = robotState().getMeasuredTorque();
    hw_effort_.assign(effort, effort+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    const double* external_torque = robotState().getExternalTorque();
    hw_external_torque_.assign(external_torque, external_torque+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

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

    if (new_state == KUKA::FRI::ESessionState::IDLE) {
        RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "FRI stoppped. Shutting down.");
        this->stop();
        rclcpp::shutdown();
    }

    if (old_state == KUKA::FRI::COMMANDING_ACTIVE) {
        fri_and_controllers_in_sync_ = false;  // log potential out of synch
    }

    // on state change, reset controllers
    if (new_state == KUKA::FRI::COMMANDING_WAIT) {  // NOT REALTIME SAFE, CONNECTION WILL BE LOST, TODO: asynch re-load
        auto re_load_ctrl = [this]() {

            // poll current controllers
            auto list_ctrl_request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();       
            auto list_ctrl_future = list_ctrl_clt_->async_send_request(list_ctrl_request);
            rclcpp::spin_until_future_complete(node_, list_ctrl_future);

            std::vector<std::string> controllers;
            for (auto& controller: list_ctrl_future.get()->controller) {
                RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "%s", controller.name.c_str());
                controllers.push_back(controller.name);
            }

            // switch current controllers
            auto switch_ctrl_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            switch_ctrl_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
            switch_ctrl_request->start_asap = true;
            switch_ctrl_request->stop_controllers = controllers;
            switch_ctrl_request->start_controllers = controllers;
            auto switch_ctrl_future = switch_ctrl_clt_->async_send_request(switch_ctrl_request);

            // wait until switch_ctrl_future
            if (rclcpp::spin_until_future_complete(node_, switch_ctrl_future) == rclcpp::FutureReturnCode::SUCCESS) {
                if (switch_ctrl_future.get()->ok) {
                    RCLCPP_INFO(rclcpp::get_logger(FRI_HW_LOGGER), "Re-loaded controllres.");
                    fri_and_controllers_in_sync_ = true;
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to re-load controllers.");
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Failed to call service %s.", switch_ctrl_clt_->get_service_name());
            }
        };

        auto re_load_ctrl_thread = std::thread(re_load_ctrl);
        re_load_ctrl_thread.detach();
    }
}

void FRIHardwareInterface::waitForCommand() {
    KUKA::FRI::LBRClient::waitForCommand();

    switch (robotState().getClientCommandMode()) {
        case KUKA::FRI::EClientCommandMode::TORQUE:
            // < 5ms
            robotCommand().setTorque(JOINT_ZEROS.data());
            break;
        case KUKA::FRI::EClientCommandMode::WRENCH:
            // <= 5ms, cartesian_impedance_control
            robotCommand().setWrench(WRENCH_ZEROS.data());
            break;
        default:
            break;
    }
}

void FRIHardwareInterface::command() {
    switch (robotState().getClientCommandMode()) {
        case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
            RCLCPP_FATAL(rclcpp::get_logger(FRI_HW_LOGGER), "No client command mode available.");
            break;
        case KUKA::FRI::EClientCommandMode::POSITION:
            if (std::isnan(hw_position_command_[0]) || !fri_and_controllers_in_sync_) {
                KUKA::FRI::LBRClient::command();
            }
            else {
                robotCommand().setJointPosition(hw_position_command_.data()); // control manager stores internal value, which needs to be updated
            }
            break;
        case KUKA::FRI::EClientCommandMode::TORQUE:
            if (std::isnan(hw_position_command_[0]) || !fri_and_controllers_in_sync_) {
                KUKA::FRI::LBRClient::command();
                robotCommand().setTorque(JOINT_ZEROS.data());
            } 
            else {
                robotCommand().setJointPosition(hw_position_command_.data());
                robotCommand().setTorque(JOINT_ZEROS.data());
            }
            break;
        case KUKA::FRI::EClientCommandMode::WRENCH:
            if (std::isnan(hw_position_command_[0]) || !fri_and_controllers_in_sync_) {
                KUKA::FRI::LBRClient::command();
                robotCommand().setWrench(WRENCH_ZEROS.data());
            } 
            else {
                robotCommand().setJointPosition(hw_position_command_.data());
                robotCommand().setWrench(WRENCH_ZEROS.data());
            }
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger(FRI_HW_LOGGER), "Unkown KUKA::FRI::EClientCommandMode provided.");
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
