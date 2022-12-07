#include "lbr_hardware_interface/lbr_hardware_interface.hpp"

namespace lbr_hardware_interface
{
    LBRHardwareInterface::~LBRHardwareInterface()
    {
        if (!disconnect_())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect from LBR.");
        }
    }

    controller_interface::CallbackReturn LBRHardwareInterface::on_init(const hardware_interface::HardwareInfo &system_info)
    {
        node_ = std::make_shared<rclcpp::Node>("fri_hardware_interface_node");

        auto ret = hardware_interface::SystemInterface::on_init(system_info);
        if (ret != controller_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize SystemInterface.");
            return ret;
        }

        if (!init_command_interfaces_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!init_state_interfaces_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!verify_number_of_joints_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!verify_joint_command_interfaces_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!verify_joint_state_interfaces_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!verify_sensors_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // read port_id and remote_host from lbr.ros2_control.xacro
        port_id_ = std::stoul(info_.hardware_parameters["port_id"]);
        info_.hardware_parameters["remote_host"] == "INADDR_ANY" ? remote_host_ = NULL : remote_host_ = info_.hardware_parameters["remote_host"].c_str();

        if (port_id_ < 30200 ||
            port_id_ > 30209)
        {
            RCLCPP_ERROR(node_->get_logger(), "Expected port_id in [30200, 30209]. Found %d.", port_id_);
            return controller_interface::CallbackReturn::ERROR;
        }

        // TO BE MOVED INTO FUNCTION AND VERIFIED / RESET ETC
        lbr_command_ = std::make_shared<lbr_fri_msgs::msg::LBRCommand>();
        rt_lbr_state_buf_ = std::make_shared<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRState::SharedPtr>>(nullptr);
        lbr_state_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRState>(
            "/lbr_state", rclcpp::SystemDefaultsQoS(), std::bind(&LBRHardwareInterface::lbr_state_cb_, this, std::placeholders::_1));
        lbr_command_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRCommand>(
            "/lbr_command", rclcpp::SystemDefaultsQoS());
        rt_lbr_command_pub_ = std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRCommand>>(lbr_command_pub_);
        // TO BE MOVED

        if (!spawn_clients_())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        auto node_thread = [this]()
        {
            rclcpp::spin(node_);
            rclcpp::shutdown();
        };

        node_thread_ = std::make_unique<std::thread>(node_thread);

        if (!connect_())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to connect to LBR.");
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> LBRHardwareInterface::export_state_interfaces()
    {
        RCLCPP_INFO(node_->get_logger(), "Exporting state interfaces...");
        std::vector<hardware_interface::StateInterface> state_interfaces;

        const auto &lbr_fri_sensor = info_.sensors[0];
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_SAMPLE_TIME, &hw_sample_time_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_SESSION_STATE, &hw_session_state_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_CONNECTION_QUALITY, &hw_connection_quality_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_SAFETY_STATE, &hw_safety_state_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_OPERATION_MODE, &hw_operation_mode_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_DRIVE_STATE, &hw_drive_state_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_CLIENT_COMMAND_MODE, &hw_client_command_mode_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_OVERLAY_TYPE, &hw_overlay_type_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_CONTROL_MODE, &hw_control_mode_);

        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_TIME_STAMP_SEC, &hw_time_stamp_sec_);
        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_TIME_STAMP_NANO_SEC, &hw_time_stamp_nano_sec_);

        for (std::size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_COMMANDED_JOINT_POSITION, &hw_commanded_joint_position_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_COMMANDED_TORQUE, &hw_commanded_torque_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_EXTERNAL_TORQUE, &hw_external_torque_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_IPO_JOINT_POSITION, &hw_ipo_joint_position_[i]);
        }

        state_interfaces.emplace_back(
            lbr_fri_sensor.name, HW_IF_TRACKING_PERFORMANCE, &hw_tracking_performance_);

        RCLCPP_INFO(node_->get_logger(), "Done.");
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> LBRHardwareInterface::export_command_interfaces()
    {
        RCLCPP_INFO(node_->get_logger(), "Exporting command interfaces...");
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (std::size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_command_[i]);

            command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_command_[i]);
        }

        RCLCPP_INFO(node_->get_logger(), "Done.");
        return command_interfaces;
    }

    hardware_interface::return_type LBRHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/)
    {

        // if (!command_mode_init_)
        // {
        //     command_mode_init_ = true;
        //     return hardware_interface::return_type::OK;
        // }
        // else
        // {
        // if (start_interfaces != stop_interfaces)
        // {
        //     RCLCPP_ERROR(
        //         node_->get_logger(),
        //         "FRI does not support command mode switches.");
        //     return hardware_interface::return_type::ERROR;
        // }
        // else
        // {
        //     return hardware_interface::return_type::OK;
        // }
        // }
        return hardware_interface::return_type::OK;
    }

    controller_interface::CallbackReturn LBRHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        if (!connect_())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to connect to robot.");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LBRHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        if (!disconnect_())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect from robot.");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type LBRHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // rt safe buffer
        // |
        // V
        // write into hw if
        RCLCPP_INFO(node_->get_logger(), "read...");
        auto lbr_state = rt_lbr_state_buf_->readFromRT();
        // TODO: logic: write into hw if

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LBRHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // read from hw if
        // |
        // V
        RCLCPP_INFO(node_->get_logger(), "write...");
        // rt safe pub

        return hardware_interface::return_type::OK;
    }

    // TODO: re-load controllers
    // auto re_load_ctrl = [this]() -> void {

    //     // poll current controllers
    //     auto list_ctrl_request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    //     auto list_ctrl_future = list_ctrl_clt_->async_send_request(list_ctrl_request);
    //     if (rclcpp::spin_until_future_complete(node_, list_ctrl_future, std::chrono::milliseconds(100)) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s.", list_ctrl_clt_->get_service_name());
    //         return;
    //     };

    //     std::vector<std::string> controllers;
    //     for (auto& controller: list_ctrl_future.get()->controller) {
    //         RCLCPP_INFO(node_->get_logger(), "Found controller: %s", controller.name.c_str());
    //         controllers.push_back(controller.name);
    //     }

    //     if (controllers.size() == 0) {
    //         RCLCPP_INFO(node_->get_logger(), "No controllers found.");
    //         return;
    //     }

    //     // switch current controllers
    //     auto switch_ctrl_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    //     switch_ctrl_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    //     switch_ctrl_request->start_asap = true;
    //     switch_ctrl_request->stop_controllers = controllers;
    //     switch_ctrl_request->start_controllers = controllers;
    //     auto switch_ctrl_future = switch_ctrl_clt_->async_send_request(switch_ctrl_request);

    //     // wait until switch_ctrl_future
    //     if (rclcpp::spin_until_future_complete(node_, switch_ctrl_future, std::chrono::milliseconds(100)) == rclcpp::FutureReturnCode::SUCCESS) {
    //         if (switch_ctrl_future.get()->ok) {
    //             RCLCPP_INFO(node_->get_logger(), "Re-loaded controllers.");

    //             // set commanded state to current state, see https://github.com/ros-controls/ros2_control/issues/674
    //             hw_position_command_ = hw_position_;
    //             hw_effort_command_ = hw_effort_;
    //             fri_and_controllers_in_sync_ = true;
    //         } else {
    //             RCLCPP_ERROR(node_->get_logger(), "Failed to re-load controllers.");
    //         }
    //     } else {
    //         RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s.", switch_ctrl_clt_->get_service_name());
    //     }
    // };

    // auto re_load_ctrl_thread = std::thread(re_load_ctrl);
    // re_load_ctrl_thread.detach();

    template <typename ServiceT>
    bool LBRHardwareInterface::wait_for_service_(
        const typename rclcpp::Client<ServiceT>::SharedPtr client,
        const uint8_t &attempts, const std::chrono::seconds &timeout)
    {
        bool spawned = false;
        uint8_t attempt = 0;
        while (attempt < attempts && !spawned && rclcpp::ok())
        {
            RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", client->get_service_name());
            spawned = client->wait_for_service(timeout);
            ++attempt;
        }
        return spawned;
    }

    bool LBRHardwareInterface::init_command_interfaces_()
    {
        try
        {
            hw_position_command_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_effort_command_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize command interfaces.\n%s", e.what());
            return false;
        }
        return true;
    }

    bool LBRHardwareInterface::init_state_interfaces_()
    {
        try
        {
            hw_sample_time_ = std::numeric_limits<double>::quiet_NaN();
            hw_session_state_ = std::numeric_limits<double>::quiet_NaN();
            hw_connection_quality_ = std::numeric_limits<double>::quiet_NaN();
            hw_safety_state_ = std::numeric_limits<double>::quiet_NaN();
            hw_operation_mode_ = std::numeric_limits<double>::quiet_NaN();
            hw_drive_state_ = std::numeric_limits<double>::quiet_NaN();
            hw_client_command_mode_ = std::numeric_limits<double>::quiet_NaN();
            hw_overlay_type_ = std::numeric_limits<double>::quiet_NaN();
            hw_control_mode_ = std::numeric_limits<double>::quiet_NaN();

            hw_time_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
            hw_time_stamp_nano_sec_ = std::numeric_limits<double>::quiet_NaN();

            hw_position_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_commanded_joint_position_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_effort_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_commanded_torque_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_external_torque_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_ipo_joint_position_.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            hw_tracking_performance_ = std::numeric_limits<double>::quiet_NaN();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize state interfaces.\n%s", e.what());
            return false;
        }
        return true;
    }

    bool LBRHardwareInterface::verify_number_of_joints_()
    {
        if (info_.joints.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Expected %d joints in URDF. Found %ld.", KUKA::FRI::LBRState::NUMBER_OF_JOINTS, info_.joints.size());
            return false;
        }
        return true;
    }

    bool LBRHardwareInterface::verify_joint_command_interfaces_()
    {
        // check command interfaces
        for (auto &joint : info_.joints)
        {
            if (joint.command_interfaces.size() != LBR_FRI_COMMAND_INTERFACE_SIZE)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Joint %s received invalid number of command interfaces. Received %ld, expected %d.", joint.name.c_str(), joint.command_interfaces.size(), LBR_FRI_COMMAND_INTERFACE_SIZE);
                return false;
            }
            for (auto &ci : joint.command_interfaces)
            {
                if (ci.name != hardware_interface::HW_IF_POSITION &&
                    ci.name != hardware_interface::HW_IF_EFFORT)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Joint %s received invalid command interface: %s. Expected %s or %s.",
                        joint.name.c_str(), ci.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT);
                    return false;
                }
            }
        }
        return true;
    }
    bool LBRHardwareInterface::verify_joint_state_interfaces_()
    {
        // check state interfaces
        for (auto &joint : info_.joints)
        {
            if (joint.state_interfaces.size() != LBR_FRI_STATE_INTERFACE_SIZE)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Joint %s received invalid number of state interfaces. Received %ld, expected %d.", joint.name.c_str(), joint.state_interfaces.size(), LBR_FRI_STATE_INTERFACE_SIZE);
                return false;
            }
            for (auto &si : joint.state_interfaces)
            {
                if (si.name != hardware_interface::HW_IF_POSITION &&
                    si.name != HW_IF_COMMANDED_JOINT_POSITION &&
                    si.name != hardware_interface::HW_IF_EFFORT &&
                    si.name != HW_IF_COMMANDED_TORQUE &&
                    si.name != HW_IF_EXTERNAL_TORQUE &&
                    si.name != HW_IF_IPO_JOINT_POSITION)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Joint %s received invalid state interface: %s. Expected %s, %s, %s, %s, %s or %s",
                        joint.name.c_str(), si.name.c_str(),
                        hardware_interface::HW_IF_POSITION,
                        HW_IF_COMMANDED_JOINT_POSITION,
                        hardware_interface::HW_IF_EFFORT,
                        HW_IF_COMMANDED_TORQUE,
                        HW_IF_EXTERNAL_TORQUE,
                        HW_IF_IPO_JOINT_POSITION);
                    return false;
                }
            }
        }
        return true;
    }

    bool LBRHardwareInterface::verify_sensors_()
    {
        // check lbr specific state interfaces
        if (info_.sensors.size() > 1)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Expected 1 sensor, got %ld", info_.sensors.size());
            return false;
        }

        // check all interfaces are defined in lbr.ros2_control.xacro
        const auto &lbr_fri_sensor = info_.sensors[0];
        if (lbr_fri_sensor.state_interfaces.size() != LBR_FRI_SENSOR_SIZE)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Sensor %s received invalid state interface. Received %ld, expected %d. ",
                lbr_fri_sensor.name.c_str(), lbr_fri_sensor.state_interfaces.size(),
                LBR_FRI_SENSOR_SIZE);
            return false;
        }

        // check only valid interfaces are defined
        for (const auto &si : lbr_fri_sensor.state_interfaces)
        {
            if (
                si.name != HW_IF_SAMPLE_TIME &&
                si.name != HW_IF_SESSION_STATE &&
                si.name != HW_IF_CONNECTION_QUALITY &&
                si.name != HW_IF_SAFETY_STATE &&
                si.name != HW_IF_OPERATION_MODE &&
                si.name != HW_IF_DRIVE_STATE &&
                si.name != HW_IF_CLIENT_COMMAND_MODE &&
                si.name != HW_IF_OVERLAY_TYPE &&
                si.name != HW_IF_CONTROL_MODE &&
                si.name != HW_IF_TIME_STAMP_SEC &&
                si.name != HW_IF_TIME_STAMP_NANO_SEC &&
                si.name != HW_IF_COMMANDED_JOINT_POSITION &&
                si.name != HW_IF_COMMANDED_TORQUE &&
                si.name != HW_IF_EXTERNAL_TORQUE &&
                si.name != HW_IF_IPO_JOINT_POSITION &&
                si.name != HW_IF_TRACKING_PERFORMANCE)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Sensor %s received invalid state interface %s.",
                    lbr_fri_sensor.name.c_str(), si.name.c_str());

                return false;
            }
        }
        return true;
    }

    bool LBRHardwareInterface::spawn_clients_()
    {
        if (!node_)
        {
            RCLCPP_ERROR(node_->get_logger(), "No node provided.");
            return false;
        }

        // list_ctrl_clt_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers", rmw_qos_profile_system_default);
        // if (!wait_for_service_<controller_manager_msgs::srv::ListControllers>(list_ctrl_clt_))
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Failed.");
        //     return false;
        // }
        // RCLCPP_INFO(node_->get_logger(), "Done.");

        // switch_ctrl_clt_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller", rmw_qos_profile_system_default);
        // if (!wait_for_service_<controller_manager_msgs::srv::SwitchController>(switch_ctrl_clt_))
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Failed.");
        //     return false;
        // }
        // RCLCPP_INFO(node_->get_logger(), "Done.");

        // app_connect_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        app_connect_clt_ = node_->create_client<lbr_fri_msgs::srv::AppConnect>("/lbr_app/connect", rmw_qos_profile_system_default);
        if (!wait_for_service_<lbr_fri_msgs::srv::AppConnect>(app_connect_clt_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed.");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Done.");

        app_disconnect_clt_ = node_->create_client<lbr_fri_msgs::srv::AppDisconnect>("/lbr_app/disconnect", rmw_qos_profile_system_default);
        if (!wait_for_service_<lbr_fri_msgs::srv::AppDisconnect>(app_disconnect_clt_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed.");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Done.");
        return true;
    }

    bool LBRHardwareInterface::connect_()
    {
        auto connect_request = std::make_shared<lbr_fri_msgs::srv::AppConnect::Request>();
        connect_request->port_id = port_id_;
        connect_request->remote_host = remote_host_ ? remote_host_ : "";
        auto future = app_connect_clt_->async_send_request(connect_request);
        auto status = future.wait_for(std::chrono::seconds(1));
        if (status != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to request connect service %s.", app_connect_clt_->get_service_name());
            return false;
        }
        auto response = future.get();
        if (!response->connected)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to connect.\n%s", response->message.c_str());
        }
        return response->connected;
    }

    bool LBRHardwareInterface::disconnect_()
    {
        auto disconnect_request = std::make_shared<lbr_fri_msgs::srv::AppDisconnect::Request>();
        auto future = app_disconnect_clt_->async_send_request(disconnect_request);
        auto status = future.wait_for(std::chrono::seconds(1));
        if (status != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to request connect service %s.", app_disconnect_clt_->get_service_name());
            return false;
        }
        auto response = future.get();
        if (!response->disconnected)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to disconnect.\n%s", response->message.c_str());
        }
        return response->disconnected;
    }

    void LBRHardwareInterface::lbr_state_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state)
    {
        RCLCPP_INFO(node_->get_logger(), "cb called");
        rt_lbr_state_buf_->writeFromNonRT(lbr_state);
    }
} // end of namespace lbr_hardware_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    lbr_hardware_interface::LBRHardwareInterface,
    hardware_interface::SystemInterface)
