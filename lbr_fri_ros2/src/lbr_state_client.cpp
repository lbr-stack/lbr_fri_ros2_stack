#include "lbr_fri_ros2/lbr_state_client.hpp"

#include "utilities.hpp"

namespace lbr_hardware
{
    LBRStateClient::LBRStateClient(const std::string &node_name)
    {
        node_ = std::make_shared<rclcpp::Node>(node_name);

        init_lbr_command_(lbr_command_);
        init_lbr_state_(lbr_state_);

        lbr_command_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRCommand>(
            "/lbr_command", 1, std::bind(&LBRStateClient::lbr_command_cb_, this, std::placeholders::_1));
        lbr_state_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRState>(
            "/lbr_state", 1);
        rt_lbr_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>(lbr_state_pub_);

        auto node_thread = [this]()
        {
            rclcpp::spin(node_);
            reset_lbr_command_(lbr_command_);
            reset_lbr_state_(lbr_state_);
            rclcpp::shutdown();
        };
        node_thread_ = std::make_unique<std::thread>(node_thread);
        node_thread_->detach();
    }

    void LBRStateClient::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state)
    {
        RCLCPP_INFO(node_->get_logger(), "State change from %s to %s.", e_session_state_to_string(old_state).c_str(), e_session_state_to_string(new_state).c_str());
        reset_lbr_command_(lbr_command_);
        reset_lbr_state_(lbr_state_);
    }

    void LBRStateClient::monitor()
    {
        KUKA::FRI::LBRClient::monitor();
        robot_state_to_lbr_state_();
        publish_lbr_state_();
    }

    void LBRStateClient::waitForCommand()
    {
        KUKA::FRI::LBRClient::waitForCommand();
        // maybe set torque / wrenches zero here
        robot_state_to_lbr_state_();
        publish_lbr_state_();
    }

    void LBRStateClient::command()
    {
        robot_state_to_lbr_state_();
        publish_lbr_state_();

        KUKA::FRI::LBRClient::command();

        switch (robotState().getClientCommandMode())
        {
        case KUKA::FRI::EClientCommandMode::POSITION:
            if (!std::isnan(lbr_command_.joint_position[0]))
            {
                robotCommand().setJointPosition(lbr_command_.joint_position.data());
            }
            break;
        case KUKA::FRI::EClientCommandMode::TORQUE:
            if (!std::isnan(lbr_command_.joint_position[0]))
            {
            }
            break;
        case KUKA::FRI::EClientCommandMode::WRENCH:
            if (!std::isnan(lbr_command_.joint_position[0]))
            {
            }
            break;
        default:
            std::string error_msg = "Unknown EClientCommandMode provided.";
            RCLCPP_ERROR(node_->get_logger(), error_msg.c_str());
            throw std::runtime_error(error_msg);
        }
    }

    void LBRStateClient::lbr_command_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command)
    {
        if (lbr_command->client_command_mode != robotState().getClientCommandMode())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Expected client command mode %d, got %d.", robotState().getClientCommandMode(),
                lbr_command->client_command_mode);
            reset_lbr_command_(lbr_command_);
            return;
        }
        lbr_command_ = *lbr_command.get();
    }

    void LBRStateClient::publish_lbr_state_()
    {
        if (rt_lbr_state_pub_->trylock())
        {
            rt_lbr_state_pub_->msg_ = lbr_state_;
            rt_lbr_state_pub_->unlockAndPublish();
        }
    }

    void LBRStateClient::robot_state_to_lbr_state_()
    {
        lbr_state_.sample_time = robotState().getSampleTime();
        lbr_state_.session_state = robotState().getSessionState();
        lbr_state_.connection_quality = robotState().getConnectionQuality();
        lbr_state_.safety_state = robotState().getSafetyState();
        lbr_state_.operation_mode = robotState().getOperationMode();
        lbr_state_.drive_state = robotState().getDriveState();
        lbr_state_.client_command_mode = robotState().getClientCommandMode();
        lbr_state_.overlay_type = robotState().getOverlayType();
        lbr_state_.control_mode = robotState().getControlMode();

        lbr_state_.time_stamp_sec = robotState().getTimestampSec();
        lbr_state_.time_stamp_nano_sec = robotState().getTimestampNanoSec();

        auto *measured_joint_position = robotState().getMeasuredJointPosition();
        lbr_state_.measured_joint_position.assign(measured_joint_position, measured_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

        auto *commanded_joint_position = robotState().getCommandedJointPosition();
        lbr_state_.commanded_joint_position.assign(commanded_joint_position, commanded_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

        auto *measured_torque = robotState().getMeasuredTorque();
        lbr_state_.measured_torque.assign(measured_torque, measured_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

        auto *commanded_torque = robotState().getCommandedTorque();
        lbr_state_.commanded_torque.assign(commanded_torque, commanded_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

        auto *external_torque = robotState().getExternalTorque();
        lbr_state_.external_torque.assign(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

        if (robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
            robotState().getSessionState() == KUKA::FRI::ESessionState::COMMANDING_ACTIVE)
        {
            auto *ipo_joint_position = robotState().getIpoJointPosition();
            lbr_state_.ipo_joint_position.assign(ipo_joint_position, ipo_joint_position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        }

        lbr_state_.tracking_performance = robotState().getTrackingPerformance();
    }

    void LBRStateClient::init_lbr_command_(lbr_fri_msgs::msg::LBRCommand &lbr_command)
    {
        try
        {
            lbr_command.client_command_mode = std::numeric_limits<int>::quiet_NaN();
            lbr_command.joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_command.torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_command.wrench.force.x = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.force.y = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.force.z = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.torque.x = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.torque.y = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.torque.z = std::numeric_limits<double>::quiet_NaN();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), e.what());
            throw std::runtime_error("Failed to initialize lbr_command.");
        }
    }

    void LBRStateClient::init_lbr_state_(lbr_fri_msgs::msg::LBRState &lbr_state)
    {
        try
        {
            lbr_state.sample_time = std::numeric_limits<double>::quiet_NaN();
            lbr_state.session_state = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.connection_quality = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.safety_state = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.operation_mode = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.drive_state = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.client_command_mode = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.overlay_type = std::numeric_limits<int8_t>::quiet_NaN();
            lbr_state.control_mode = std::numeric_limits<int8_t>::quiet_NaN();

            lbr_state.time_stamp_sec = std::numeric_limits<int32_t>::quiet_NaN();
            lbr_state.time_stamp_nano_sec = std::numeric_limits<int32_t>::quiet_NaN();

            lbr_state.measured_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.commanded_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.measured_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.commanded_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.external_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state_.ipo_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state_.tracking_performance = std::numeric_limits<double>::quiet_NaN();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), e.what());
            throw std::runtime_error("Failed to initialize lbr_state.");
        }
    }

    void LBRStateClient::reset_lbr_command_(lbr_fri_msgs::msg::LBRCommand &lbr_command)
    {
        try
        {
            if (lbr_command.joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS &&
                lbr_command.torque.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS)
            {
                throw std::length_error("Found lbr_command of invalid size.");
            }
            lbr_command.client_command_mode = std::numeric_limits<int>::quiet_NaN();
            std::fill(lbr_command.joint_position.begin(), lbr_command.joint_position.end(), std::numeric_limits<double>::quiet_NaN());
            std::fill(lbr_command.torque.begin(), lbr_command.torque.end(), std::numeric_limits<double>::quiet_NaN());
            lbr_command.wrench.force.x = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.force.y = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.force.z = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.torque.x = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.torque.y = std::numeric_limits<double>::quiet_NaN();
            lbr_command.wrench.torque.z = std::numeric_limits<double>::quiet_NaN();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), e.what());
            throw std::runtime_error("Failed to reset lbr_command.");
        }
    }

    void LBRStateClient::reset_lbr_state_(lbr_fri_msgs::msg::LBRState &lbr_state)
    {
        try
        {

            lbr_state.measured_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.commanded_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.measured_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.commanded_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.external_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state_.ipo_joint_position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

            if (lbr_state.measured_joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS ||
                lbr_state.commanded_joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS ||
                lbr_state.measured_torque.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS ||
                lbr_state.commanded_torque.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS ||
                lbr_state.external_torque.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS ||
                lbr_state.ipo_joint_position.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS)
            {
                throw std::length_error("Found lbr_state of invalid size.");
            }
            std::fill(lbr_state.measured_joint_position.begin(), lbr_state.measured_joint_position.end(), std::numeric_limits<double>::quiet_NaN());
            std::fill(lbr_state.ipo_joint_position.begin(), lbr_state.ipo_joint_position.end(), std::numeric_limits<double>::quiet_NaN());
            std::fill(lbr_state.measured_torque.begin(), lbr_state.measured_torque.end(), std::numeric_limits<double>::quiet_NaN());
            std::fill(lbr_state.external_torque.begin(), lbr_state.external_torque.end(), std::numeric_limits<double>::quiet_NaN());
            lbr_state.sample_time = std::numeric_limits<double>::quiet_NaN();
            lbr_state.time_stamp_sec = std::numeric_limits<uint32_t>::quiet_NaN();
            lbr_state.time_stamp_nano_sec = std::numeric_limits<uint32_t>::quiet_NaN();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), e.what());
            throw std::runtime_error("Failed to reset lbr_state.");
        }
    }

    bool LBRStateClient::verify_lbr_command_(const lbr_fri_msgs::msg::LBRCommand &lbr_command)
    {
        // TODO: modify for different control modes
        switch (lbr_command.client_command_mode)
        {
        case KUKA::FRI::EClientCommandMode::POSITION:
            if (std::isnan(lbr_command.joint_position[0]))
            {
                RCLCPP_ERROR(node_->get_logger(), "Attempted to command invalid joint position.");
                return false;
            }
            break;
        case KUKA::FRI::EClientCommandMode::TORQUE:
            /* code */
            break;
        case KUKA::FRI::EClientCommandMode::WRENCH:
            /* code */
            break;
        default:
            std::string error_msg = "Unknown EClientCommandMode provided.";
            RCLCPP_ERROR(node_->get_logger(), error_msg.c_str());
            throw std::runtime_error(error_msg);
        }
        return true;
    }
} // end of namespace lbr_hardware
