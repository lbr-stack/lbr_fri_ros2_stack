#include "lbr_hardware_new/lbr_state_client.hpp"

#include "utilities.hpp"

namespace lbr_hardware
{
    LBRStateClient::LBRStateClient(const std::string &node_name)
    {
        node_ = std::make_shared<rclcpp::Node>(node_name);

        reset_lbr_state_(lbr_state_);
        reset_lbr_command_(lbr_command_);

        lbr_command_sub_ = node_->create_subscription<lbr_fri_msgs::msg::LBRCommand>(
            "/lbr_command", rclcpp::SystemDefaultsQoS(), std::bind(&LBRStateClient::lbr_command_cb_, this, std::placeholders::_1));
        lbr_state_pub_ = node_->create_publisher<lbr_fri_msgs::msg::LBRState>(
            "/lbr_state", rclcpp::SystemDefaultsQoS());
        rt_lbr_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>>(lbr_state_pub_);
    }

    void LBRStateClient::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state)
    {
        RCLCPP_INFO(node_->get_logger(), "State change from %s to %s.", e_session_state_to_string(old_state).c_str(), e_session_state_to_string(new_state).c_str());
        reset_lbr_state_(lbr_state_);
        reset_lbr_command_(lbr_command_);
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
        robot_state_to_lbr_state_();
        publish_lbr_state_();
    }

    void LBRStateClient::command()
    {
        robot_state_to_lbr_state_();
        KUKA::FRI::LBRClient::command();
        switch (robotState().getClientCommandMode())
        {
        case KUKA::FRI::EClientCommandMode::POSITION:
            if (!std::isnan(lbr_command_.position[0]))
            {
            }
            break;
        case KUKA::FRI::EClientCommandMode::TORQUE:
            if (!std::isnan(lbr_command_.position[0]))
            {
            }
            break;
        case KUKA::FRI::EClientCommandMode::WRENCH:
            if (!std::isnan(lbr_command_.position[0]))
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
        if (lbr_command->e_client_command_mode != robotState().getClientCommandMode())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Expected client command mode %d, got %d.", robotState().getClientCommandMode(),
                lbr_command->e_client_command_mode);
            return;
        }


    }

    void LBRStateClient::publish_lbr_state_()
    {
        if (rt_lbr_state_pub_->trylock())
        {
            rt_lbr_state_pub_->msg_ = lbr_state_;
            rt_lbr_state_pub_->unlockAndPublish();
        }
    }

    void LBRStateClient::lbr_command_to_robot_command_()
    {
    }

    void LBRStateClient::robot_state_to_lbr_state_()
    {
        auto *position = robotState().getMeasuredJointPosition();
        lbr_state_.position.assign(position, position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        auto *torque = robotState().getMeasuredTorque();
        lbr_state_.torque.assign(torque, torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        auto *external_torque = robotState().getExternalTorque();
        lbr_state_.external_torque.assign(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
        lbr_state_.sample_time = robotState().getSampleTime();
        lbr_state_.time_stamp_sec = robotState().getTimestampSec();
        lbr_state_.time_stamp_nano_sec = robotState().getTimestampNanoSec();
        lbr_state_.e_client_command_mode = robotState().getClientCommandMode();
    }

    void LBRStateClient::reset_lbr_command_(lbr_fri_msgs::msg::LBRCommand &lbr_command)
    {
        try
        {
            lbr_command.position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_command.torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
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
            lbr_state.position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
            lbr_state.external_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
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
} // end of namespace lbr_hardware
