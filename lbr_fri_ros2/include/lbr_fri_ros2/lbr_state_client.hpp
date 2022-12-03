#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "fri/friClientIf.h"
#include "fri/friLBRClient.h"
#include "fri/friLBRState.h"

namespace lbr_fri_ros2
{
    class LBRStateClient : public KUKA::FRI::LBRClient
    {
    public:
        LBRStateClient(const std::string &node_name);

        void onStateChange(KUKA::FRI::ESessionState oldstate, KUKA::FRI::ESessionState new_state) override;
        void monitor() override;
        void waitForCommand() override;
        void command() override;

        inline const lbr_fri_msgs::msg::LBRState &lbr_state() const { return lbr_state_; };
        inline lbr_fri_msgs::msg::LBRCommand &lbr_command() { return lbr_command_; };

    protected:
        void lbr_command_cb_(const lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command);
        void publish_lbr_state_();

        void robot_state_to_lbr_state_();

        void init_lbr_command_(lbr_fri_msgs::msg::LBRCommand &lbr_command);
        void init_lbr_state_(lbr_fri_msgs::msg::LBRState &lbr_state);

        void reset_lbr_command_(lbr_fri_msgs::msg::LBRCommand &lbr_command);
        void reset_lbr_state_(lbr_fri_msgs::msg::LBRState &lbr_state);

        bool verify_lbr_command_(const lbr_fri_msgs::msg::LBRCommand &lbr_command);

        std::shared_ptr<rclcpp::Node> node_;

        lbr_fri_msgs::msg::LBRCommand lbr_command_;
        lbr_fri_msgs::msg::LBRState lbr_state_;

        rclcpp::Subscription<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_sub_;
        rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_pub_;
        std::shared_ptr<realtime_tools::RealtimeBuffer<lbr_fri_msgs::msg::LBRCommand>> rt_lbr_command_buf_;
        std::shared_ptr<realtime_tools::RealtimePublisher<lbr_fri_msgs::msg::LBRState>> rt_lbr_state_pub_;

        std::unique_ptr<std::thread> node_thread_;
    };
} // end of namespace lbr_fri_ros2
