#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <fri/friLBRClient.h>
#include <lbr.h>
#include <lbr_msgs/msg/lbr_state.hpp>

class LBRClient : public KUKA::FRI::LBRClient {
    public:
        LBRClient(std::shared_ptr<LBR> lbr) : lbr_(lbr) {   };

        auto onStateChange(KUKA::FRI::ESessionState old_mode, KUKA::FRI::ESessionState new_mode) -> void override {
            printf("LBR switched from %s to %s.\n", translate(old_mode).c_str(), translate(new_mode).c_str());
        };

        auto monitor() -> void override {
            update_state();
        };

        auto waitForCommand() -> void override {
            update_state();
            KUKA::FRI::LBRClient::waitForCommand();
        };

        auto command() -> void override {
            update_state();

            switch (robotState().getClientCommandMode()) {
                case KUKA::FRI::POSITION:
                    if (!lbr_->get_commanded_state().joint_positions.empty())
                        robotCommand().setJointPosition(lbr_->get_commanded_state().joint_positions.data());
                    else
                        KUKA::FRI::LBRClient::command();
                    break;
                case KUKA::FRI::TORQUE:
                    if (!lbr_->get_commanded_state().torques.empty())
                        robotCommand().setTorque(lbr_->get_commanded_state().torques.data());
                    else
                        KUKA::FRI::LBRClient::command();
                    break;
                case KUKA::FRI::WRENCH:
                    printf("Wrench command mode not supported.\n");
                default:
                    break;
            }
        };

        auto update_state() -> void {
            const double* ja = robotState().getMeasuredJointPosition();
            state_.joint_positions.assign(ja, ja+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
            const double* t = robotState().getMeasuredTorque();
            state_.torques.assign(t, t+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
            state_.time_stamp = robotState().getTimestampNanoSec();

            lbr_->set_current_state(state_);
        }

        auto translate(int mode) -> std::string {
            switch (mode) {
                case KUKA::FRI::ESessionState::IDLE:
                    return "idle";
                case KUKA::FRI::ESessionState::MONITORING_WAIT:
                    return "monitoring wait";
                case KUKA::FRI::ESessionState::MONITORING_READY:
                    return "monitoring ready";
                case KUKA::FRI::ESessionState::COMMANDING_WAIT:
                    return "commanding wait";
                case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
                    return "commanding active";
                default:
                    return "undefined";
            }
        }

    private:
        std::shared_ptr<LBR> lbr_;
        lbr_msgs::msg::LBRState state_;
};
