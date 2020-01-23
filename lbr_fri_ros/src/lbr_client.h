#pragma once

#include <memory>

#include <fri/friLBRClient.h>
#include <lbr.h>
#include <lbr_msgs/msg/lbr_state.hpp>

class LBRClient : public KUKA::FRI::LBRClient {
    public:
        LBRClient(std::shared_ptr<LBR> lbr) : lbr_(lbr) {   };

        void monitor() override {
            const double* ja = robotState().getMeasuredJointPosition();
            state_.joint_positions.assign(ja, ja+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
            const double* t = robotState().getMeasuredTorque();
            state_.torques.assign(t, t+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
            state_.time_stamp = robotState().getTimestampNanoSec();

            lbr_->set_current_state(state_);
        };

        void command() override {
            switch (robotState().getClientCommandMode()) {
                case KUKA::FRI::POSITION:
                    if (!lbr_->get_commanded_state().joint_positions.data())
                        robotCommand().setJointPosition(lbr_->get_commanded_state().joint_positions.data());
                    break;
                case KUKA::FRI::TORQUE:
                    if (!lbr_->get_commanded_state().torques.data())
                        robotCommand().setTorque(lbr_->get_commanded_state().torques.data());
                    break;
                case KUKA::FRI::WRENCH:
                    printf("Wrench command mode not supported./n");
                default:
                    break;
            }
        };

    private:
        std::shared_ptr<LBR> lbr_;
        lbr_msgs::msg::LBRState state_;
};
