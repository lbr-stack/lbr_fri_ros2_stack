#pragma once

#include <memory>

#include <fri/friLBRClient.h>
#include <lbr.h>

class LBRClient : public KUKA::FRI::LBRClient {
    public:
        LBRClient(std::shared_ptr<LBR> lbr)
            : lbr_(lbr) {
            
        };

        void monitor() override {
            lbr_->set_state(robotState());
        };

        void command() override {
            switch (robotState().getClientCommandMode()) {
                case KUKA::FRI::POSITION:
                    robotCommand().setJointPosition(lbr_->get_state().getCommandedJointPosition());
                    break;
                case KUKA::FRI::TORQUE:
                    robotCommand().setTorque(lbr_->get_state().getCommandedTorque());
                    break;
                case KUKA::FRI::WRENCH:
                    printf("Wrench command mode not supported./n");
                default:
                    break;
            }
        };

    private:
        std::shared_ptr<LBR> lbr_;
};