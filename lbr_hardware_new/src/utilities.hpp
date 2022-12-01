#pragma once
#include <string>
#include <stdexcept>

#include "fri/friClientIf.h"

namespace lbr_hardware
{
    std::string e_session_state_to_string(const KUKA::FRI::ESessionState &state)
    {
        switch (state)
        {
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
            throw std::runtime_error("Reveived unknown state.");
        }
    }

}
