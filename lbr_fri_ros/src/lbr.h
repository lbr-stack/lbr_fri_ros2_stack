#pragma once

#include <lbr_msgs/LBRState.h>

/**
 * @brief A thread-safe LBR object that represents the real robot
 * 
 * A LBR object is ment to be shared between a LBRClient and a FriRos node, so
 * to allow data exchange between the real robot and ROS2.
**/
class LBR {
    public:
        LBR() = default;

        /**
         * @brief Returns the current state of this LBR
        **/
        inline auto get_current_state() -> const lbr_msgs::LBRState& {
            return this->current_state_;
        };

        /**
         * @brief Returns the commanded state of this LBR
        **/
        inline auto get_commanded_state() -> const lbr_msgs::LBRState& {    
            return this->commanded_state_;
        };

        /**
         * @brief Sets the current state of this LBR
        **/
        auto set_current_state(const lbr_msgs::LBRState& s) -> void {
            this->current_state_ = s;
        };
        /**
         * @brief Sets the commanded state of this LBR
        **/
        auto set_commanded_state(const lbr_msgs::LBRState& s) -> void {
            this->commanded_state_ = s;
        };

    private:
        // current
        lbr_msgs::LBRState current_state_;

        // commanded
        lbr_msgs::LBRState commanded_state_;
};
