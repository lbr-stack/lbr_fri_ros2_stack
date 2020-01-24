#pragma once

#include <mutex>
#include <lbr_msgs/msg/lbr_state.hpp>

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
        inline auto get_current_state() -> const lbr_msgs::msg::LBRState& {
            std::lock_guard<std::mutex> lk(this->current_mutex_);

            return this->current_state_;
        };

        /**
         * @brief Returns the commanded state of this LBR
        **/
        inline auto get_commanded_state() -> const lbr_msgs::msg::LBRState& {
            std::lock_guard<std::mutex> lk(this->commanded_mutex_);
            
            return this->commanded_state_;
        };

        /**
         * @brief Sets the current state of this LBR
        **/
        auto set_current_state(const lbr_msgs::msg::LBRState& s) -> void {
            std::lock_guard<std::mutex> lk(this->current_mutex_);

            this->current_state_ = s;
        };
        /**
         * @brief Sets the commanded state of this LBR
        **/
        auto set_commanded_state(const lbr_msgs::msg::LBRState& s) {
            std::lock_guard<std::mutex> lk(this->commanded_mutex_);

            this->commanded_state_ = s;
        };

    private:
        // current
        std::mutex current_mutex_;
        lbr_msgs::msg::LBRState current_state_;

        // commanded
        std::mutex commanded_mutex_;
        lbr_msgs::msg::LBRState commanded_state_;
};
