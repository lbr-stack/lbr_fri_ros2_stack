#pragma once

#include <mutex>
#include <lbr_msgs/msg/lbr_state.hpp>

// TODO: add tests for 2 threads that access and manipulate the data
//       test size of joints angles and torque
//       test get and set without threads

class LBR {
    public:
        LBR() = default;

        inline auto get_current_state() const -> const lbr_msgs::msg::LBRState& {
            return current_state_;
        };

        inline auto get_commanded_state() const -> const lbr_msgs::msg::LBRState& {
            return commanded_state_;
        };

        auto set_current_state(const lbr_msgs::msg::LBRState& s) -> void {
            std::lock_guard<std::mutex> lk(current_mutex_);

            current_state_ = s;
        };

        auto set_commanded_state(const lbr_msgs::msg::LBRState& s) {
            std::lock_guard<std::mutex> lk(commanded_mutex_);

            commanded_state_ = s;
        };

    private:
        // current
        std::mutex current_mutex_;
        lbr_msgs::msg::LBRState current_state_;

        // commanded
        std::mutex commanded_mutex_;
        lbr_msgs::msg::LBRState commanded_state_;
};
