#pragma once

#include <mutex>
#include <eigen3/Eigen/Core>

using Joints = Eigen::Matrix<double, uint(KUKA::FRI::LBRState::NUMBER_OF_JOINTS), 1>;

struct State {
    State() = default;
    State(Joints& ja, Joints& jt, uint ts) : joint_angles(ja), joint_torques(jt), time_stamp(ts) { };
    State(const double* ja, const double* jt, uint ts) : joint_angles(ja), joint_torques(jt), time_stamp(ts) {  };
    
    Joints joint_angles = Joints::Zero();
    Joints joint_torques = Joints::Zero();
    uint time_stamp = 0;

};

// TODO: add tests for 2 threads that access and manipulate the data
//       test size of joints angles and torque
//       test get and set without threads
class LBR {
    public:
        LBR() : n_joints_(uint(KUKA::FRI::LBRState::NUMBER_OF_JOINTS)) {    };

        inline auto get_current_state() const -> const State& {
            return current_state_;
        };

        inline auto get_commanded_state() const -> const State& {
            return commanded_state_;
        };

        auto set_current_state(State& s) -> void {
            std::lock_guard<std::mutex> lk(current_mutex_);

            current_state_ = s;
        };

        auto set_commanded_state(State& s) {
            std::lock_guard<std::mutex> lk(commanded_mutex_);

            commanded_state_ = s;
        };

    private:
        uint n_joints_;

        // current
        std::mutex current_mutex_;
        State current_state_;

        // commanded
        std::mutex commanded_mutex_;
        State commanded_state_;
};
