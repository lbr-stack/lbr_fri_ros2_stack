#pragma once

#include <mutex>

#include <fri/friLBRState.h>
#include <fri/friLBRCommand.h>

class LBR {
    public:
        LBR() = default;

        auto inline get_state() -> const KUKA::FRI::LBRState& {
            std::lock_guard<std::mutex> lk(mutex_);
            return lbr_state_;
        };

        auto set_state(const KUKA::FRI::LBRState lbr_state) -> void {
            std::lock_guard<std::mutex> lk(mutex_);
            lbr_state_ = lbr_state;
        };

    private:
        std::mutex mutex_;

        KUKA::FRI::LBRState lbr_state_;
};

// using state = std::tuple<double* /*joint angles*/, double* /*torque*/, uint /*time stamp*/>;

// // TODO: add tests for 2 threads that access and manipulate the data
// //       test size of joints angles and torque
// //       test get and set without threads
// class LBR {
//     public:
//         LBR() {
//             current_joint_angles_ = new double[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
//             current_torques_ = new double[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

//             commanded_joint_angles_ = new double[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
//             commanded_torques_ = new double[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
//         };

//         ~LBR() {
//             delete[] current_joint_angles_;
//             delete[] current_torques_;

//             delete[] commanded_joint_angles_;
//             delete[] commanded_torques_;
//         }

//         inline auto get_current_state() const -> const state {
//             return std::make_tuple(current_joint_angles_, current_torques_, current_time_stamp_);
//         };

//         inline auto get_commanded_state() const -> const state {
//             return std::make_tuple(commanded_joint_angles_, commanded_torques_, commanded_time_stamp_);
//         };

//         auto set_current_state(const double* joint_angles, const double* torques, const int time_stamp) -> void {
//             std::lock_guard<std::mutex> lk(current_mutex_);
            
//             for (uint i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
//                 current_joint_angles_[i] = joint_angles[i];
//                 current_torques_[i] = torques[i];
//             }

//             current_time_stamp_ = time_stamp;
//         };

//         auto set_commanded_state(const double* joint_angles, const double* torques, const int time_stamp) {
//             std::lock_guard<std::mutex> lk(commanded_mutex_);

//             for (uint i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++) {
//                 commanded_joint_angles_[i] = joint_angles[i];
//                 commanded_torques_[i] = torques[i];
//             }

//             commanded_time_stamp_ = time_stamp;
//         };

//     private:
//         // current
//         std::mutex current_mutex_;
//         double* current_joint_angles_;
//         double* current_torques_;
//         uint current_time_stamp_;

//         // commanded
//         std::mutex commanded_mutex_;
//         double* commanded_joint_angles_;
//         double* commanded_torques_;
//         uint commanded_time_stamp_;
// };