#ifndef LBR_FRI_ROS2__LBR_CLIENT_HPP_
#define LBR_FRI_ROS2__LBR_CLIENT_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>

#include "fri/friLBRClient.h"
#include "fri/friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr.hpp"

namespace lbr_fri_ros2 {
class LBRClient : public KUKA::FRI::LBRClient {
public:
  LBRClient(std::shared_ptr<LBR> lbr);

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

  // setting the command and getting the state
  inline lbr_fri_msgs::msg::LBRCommand::SharedPtr lbr_command() { return lbr_->command; };
  inline const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state() const { return lbr_->state; };

protected:
  // reset
  bool reset_lbr_command_();

  // pass through from lbr_fri_msgs to robot
  bool robot_state_to_lbr_state_();
  bool lbr_command_to_robot_command_();

  std::string session_state_to_string(const KUKA::FRI::ESessionState &state);

  // shared lbr object
  std::shared_ptr<LBR> lbr_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_CLIENT_HPP_
