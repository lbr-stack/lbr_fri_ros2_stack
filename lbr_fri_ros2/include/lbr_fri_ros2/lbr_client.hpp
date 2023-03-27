#ifndef LBR_FRI_ROS2__LBR_CLIENT_HPP_
#define LBR_FRI_ROS2__LBR_CLIENT_HPP_

#include <memory>
#include <stdexcept>

#include "fri/friClientIf.h"
#include "fri/friLBRClient.h"

#include "lbr_fri_ros2/lbr_intermediary.hpp"

namespace lbr_fri_ros2 {
class LBRClient : public KUKA::FRI::LBRClient {
public:
  LBRClient(const std::shared_ptr<lbr_fri_ros2::LBRIntermediary> lbr_intermediary);

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

protected:
  void zero_command_();
  void buffer_to_command_();
  void state_to_buffer_();

  std::string session_state_to_string_(const KUKA::FRI::ESessionState &state);

  const std::shared_ptr<lbr_fri_ros2::LBRIntermediary> lbr_intermediary_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_CLIENT_HPP_
