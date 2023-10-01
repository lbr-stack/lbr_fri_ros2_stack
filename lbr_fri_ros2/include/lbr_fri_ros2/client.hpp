#ifndef LBR_FRI_ROS2__CLIENT_HPP_
#define LBR_FRI_ROS2__CLIENT_HPP_

#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "friLBRClient.h"

#include "lbr_fri_ros2/command_interface.hpp"
#include "lbr_fri_ros2/state_interface.hpp"

namespace lbr_fri_ros2 {
class Client : public KUKA::FRI::LBRClient {
public:
  Client() = delete;
  Client(const rclcpp::Node::SharedPtr node_ptr);

  inline CommandInterface &get_command_interface() { return command_interface_; }
  inline StateInterface &get_state_interface() { return state_interface_; }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override;
  void monitor() override;
  void waitForCommand() override;
  void command() override;

protected:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_ptr_;

  CommandInterface command_interface_;
  StateInterface state_interface_;

  bool open_loop_;

private:
  std::map<int, std::string> KUKA_FRI_STATE_MAP{
      {KUKA::FRI::ESessionState::IDLE, "IDLE"},
      {KUKA::FRI::ESessionState::MONITORING_WAIT, "MONITORING_WAIT"},
      {KUKA::FRI::ESessionState::MONITORING_READY, "MONITORING_READY"},
      {KUKA::FRI::ESessionState::COMMANDING_WAIT, "COMMANDING_WAIT"},
      {KUKA::FRI::ESessionState::COMMANDING_ACTIVE, "COMMANDING_ACTIVE"},
  };
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__CLIENT_HPP_
