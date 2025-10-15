#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/types.hpp"

using CallbackReturn =
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using vector = std::array<double, lbr_fri_ros2::N_JNTS>;

/** The effort example controller moves configured joints to a starting position
 * and then in a periodic movement, using a simple regulation of effort.
 */

class EffortExampleController :
    public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration()
  const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration()
  const override;

  controller_interface::return_type update(
    const rclcpp::Time &time,
    const rclcpp::Duration &period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State &previous_state) override;

private:
  // parameters
  std::string robot_name_;
  vector p_gains_{};
  vector d_gains_{};
  vector q_amplitudes_{};
  vector q_periods_{};
  vector effort_max_{};
  double dq_filt_alpha_{};
  double report_period_{};
  double start_time_{};
  vector start_q_{};

  // state variables
  vector q_{};
  vector effort_{};
  vector ext_effort_{};
  vector initial_q_{};

  vector diff_q_{};
  vector dq_{};
  vector dq_filtered_{};
  vector goals_{};

  double elapsed_time_{0.0};
  double last_time_{0.0};

  void updateJointStates();

  bool param_array(vector &data, const char *name);

  rclcpp::Publisher<lbr_fri_idl::msg::LBRState>::SharedPtr state_publisher_;
};

void set_vector(vector &v, const double val) {
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i)
    v[i] = val;
}

controller_interface::InterfaceConfiguration
EffortExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= lbr_fri_ros2::N_JNTS; ++i) {
    config.names.push_back(robot_name_ + "_A" + std::to_string(i) + "/"
                           + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::InterfaceConfiguration
EffortExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= lbr_fri_ros2::N_JNTS; ++i) {
    config.names.push_back(robot_name_ + "_A" + std::to_string(i) + "/"
                           + hardware_interface::HW_IF_POSITION);
    config.names.push_back(robot_name_ + "_A" + std::to_string(i) + "/"
                           + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(robot_name_ + "_A" + std::to_string(i) + "/"
                           + hardware_interface::HW_IF_EFFORT);
    config.names.push_back(
      robot_name_ + "_A" + std::to_string(i) + "/external_torque");
  }
  return config;
}

controller_interface::return_type EffortExampleController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration &period) {
  updateJointStates();
  double dt = period.seconds();
  elapsed_time_ = elapsed_time_ + dt;

  vector efforts;
  vector goals;

  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    dq_filtered_[i] = (1 - dq_filt_alpha_) * dq_filtered_[i]
                      + dq_filt_alpha_ * dq_[i];

    if (elapsed_time_ < start_time_) {
      // goal: interpolated motion from initial to start position
      goals[i] = (elapsed_time_ * start_q_[i] +
                  (start_time_ - elapsed_time_) * initial_q_[i]) /
                 start_time_;
    } else {
      // goal: periodic motion around start position
      goals[i] = start_q_[i] +
                 q_amplitudes_[i] *
                 std::sin(
                   2.0 * M_PI * (elapsed_time_ - start_time_) / q_periods_[i]);
    }

    // speed goal
    double dgoal = (goals[i] - goals_[i]) / dt;

    // computed effort, limited to bounds
    double effort = p_gains_[i] * (goals[i] - q_[i]) +
                    d_gains_[i] * (dgoal - dq_filtered_[i]);

    if (effort > effort_max_[i])
      effort = effort_max_[i];
    if (effort < -effort_max_[i])
      effort = -effort_max_[i];

    efforts[i] = effort;

    command_interfaces_[i].set_value(effort);
  }

  // publish status
  if (report_period_ >= 0.0)
    if (elapsed_time_ - last_time_ > report_period_) {
      last_time_ = elapsed_time_;
      auto message = lbr_fri_idl::msg::LBRState();
      message.commanded_torque = efforts;
      message.measured_torque = effort_;
      message.commanded_joint_position = goals;
      message.measured_joint_position = q_;
      message.external_torque = ext_effort_;
      state_publisher_->publish(message);
    }

  // store goals to compute speed goal next iteration
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i)
    goals_[i] = goals[i];

  return controller_interface::return_type::OK;
}

CallbackReturn EffortExampleController::on_init() {
  try {
    auto_declare<std::string>("robot_name", "lbr");
    auto_declare<double>("dq_filt_alpha", 0.8);
    auto_declare<double>("report_period", 0.1);
    auto_declare<double>("init_time", 10.0);

    auto_declare<std::vector<double> >("p_gains", {});
    auto_declare<std::vector<double> >("d_gains", {});
    auto_declare<std::vector<double> >("amplitudes", {});
    auto_declare<std::vector<double> >("periods", {});
    auto_declare<std::vector<double> >("effort_max", {});
    auto_declare<std::vector<double> >("init_pos", {});
  } catch (const std::exception &e) {
    fprintf(stderr, "on_init: failed: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EffortExampleController::on_configure(
  const rclcpp_lifecycle::State &) {
  // set default values of vector parameters
  set_vector(start_q_, 0.0);
  set_vector(p_gains_, 200.0);
  set_vector(d_gains_, 0.0);
  set_vector(q_amplitudes_, 0.0);
  q_amplitudes_[2] = 0.1;
  q_amplitudes_[3] = 0.1;
  set_vector(q_periods_, 5.0);
  set_vector(effort_max_, 10.0);

  // read parameters
  robot_name_ = get_node()->get_parameter("robot_name").as_string();
  dq_filt_alpha_ = get_node()->get_parameter("dq_filt_alpha").as_double();
  report_period_ = get_node()->get_parameter("report_period").as_double();
  start_time_ = get_node()->get_parameter("init_time").as_double();

  if (param_array(start_q_, "init_pos"))
    return CallbackReturn::FAILURE;

  if (param_array(p_gains_, "p_gains"))
    return CallbackReturn::FAILURE;

  if (param_array(d_gains_, "d_gains"))
    return CallbackReturn::FAILURE;

  if (param_array(q_amplitudes_, "amplitudes"))
    return CallbackReturn::FAILURE;

  if (param_array(q_periods_, "periods"))
    return CallbackReturn::FAILURE;

  if (param_array(effort_max_, "effort_max"))
    return CallbackReturn::FAILURE;

  state_publisher_ = get_node()->create_publisher<
    lbr_fri_idl::msg::LBRState>("controller_state", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn EffortExampleController::on_activate(
  const rclcpp_lifecycle::State &) {
  updateJointStates();

  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    diff_q_[i] = 0.0;
    dq_filtered_[i] = 0.0;
    initial_q_[i] = q_[i];
    goals_[i] = q_[i];
  }

  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

void EffortExampleController::updateJointStates() {
  for (auto i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    const auto &position_interface = state_interfaces_[4 * i];
    const auto &velocity_interface = state_interfaces_[4 * i + 1];
    const auto &effort_interface = state_interfaces_[4 * i + 2];
    const auto &ext_effort_interface = state_interfaces_[4 * i + 3];

    q_[i] = position_interface.get_value();
    dq_[i] = velocity_interface.get_value();
    effort_[i] = effort_interface.get_value();
    ext_effort_[i] = ext_effort_interface.get_value();
  }
}

/// Check and read array of lbr_fri_ros2::N_JNTS double values from the name parameter.
bool EffortExampleController::param_array(vector &data, const char *name) {
  auto arr = get_node()->get_parameter(name).as_double_array();

  if (arr.empty()) // keep default values
    return false;

  if (arr.size() != static_cast<unsigned>(lbr_fri_ros2::N_JNTS)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "parameter %s should be of size %d (not %ld)",
                 name, lbr_fri_ros2::N_JNTS, arr.size());
    return true;
  }

  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i)
    data[i] = arr[i];

  return false;
}

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(EffortExampleController,
                       controller_interface::ControllerInterface)
