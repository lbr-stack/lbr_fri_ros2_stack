#include <lbr_state_broadcaster/lbr_state_broadcaster.hpp>


namespace lbr_state_broadcaster {

controller_interface::return_type
LBRStateBroadcaster::init(const std::string& controller_name) {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }

    try { // these parameters can be parsed via the YAML configurations
        use_local_topics_ = this->auto_declare<bool>("use_local_topics", false);
        lbr_state_topic_ = this->auto_declare<std::string>("lbr_state_topic", "lbr_state");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->node_->get_logger(), "Failed to declare parameters.\n%s", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
LBRStateBroadcaster::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE
    };
}

controller_interface::InterfaceConfiguration
LBRStateBroadcaster::state_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::ALL
    };
}

controller_interface::return_type 
LBRStateBroadcaster::update() {
    try {
        lbr_state_.header.stamp = this->node_->now();
        lbr_state_.sample_time = this->sample_time_interface_ptr_->get_value();
        lbr_state_.time_stamp_sec = this->time_stamp_sec_interface_ptr_->get_value();
        lbr_state_.time_stamp_nano_sec = this->time_stamp_nano_sec_interface_ptr_->get_value();

        for (int i=0; i<KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i) {
            this->lbr_state_.position[i] = this->position_interfaces_[i].get().get_value();
            this->lbr_state_.torque[i] = this->effort_interfaces_[i].get().get_value();
            this->lbr_state_.external_torque[i] = this->external_torque_interfaces_[i].get().get_value();
        }

        if (this->realtime_lbr_state_publisher_->trylock()) {
            this->realtime_lbr_state_publisher_->msg_ = lbr_state_;
            this->realtime_lbr_state_publisher_->unlockAndPublish();
        }
    } catch (const std::exception& e) {
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LBRStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/
) {
    try { // initialize realtime publishers of lbr_state_msgs::msg::LBRState
        std::string topic_prefix = this->use_local_topics_ ? "~/" : "";
        this->lbr_state_publisher_ = rclcpp::create_publisher<lbr_state_msgs::msg::LBRState>(
            this->node_, topic_prefix + this->lbr_state_topic_, rclcpp::SystemDefaultsQoS()
        );
        this->realtime_lbr_state_publisher_ = std::make_shared<
            realtime_tools::RealtimePublisher<lbr_state_msgs::msg::LBRState>
        >(this->lbr_state_publisher_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->node_->get_logger(), "Failed to initialize publishers.\n%s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    lbr_state_.name.reserve(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    lbr_state_.position.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    lbr_state_.torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());
    lbr_state_.external_torque.resize(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, std::numeric_limits<double>::quiet_NaN());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LBRStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/
) {
    try { // initialize references to the state interfaces for simplicity
        for (auto& state_interface: this->state_interfaces_) {
            if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
                this->position_interfaces_.emplace_back(
                    std::ref(state_interface)
                );
                this->joint_names_.emplace_back(
                    state_interface.get_name()
                );
                continue;
            }
            if (state_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
                this->effort_interfaces_.emplace_back(
                    std::ref(state_interface)
                );
                continue;
            }
            if (state_interface.get_interface_name() == LBR::HW_IF_EXTERNAL_TORQUE) {
                this->external_torque_interfaces_.emplace_back(
                    std::ref(state_interface)
                );
                continue;
            }
            if (state_interface.get_interface_name() == LBR::HW_IF_SAMPLE_TIME) {
                this->sample_time_interface_ptr_ = std::make_unique<
                    hardware_interface::LoanedStateInterface>(std::move(state_interface));
                continue;
            }
            if (state_interface.get_interface_name() == LBR::HW_IF_TIME_STAMP_SEC) {
                this->time_stamp_sec_interface_ptr_ = std::make_unique<
                    hardware_interface::LoanedStateInterface>(std::move(state_interface));
                continue;
            }
            if (state_interface.get_interface_name() == LBR::HW_IF_TIME_STAMP_NANO_SEC) {
                this->time_stamp_nano_sec_interface_ptr_ = std::make_unique<
                    hardware_interface::LoanedStateInterface>(std::move(state_interface));
                continue;
            }

            std::string e_msg = "Received unhandled state interface " + state_interface.get_name() + ".";
            throw std::runtime_error(e_msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->node_->get_logger(), "Error while iterating state interfaces.\n%s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (this->joint_names_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
        RCLCPP_ERROR(
            this->node_->get_logger(), 
            "Got invalid number of joint names. Got %d, expected %d.",
            this->joint_names_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS
        );
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (this->position_interfaces_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
        RCLCPP_ERROR(
            this->node_->get_logger(), 
            "Got invalid number of position interfaces. Got %d, expected %d.",
            this->position_interfaces_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS
        );
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (this->effort_interfaces_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
        RCLCPP_ERROR(
            this->node_->get_logger(), 
            "Got invalid number of effort interfaces. Got %d, expected %d.",
            this->effort_interfaces_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS
        );
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (this->external_torque_interfaces_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
        RCLCPP_ERROR(
            this->node_->get_logger(), 
            "Got invalid number of external torque interfaces. Got %d, expected %d.",
            this->external_torque_interfaces_.size(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS
        );
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    try {
        this->lbr_state_.name = joint_names_;
        std::fill(this->lbr_state_.position.begin(), lbr_state_.position.end(), std::numeric_limits<double>::quiet_NaN());
        std::fill(this->lbr_state_.torque.begin(), lbr_state_.position.end(), std::numeric_limits<double>::quiet_NaN());
        std::fill(this->lbr_state_.external_torque.begin(), lbr_state_.position.end(), std::numeric_limits<double>::quiet_NaN());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->node_->get_logger(), "Failed to initialize the LBRState.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LBRStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/
) {
    try {
        this->joint_names_.clear();
        this->position_interfaces_.clear();
        this->effort_interfaces_.clear();
        this->external_torque_interfaces_.clear();
        this->sample_time_interface_ptr_.release();
        this->time_stamp_sec_interface_ptr_.release();
        this->time_stamp_nano_sec_interface_ptr_.release();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->node_->get_logger(), "Failed to clear interfaces.\n%s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // end of namespace lbr_state_broadcaster

PLUGINLIB_EXPORT_CLASS(
    lbr_state_broadcaster::LBRStateBroadcaster,
    controller_interface::ControllerInterface
)
