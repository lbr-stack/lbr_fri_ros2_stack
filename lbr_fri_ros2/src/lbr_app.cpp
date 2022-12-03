#include "lbr_fri_ros2/lbr_app.hpp"

namespace lbr_fri_ros2
{
    LBRApp::LBRApp(const std::string &node_name, const int &port_id, const char *const remote_host)
        : rclcpp::Node(node_name)
    {
        if (!valid_port_(port_id))
        {
            throw std::range_error("Invalid port_id provided.");
        }
        port_id_ = port_id;
        remote_host_ = remote_host;

        connected_ = false;

        app_connect_srv_ = this->create_service<lbr_fri_msgs::srv::AppConnect>(
            "app/connect",
            std::bind(&LBRApp::app_connect_cb_, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_system_default);

        app_disconnect_srv_ = this->create_service<std_srvs::srv::Empty>(
            "app/disconnect",
            std::bind(&LBRApp::app_disconnect_cb_, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_system_default);

        lbr_state_client_ = std::make_unique<lbr_fri_ros2::LBRStateClient>("lbr_state_client_node");
        connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
        app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_.get(), *lbr_state_client_.get());
    }

    LBRApp::~LBRApp()
    {
        disconnect_();
    }

    void LBRApp::app_connect_cb_(
        const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
        lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response)
    {
        RCLCPP_INFO(get_logger(), "Attempting to open UDP connection for LBR server...");
        const char *remote_host = request->remote_host.empty() ? NULL : request->remote_host.c_str();
        response->established = connect_(request->port_id, remote_host);
        if (!response->established)
        {
            RCLCPP_ERROR(get_logger(), "Failed.");
            return;
        };
        RCLCPP_INFO(get_logger(), "Done.");
    }

    void LBRApp::app_disconnect_cb_(
        const std_srvs::srv::Empty::Request::SharedPtr /*request*/,
        std_srvs::srv::Empty::Response::SharedPtr /*response*/)
    {
        RCLCPP_INFO(get_logger(), "Attempting to close UDP connection for LBR server...");
        if (!disconnect_())
        {
            RCLCPP_ERROR(get_logger(), "Failed.");
            return;
        };
        RCLCPP_INFO(get_logger(), "Done.");
    }

    bool LBRApp::valid_port_(const int &port_id)
    {
        if (30200 > port_id ||
            30209 <= port_id)
        {
            RCLCPP_ERROR(get_logger(), "Expected port_id id in [30200, 30209] or -1, got %d.", port_id);
            return false;
        }
        return true;
    }

    bool LBRApp::connect_(const int &port_id, const char *const remote_host)
    {
        if (!connected_)
        {
            if (!valid_port_(port_id))
            {
                throw std::range_error("Invalid port_id provided.");
            }
            connected_ = app_->connect(port_id, remote_host);
            if (connected_)
            {
                port_id_ = port_id;
                remote_host_ = remote_host;

                auto app_step = [this]()
                {
                    bool success = true;
                    while (success && connected_ && rclcpp::ok())
                    {
                        try
                        {
                            success = app_->step();
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_ERROR(get_logger(), e.what());
                            break;
                        }
                    }
                    disconnect_();
                };

                app_step_thread_ = std::make_unique<std::thread>(app_step);
                app_step_thread_->detach();
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Attempted to connect to LBR server when already connected.");
        }
        return connected_;
    }

    bool LBRApp::disconnect_()
    {
        if (connected_)
        {
            app_->disconnect();
            connected_ = false;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Attempted to disconnect from LBR server when already disconnected.");
        }
        return !connected_;
    }
} // end of namespace lbr_fri_ros2
