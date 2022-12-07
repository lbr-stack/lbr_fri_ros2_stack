#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_ros2/lbr_app.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto lbr_app = std::make_shared<lbr_fri_ros2::LBRApp>("lbr_app_node");
    rclcpp::spin(lbr_app);
    rclcpp::shutdown();
    return 0;
}