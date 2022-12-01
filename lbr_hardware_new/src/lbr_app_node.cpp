#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lbr_hardware_new/lbr_app.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto lbr_app = std::make_shared<lbr_hardware::LBRApp>("lbr_app_node");
    rclcpp::spin(lbr_app);
    rclcpp::shutdown();
    return 0;
}
