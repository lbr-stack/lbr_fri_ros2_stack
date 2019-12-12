#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"



class FRINode : public rclcpp::Node {
    public:
        FRINode() : Node("lbr_subscriber") {
            sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/lbr_in", 10, std::bind(&FRINode::timer_callback, this, std::placeholders::_1)
            );
        }

        ~FRINode() {

        }


    private:
        void timer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
            for (const auto& dim : msg->layout.dim) {
                RCLCPP_INFO(this->get_logger(), "%d", dim.size);
            }
        }

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};



int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FRINode>());
    rclcpp::shutdown();
    return 0;

}