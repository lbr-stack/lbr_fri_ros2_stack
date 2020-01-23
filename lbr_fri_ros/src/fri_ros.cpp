#include <rclcpp/rclcpp.hpp>
#include <lbr_msgs/msg/lbr_state.hpp>
#include <memory>
#include <chrono>
#include <thread>

#include <fri/friLBRState.h>
#include <lbr.h>
#include <lbr_client.h>

class FriRos : public rclcpp::Node {
    public:
        FriRos(std::shared_ptr<LBR> lbr) : Node("fri_ros"), lbr_(lbr) {
            using namespace std::chrono_literals;
            publisher_ = this->create_publisher<lbr_msgs::msg::LBRState>("fri_ros/get_state", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&FriRos::timer_callback, this));
            subscription_ = this->create_subscription<lbr_msgs::msg::LBRState>(
                "fri_ros/set_state", 10, std::bind(&FriRos::topic_callback, this, std::placeholders::_1));
        };

    private:
        void timer_callback() {
            auto msg = lbr_msgs::msg::LBRState();
            msg.joint_positions = lbr_->get_current_state().joint_positions;
            msg.torques = lbr_->get_current_state().torques;
            msg.time_stamp = lbr_->get_current_state().time_stamp;
            publisher_->publish(msg);
        };

        void topic_callback(const lbr_msgs::msg::LBRState::SharedPtr msg) const {  
            // lbr_->set_commanded_state(msg);
        };

        std::shared_ptr<LBR> lbr_;

        rclcpp::Publisher<lbr_msgs::msg::LBRState>::SharedPtr publisher_;
        rclcpp::Subscription<lbr_msgs::msg::LBRState>::SharedPtr subscription_;

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    auto lbr = std::make_shared<LBR>();
    rclcpp::init(argc, argv);
    // FriRos fri_ros(lbr); // run in thread 1
    LBRClient lbr_client(lbr); // run in thread 2

    // test lbr struct... test with multiple threads
    std::vector<double> el(7, 0);
    lbr_msgs::msg::LBRState s;
    s.joint_positions = el;
    s.torques = el;
    s.time_stamp = 0;

    if (lbr->get_current_state().joint_positions.empty()) {
        std::cout << "not init yet" << std::endl;
    }

    if (!lbr->get_current_state().joint_positions.data())
        std::cout << "not init yet" << std::endl;


    s.joint_positions[0] = 1.;
    lbr->set_current_state(s);
    for (int i = 0; i < lbr->get_current_state().joint_positions.size(); i++) {
        std::cout << "set state: " << lbr->get_current_state().joint_positions[i] << " " << s.joint_positions[i] << std::endl;
    }

    s.joint_positions[0] = 2.;
    for (int i = 0; i < lbr->get_current_state().joint_positions.size(); i++) {
        std::cout << "externally manipulated state: " << lbr->get_current_state().joint_positions[i] << " " << s.joint_positions[i] << std::endl;
    }


    
    // setup connection as usual

    // auto state = lbr->get_state(); // get state and feed to ros
    // lbr->set_state(state); // take state from ros and set it
    // // lbr.set_state();

    printf("hello world!\n");
    return 0;
}