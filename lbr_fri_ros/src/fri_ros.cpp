#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <lbr_msgs/msg/lbr_state.hpp>
#include <fri/friLBRState.h>
#include <fri/friUdpConnection.h>
#include <fri/friClientApplication.h>
#include <lbr.h>
#include <lbr_client.h>

/**
 * @brief Creates a ROS2 node that publishes and changes the state of a shared LBR object
 * @param lbr A shared pointer to a LBR object
 * 
 * The FriRos node changes the state of a shared LBR object, which also is accessed by the
 * LBRClient. The FriRos node, therefore, allows to access and manipulate the state of the
 * real robot.
**/
class FriRos : public rclcpp::Node {
    public:
        FriRos(std::shared_ptr<LBR> lbr) : Node("fri_ros"), lbr_(lbr) {
            using namespace std::chrono_literals;
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&FriRos::timer_callback, this));
            subscription_ = this->create_subscription<lbr_msgs::msg::LBRState>(
                "/command_states", 10, std::bind(&FriRos::topic_callback, this, std::placeholders::_1));
        };

    private:
        /**
         * @brief Periodic callback that publishes the shared state of the LBR object to ROS2
        **/
        auto timer_callback() -> void {
            auto msg = sensor_msgs::msg::JointState();
            
            msg.header.stamp.sec = lbr_->get_current_state().stamp.sec;
            msg.header.stamp.nanosec = lbr_->get_current_state().stamp.nanosec;
            msg.position = lbr_->get_current_state().position;

            publisher_->publish(msg);
        };

        /**
         * @brief Callback on subscription to ROS2 topic that commands the state of the shared LBR object
         * @param msg LBRState ROS2 message
        **/
        auto topic_callback(const lbr_msgs::msg::LBRState::SharedPtr msg) const -> void {  
            lbr_->set_commanded_state(*msg);
        };

        std::shared_ptr<LBR> lbr_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::Subscription<lbr_msgs::msg::LBRState>::SharedPtr subscription_;

        rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Runs a ClientApplication in a seperated thread
**/
auto fri_thread(std::shared_ptr<LBR> lbr) -> void {
    LBRClient lbr_client(lbr);
    KUKA::FRI::UdpConnection connection;

    KUKA::FRI::ClientApplication app(connection, lbr_client);
    app.connect(30200 /*default port id*/, NULL /*host name*/);

    bool succes = true;
    while (succes && !rclcpp::ok()) {
        succes = app.step();
    }
    app.disconnect();
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto lbr = std::make_shared<LBR>();
    
    // start thread that connects to the robot via the Fast Robot Interface
    std::thread thread(fri_thread, lbr); 

    // start a thread that reads and write to the lbr object
    rclcpp::spin(std::make_shared<FriRos>(lbr));

    // shutdown
    rclcpp::shutdown();
    thread.join();

    return 0;
}