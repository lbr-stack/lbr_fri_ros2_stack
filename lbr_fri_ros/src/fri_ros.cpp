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
 * @brief Runs a FriRos in a seperated thread. Notifies lbr_client_thread() on exit
**/
auto fri_ros_thread(std::shared_ptr<LBR> lbr) -> void {
    LBRClient lbr_client(lbr);
    KUKA::FRI::UdpConnection connection;

    KUKA::FRI::ClientApplication app(connection, lbr_client);
    app.connect(30200 /*default port id*/, NULL /*host name*/);

    bool succes = true;
    while (succes) {
        succes = app.step();
    }
    app.disconnect();
};

/**
 * @brief Runs a LBRClient in a seperated thread. Notifies fri_ros_thread() on exit
**/
auto lbr_client_thread(std::shared_ptr<LBR> lbr) -> void {
    rclcpp::spin(std::make_shared<FriRos>(lbr));
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto lbr = std::make_shared<LBR>();
    
    std::thread thread1(lbr_client_thread, lbr);
    std::thread thread2(fri_ros_thread, lbr); 

    thread1.join();
    thread2.join();

    rclcpp::shutdown();










// TODO: add tests for 2 threads that access and manipulate the data LBR class
//       test size of joints angles and torque
//       test get and set without threads

    // rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<FriRos>(lbr));



    // 2 threads, finish when quit or error
    // notify the other on quit






    // rclcpp::shutdown();

    // // test lbr struct... test with multiple threads
    // std::vector<double> el(7, 0);
    // lbr_msgs::msg::LBRState s;
    // s.position = el;
    // s.torque = el;
    // s.stamp.nanosec = 0;

    // if (lbr->get_current_state().position.empty()) {
    //     std::cout << "not init yet" << std::endl;
    // }

    // if (!lbr->get_current_state().position.data())
    //     std::cout << "not init yet" << std::endl;


    // s.position[0] = 1.;
    // lbr->set_current_state(s);
    // for (int i = 0; i < lbr->get_current_state().position.size(); i++) {
    //     std::cout << "set state: " << lbr->get_current_state().position[i] << " " << s.position[i] << std::endl;
    // }

    // s.position[0] = 2.;
    // for (int i = 0; i < lbr->get_current_state().position.size(); i++) {
    //     std::cout << "externally manipulated state: " << lbr->get_current_state().position[i] << " " << s.position[i] << std::endl;
    // }


    
    // setup connection as usual

    // auto state = lbr->get_state(); // get state and feed to ros
    // lbr->set_state(state); // take state from ros and set it
    // // lbr.set_state();

    return 0;
}