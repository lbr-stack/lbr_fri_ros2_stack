#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include <fri/friLBRState.h>
#include <lbr_client.h>


// node with publisher and subscriber
// publisher -> push lbr.get_state() to network
// subscriber -> lbr.set_state() from network, possibly accessed via python
class FriRos : public rclcpp::Node {
    public:
        FriRos(std::shared_ptr<LBR> lbr) : Node("fri_ros"), lbr_(lbr) {  };
    private:
        std::shared_ptr<LBR> lbr_;

        rclcpp::Publisher<...> publisher_; // TODO use user msg
        rclcpp::Subscription<...> subscription;
};



int main() {
    auto lbr = std::make_shared<LBR>();

    FriRos fri_ros(lbr); // run in thread 1
    LBRClient lbr_client(lbr); // run in thread 2

    // setup connection as usual

    // auto state = lbr->get_state(); // get state and feed to ros
    // lbr->set_state(state); // take state from ros and set it
    // // lbr.set_state();

    printf("hello world!\n");
    return 0;
}