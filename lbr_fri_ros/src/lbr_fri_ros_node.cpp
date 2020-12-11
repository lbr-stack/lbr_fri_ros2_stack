#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <fri/friUdpConnection.h>
#include <fri/friClientApplication.h>

#include <lbr_hardware_interface.h>
#include <lbr_client.h>
#include <lbr.h>

void control_loop(controller_manager::ControllerManager& cm, LBRHardwareInterface& lbr_hw, ros::Time& time, ros::Duration& period, bool state_change=true) {
    lbr_hw.read();
    cm.update(time, period, state_change);
    lbr_hw.write();
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "lbr_fri_ros");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto lbr = std::make_shared<LBR>();
    LBRClient lbr_client(lbr);
    KUKA::FRI::UdpConnection connection;
    KUKA::FRI::ClientApplication app(connection, lbr_client);
    app.connect(30200 /*default port id*/, NULL /*host name*/);
    bool success = true;

    LBRHardwareInterface lbr_hw(lbr);
    if (!lbr_hw.init(nh)) {
        ROS_ERROR("Failed to initialize HardwareInterface.");
        std::exit(-1);
    }

    controller_manager::ControllerManager cm(&lbr_hw, nh);

    ros::Time time = ros::Time::now();
    auto last_state = KUKA::FRI::ESessionState::MONITORING_WAIT;

    while (success && ros::ok()) {
        success = app.step();

        auto period = ros::Duration(lbr_client.robotState().getSampleTime());
        auto current_state = lbr_client.robotState().getSessionState();
        switch (current_state) {
            case KUKA::FRI::ESessionState::MONITORING_WAIT: case KUKA::FRI::ESessionState::MONITORING_READY:
                time = ros::Time::now();
                lbr->set_commanded_state(lbr->get_current_state());
                control_loop(cm, lbr_hw, time, period, (last_state != current_state));
                last_state = current_state;
                break;
            case KUKA::FRI::ESessionState::COMMANDING_WAIT: case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
                if (success) {
                    time = ros::Time::now();
                    control_loop(cm, lbr_hw, time, period, (last_state != current_state));
                    last_state = current_state;
                }
                break;     
            case KUKA::FRI::ESessionState::IDLE:
                success = false;
                break;  
        }
    }

    app.disconnect();
    spinner.stop();
    ros::shutdown();

    return 0;
}