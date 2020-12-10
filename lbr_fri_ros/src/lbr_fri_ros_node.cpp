#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <fri/friUdpConnection.h>
#include <fri/friClientApplication.h>

#include <lbr_hardware_interface.h>
#include <lbr_client.h>
#include <lbr.h>

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

    ROS_INFO("Awaiting good or excellent connection...");
    while (lbr_client.robotState().getConnectionQuality() < KUKA::FRI::EConnectionQuality::GOOD) {
        success = app.step();
    }

    LBRHardwareInterface lbr_hw(lbr);
    if (!lbr_hw.init(nh)) {
        ROS_ERROR("Failed to initialize HardwareInterface.");
        std::exit(-1);
    }

    controller_manager::ControllerManager cm(&lbr_hw, nh);

    ros::Time time = ros::Time::now();
    auto period = ros::Duration(lbr_client.robotState().getSampleTime());

    while (success && ros::ok()) {
        success = app.step();

        if (success) {
            time = ros::Time::now();

            lbr_hw.read();
            cm.update(time, period);
            lbr_hw.write();
        }
    }

    app.disconnect();
    spinner.stop();
    ros::shutdown();

    return 0;
}