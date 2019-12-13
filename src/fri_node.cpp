#include <memory>
#include <cstring>

// FRI
#include <fri/friClientIf.h>
#include <fri/friConnectionIf.h>
#include <friClientData.h>
#include <FRIMessages.pb.h>
#include <fri/friLBRState.h>
#include <fri/friUdpConnection.h>
#include <fast_robot_interface_ros2/lbr_servo.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


namespace KUKA {
namespace FRI {
    
class ClientROSApplication : public rclcpp::Node {
    public:
        ClientROSApplication(IConnection& connection, IClient& client) 
            : Node("fri_node"),
              connection_(connection),
              client_(&client) {

            data_ = client_->createData();
            this->connect(30200);

            // Create subscription to the joint angle topic
            sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/lbr_joint_angle_in", 10, std::bind(&ClientROSApplication::callback, this, std::placeholders::_1)
            );
        }

        ~ClientROSApplication() {
            disconnect();
            delete data_;
        }


    private:
        void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
            // Read msg from subscription
            double in[LBRState::NUMBER_OF_JOINTS];
            double out[LBRState::NUMBER_OF_JOINTS];

            std::memcpy(in, msg->data.data(), sizeof(double)*LBRState::NUMBER_OF_JOINTS);

            // Send msg via UDP to LBR
            // bool success = send(in, out);

            // Publish robot state
            // std::memcpy(out, msg->data.data(), sizeof(double)*LBRState::NUMBER_OF_JOINTS);
        }

        bool connect(int port, const char *remoteHost = NULL) {
            if (connection_.isOpen()) 
            {
                printf("Warning: client application already connected!\n");
                return true;
            }
            
            return connection_.open(port, remoteHost);
        };
        
        void disconnect() {
            if (connection_.isOpen()) connection_.close();
        };

        bool send(double* in, double* out) {
            if (!connection_.isOpen())
            {
                printf("Error: client application is not connected!\n");
                return false;
            }

            // **************************************************************************
            // Receive and decode new monitoring message
            // **************************************************************************
            int size = connection_.receive(data_->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);
            
            if (size <= 0)
            {  // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
                printf("Error: failed while trying to receive monitoring message!\n");
                return false;
            }
            
            if (!data_->decoder.decode(data_->receiveBuffer, size))
            {
                return false;
            }
            
            // check message type (so that our wrappers match)
            if (data_->expectedMonitorMsgID != data_->monitoringMsg.header.messageIdentifier)
            {
                printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
                        (int)data_->monitoringMsg.header.messageIdentifier,
                        (int)data_->expectedMonitorMsgID);
                return false;
            }   
            
            // **************************************************************************
            // callbacks
            // **************************************************************************
            // reset commmand message before callbacks
            data_->resetCommandMessage();
            
            // callbacks for robot client
            ESessionState currentState = (ESessionState)data_->monitoringMsg.connectionInfo.sessionState;
            
            if (data_->lastState != currentState)
            {
                client_->onStateChange(data_->lastState, currentState);
                data_->lastState = currentState;
            }
            
            switch (currentState)
            {
                case MONITORING_WAIT:
                case MONITORING_READY:
                    client_->monitor();
                    break;
                case COMMANDING_WAIT:
                    client_->waitForCommand();
                    break;
                case COMMANDING_ACTIVE:
                    client_->command(in, out);
                    break;
                case IDLE:
                default:
                    return true; // nothing to send back
            }

            // **************************************************************************
            // Encode and send command message
            // **************************************************************************
            
            data_->lastSendCounter++;
            // check if its time to send an answer
            if (data_->lastSendCounter >= data_->monitoringMsg.connectionInfo.receiveMultiplier)
            {
                data_->lastSendCounter = 0;
                
                // set sequence counters
                data_->commandMsg.header.sequenceCounter = data_->sequenceCounter++;
                data_->commandMsg.header.reflectedSequenceCounter = 
                        data_->monitoringMsg.header.sequenceCounter;
                
                if (!data_->encoder.encode(data_->sendBuffer, size))
                {
                    return false;
                }
                
                if (!connection_.send(data_->sendBuffer, size))
                {
                    printf("Error: failed while trying to send command message!\n");
                    return false;
                }
            }
            
            return true;
        };

        // FRI
        IConnection& connection_;    
        IClient* client_;    
        ClientData* data_;

        // ROS
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

};

} // namespace FRI
} // namespace KUKA

int main(int argc, char** argv) {

    LBRServoClient client;
    KUKA::FRI::UdpConnection connection;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KUKA::FRI::ClientROSApplication>(connection, client));
    rclcpp::shutdown();
    return 0;

}