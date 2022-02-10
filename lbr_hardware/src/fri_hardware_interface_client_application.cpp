#include <fri_hardware_interface_client_application.h>


namespace KUKA {
    namespace FRI {

        FRIHardwareInterfaceClientApplication::FRIHardwareInterfaceClientApplication(IConnection& connection, IClient& client) 
            : ClientApplication(connection, client), buffer_size_(0) {

        }

        bool FRIHardwareInterfaceClientApplication::receiveAndDecode() {
            if (!_connection.isOpen()) {
                RCLCPP_ERROR(rclcpp::get_logger(CLIENT_APPLIATION_LOGGER), "Client application is not connected.");
                return false;
            }

            // **************************************************************************
            // Receive and decode new monitoring message
            // **************************************************************************
            buffer_size_ = _connection.receive(_data->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);

            if (buffer_size_ <= 0) {  // TODO: buffer_size_ == 0 -> connection closed (maybe go to IDLE instead of stopping?)
                RCLCPP_ERROR(rclcpp::get_logger(CLIENT_APPLIATION_LOGGER), "Failed while trying to receive monitoring message");
                return false;
            }

            if (!_data->decoder.decode(_data->receiveBuffer, buffer_size_)) {
                return false;
            }

            // check message type (so that our wrappers match)
            if (_data->expectedMonitorMsgID != _data->monitoringMsg.header.messageIdentifier)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger(CLIENT_APPLIATION_LOGGER),
                    "Error: incompatible IDs for received message (got: %d expected %d).",
                    (int)_data->monitoringMsg.header.messageIdentifier,
                    (int)_data->expectedMonitorMsgID
                );
                return false;
            }

            return true;
        }

        void FRIHardwareInterfaceClientApplication::callback() {
            // **************************************************************************
            // callbacks
            // **************************************************************************
            // reset commmand message before callbacks
            _data->resetCommandMessage();

            // callbacks for robot client
            ESessionState currentState = (ESessionState)_data->monitoringMsg.connectionInfo.sessionState;

            if (_data->lastState != currentState) {
                _robotClient->onStateChange(_data->lastState, currentState);
                _data->lastState = currentState;
            }

            switch (currentState) {
                case MONITORING_WAIT:
                case MONITORING_READY:
                    _robotClient->monitor();
                    break;
                case COMMANDING_WAIT:
                    _robotClient->waitForCommand();
                    break;
                case COMMANDING_ACTIVE:
                    _robotClient->command();
                    break;
                case IDLE:
                    default:
                    return; // nothing to send back
            }

            // callback for transformation client
            if(_trafoClient != NULL) {
                _trafoClient->provide();
            }
        }

        bool FRIHardwareInterfaceClientApplication::encodeAndSend() {
            // **************************************************************************
            // Encode and send command message
            // **************************************************************************

            _data->lastSendCounter++;
            // check if its time to send an answer
            if (_data->lastSendCounter >= _data->monitoringMsg.connectionInfo.receiveMultiplier) {
                _data->lastSendCounter = 0;

                // set sequence counters
                _data->commandMsg.header.sequenceCounter = _data->sequenceCounter++;
                _data->commandMsg.header.reflectedSequenceCounter = 
                _data->monitoringMsg.header.sequenceCounter;

                if (!_data->encoder.encode(_data->sendBuffer, buffer_size_)) {
                    return false;
                }

                if (!_connection.send(_data->sendBuffer, buffer_size_)) {
                    printf("Error: failed while trying to send command message!\n");
                    return false;
                }
            }

            return true;
        }

    }  // end of namespace FRI
}  // end of namespace KUKA

