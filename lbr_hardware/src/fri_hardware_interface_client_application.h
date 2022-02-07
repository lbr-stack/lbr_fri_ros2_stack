#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

#include <fri/friClientApplication.h>
#include <fri/friConnectionIf.h>
#include <fri/friClientIf.h>
#include <fri/friTransformationClient.h>
#include <fri/friClientData.h>


namespace KUKA {
    namespace FRI {

        class FRIHardwareInterfaceClientApplication : public ClientApplication {
            public:
                /**
                 * @brief Construct a new FRIHardwareInterfaceClientApplication object. 
                 * 
                 * This is a dummy class that splits KUKA::FRI::ClientApplication::step() into separate methods.
                 * This way, LBR::FRIHardwareInterface can call the steps in LBR::FRIHardwareInterface::read()
                 * and LBR::FRIHardwareInterface::write() separately.
                 * 
                 * @param connection UDP connection to controller
                 * @param client LBRClient for callbacks in FRIHardwareInterfaceClientApplication::callback()
                 */
                FRIHardwareInterfaceClientApplication(IConnection& connection, IClient& client);

                bool receiveAndDecode();
                void callback();
                bool encodeAndSend();

            private:
                int buffer_size_;

                std::string CLIENT_APPLIATION_LOGGER = "FRIHardwareInterfaceClientApplication";
        };

    }  // end of namespace FRI
}  // end of namespace KUKA
