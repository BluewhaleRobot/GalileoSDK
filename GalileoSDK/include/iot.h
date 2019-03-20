#ifndef __IOT_H__
#define __IOT_H__
#include "dev_sign_api.h"
#include "mqtt_api.h"
#include "wrapper.h"

namespace GalileoSDK {
    class IOTClient {
    public:
        IOTClient(std::string productID, std::string deviceName, std::string deviceSecret);
        void SetOnStatusUpdated(void(*OnStatusUpdate)(galileo_serial_server::GalileoStatus status));
        void SendGalileoCmd(std::vector<uint8_t> cmds);
    };
}
#endif // !__IOT_H__