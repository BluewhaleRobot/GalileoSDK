#ifndef __IOT_H__
#define __IOT_H__
#include <string>
#include <vector>
#include <thread>
#include "galileo_serial_server/GalileoStatus.h"
#include "dev_sign_api.h"
#include "mqtt_api.h"
#include "wrapper.h"
#include "json.hpp"
#include "mutils.h"
#include "GalileoReturnCode.h"

namespace GalileoSDK {

    class IOTClient {
    public:
        IOTClient(std::string productID, std::string deviceName, std::string deviceSecret);
        int64_t AddOnStatusUpdatedListener(std::function<void(galileo_serial_server::GalileoStatus status)> OnStatusUpdate);
        void RemoveOnStatusUpdatedListener(int64_t id);
        bool SendGalileoCmd(std::vector<uint8_t> cmds);
        bool SendTestCmd();
        bool SendAudioCmd(char audio[]);
        bool SendSpeedCmd(float vLinear, float vAngle);
        bool IsRunning();
        bool IsConnected();
        void SetOnConnectCB(std::function<void(GALILEO_RETURN_CODE, std::string)> cb);
        void SetOnDisonnectCB(std::function<void(GALILEO_RETURN_CODE, std::string)> cb);
        ~IOTClient();
    private:
        std::map<int64_t, std::function<void(galileo_serial_server::GalileoStatus status)>> statusUpdateCallbacks;
        std::function<void(GALILEO_RETURN_CODE, std::string)> OnConnectCB;
        std::function<void(GALILEO_RETURN_CODE, std::string)> OnDisconnecCB;
        int64_t callbackIndex;
        std::string productID;
        std::string deviceName;
        std::string deviceSecret;
        void LoopThread();
        void *pclient;
        bool runningFlag;
        bool exitFlag;
        std::string galileoStatusTopic;
        std::string galileoCmdTopic;
        std::string testTopic;
        std::string audioTopic;
        std::string speedTopic;
        bool connectedFlag;
    };
}
#endif // !__IOT_H__