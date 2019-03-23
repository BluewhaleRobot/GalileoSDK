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
        size_t AddOnStatusUpdatedListener(std::function<void(galileo_serial_server::GalileoStatus status)> OnStatusUpdate);
        void RemoveOnStatusUpdatedListener(size_t id);
        bool SendGalileoCmd(std::vector<uint8_t> cmds);
        bool SendTestCmd();
        bool SendAudioCmd(char audio[]);
        bool SendSpeedCmd(float vLinear, float vAngle);
        static IOTClient* GetInstance();
        static IOTClient* GetInstance(std::string productID, std::string deviceName, std::string deviceSecret);
        bool IsRunning();
        bool IsConnected();
        void SetOnConnectCB(std::function<void(GALILEO_RETURN_CODE, std::string)> cb);
        void SetOnDisonnectCB(std::function<void(GALILEO_RETURN_CODE, std::string)> cb);
        ~IOTClient();
    private:
        static std::map<size_t, std::function<void(galileo_serial_server::GalileoStatus status)>> statusUpdateCallbacks;
        std::function<void(GALILEO_RETURN_CODE, std::string)> OnConnectCB;
        std::function<void(GALILEO_RETURN_CODE, std::string)> OnDisconnecCB;
        static size_t callbackIndex;
        std::string productID;
        std::string deviceName;
        std::string deviceSecret;
        static IOTClient* instance;
        void LoopThread();
        void *pclient;
        bool runningFlag;
        bool exitFlag;
        static std::string galileoStatusTopic;
        static std::string galileoCmdTopic;
        static std::string testTopic;
        static std::string audioTopic;
        static std::string speedTopic;
        bool connectedFlag;
    };
}
#endif // !__IOT_H__