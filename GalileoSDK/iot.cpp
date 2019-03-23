#include "iot.h"

#define EXAMPLE_TRACE(fmt, ...)  \
    do { \
        printf("%s|%03d :: ", __func__, __LINE__); \
        printf(fmt, ##__VA_ARGS__); \
        printf("%s", "\r\n"); \
    } while(0)

namespace GalileoSDK{

    std::map<size_t, std::function<void(galileo_serial_server::GalileoStatus status)>> IOTClient::statusUpdateCallbacks;
    size_t IOTClient::callbackIndex = 0;
    IOTClient* IOTClient::instance = NULL;
    std::string IOTClient::galileoStatusTopic = "";
    std::string IOTClient::galileoCmdTopic = "";
    std::string IOTClient::testTopic = "";
    std::string IOTClient::audioTopic = "";
    std::string IOTClient::speedTopic = "";

    IOTClient::IOTClient(std::string productID, std::string deviceName, std::string deviceSecret)
        :OnConnectCB(NULL), OnDisconnecCB(NULL), productID(productID), deviceName(deviceName), deviceSecret(deviceSecret), runningFlag(false), exitFlag(true), connectedFlag(false){
        if (instance != NULL)
            return;
        void *pclient = NULL;
        int res = 0;
        int loop_cnt = 0;
        iotx_mqtt_param_t mqtt_params;

        SetDeviceName(deviceName.c_str(), deviceName.size());
        SetDeviceSecret(deviceSecret.c_str(), deviceSecret.size());
        SetProductKey(productID.c_str(), productID.size());
        SetFirewareVersion("1.0.0", 5);
        
        memset(&mqtt_params, 0x0, sizeof(mqtt_params));

        mqtt_params.handle_event.h_fp = [](void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg) {
            //EXAMPLE_TRACE("msg->event_type : %d", msg->event_type);
        };

        pclient = IOT_MQTT_Construct(&mqtt_params);
        if (NULL == pclient) {
            EXAMPLE_TRACE("MQTT construct failed");
        }
        
        galileoStatusTopic = "/" + productID + "/" + deviceName + "/" + "user/galileo/status";
        galileoCmdTopic = "/" + productID + "/" + deviceName + "/" + "user/galileo/cmds";
        testTopic = "/" + productID + "/" + deviceName + "/" + "user/test";
        audioTopic = "/" + productID + "/" + deviceName + "/" + "user/audio";
        speedTopic = "/" + productID + "/" + deviceName + "/" + "user/speed";

        res = IOT_MQTT_Subscribe(pclient, galileoStatusTopic.c_str(), IOTX_MQTT_QOS0,
            [](void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg) {
            iotx_mqtt_topic_info_t *topic_info = (iotx_mqtt_topic_info_pt)msg->msg;
            if (msg->event_type == IOTX_MQTT_EVENT_PUBLISH_RECEIVED) {
                std::string topic(topic_info->ptopic, topic_info->topic_len);
                if (topic == galileoStatusTopic) {
                    nlohmann::json j = nlohmann::json::parse(std::string(topic_info->payload, topic_info->payload_len));
                    auto status = Utils::jsonToStatus(j);
                    if (statusUpdateCallbacks.size() != 0) {
                        for (auto it = statusUpdateCallbacks.begin(); it != statusUpdateCallbacks.end(); ++it) {
                            it->second(status);
                        }
                    }
                }
            }
        }, NULL);
        if (res < 0) {
            IOT_MQTT_Destroy(&pclient);
            return;
        }
        runningFlag = true;
        new std::thread(&IOTClient::LoopThread, this);
        instance = this;
    }

    void IOTClient::LoopThread() {
        exitFlag = false;
        while (runningFlag) {
            IOT_MQTT_Yield(pclient, 200);
            bool currentStatus = IOT_MQTT_CheckStateNormal(pclient);
            if (connectedFlag == false && currentStatus == true && OnConnectCB != NULL) {
                OnConnectCB(GALILEO_RETURN_CODE::OK, deviceName);
            }
            if (connectedFlag == true && currentStatus == false && OnDisconnecCB != NULL) {
                OnConnectCB(GALILEO_RETURN_CODE::NETWORK_ERROR, deviceName);
            }
            connectedFlag = currentStatus;
        }
        exitFlag = true;

    }

    void IOTClient::SetOnConnectCB(std::function<void(GALILEO_RETURN_CODE, std::string)> cb) {
        OnConnectCB = cb;
    }

    void IOTClient::SetOnDisonnectCB(std::function<void(GALILEO_RETURN_CODE, std::string)> cb) {
        OnDisconnecCB = cb;
    }

    size_t IOTClient::AddOnStatusUpdatedListener(std::function<void(galileo_serial_server::GalileoStatus status)> OnStatusUpdate) {
        size_t id = callbackIndex;
        callbackIndex += 1;
        statusUpdateCallbacks.insert(std::pair<size_t, std::function<void(galileo_serial_server::GalileoStatus)>>(id, OnStatusUpdate));
        return id;
    }


    void IOTClient::RemoveOnStatusUpdatedListener(size_t id) {
        statusUpdateCallbacks.erase(id);
    }

    bool IOTClient::SendGalileoCmd(std::vector<uint8_t> cmds) {
        int res = IOT_MQTT_Publish_Simple(0, galileoCmdTopic.c_str(), IOTX_MQTT_QOS0,
            cmds.data(), cmds.size());
        return res == 0;
    }

    bool IOTClient::SendTestCmd() {
        std::stringstream ss;
        ss << "Galileo SDK pub test " << Utils::GetCurrentTimestamp();
        std::string testMsg = ss.str();
        int res = IOT_MQTT_Publish_Simple(0, testTopic.c_str(), IOTX_MQTT_QOS0,
            (void*)(testMsg.c_str()), testMsg.size());
        return res == 0;
    }

    bool IOTClient::SendAudioCmd(char audio[]) {
        int res = IOT_MQTT_Publish_Simple(0, audioTopic.c_str(), IOTX_MQTT_QOS0,
            (void*)(audio), strlen(audio));
        return res == 0;
    }

    bool IOTClient::SendSpeedCmd(float vLinear, float vAngle) {
        nlohmann::json speed_json = {
            {"vLinear", vLinear},
        {"vAngle", vAngle}
        };
        std::string speed_str = speed_json.dump(4);
        int res = IOT_MQTT_Publish_Simple(0, speedTopic.c_str(), IOTX_MQTT_QOS0,
            (void*)(speed_str.c_str()), speed_str.size());
        return res == 0;
    }

    IOTClient* IOTClient::GetInstance() {
        return instance;
    }

    IOTClient* IOTClient::GetInstance(std::string productID, std::string deviceName, std::string deviceSecret) {
        if (instance == NULL)
            return new IOTClient(productID, deviceName, deviceSecret);
        else
            return instance;
    }

    bool IOTClient::IsRunning() {
        return runningFlag;
    }

    bool IOTClient::IsConnected() {
        return connectedFlag;
    }

    IOTClient::~IOTClient() {
        runningFlag = false;
        // µÈ´ýÍË³ö
        while (!exitFlag)
        {
            Sleep(100);
        }
        IOT_MQTT_Destroy(&pclient);
        delete instance;
        instance = NULL;
    }

}