#ifndef __GALILEO_SDK_H__
#define __GALILEO_SDK_H__

#include <iostream>
#include <fstream>

#ifdef _WIN32
#ifndef WIN32
#define WIN32
#endif
#pragma comment(lib, "Ws2_32.lib")
#endif

#include "Dll.h"
#include "galileo_serial_server/GalileoNativeCmds.h"
#include "galileo_serial_server/GalileoStatus.h"
#include "audio_common_msgs/AudioData.h"
#include "geometry_msgs/Twist.h"
#include "models/ServerInfo.h"
#include "ros/ros.h"
#include "ros/network.h"
#include "std_msgs/String.h"
#include "tasks/BroadcastReceiver.h"
#include "json.hpp"
#include "iot.h"
#include "HttpConnection.h"
#include "GalileoReturnCode.h"
#include "mutils.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#ifdef __ANDROID__
#include "spdlog/sinks/android_sink.h"
#endif
#include <mutex>
#include <thread>
#include <stdexcept>

namespace GalileoSDK
{

class DLL_PUBLIC GalileoSDK
{
  public:
    GalileoSDK();
    GALILEO_RETURN_CODE
    Connect(std::string targetID, bool auto_connect, int timeout,
        std::function<void(GALILEO_RETURN_CODE, std::string)> OnConnect,
        std::function<void(GALILEO_RETURN_CODE, std::string)> OnDisonnect
    );
    GALILEO_RETURN_CODE Connect(ServerInfo server);
    GALILEO_RETURN_CODE
    Connect(std::string targetID, int timeout, std::string password,
        std::function<void(GALILEO_RETURN_CODE, std::string)>OnConnectCB,
        std::function<void(GALILEO_RETURN_CODE, std::string)>OnDisconnectCB);
    GALILEO_RETURN_CODE
    ConnectIOT(std::string targetID, int timeout, std::string password);
    std::vector<ServerInfo> GetServersOnline();
    GALILEO_RETURN_CODE PublishTest();
    ServerInfo *GetCurrentServer();
    GalileoSDK *GetInstance();
    void broadcastOfflineCallback(std::string id);
    // Galileo commnads related
    GALILEO_RETURN_CODE SendCMD(uint8_t[], int);
    GALILEO_RETURN_CODE StartNav();
    GALILEO_RETURN_CODE StopNav();
    GALILEO_RETURN_CODE SetGoal(int);
    GALILEO_RETURN_CODE PauseGoal();
    GALILEO_RETURN_CODE ResumeGoal();
    GALILEO_RETURN_CODE CancelGoal();
    GALILEO_RETURN_CODE InsertGoal(float x, float y);
    GALILEO_RETURN_CODE ResetGoal();
    GALILEO_RETURN_CODE SetSpeed(float vLinear, float vAngle);
    GALILEO_RETURN_CODE Shutdown();
    GALILEO_RETURN_CODE SetAngle(uint8_t sign, uint8_t angle);
    GALILEO_RETURN_CODE StartLoop();
    GALILEO_RETURN_CODE StopLoop();
    GALILEO_RETURN_CODE SetLoopWaitTime(uint8_t time);
    GALILEO_RETURN_CODE StartMapping();
    GALILEO_RETURN_CODE StopMapping();
    GALILEO_RETURN_CODE SaveMap();
    GALILEO_RETURN_CODE UpdateMap();
    GALILEO_RETURN_CODE StartChargeLocal();
    GALILEO_RETURN_CODE StopChargeLocal();
    GALILEO_RETURN_CODE SaveChargeBasePosition();
    GALILEO_RETURN_CODE StartCharge(float x, float y);
    GALILEO_RETURN_CODE StopCharge();
    GALILEO_RETURN_CODE MoveTo(float x, float y, uint8_t *goalNum);
    GALILEO_RETURN_CODE GetGoalNum(uint8_t *goalNum);
    GALILEO_RETURN_CODE GetCurrentStatus(galileo_serial_server::GalileoStatus *);
    void SetCurrentStatusCallback(std::function<void(GALILEO_RETURN_CODE, galileo_serial_server::GalileoStatus)> callback);
    void SetGoalReachedCallback(std::function<void(int goalID, galileo_serial_server::GalileoStatus)> callback);
    GALILEO_RETURN_CODE WaitForGoal(int goalID);
    GALILEO_RETURN_CODE SendAudio(const char audio[]);
	GALILEO_RETURN_CODE SendRawAudio(const uint8_t audio[], int length);
	GALILEO_RETURN_CODE EnableGreeting(bool flag);
    bool CheckServerOnline(std::string targetid);
    int64_t GetStatusUpdateStamp();
    void Dispose();
    ~GalileoSDK();
    static std::shared_ptr<spdlog::logger> logger;

  private:
    ServerInfo *currentServer;
    static BroadcastReceiver* broadcastReceiver;
    ros::NodeHandle *nh;
    ros::Publisher testPub;
    ros::Publisher cmdPub;
    ros::Publisher audioPub;
    ros::Publisher speedPub;
	ros::Publisher audioRawPub;
    ros::Subscriber galileoStatusSub;
    void UpdateGalileoStatus(
        const galileo_serial_server::GalileoStatusConstPtr &status);
    galileo_serial_server::GalileoStatusConstPtr currentStatus;
    std::mutex statusLock;
    std::mutex serverLock;
    void SpinThread();
    std::function<void(GALILEO_RETURN_CODE, std::string)> OnDisconnect;
    std::function<void(GALILEO_RETURN_CODE, std::string)> OnConnect;
    std::function<void(GALILEO_RETURN_CODE, galileo_serial_server::GalileoStatus)> CurrentStatusCallback;
    std::function<void(int, galileo_serial_server::GalileoStatus)> GoalReachedCallback;
    bool connectingTaskFlag;
    static std::vector<GalileoSDK*> instances;
    // Connect related params
    std::string targetID;
    bool auto_connect;
    int timeout;
    IOTClient* iotclient;
    std::string password;
    int64_t statusUpdateStamp;
};

// export c functions
extern "C"
{
    DLL_PUBLIC void *__stdcall CreateInstance();
    DLL_PUBLIC void __stdcall ReleaseInstance(void *instance);
	DLL_PUBLIC void __stdcall Dispose(void* instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall Connect(
        void *instance, uint8_t *targetID, int64_t length, bool auto_connect, int timeout,
        void (*OnConnect)(GALILEO_RETURN_CODE, uint8_t *, int64_t),
        void (*OnDisconnect)(GALILEO_RETURN_CODE, uint8_t *, int64_t));
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall
        ConnectIOT(void *instance, uint8_t *targetID, int64_t length, int timeout, uint8_t *password, int64_t pass_length,
            void(*OnConnect)(GALILEO_RETURN_CODE, uint8_t *, int64_t),
            void(*OnDisconnect)(GALILEO_RETURN_CODE, uint8_t *, int64_t));
    DLL_PUBLIC void __stdcall GetServersOnline(void *instance, uint8_t *servers_json, int64_t &length);
    DLL_PUBLIC void __stdcall GetCurrentServer(void *instance, uint8_t *servers_json, int64_t &length);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall PublishTest(void *instance);
    DLL_PUBLIC void *__stdcall GetInstance();
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SendCMD(void *instance, uint8_t *, int);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartNav(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopNav(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetGoal(void *instance, int);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall PauseGoal(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall ResumeGoal(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall CancelGoal(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall InsertGoal(void *instance, float x, float y);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall ResetGoal(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetSpeed(void *instance, float vLinear, float vAngle);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall Shutdown(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetAngle(void *instance, uint8_t sign, uint8_t angle);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartLoop(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopLoop(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetLoopWaitTime(void *instance, uint8_t time);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartMapping(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopMapping(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SaveMap(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall UpdateMap(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartChargeLocal(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopChargeLocal(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SaveChargeBasePosition(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartCharge(void *instance, float x, float y);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopCharge(void *instance);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall MoveTo(void *instance, float x, float y, uint8_t &goalNum);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall GetGoalNum(void *instance, uint8_t &goalNum);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall GetCurrentStatus(void *instance, uint8_t *status_json, int64_t &length);
    DLL_PUBLIC void __stdcall SetCurrentStatusCallback(void *instance, void (*callback)(
                                                                           GALILEO_RETURN_CODE, uint8_t *status_json, int64_t length));
    DLL_PUBLIC void __stdcall SetGoalReachedCallback(
        void *instance,
        void (*callback)(int goalID, uint8_t *status_json, int64_t length));
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall WaitForGoal(void *instance, int goalID);
    DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SendAudio(void *instance, uint8_t* audio, int64_t length);
	DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SendRawAudio(void* instance, uint8_t* audio, int64_t length);
	DLL_PUBLIC GALILEO_RETURN_CODE __stdcall EnableGreeting(void* instance, bool flag);
    DLL_PUBLIC bool __stdcall CheckServerOnline(void *instance, uint8_t *targetID, int64_t length);
}

}; // namespace GalileoSDK

#endif // !__GALILEO_SDK_H__
