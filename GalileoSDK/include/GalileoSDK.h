#ifndef __GALILEO_SDK_H__
#define __GALILEO_SDK_H__

#include <iostream>

#ifdef _WIN32
#define WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif

#include "Dll.h"
#include "galileo_serial_server/GalileoNativeCmds.h"
#include "galileo_serial_server/GalileoStatus.h"
#include "geometry_msgs/Twist.h"
#include "models/ServerInfo.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tasks/BroadcastReceiver.h"
#include <json.hpp>
#include <mutex>
#include <thread>
#include <stdexcept>

namespace GalileoSDK {

    enum GALILEO_RETURN_CODE {
        OK,
        NOT_CONNECTED,
        INVALIDE_STATE,
        NO_SERVER_FOUND,
        MULTI_SERVER_FOUND,
        NETWORK_ERROR,
        ALREADY_CONNECTED,
        TIMEOUT,
        SERVER_ERROR,
        GOAL_CANCELLED,
        INVALIDE_GOAL,
        INVALIDE_PARAMS
    };

    std::string GalileoReturnCodeToString(GALILEO_RETURN_CODE status) {
        switch (status)
        {
        case GALILEO_RETURN_CODE::OK:
            return "OK";
        case GALILEO_RETURN_CODE::NOT_CONNECTED:
            return "NOT_CONNECTED";
        case GALILEO_RETURN_CODE::INVALIDE_STATE:
            return "INVALIDE_STATE";
        case GALILEO_RETURN_CODE::NO_SERVER_FOUND:
            return "NO_SERVER_FOUND";
        case GALILEO_RETURN_CODE::MULTI_SERVER_FOUND:
            return "MULTI_SERVER_FOUND";
        case GALILEO_RETURN_CODE::NETWORK_ERROR:
            return "NETWORK_ERROR";
        case GALILEO_RETURN_CODE::ALREADY_CONNECTED:
            return "ALREADY_CONNECTED";
        case GALILEO_RETURN_CODE::TIMEOUT:
            return "TIMEOUT";
        case GALILEO_RETURN_CODE::SERVER_ERROR:
            return "SERVER_ERROR";
        case GALILEO_RETURN_CODE::GOAL_CANCELLED:
            return "GOAL_CANCELLED";
        case GALILEO_RETURN_CODE::INVALIDE_GOAL:
            return "INVALIDE_GOAL";
        case GALILEO_RETURN_CODE::INVALIDE_PARAMS:
            return "INVALIDE_PARAMS";
        default:
            return "OK";
        }
    }

    class DLL_PUBLIC GalileoSDK {
    public:
        GalileoSDK();
        GALILEO_RETURN_CODE
            Connect(std::string targetID, bool auto_connect, int timeout,
                void(*OnConnect)(GALILEO_RETURN_CODE, std::string),
                void(*OnDisconnect)(GALILEO_RETURN_CODE, std::string));
        GALILEO_RETURN_CODE Connect(ServerInfo server);
        /*GALILEO_RETURN_CODE WaitForConnect(std::string targetID, bool auto_connect,
        bool reconnect, int timeout, GALILEO_RETURN_CODE(*OnConnect)(std::string),
        GALILEO_RETURN_CODE(*OnDisconnect)(std::string)); GALILEO_RETURN_CODE
        StartNavigation(GALILEO_RETURN_CODE(*OnConnect)(std::string));
        GALILEO_RETURN_CODE
        WaitForStartNavigation(GALILEO_RETURN_CODE(*OnConnect)(std::string));*/
        std::vector<ServerInfo> GetServersOnline();
        GALILEO_RETURN_CODE PublishTest();
        ServerInfo* GetCurrentServer();
        static GalileoSDK *GetInstance();
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
        void SetCurrentStatusCallback(void(*callback)(
            GALILEO_RETURN_CODE, galileo_serial_server::GalileoStatus));
        void SetGoalReachedCallback(
            void(*callback)(int goalID, galileo_serial_server::GalileoStatus));
        GALILEO_RETURN_CODE WaitForGoal(int goalID);
        bool CheckServerOnline(std::string targetid);
        void Dispose();
        ~GalileoSDK();

    private:
        ServerInfo * currentServer;
        BroadcastReceiver broadcastReceiver;
        ros::NodeHandle *nh;
        ros::Publisher testPub;
        ros::Publisher cmdPub;
        ros::Publisher audioPub;
        ros::Publisher speedPub;
        ros::Subscriber galileoStatusSub;
        void UpdateGalileoStatus(
            const galileo_serial_server::GalileoStatusConstPtr &status);
        galileo_serial_server::GalileoStatusConstPtr currentStatus;
        std::mutex statusLock;
        std::mutex serverLock;
        void SpinThread();
        void(*OnDisconnect)(GALILEO_RETURN_CODE, std::string);
        void(*OnConnect)(GALILEO_RETURN_CODE, std::string);
        void(*CurrentStatusCallback)(GALILEO_RETURN_CODE,
            galileo_serial_server::GalileoStatus);
        void(*GoalReachedCallback)(int, galileo_serial_server::GalileoStatus);
        bool connectingTaskFlag;
        static GalileoSDK *instance;
        // Connect related params
        std::string targetID;
        bool auto_connect;
        int timeout;
    };

    nlohmann::json statusToJson(galileo_serial_server::GalileoStatus);

    // export c functions
    extern "C"
    {
        DLL_PUBLIC void* __stdcall CreateInstance();
        DLL_PUBLIC void __stdcall ReleaseInstance(void *instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall Connect(
            void* instance, uint8_t* targetID, size_t length, bool auto_connect, int timeout,
            void(*OnConnect)(GALILEO_RETURN_CODE, uint8_t*, size_t),
            void(*OnDisconnect)(GALILEO_RETURN_CODE, uint8_t*, size_t));
        DLL_PUBLIC void __stdcall GetServersOnline(void * instance, uint8_t* servers_json, size_t &length);
        DLL_PUBLIC void __stdcall GetCurrentServer(void * instance, uint8_t* servers_json, size_t &length);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall PublishTest(void * instance);
        DLL_PUBLIC void* __stdcall GetInstance();
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SendCMD(void * instance, uint8_t*, int);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartNav(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopNav(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetGoal(void * instance, int);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall PauseGoal(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall ResumeGoal(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall CancelGoal(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall InsertGoal(void * instance, float x, float y);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall ResetGoal(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetSpeed(void * instance, float vLinear, float vAngle);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall Shutdown(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetAngle(void * instance, uint8_t sign, uint8_t angle);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartLoop(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopLoop(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SetLoopWaitTime(void * instance, uint8_t time);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartMapping(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopMapping(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SaveMap(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall UpdateMap(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartChargeLocal(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopChargeLocal(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall SaveChargeBasePosition(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StartCharge(void * instance, float x, float y);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall StopCharge(void * instance);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall MoveTo(void * instance, float x, float y, uint8_t &goalNum);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall GetGoalNum(void * instance, uint8_t &goalNum);
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall GetCurrentStatus(void * instance, uint8_t* status_json, size_t &length);
        DLL_PUBLIC void __stdcall SetCurrentStatusCallback(void * instance, void(*callback)(
            GALILEO_RETURN_CODE, uint8_t* status_json, size_t length));
        DLL_PUBLIC void __stdcall SetGoalReachedCallback(
            void * instance,
            void(*callback)(int goalID, uint8_t* status_json, size_t length));
        DLL_PUBLIC GALILEO_RETURN_CODE __stdcall WaitForGoal(void * instance, int goalID);
    }
}; // namespace GalileoSDK

#endif // !__GALILEO_SDK_H__
