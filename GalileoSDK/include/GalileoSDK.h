#pragma once
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
#include <mutex>
#include <thread>

namespace GalileoSDK {

    //enum GALILEO_RETURN_CODE {
    //    OK,                 // 执行成功
    //    NOT_CONNECTED,      // 没有连接到机器人
    //    INVALIDE_STATE,     // 当前状态无法执行此操作
    //    NO_SERVER_FOUND,    // 没有发现机器人
    //    MULTI_SERVER_FOUND, // 发现多个机器人
    //    NETWORK_ERROR,      // 网络环境错误
    //    ALREADY_CONNECTED,  // 已经连接过了
    //    TIMEOUT,            // 操作超时
    //    SERVER_ERROR,       // 服务端错误
    //    GOAL_CANCELLED,     // 任务取消
    //    INVALIDE_GOAL,      // 无效目标
    //};

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
    };

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
        ServerInfo *currentServer;
        GALILEO_RETURN_CODE PublishTest();
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
        GALILEO_RETURN_CODE stopChargeLocal();
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
        ~GalileoSDK();

    private:
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
}; // namespace GalileoSDK

#endif // !__GALILEO_SDK_H__
