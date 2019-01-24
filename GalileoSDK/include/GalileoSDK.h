#pragma once
#ifndef __GALILEO_SDK_H__
#define __GALILEO_SDK_H__

#include <iostream>

#ifdef _WIN32
#define WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif

#include "Dll.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "galileo_serial_server/GalileoNativeCmds.h"
#include "galileo_serial_server/GalileoStatus.h"
#include "models/ServerInfo.h"
#include "tasks/BroadcastReceiver.h"
#include <thread>
#include <mutex>

namespace GalileoSDK {

	enum GALILEO_RETURN_CODE {
		OK, // 执行成功
		NOT_CONNECTED, // 没有连接到机器人
		INVALIDE_STATE, // 当前状态无法执行此操作
		NO_SERVER_FOUND, // 没有发现机器人
		MULTI_SERVER_FOUND, // 发现多个机器人
		NETWORK_ERROR, //网络环境错误
		ALREADY_CONNECTED, // 已经连接过了
		TIMEOUT, // 操作超时
	};

	class DLL_PUBLIC GalileoSDK {
	public:
		GalileoSDK();
		GALILEO_RETURN_CODE Connect(std::string targetID, bool auto_connect, bool reconnect, int timeout,
			void(*OnConnect)(GALILEO_RETURN_CODE, std::string), void(*OnDisconnect)(GALILEO_RETURN_CODE, std::string));
		GALILEO_RETURN_CODE Connect(ServerInfo server);
		GALILEO_RETURN_CODE WaitForConnect(std::string targetID, bool auto_connect, bool reconnect, int timeout,
			GALILEO_RETURN_CODE(*OnConnect)(std::string), GALILEO_RETURN_CODE(*OnDisconnect)(std::string));
		GALILEO_RETURN_CODE StartNavigation(GALILEO_RETURN_CODE(*OnConnect)(std::string));
		GALILEO_RETURN_CODE WaitForStartNavigation(GALILEO_RETURN_CODE(*OnConnect)(std::string));
		//DLL_PUBLIC std::vector<ServerInfo>  GetServersOnline();
		std::vector<ServerInfo> GetServersOnline();
		ServerInfo* currentServer;
		GALILEO_RETURN_CODE PublishTest();
		GALILEO_RETURN_CODE GetCurrentStatus(galileo_serial_server::GalileoStatus* status);
		static GalileoSDK* GetInstance();
		void broadcastOfflineCallback(std::string id);
		~GalileoSDK();

	private:
		BroadcastReceiver broadcastReceiver;
		ros::NodeHandle* nh;
		ros::Publisher testPub;
		ros::Subscriber galileoStatusSub;
		void UpdateGalileoStatus(const galileo_serial_server::GalileoStatusConstPtr &status);
		galileo_serial_server::GalileoStatusConstPtr currentStatus;
		std::mutex statusLock;
		void SpinThread();
		void(*OnDisconnect)(GALILEO_RETURN_CODE, std::string);
		void(*OnConnect)(GALILEO_RETURN_CODE, std::string);
		bool connectingTaskFlag;
		bool reconnectFlag;
		static GalileoSDK* instance;
	};
};

#endif // !__GALILEO_SDK_H__
