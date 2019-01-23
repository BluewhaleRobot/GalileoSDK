#pragma once
#ifndef __GALILEO_SDK_H__
#define __GALILEO_SDK_H__

#include "stdafx.h"
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

namespace GalileoSDK {

	enum GALILEO_RETURN_CODE {
		OK, // 执行成功
		NOT_CONNECTED, // 没有连接到机器人
		INVALIDE_STATE, // 当前状态无法执行此操作
		NO_SERVER_FOUND, // 没有发现机器人
		MULTI_SERVER_FOUND, // 发现多个机器人
	};

	class DLL_PUBLIC GalileoSDK {
	public:
		GalileoSDK();
		GALILEO_RETURN_CODE Connect(std::string targetID, bool auto_connect, bool reconnect, int timeout,
			GALILEO_RETURN_CODE(*OnConnect)(std::string), GALILEO_RETURN_CODE (*OnDisconnect)(std::string));
		GALILEO_RETURN_CODE WaitForConnect(std::string targetID, bool auto_connect, bool reconnect, int timeout,
			GALILEO_RETURN_CODE(*OnConnect)(std::string), GALILEO_RETURN_CODE(*OnDisconnect)(std::string));
		GALILEO_RETURN_CODE StartNavigation(GALILEO_RETURN_CODE(*OnConnect)(std::string));
		GALILEO_RETURN_CODE WaitForStartNavigation(GALILEO_RETURN_CODE(*OnConnect)(std::string));
		//DLL_PUBLIC std::vector<ServerInfo>  GetServersOnline();
		std::vector<ServerInfo> GetServersOnline();
		ServerInfo currentServer;
	private:
		BroadcastReceiver broadcastReceiver;
	};

	extern "C" DLL_PUBLIC GalileoSDK* GetSDK();
};

#endif // !__GALILEO_SDK_H__
