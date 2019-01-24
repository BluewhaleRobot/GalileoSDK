#pragma once
#ifndef __BROADCAST_RECEIVER_H__
#define __BROADCAST_RECEIVER_H__

#include <vector>
#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#endif
#define BUFLEN 512 * 1024
#define BROADCAST_PORT 22002
#include "ros/ros.h"
#include "models/ServerInfo.h"
#include <json/json.h>
#include <regex>
#include <iostream>
#include <Ws2tcpip.h>
#include "Utils.h"

namespace GalileoSDK {
	class GalileoSDK;
	class BroadcastReceiver {
	public:
		BroadcastReceiver();
		static void StopAll();
		void StopTask();
		static std::vector<ServerInfo> GetServers();
		void Run();
		void SetSDK(GalileoSDK*);
	private:
		SOCKET serverSocket;
		struct sockaddr_in server, si_other;
		bool runningFlag;
		std::vector<ServerInfo> serverList;
		static BroadcastReceiver* instance;
		GalileoSDK* sdk;
	};
}

#endif // !__BROADCAST_RECEIVER_H__
