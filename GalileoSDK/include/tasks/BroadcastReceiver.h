#pragma once
#ifndef __BROADCAST_RECEIVER_H__
#define __BROADCAST_RECEIVER_H__

#include <vector>
#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#endif
#define BUFLEN 512 * 1024
#define BROADCAST_PORT 22002
#include "Utils.h"
#include "models/ServerInfo.h"
#include "ros/ros.h"
#include <Ws2tcpip.h>
#include <iostream>
#include <json/json.h>
#include <mutex>
#include <regex>

namespace GalileoSDK {
class GalileoSDK;
class BroadcastReceiver {
public:
  BroadcastReceiver();
  static void StopAll();
  void StopTask();
  static std::vector<ServerInfo> GetServers();
  void Run();
  void SetSDK(GalileoSDK *);

private:
  SOCKET serverSocket;
  struct sockaddr_in server, si_other;
  bool runningFlag;
  std::vector<ServerInfo> serverList;
  static BroadcastReceiver *instance;
  GalileoSDK *sdk;
  std::mutex serversLock;
};
} // namespace GalileoSDK

#endif // !__BROADCAST_RECEIVER_H__
