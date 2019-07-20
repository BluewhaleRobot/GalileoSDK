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
#include "../mutils.h"
#ifdef _WIN32
#include <Ws2tcpip.h>
#endif
#include "../models/ServerInfo.h"
#include "ros/ros.h"
#include <iostream>
#include "../json.hpp"
#include <mutex>
#include <regex>

namespace GalileoSDK
{
class GalileoSDK;
class BroadcastReceiver
{
public:
  BroadcastReceiver();
  static void StopAll();
  void StopTask();
  static std::vector<ServerInfo> GetServers();
  void Run();
  void AddSDK(GalileoSDK *);
  void RemoveSDK(GalileoSDK *);

private:
#ifdef _WIN32
  SOCKET serverSocket;
#else
  int serverSocket;
#endif
  struct sockaddr_in server, si_other;
  bool runningFlag;
  std::vector<ServerInfo> serverList;
  static BroadcastReceiver *instance;
  static std::set<GalileoSDK *> sdks;
  std::recursive_mutex serversLock;
  bool stoppedFlag;
};
} // namespace GalileoSDK

#endif // !__BROADCAST_RECEIVER_H__
