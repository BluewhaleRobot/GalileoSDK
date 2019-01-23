#pragma once
#ifndef __UTILS_H__
#define __UTILS_H__

#include <chrono>
#include <Iphlpapi.h>
#pragma comment(lib, "IPHLPAPI.lib")

namespace GalileoSDK{
	class Utils {
	public:
		static __int64 GetCurrentTimestamp() {
			return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		}

		static std::string GetMyIP() {
			IP_ADAPTER_ADDRESSES myIP;
			ULONG buf_size;
			buf_size = sizeof(myIP);
			GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_UNICAST, NULL, &myIP, &buf_size);
		}

//		struct IPv4
//		{
//			unsigned char b1, b2, b3, b4;
//		};
//
//		static bool getMyIP(IPv4 & myIP)
//		{
//			char szBuffer[1024];
//
//#ifdef WIN32
//			WSADATA wsaData;
//			WORD wVersionRequested = MAKEWORD(2, 0);
//			if (::WSAStartup(wVersionRequested, &wsaData) != 0)
//				return false;
//#endif
//
//
//			if (gethostname(szBuffer, sizeof(szBuffer)) == SOCKET_ERROR)
//			{
//#ifdef WIN32
//				WSACleanup();
//#endif
//				return false;
//			}
//
//			struct hostent *host = gethostbyname(szBuffer);
//			if (host == NULL)
//			{
//#ifdef WIN32
//				WSACleanup();
//#endif
//				return false;
//			}
//
//			//Obtain the computer's IP
//			myIP.b1 = ((struct in_addr *)(host->h_addr))->S_un.S_un_b.s_b1;
//			myIP.b2 = ((struct in_addr *)(host->h_addr))->S_un.S_un_b.s_b2;
//			myIP.b3 = ((struct in_addr *)(host->h_addr))->S_un.S_un_b.s_b3;
//			myIP.b4 = ((struct in_addr *)(host->h_addr))->S_un.S_un_b.s_b4;
//
//#ifdef WIN32
//			WSACleanup();
//#endif
//			return true;
//		}
//
//		static std::string getMyIP() {
//			IPv4 myIP;
//			if (!getMyIP(myIP))
//				return "";
//			std::stringstream ipStr;
//			ipStr << myIP.b1 << "." << myIP.b2 << "." << myIP.b3 << "." << myIP.b4;
//			return ipStr.str();
//		}
	};
}

#endif // !__UTILS_H__
