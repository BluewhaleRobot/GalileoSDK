#ifndef __UTILS_H__
#define __UTILS_H__
#include <vector>
#ifdef _WIN32
#include <Iphlpapi.h>
#pragma comment(lib, "IPHLPAPI.lib")
#else
#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include <chrono>
#include <mutex>
#if __ANDROID_API__ < 24
#include "ifaddrs1.h"
#endif /* __ANDROID_API__ < 24 */

namespace GalileoSDK
{
class Utils
{
  public:
#ifdef _WIN32
    static __int64 GetCurrentTimestamp()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    static std::vector<std::string> ListIpAddresses()
    {

        std::vector<std::string> myIPs;

        PIP_ADAPTER_INFO pAdapterInfo;
        pAdapterInfo = (IP_ADAPTER_INFO *)malloc(sizeof(IP_ADAPTER_INFO));
        ULONG buflen = sizeof(IP_ADAPTER_INFO);

        if (GetAdaptersInfo(pAdapterInfo, &buflen) == ERROR_BUFFER_OVERFLOW)
        {
            free(pAdapterInfo);
            pAdapterInfo = (IP_ADAPTER_INFO *)malloc(buflen);
        }

        if (GetAdaptersInfo(pAdapterInfo, &buflen) == NO_ERROR)
        {
            PIP_ADAPTER_INFO pAdapter = pAdapterInfo;
            while (pAdapter)
            {
                std::string desp = std::string(pAdapter->Description);
                if (desp.find("Virtual") != std::string::npos ||
                    desp.find("Loopback") != std::string::npos ||
                    desp.find("virtual") != std::string::npos ||
                    desp.find("loopback") != std::string::npos)
                {
                    pAdapter = pAdapter->Next;
                    continue;
                }
                myIPs.push_back(pAdapter->IpAddressList.IpAddress.String);
                pAdapter = pAdapter->Next;
            }
        }

        return myIPs;
    }
#else
    static int64_t GetCurrentTimestamp()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    static std::vector<std::string> ListIpAddresses()
    {
        struct ifaddrs *ifAddrStruct = NULL;
        struct ifaddrs *ifa = NULL;
        void *tmpAddrPtr = NULL;
        std::vector<std::string> ips;

        getifaddrs(&ifAddrStruct);

        for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (!ifa->ifa_addr)
            {
                continue;
            }
            if (ifa->ifa_addr->sa_family == AF_INET)
            { // check it is IP4
                // is a valid IP4 Address
                tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
                char addressBuffer[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
                std::string ifname(ifa->ifa_name);
                if(ifname.find("lo") == 0 || ifname.find("docker") == 0 || ifname.find("virtual") == 0)
                    continue;
                ips.push_back(addressBuffer);
            }
        }
        if (ifAddrStruct != NULL)
            freeifaddrs(ifAddrStruct);
        return ips;
    }
#endif
};

} // namespace GalileoSDK

#endif // !__UTILS_H__
