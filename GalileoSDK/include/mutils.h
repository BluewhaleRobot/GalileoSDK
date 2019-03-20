#ifndef __UTILS_H__
#define __UTILS_H__
#include <vector>
#include <stdio.h>
#ifdef _WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#include <Iphlpapi.h>
#pragma comment(lib, "IPHLPAPI.lib")
#else
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
#include <chrono>
#include <mutex>
#include <random>
#include <sstream>
#include <iostream>

#if defined(__ANDROID__) && __ANDROID_API__ < 24
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

    static std::vector<std::string> ListMac() {
        std::vector<std::string> myMacs;

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

                std::stringstream mac_ss;
                for (int i = 0; i < 6; i++)
                {
                    int mac_byte = (int)(pAdapter->Address[i]);
                    if ((uint8_t)(pAdapter->Address[i]) < 16) {
                        mac_ss << 0 << std::uppercase << std::hex << mac_byte;
                    }
                    else 
                    {
                        mac_ss << std::uppercase << std::hex << mac_byte;
                    }
                }
                myMacs.push_back(mac_ss.str());
                pAdapter = pAdapter->Next;
            }
        }

        return myMacs;
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

    static std::string GenID() {
        std::stringstream id;
        std::stringstream mac;
        for (int i = 0; i < 64; i++) {
            id << "A";
        }
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dice(0, 255);
        for (int i = 0; i < 8; i++) {
            uint8_t dice_roll = dice(rng);
            if(dice_roll < 16)
                mac << 0 << std::uppercase << std::hex << dice_roll;
            else
                mac << std::uppercase <<  std::hex << (unsigned int)dice_roll;
        }
        auto macs = ListMac();
        if (macs.size() != 0) {
            id << macs[0];
        }
        else {
            id << mac.str();
        }
        return id.str();
    }



    static std::string GetCurrentWorkingDir(void) {
        char buff[FILENAME_MAX];
        GetCurrentDir(buff, FILENAME_MAX);
        std::string current_working_dir(buff);
        return current_working_dir;
    }
};

} // namespace GalileoSDK

#endif // !__UTILS_H__
