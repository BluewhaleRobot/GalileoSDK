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
#include <linux/if_packet.h>
#endif
#include <chrono>
#include <mutex>
#include <random>
#include <sstream>
#include <iostream>
#include "json.hpp"
#include "galileo_serial_server/GalileoStatus.h"

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

    static std::vector<std::string> ListMac()
    {
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
                    if ((uint8_t)(pAdapter->Address[i]) < 16)
                    {
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
                if (ifname.find("lo") == 0 || ifname.find("docker") == 0 || ifname.find("virtual") == 0)
                    continue;
                ips.push_back(addressBuffer);
            }
        }
        if (ifAddrStruct != NULL)
            freeifaddrs(ifAddrStruct);
        return ips;
    }

    static std::vector<std::string> ListMac()
    {
        struct ifaddrs *ifAddrStruct = NULL;
        struct ifaddrs *ifa = NULL;
        std::vector<std::string> macs;

        getifaddrs(&ifAddrStruct);

        for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (!ifa->ifa_addr)
            {
                continue;
            }
            if (ifa->ifa_addr->sa_family == AF_PACKET)
            { // check it is IP4
                // is a valid IP4 Address
                std::string ifname(ifa->ifa_name);
                if (ifname.find("lo") == 0 || ifname.find("docker") == 0 || ifname.find("virtual") == 0)
                    continue;
                struct sockaddr_ll *s = (struct sockaddr_ll*)ifa->ifa_addr;

                std::stringstream mac_ss;
                for (int i = 0; i < 6; i++)
                {
                    if (s->sll_addr[i] < 16)
                    {
                        mac_ss << 0 << std::uppercase << std::hex << (int)(s->sll_addr[i]);
                    }
                    else
                        mac_ss << std::uppercase << std::hex << (int)(s->sll_addr[i]);
                }
                macs.push_back(mac_ss.str());
            }
        }
        if (ifAddrStruct != NULL)
            freeifaddrs(ifAddrStruct);
        return macs;
    }
#endif

    static std::string GenID()
    {
        std::stringstream id;
        std::stringstream mac;
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dice(0, 255);
        for (int i = 0; i < 8; i++)
        {
            uint8_t dice_roll = dice(rng);
            if (dice_roll < 16)
                mac << 0 << std::uppercase << std::hex << dice_roll;
            else
                mac << std::uppercase << std::hex << (unsigned int)dice_roll;
        }
        auto macs = ListMac();
        if (macs.size() != 0)
        {
            id << macs[0];
        }
        else
        {
            id << mac.str();
        }
        for (int i = 0; i < 64; i++)
        {
            id << "A";
        }
        return id.str();
    }

    static std::string GetCurrentWorkingDir(void)
    {
        char buff[FILENAME_MAX];
		if (GetCurrentDir(buff, FILENAME_MAX) != 0) {
			return "";
		}
        std::string current_working_dir(buff);
        return current_working_dir;
    }

    static std::string IDToDeviceName(std::string id)
    {
        return id.substr(0, 32);
    }

    static nlohmann::json statusToJson(galileo_serial_server::GalileoStatus status)
    {
        nlohmann::json rootValue;
        rootValue["timestamp"] = (status.header.stamp.toNSec() / 1000 / 1000);
        rootValue["angleGoalStatus"] = status.angleGoalStatus;
        rootValue["busyStatus"] = status.busyStatus;
        rootValue["chargeStatus"] = status.chargeStatus;
        rootValue["controlSpeedTheta"] = status.controlSpeedTheta;
        rootValue["controlSpeedX"] = status.controlSpeedX;
        rootValue["currentAngle"] = status.currentAngle;
        rootValue["currentPosX"] = status.currentPosX;
        rootValue["currentPosY"] = status.currentPosY;
        rootValue["currentSpeedTheta"] = status.currentSpeedTheta;
        rootValue["currentSpeedX"] = status.currentSpeedX;
        rootValue["gbaStatus"] = status.gbaStatus;
        rootValue["gcStatus"] = status.gcStatus;
        rootValue["loopStatus"] = status.loopStatus;
        rootValue["mapStatus"] = status.mapStatus;
        rootValue["navStatus"] = status.navStatus;
        rootValue["power"] = status.power;
        rootValue["targetDistance"] = status.targetDistance;
        rootValue["targetNumID"] = status.targetNumID;
        rootValue["targetStatus"] = status.targetStatus;
        rootValue["visualStatus"] = status.visualStatus;
        return rootValue;
    }

    static galileo_serial_server::GalileoStatus jsonToStatus(nlohmann::json j)
    {
        galileo_serial_server::GalileoStatus status;
        size_t timestamp = j["timestamp"];
        status.header.stamp.fromNSec(timestamp * 1000 * 1000);
        j.at("angleGoalStatus").get_to(status.angleGoalStatus);
        j.at("busyStatus").get_to(status.busyStatus);
        j.at("chargeStatus").get_to(status.chargeStatus);
        j.at("controlSpeedTheta").get_to(status.controlSpeedTheta);
        j.at("controlSpeedX").get_to(status.controlSpeedX);
        j.at("currentAngle").get_to(status.currentAngle);
        j.at("currentPosX").get_to(status.currentPosX);
        j.at("currentPosY").get_to(status.currentPosY);
        j.at("currentSpeedTheta").get_to(status.currentSpeedTheta);
        j.at("currentSpeedX").get_to(status.currentSpeedX);
        j.at("gbaStatus").get_to(status.gbaStatus);
        j.at("gcStatus").get_to(status.gcStatus);
        j.at("loopStatus").get_to(status.loopStatus);
        j.at("mapStatus").get_to(status.mapStatus);
        j.at("navStatus").get_to(status.navStatus);
        j.at("power").get_to(status.power);
        j.at("targetDistance").get_to(status.targetDistance);
        j.at("targetNumID").get_to(status.targetNumID);
        j.at("targetStatus").get_to(status.targetStatus);
        j.at("visualStatus").get_to(status.visualStatus);
        return status;
    }
};

} // namespace GalileoSDK

#endif // !__UTILS_H__
