#pragma once
#ifndef __UTILS_H__
#define __UTILS_H__

#include <Iphlpapi.h>
#include <chrono>
#include <mutex>
#pragma comment(lib, "IPHLPAPI.lib")

namespace GalileoSDK {
class Utils {
public:
  static __int64 GetCurrentTimestamp() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static std::vector<std::string> ListIpAddresses() {

    std::vector<std::string> myIPs;

    PIP_ADAPTER_INFO pAdapterInfo;
    pAdapterInfo = (IP_ADAPTER_INFO *)malloc(sizeof(IP_ADAPTER_INFO));
    ULONG buflen = sizeof(IP_ADAPTER_INFO);

    if (GetAdaptersInfo(pAdapterInfo, &buflen) == ERROR_BUFFER_OVERFLOW) {
      free(pAdapterInfo);
      pAdapterInfo = (IP_ADAPTER_INFO *)malloc(buflen);
    }

    if (GetAdaptersInfo(pAdapterInfo, &buflen) == NO_ERROR) {
      PIP_ADAPTER_INFO pAdapter = pAdapterInfo;
      while (pAdapter) {
        std::string desp = std::string(pAdapter->Description);
        if (desp.find("Virtual") != std::string::npos ||
            desp.find("Loopback") != std::string::npos ||
            desp.find("virtual") != std::string::npos ||
            desp.find("loopback") != std::string::npos) {
          pAdapter = pAdapter->Next;
          continue;
        }
        myIPs.push_back(pAdapter->IpAddressList.IpAddress.String);
        pAdapter = pAdapter->Next;
      }
    }

    return myIPs;
  }
};

} // namespace GalileoSDK

#endif // !__UTILS_H__
