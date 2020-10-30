#ifndef __M_UTILS_H__
#define __M_UTILS_H__
#include <vector>
#include <stdio.h>
#include <chrono>
#include <mutex>
#include <random>
#include <sstream>
#include <iostream>
#include <string>
#include "json.hpp"
#include "galileo_serial_server/GalileoStatus.h"

#if defined(__ANDROID__) && __ANDROID_API__ < 24
#include "ifaddrs1.h"
/* __ANDROID_API__ < 24 */
#elif !defined(_WIN32)
#include <ifaddrs.h>
/* __ANDROID_API__ > 24 or Linux*/
#endif 

namespace GalileoSDK
{
class Utils
{
public:
#ifdef _WIN32
	static __int64 GetCurrentTimestamp();

#else
	static int64_t GetCurrentTimestamp();
#endif
	static std::vector<std::string> ListIpAddresses();

	static std::vector<std::string> ListMac();

	static std::string GenID();

	static std::string GetCurrentWorkingDir(void);

	static std::string IDToDeviceName(std::string id);

	static nlohmann::json statusToJson(galileo_serial_server::GalileoStatus status);

	static galileo_serial_server::GalileoStatus jsonToStatus(nlohmann::json j);

	static void mkdirs(std::string path);

    static unsigned int RandomChar() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 255);
        return dis(gen);
    }

    static std::string GenHex(const unsigned int len) {
        std::stringstream ss;
        for (auto i = 0; i < len; i++) {
            const auto rc = RandomChar();
            std::stringstream hexstream;
            hexstream << std::hex << rc;
            auto hex = hexstream.str();
            ss << (hex.length() < 2 ? '0' + hex : hex);
        }
        return ss.str();
    }

    static std::vector<std::string> Split(const std::string& text, char spliter) {
        std::string tmp;
        std::vector<std::string> stk;
        std::stringstream ss(text);
        while (getline(ss, tmp, spliter)) {
            stk.push_back(tmp);
        }
        return stk;
    }
};

} // namespace GalileoSDK

#endif // !__M_UTILS_H__
