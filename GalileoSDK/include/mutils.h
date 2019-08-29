#ifndef __M_UTILS_H__
#define __M_UTILS_H__
#include <vector>
#include <stdio.h>
#include <chrono>
#include <mutex>
#include <random>
#include <sstream>
#include <iostream>
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
};

} // namespace GalileoSDK

#endif // !__M_UTILS_H__
