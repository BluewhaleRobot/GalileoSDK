#pragma once
#ifndef __UTILS_H__
#define __UTILS_H__

#include <chrono>

namespace GalileoSDK{
	class Utils {
	public:
		static __int64 GetCurrentTimestamp() {
			return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		}
	};
}

#endif // !__UTILS_H__
