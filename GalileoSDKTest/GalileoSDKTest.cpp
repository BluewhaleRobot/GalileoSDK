// GalileoSDKTest.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "GalileoSDK.h"

int main()
{
	GalileoSDK::GalileoSDK sdk;
	while (true) {
		auto servers = sdk.GetServersOnline();
		for (auto it = servers.begin(); it < servers.end(); it++) {
			std::cout << it->getID() << std::endl;
		}
		Sleep(1000);
	}
    return 0;
}

