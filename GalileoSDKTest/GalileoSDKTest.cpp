// GalileoSDKTest.cpp: 定义控制台应用程序的入口点。
//
#include "GalileoSDK.h"
#include "galileo_serial_server/GalileoStatus.h"


/// 测试发布消息
void testPub() {
    GalileoSDK::GalileoSDK sdk;
    auto servers = sdk.GetServersOnline();
    for (auto it = servers.begin(); it < servers.end(); it++) {
        std::cout << it->getID() << std::endl;
        sdk.Connect("", true, 10000, NULL, NULL);
    }
    sdk.PublishTest();
}

/// 测试订阅消息
void testSub() {
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("", true, 10000, NULL, NULL);
    while (true) {
        galileo_serial_server::GalileoStatus *status =
            new galileo_serial_server::GalileoStatus();
        if (sdk.GetCurrentStatus(status) == GalileoSDK::OK) {
            std::cout << status->power << std::endl;
        }
        else {
            std::cout << "Get status failed" << std::endl;
        }
        Sleep(1000);
    }
}

bool connected = false;
void testConnectWithCallback() {
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("", true, 10000,
        [](GalileoSDK::GALILEO_RETURN_CODE res, std::string id) -> void {
        std::cout << "OnConnect Callback: result " << res << std::endl;
        std::cout << "OnConnect Callback: connected to " << id
            << std::endl;
        connected = true;
    },
        NULL);
    while (!connected) {
        Sleep(1);
    }
}

bool connectCallbackFlag = false;
void testReconnect() {
    connected = false;
    connectCallbackFlag = false;
    GalileoSDK::GalileoSDK sdk;
    while (true) {
        if (!connected) {
            sdk.Connect(
                "", true, 10000,
                [](GalileoSDK::GALILEO_RETURN_CODE res, std::string id) -> void {
                std::cout << "OnConnect Callback: result " << res << std::endl;
                std::cout << "OnConnect Callback: connected to " << id << std::endl;
                connectCallbackFlag = true;
                if (res == GalileoSDK::GALILEO_RETURN_CODE::OK) {
                    connected = true;
                }
            },
                [](GalileoSDK::GALILEO_RETURN_CODE res, std::string id) -> void {
                std::cout << "OnDisconnect Callback: result " << res << std::endl;
                std::cout << "OnDisconnect Callback: server " << id << std::endl;
                connected = false;
            });
            while (!connectCallbackFlag) {
                Sleep(1000);
            }
            connectCallbackFlag = false;
        }
        Sleep(1000);
        std::cout << "waitting" << std::endl;
    }
}

void testSendGalileoCmd() {
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
        "C730A614FF2790C1",
        true, 10000, NULL,
        NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK) {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true) {
        uint8_t cmd[] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
        sdk.SendCMD(cmd, 6);
        Sleep(1000);
    }
}

void testStartNav() {
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
        "C730A614FF2790C1",
        true, 10000, NULL,
        NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK) {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        std::cout << "status: " << status.navStatus << std::endl;
        std::cout << "start nav" << std::endl;
        sdk.StartNav();
        Sleep(1000 * 10);
        sdk.GetCurrentStatus(&status);
        std::cout << "status: " << status.navStatus << std::endl;
        std::cout << "stop nav" << std::endl;
        sdk.StopNav();
        Sleep(1000 * 10);
        sdk.GetCurrentStatus(&status);
        std::cout << "status: " << status.navStatus << std::endl;
    }
}

int main() {
    testStartNav();
    return 0;
}
