// GalileoSDKTest.cpp: 定义控制台应用程序的入口点。
//
#define INFRA_LOG
#include "GalileoSDK.h"
#include "galileo_serial_server/GalileoStatus.h"

#ifndef _WIN32
void Sleep(uint64_t miliseconds)
{
    usleep(miliseconds * 1000);
}
#endif

/// 测试发布消息
void testPub()
{
    GalileoSDK::GalileoSDK sdk;
    while (true)
    {
        auto servers = sdk.GetServersOnline();
        if(servers.size() == 0){
            std::cout << "No server found" << std::endl;
        }
        for (auto it = servers.begin(); it < servers.end(); it++)
        {
            std::cout << it->getID() << std::endl;
            sdk.Connect("", true, 10000, NULL, NULL);
        }
        sdk.PublishTest();
        Sleep(1000);
    }
}

/// 测试订阅消息
void testSub()
{
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("", true, 10000, NULL, NULL);
    while (true)
    {
        galileo_serial_server::GalileoStatus *status =
            new galileo_serial_server::GalileoStatus();
        if (sdk.GetCurrentStatus(status) == GalileoSDK::OK)
        {
            std::cout << status->power << std::endl;
        }
        else
        {
            std::cout << "Get status failed" << std::endl;
        }
        Sleep(1000);
    }
}

bool connected = false;
void testConnectWithCallback()
{
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
    while (!connected)
    {
        Sleep(1);
    }
}

bool connectCallbackFlag = false;
void testReconnect()
{
    connected = false;
    connectCallbackFlag = false;
    GalileoSDK::GalileoSDK sdk;
	galileo_serial_server::GalileoStatus status;
    while (true)
    {
        if (!connected)
        {
			sdk.Dispose();
			std::cout << "Start connect" << std::endl;
            auto res = sdk.Connect(
                "8FB56D27D6C961E9036F62182ADE9544D71E23C31E5DF4C7DD692B9E4296A131434B1066D365", true, 3000,
                [](GalileoSDK::GALILEO_RETURN_CODE res, std::string id) -> void {
                    std::cout << "OnConnect Callback: result " << res << std::endl;
                    std::cout << "OnConnect Callback: connected to " << id << std::endl;
                    connectCallbackFlag = true;
                    if (res == GalileoSDK::GALILEO_RETURN_CODE::OK || res == GalileoSDK::GALILEO_RETURN_CODE::ALREADY_CONNECTED)
                    {
                        connected = true;
                    }
                },
                [](GalileoSDK::GALILEO_RETURN_CODE res, std::string id) -> void {
                    std::cout << "OnDisconnect Callback: result " << res << std::endl;
                    std::cout << "OnDisconnect Callback: server " << id << std::endl;
                    connected = false;
                });
			std::cout << "Connect RES: " << res << std::endl;
			auto servers = sdk.GetServersOnline();
			for (int i = 0; i < servers.size(); i++) {
				std::cout << servers[i].getID() << std::endl;
			}
            while (!connectCallbackFlag)
            {
                Sleep(1000);
				std::cout << "Waitting callback" << std::endl;
            }
            connectCallbackFlag = false;
        }
        Sleep(1000);
        std::cout << "waitting" << std::endl;
		if (sdk.GetCurrentServer() != NULL) {
			std::cout << sdk.GetCurrentServer()->getID() << std::endl;
			sdk.GetCurrentStatus(&status);
			std::cout << status.power << std::endl;
		}
		else {
			std::cout << "current server is null" << std::endl;
		}
		
    }
}

void testSendGalileoCmd()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        uint8_t cmd[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
        sdk.SendCMD(cmd, 6);
        Sleep(1000);
    }
}

void testStartNav()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        std::cout << "status: " << status.navStatus << std::endl;
        std::cout << "start nav" << std::endl;
        sdk.StartNav();
        Sleep(1000 * 20);
        sdk.GetCurrentStatus(&status);
        std::cout << "status: " << status.navStatus << std::endl;
        std::cout << "stop nav" << std::endl;
        sdk.StopNav();
        Sleep(1000 * 20);
        sdk.GetCurrentStatus(&status);
        std::cout << "status: " << status.navStatus << std::endl;
    }
}

void testSetSpeed()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0.1, 0);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0, 1);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(-0.1, 0);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0, -1);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0, 0);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
    }
}

void testSetAngle()
{
}

void testGoals()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        // 开启导航
        sdk.StartNav();
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        // 等待正常追踪
        while (status.visualStatus != 1 || status.navStatus != 1)
        {
            std::cout << "Wait for navigation initialization" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }

        // 设置目标点
        sdk.SetGoal(0);
        // 等待goal status
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 1)
        {
            std::cout << "Wait for goal start" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal started" << std::endl;
        // 暂停目标
        Sleep(2 * 1000);
        sdk.PauseGoal();
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 2)
        {
            std::cout << "Wait for goal pause" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal paused" << std::endl;
        // 继续目标
        Sleep(2 * 1000);
        sdk.ResumeGoal();
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 1)
        {
            std::cout << "Wait for goal resume" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal resumed" << std::endl;
        // 取消目标
        Sleep(2 * 1000);
        sdk.CancelGoal();
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 0 || status.targetNumID != -1)
        {
            std::cout << "Wait for goal cancel" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal cancelled" << std::endl;
        // 再次设置目标
        Sleep(2 * 1000);
        std::cout << "Set goal again" << std::endl;
        sdk.SetGoal(0);
        // 完成目标
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 1)
        {
            std::cout << "Wait for goal start" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal started" << std::endl;
        while (status.targetStatus != 0 || status.targetNumID != 0)
        {
            std::cout << "Wait for goal complete" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        // 移动到特定目标
        // 获取当前坐标
        sdk.GetCurrentStatus(&status);
        float pos0_x = status.currentPosX;
        float pos0_y = status.currentPosY;
        // 再次设置目标 ，移动至1号目标点
        Sleep(2 * 1000);
        std::cout << "Set goal again" << std::endl;
        sdk.SetGoal(1);
        // 完成目标
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 1)
        {
            std::cout << "Wait for goal start" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal started" << std::endl;
        while (status.targetStatus != 0 || status.targetNumID != 1)
        {
            std::cout << "Wait for goal complete" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        // 再次获取坐标
        sdk.GetCurrentStatus(&status);
        float pos1_x = status.currentPosX;
        float pos1_y = status.currentPosY;
        // 然后移动至0号和1号之间
        uint8_t goalNum;
        std::cout << "Start move to " << (pos0_x + pos1_x) / 2 << " " << (pos0_y + pos1_y) / 2 << std::endl;
        sdk.MoveTo((pos0_x + pos1_x) / 2, (pos0_y + pos1_y) / 2, &goalNum);
        // 等待移动完成
        sdk.GetCurrentStatus(&status);
        while (status.targetStatus != 1)
        {
            std::cout << "Wait for goal start" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Goal started" << std::endl;
        while (status.targetStatus != 0 || status.targetNumID != goalNum)
        {
            std::cout << "Wait for goal complete" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Move to " << (pos0_x + pos1_x) / 2 << " " << (pos0_y + pos1_y) / 2 << " complete" << std::endl;
        Sleep(10 * 1000);
        std::cout << "All complete" << std::endl;
    }
}

void testLoop()
{
}

void testMapping()
{
}

void testAutoCharge()
{
}

void testShutdown()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        std::cout << "Send Shutdown" << std::endl;
        sdk.Shutdown();
        Sleep(10 * 1000);
    }
}

void testGetParams()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        uint8_t goalnum;
        sdk.GetGoalNum(&goalnum);
        std::cout << "/galileo/goal_num: " << (int)goalnum << std::endl;
        Sleep(10 * 1000);
    }
}

void testNavigationLoop()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        // 开启导航
        sdk.StartNav();
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        // 等待正常追踪
        while (status.visualStatus != 1 || status.navStatus != 1)
        {
            std::cout << "Wait for navigation initialization" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Start loop" << std::endl;
        sdk.StartLoop();
        Sleep(2 * 60 * 1000);
        std::cout << "Stop loop" << std::endl;
        sdk.StopLoop();
        std::cout << "Set wait time to 10s" << std::endl;
        sdk.SetLoopWaitTime(10);
        std::cout << "Start loop" << std::endl;
        sdk.StartLoop();
        Sleep(2 * 60 * 1000);
        std::cout << "Stop loop" << std::endl;
        sdk.StopLoop();
        Sleep(2 * 60 * 1000);
    }
}

void testCreateMap()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9"
                    "C730A614FF2790C1",
                    true, 10000, NULL,
                    NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        // 开启建图
        sdk.StartMapping();
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        // 等待开始建图
        while (status.visualStatus == -1 || status.mapStatus != 1)
        {
            std::cout << "Wait for map start" << std::endl;
            sdk.GetCurrentStatus(&status);
            Sleep(1000);
        }
        std::cout << "Mapping started" << std::endl;
        std::cout << "You can start to move the robot around" << std::endl;
        std::cout << "Press enter to save map" << std::endl;
        getchar();
        std::cout << "Start save map" << std::endl;
        sdk.SaveMap();
        std::cout << "Wait for save complete" << std::endl;
        // 等待保存完成
        Sleep(20 * 1000);
        // 停止建图
        sdk.StopMapping();
        Sleep(60 * 1000);
    }
}

void testRelease(){
    while(true){
        Sleep(5000);
        std::cout << "Create SDK instance" << std::endl;
        GalileoSDK::GalileoSDK sdk;
        Sleep(1000);
        auto servers = sdk.GetServersOnline();
        std::cout << "Release SDK instance" << std::endl;
    }
}

void testConnect_IOT() {
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("8FB56D27D6C961E9036F62182ADE9544D71E23C31E5DF4C7DD692B9E4296A131434B1066D365", 10000, "xiaoqiang", NULL, NULL);
    galileo_serial_server::GalileoStatus status;
    while (true)
    {
        if (sdk.GetCurrentStatus(&status) == GalileoSDK::OK)
        {
            std::cout << status.power << std::endl;
        }
        else
        {
            std::cout << "Get status failed" << std::endl;
        }
        Sleep(1000);
    }

}

void testPubIOT()
{
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang", NULL, NULL);
    while (true)
    {
        auto res = sdk.PublishTest();
        if (res == GalileoSDK::GALILEO_RETURN_CODE::OK)
            std::cout << "Send test msg" << std::endl;
        else
            std::cout << "Send test msg failed" << std::endl;
        Sleep(1000);
    }
}

void testAudioIOT() {
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang", NULL, NULL);
    while (true)
    {
        std::string string = u8"测试";
        auto res = sdk.SendAudio(string.c_str());
        if (res == GalileoSDK::GALILEO_RETURN_CODE::OK)
            std::cout << "Send audio msg" << std::endl;
        else
            std::cout << "Send audio msg failed" << std::endl;

        Sleep(4000);

    }
}


void testConnectIOTWithCallback()
{
    GalileoSDK::GalileoSDK sdk;
    connected = false;
    sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang",
        [](GalileoSDK::GALILEO_RETURN_CODE res, std::string id) -> void {
        std::cout << "OnConnect Callback: result " << res << std::endl;
        std::cout << "OnConnect Callback: connected to " << id
            << std::endl;
        connected = true;
    },
        NULL);
    while (!connected)
    {
        std::cout << "not connected" << std::endl;
        Sleep(1000);
    }

    std::cout << "connected" << std::endl;
    Sleep(2 * 1000);
}

void testConnectIOTWithWrongPassword() {
    GalileoSDK::GalileoSDK sdk;
    sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang1", NULL, NULL);
    while (true)
    {
        std::string string = u8"测试";
        auto res = sdk.SendAudio(string.c_str());
        if (res == GalileoSDK::GALILEO_RETURN_CODE::OK)
            std::cout << "Send audio msg" << std::endl;
        else
            std::cout << "Send audio msg failed" << std::endl;

        Sleep(4000);
    }
}

void testMultiSDK() {
    GalileoSDK::GalileoSDK sdk1;
    GalileoSDK::GalileoSDK sdk2;
    sdk1.Connect("", true, 10000, NULL, NULL);
    sdk2.Connect("", true, 10000, NULL, NULL);
    int timecout = 0;
    while (timecout < 10)
    {
        galileo_serial_server::GalileoStatus *status =
            new galileo_serial_server::GalileoStatus();
        if (sdk1.GetCurrentStatus(status) == GalileoSDK::OK)
        {
            std::cout << "sdk 1: " << status->power << std::endl;
        }
        else
        {
            std::cout << "sdk 1 Get status failed" << std::endl;
        }

        if (sdk2.GetCurrentStatus(status) == GalileoSDK::OK)
        {
            std::cout << "sdk 2: " << status->power << std::endl;
        }
        else
        {
            std::cout << "sdk 2 Get status failed" << std::endl;
        }

        Sleep(1000);
        timecout += 1;
    }
}

void testMultiSDKOffline() {
    GalileoSDK::GalileoSDK sdk1;
    GalileoSDK::GalileoSDK sdk2;
    sdk1.Connect("", true, 10000, NULL, NULL);
    sdk2.Connect("", true, 10000, NULL, NULL);
    while (true)
    {
        galileo_serial_server::GalileoStatus *status =
            new galileo_serial_server::GalileoStatus();
        if (sdk1.GetCurrentStatus(status) == GalileoSDK::OK)
        {
            std::cout << "sdk 1: " << status->power << std::endl;
        }
        else
        {
            std::cout << "sdk 1 Get status failed" << std::endl;
        }

        if (sdk2.GetCurrentStatus(status) == GalileoSDK::OK)
        {
            std::cout << "sdk 2: " << status->power << std::endl;
        }
        else
        {
            std::cout << "sdk 2 Get status failed" << std::endl;
        }

        Sleep(1000);
    }
}

void testMultiSDKWithCallback() {

}

void testMultiSDKDifferentServer() {

}

void testIOTSetSpeed()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang", NULL, NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        galileo_serial_server::GalileoStatus status;
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0.1, 0);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0, 1);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(-0.1, 0);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0, -1);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
        sdk.SetSpeed(0, 0);
        Sleep(5 * 1000);
        sdk.GetCurrentStatus(&status);
        std::cout << "currentSpeedX: " << status.currentSpeedX << std::endl;
        std::cout << "currentSpeedTheta: " << status.currentSpeedTheta << std::endl;
    }
}


void testIOTSendGalileoCmd()
{
    connected = false;
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang", NULL, NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        uint8_t cmd[] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
        sdk.SendCMD(cmd, 6);
        std::cout << "send galileo cmds" << std::endl;
        Sleep(1000);
    }
}

void testIOTDisconnect() {
    GalileoSDK::GalileoSDK sdk;
    if (sdk.Connect("71329A5B0F2D68364BB7B44F3F125531E4C7F5BC3BCE2694DFE39B505FF9C730A614FF2790C1", 10000, "xiaoqiang", NULL, NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
    {
        std::cout << "Connect to server failed" << std::endl;
    }
    while (true)
    {
        if (sdk.PublishTest() == GalileoSDK::GALILEO_RETURN_CODE::OK) {
            std::cout << "pub succeed" << std::endl;
        }
        else {
            std::cout << "pub failed" << std::endl;
        }
        Sleep(1000);
    }
}

void testAudio() {
	GalileoSDK::GalileoSDK sdk;
	if (sdk.Connect("F9DF41E6CA1C41CD8ECB510C3EF84A4472191922695EBA5A7514D459FC919608A2EF4FB50622",true, 10000, NULL, NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
	{
		std::cout << "Connect to server failed" << std::endl;
	}
	std::ifstream audioFile("audio.mp3", std::ios::binary);
	if (!audioFile.good()) {
		std::cout << "audio.mp3" << " not found." << std::endl;
		exit(-1);
	}

	std::vector<char> audioFileBuff((
		std::istreambuf_iterator<char>(audioFile)),
		(std::istreambuf_iterator<char>()));

	sdk.SendRawAudio((uint8_t*)(audioFileBuff.data()), audioFileBuff.size());
	Sleep(10000);
}

void testGreeting() {
	GalileoSDK::GalileoSDK sdk;
	if (sdk.Connect("8FB56D27D6C961E9036F62182ADE9544D71E23C31E5DF4C7DD692B9E4296A131434B1066D365", true, 300000, NULL, NULL) != GalileoSDK::GALILEO_RETURN_CODE::OK)
	{
		std::cout << "Connect to server failed" << std::endl;
	}
	sdk.EnableGreeting(true);
	Sleep(10 * 1000);
	sdk.EnableGreeting(false);
	Sleep(10 * 1000);
}

int main()
{
	testReconnect();
}
