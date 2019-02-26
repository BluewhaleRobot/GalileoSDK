#define BUILDING_DLL
#include "GalileoSDK.h"

namespace GalileoSDK
{

// Basic functions
#ifndef _WIN32
void Sleep(uint64_t miliseconds)
{
    usleep(miliseconds * 1000);
}
#endif

// Implementation of class GalileoSDK
// ////////////////////////////////
GalileoSDK *GalileoSDK::instance = NULL;

GalileoSDK::GalileoSDK()
    : currentServer(NULL), currentStatus(NULL), OnDisconnect(NULL), OnConnect(NULL), CurrentStatusCallback(NULL), GoalReachedCallback(NULL), connectingTaskFlag(false)
{
    new std::thread(&BroadcastReceiver::Run, &broadcastReceiver);
    instance = this;
}

GalileoSDK *GalileoSDK::GetInstance()
{
    return instance;
}

std::vector<ServerInfo> GalileoSDK::GetServersOnline()
{
    return BroadcastReceiver::GetServers();
}

GALILEO_RETURN_CODE GalileoSDK::Connect(
    std::string targetID = "",
    bool auto_connect = true,
    int timeout = 10 * 1000,
    void (*OnConnect)(GALILEO_RETURN_CODE, std::string) = NULL,
    void (*OnDisconnect)(GALILEO_RETURN_CODE, std::string) = NULL)
{
    if (currentServer != NULL)
    {
        return GALILEO_RETURN_CODE::ALREADY_CONNECTED;
    }
    if (OnDisconnect != NULL)
    {
        this->OnDisconnect = OnDisconnect;
    }
    this->auto_connect = auto_connect;
    this->timeout = timeout;
    this->targetID = targetID;

    // 设置了回调函数，采用异步方式执行
    if (OnConnect != NULL)
    {
        this->OnConnect = OnConnect;
        new std::thread([]() -> void {
            auto sdk = GetInstance();
            if (sdk->connectingTaskFlag)
                return;
            sdk->connectingTaskFlag = true;
            if (sdk->targetID.empty() && sdk->auto_connect)
            {
                int timecount = 0;
                std::vector<ServerInfo> servers;
                while (timecount < sdk->timeout)
                {
                    servers = BroadcastReceiver::GetServers();
                    if (servers.size() > 0)
                        break;
                    Sleep(100);
                    timecount += 100;
                }
                if (servers.size() == 1)
                {
                    sdk->currentServer = new ServerInfo();
                    *(sdk->currentServer) = servers.at(0);
                }
                if (servers.size() == 0 && sdk->OnConnect != NULL)
                {
                    sdk->OnConnect(GALILEO_RETURN_CODE::NO_SERVER_FOUND, "");
                    sdk->connectingTaskFlag = false;
                    return;
                }
                if (servers.size() > 1)
                {
                    sdk->OnConnect(GALILEO_RETURN_CODE::MULTI_SERVER_FOUND, "");
                    sdk->connectingTaskFlag = false;
                    return;
                }
            }
            if (!sdk->targetID.empty())
            {
                int timecount = 0;
                std::vector<ServerInfo> servers;
                while (timecount < sdk->timeout)
                {
                    servers = BroadcastReceiver::GetServers();
                    for (auto it = servers.begin(); it < servers.end(); it++)
                    {
                        if (it->getID() == sdk->targetID)
                        {
                            sdk->currentServer = new ServerInfo();
                            *(sdk->currentServer) = *it;
                            break;
                        }
                    }
                    Sleep(100);
                    timecount += 100;
                }
                if (sdk->currentServer == NULL && sdk->OnConnect != NULL)
                {
                    sdk->OnConnect(GALILEO_RETURN_CODE::NO_SERVER_FOUND, sdk->targetID);
                    sdk->connectingTaskFlag = false;
                    return;
                }
            }
            sdk->Connect(*(sdk->currentServer));
            if (sdk->OnConnect != NULL)
            {
                sdk->OnConnect(GALILEO_RETURN_CODE::OK, sdk->currentServer->getID());
            }
            sdk->connectingTaskFlag = false;
            return;
        });
        return GALILEO_RETURN_CODE::OK;
    }
    if (targetID.empty() && auto_connect)
    {
        int timecount = 0;
        std::vector<ServerInfo> servers;
        while (timecount < timeout)
        {
            servers = BroadcastReceiver::GetServers();
            if (servers.size() > 0)
                break;
            Sleep(100);
            timecount += 100;
        }
        if (servers.size() == 1)
        {
            currentServer = new ServerInfo();
            *currentServer = servers.at(0);
        }
        if (servers.size() == 0)
        {
            return GALILEO_RETURN_CODE::NO_SERVER_FOUND;
        }
        if (servers.size() > 1)
        {
            return GALILEO_RETURN_CODE::MULTI_SERVER_FOUND;
        }
    }
    if (!targetID.empty())
    {
        int timecount = 0;
        std::vector<ServerInfo> servers;
        while (timecount < timeout)
        {
            bool serverFoundFlag = false;
            servers = BroadcastReceiver::GetServers();
            for (auto it = servers.begin(); it < servers.end(); it++)
            {
                if (it->getID() == targetID)
                {
                    currentServer = new ServerInfo();
                    *currentServer = *it;
                    serverFoundFlag = true;
                    break;
                }
            }
            if (serverFoundFlag)
                break;
            Sleep(100);
            timecount += 100;
        }
        if (currentServer == NULL)
        {
            return GALILEO_RETURN_CODE::NO_SERVER_FOUND;
        }
    }
    return Connect(*currentServer);
}

GALILEO_RETURN_CODE GalileoSDK::Connect(ServerInfo server)
{
    std::map<std::string, std::string> params;
    // 设置 ros master地址
    std::stringstream masterURI;
    masterURI << "http://" << server.getIP() << ":11311";
    params.insert(
        std::pair<std::string, std::string>("__master", masterURI.str()));
    // 设置自己的地址，否则默认使用hostname会无法通信
    auto myIPs = Utils::ListIpAddresses();
    if (myIPs.size() == 0)
        return GALILEO_RETURN_CODE::NETWORK_ERROR;
    // 若有多个网络接口则连接第一个
    params.insert(std::pair<std::string, std::string>("__ip", myIPs.at(0)));
    ros::init(params, "galileo_sdk_client", ros::init_options::AnonymousName);
    std::cout << "Connected to server: " << server.getID() << std::endl;
    std::cout << "Master URI: " << masterURI.str() << std::endl;
    std::cout << "IP: " << myIPs.at(0) << std::endl;
    nh = new ros::NodeHandle();
    testPub = nh->advertise<std_msgs::String>("pub_test", 10);
    cmdPub = nh->advertise<galileo_serial_server::GalileoNativeCmds>(
        "/galileo/cmds", 10);
    audioPub = nh->advertise<std_msgs::String>("/xiaoqiang_tts/text", 10);
    speedPub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    galileoStatusSub = nh->subscribe("/galileo/status", 0,
                                     &GalileoSDK::UpdateGalileoStatus, this);
    new std::thread(&GalileoSDK::SpinThread, this);
    broadcastReceiver.SetSDK(this);
    // 等待回调更新状态，保证连接可靠
    int timecount = 0;
    while (timecount < timeout)
    {
        {
            std::unique_lock<std::mutex> lock(statusLock);
            if (currentStatus != NULL)
                break;
        }
        Sleep(100);
        timecount += 100;
    }
    if (timecount >= timeout)
        return GALILEO_RETURN_CODE::TIMEOUT;
    return GALILEO_RETURN_CODE::OK;
}

void GalileoSDK::UpdateGalileoStatus(
    const galileo_serial_server::GalileoStatusConstPtr &status)
{
    std::unique_lock<std::mutex> lock(statusLock);
    //std::cout << "GalileoSDK::UpdateGalileoStatus" << std::endl;
    if (currentStatus != NULL && currentStatus->targetStatus != 0 &&
        status->targetStatus == 0)
    {
        // 从任务未完成状态变成任务完成状态
        if (GoalReachedCallback != NULL)
            GoalReachedCallback(status->targetNumID, *status);
    }
    currentStatus = status;
    if (CurrentStatusCallback != NULL)
    {
        CurrentStatusCallback(GALILEO_RETURN_CODE::OK, *status);
    }
}

void GalileoSDK::SpinThread()
{
    while (currentServer != NULL && ros::ok())
    {
        ros::spinOnce();
        Sleep(1);
    }
    std::cout << "SpinThread exited" << std::endl;
}

void GalileoSDK::broadcastOfflineCallback(std::string id)
{
    if (GalileoSDK::GetInstance() == NULL)
        return;
    auto sdk = GalileoSDK::GetInstance();
    free(sdk->nh);
    sdk->nh = NULL;
    if (sdk->currentServer != NULL)
    {
        free(sdk->currentServer);
        sdk->currentServer = NULL;
    }
    if (sdk->OnDisconnect != NULL)
    {
        sdk->OnDisconnect(GALILEO_RETURN_CODE::OK, id);
    }
}

GALILEO_RETURN_CODE GalileoSDK::GetCurrentStatus(galileo_serial_server::GalileoStatus *status)
{
    std::unique_lock<std::mutex> lock(statusLock);
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    *status = *currentStatus;
    return GALILEO_RETURN_CODE::OK;
}

GALILEO_RETURN_CODE GalileoSDK::PublishTest()
{
    if (currentServer == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Galileo SDK pub test " << Utils::GetCurrentTimestamp();
    msg.data = ss.str();
    testPub.publish(msg);
    return GALILEO_RETURN_CODE::OK;
}

GalileoSDK::~GalileoSDK()
{
    if (currentServer != NULL)
        free(currentServer);
    if (nh != NULL)
        free(nh);
    currentServer = NULL;
    nh = NULL;
    instance = NULL;
    std::cout << "~GalileoSDK Called" << std::endl;
}

GALILEO_RETURN_CODE GalileoSDK::SendCMD(uint8_t data[], int length)
{
    if (currentServer == NULL || currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    galileo_serial_server::GalileoNativeCmds cmd;
    cmd.data = std::vector<uint8_t>(data, data + length);
    cmd.length = length;
    cmdPub.publish(cmd);
    return GALILEO_RETURN_CODE::OK;
};

GALILEO_RETURN_CODE GalileoSDK::StartNav()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->mapStatus == 1 || currentStatus->navStatus == 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'m', 0};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::StopNav()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'m', 4};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::SetGoal(int goalID)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'g', (uint8_t)goalID};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::PauseGoal()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'i', 0};
    return SendCMD(cmd, 2);
}
GALILEO_RETURN_CODE GalileoSDK::ResumeGoal()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'i', 1};
    return SendCMD(cmd, 2);
}
GALILEO_RETURN_CODE GalileoSDK::CancelGoal()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'i', 2};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::InsertGoal(float x, float y)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[10] = {0};
    cmd[0] = 'g';
    cmd[1] = 'i';
    memcpy(&cmd[2], &x, sizeof(x));
    memcpy(&cmd[6], &y, sizeof(y));
    return SendCMD(cmd, 10);
}

GALILEO_RETURN_CODE GalileoSDK::ResetGoal()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'g', 'r', 0};
    return SendCMD(cmd, 3);
}

GALILEO_RETURN_CODE GalileoSDK::SetSpeed(float vLinear, float vAngle)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    geometry_msgs::Twist speed;
    speed.linear.x = vLinear;
    speed.angular.z = vAngle;
    speedPub.publish(speed);
    return GALILEO_RETURN_CODE::OK;
}

GALILEO_RETURN_CODE GalileoSDK::Shutdown()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    uint8_t cmd[] = {0xaa, 0x44};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::SetAngle(uint8_t sign, uint8_t angle)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    uint8_t cmd[] = {'a', sign, angle};
    return SendCMD(cmd, 3);
}

GALILEO_RETURN_CODE GalileoSDK::StartLoop()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    // 动态添加的点不会添加到导航点列表
    auto res = ResetGoal();
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    uint8_t cmd[] = {'m', 5};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::StopLoop()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'m', 6};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::SetLoopWaitTime(uint8_t time)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'m', 5, time};
    return SendCMD(cmd, 3);
}

GALILEO_RETURN_CODE GalileoSDK::StartMapping()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->mapStatus != 0)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'V', 0};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::StopMapping()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->mapStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'V', 1};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::SaveMap()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->mapStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'V', 2};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::UpdateMap()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->mapStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t cmd[] = {'V', 2};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::StartChargeLocal()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    uint8_t cmd[] = {'j', 0};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::stopChargeLocal()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    uint8_t cmd[] = {'j', 1};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::SaveChargeBasePosition()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    uint8_t cmd[] = {'j', 2};
    return SendCMD(cmd, 2);
}

GALILEO_RETURN_CODE GalileoSDK::StartCharge(float x, float y)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    uint8_t goalNum;
    auto res = MoveTo(x, y, &goalNum);
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }

    res = WaitForGoal(goalNum);
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    return StartChargeLocal();
}

GALILEO_RETURN_CODE GalileoSDK::MoveTo(float x, float y, uint8_t *goalIndex)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    auto res = ResetGoal();
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    Sleep(400);
    res = InsertGoal(x, y);
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    Sleep(400);
    uint8_t goalNum;
    res = GetGoalNum(&goalNum);
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    *goalIndex = goalNum - 1;
    return SetGoal(*goalIndex);
}

GALILEO_RETURN_CODE GalileoSDK::GetGoalNum(uint8_t *goalNum)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }

    std::string goalNumStr;
    std::vector<std::string> names;
    int goal;
    if (ros::param::get("/galileo/goal_num", goal))
    {
        *goalNum = goal;
        return GALILEO_RETURN_CODE::OK;
    }

    if (!ros::param::get("/galileo/goal_num", goalNumStr))
    {
        return GALILEO_RETURN_CODE::NETWORK_ERROR;
    }
    if (goalNumStr.empty() || std::stoi(goalNumStr) == 0)
        return GALILEO_RETURN_CODE::SERVER_ERROR;
    *goalNum = std::stoi(goalNumStr);
    return GALILEO_RETURN_CODE::OK;
}

GALILEO_RETURN_CODE GalileoSDK::StopCharge()
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NOT_CONNECTED;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    auto res = CancelGoal();
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    Sleep(400);
    res = ResetGoal();
    if (res != GALILEO_RETURN_CODE::OK)
    {
        return res;
    }
    Sleep(400);
    return stopChargeLocal();
}

void GalileoSDK::SetCurrentStatusCallback(void (
    *callback)(GALILEO_RETURN_CODE, galileo_serial_server::GalileoStatus))
{
    this->CurrentStatusCallback = callback;
}

void GalileoSDK::SetGoalReachedCallback(
    void (*callback)(int goalID, galileo_serial_server::GalileoStatus))
{
    this->GoalReachedCallback = callback;
}

GALILEO_RETURN_CODE GalileoSDK::WaitForGoal(int goalID)
{
    if (currentStatus == NULL)
        return GALILEO_RETURN_CODE::NETWORK_ERROR;
    if (currentStatus->navStatus != 1)
    {
        return GALILEO_RETURN_CODE::INVALIDE_STATE;
    }
    if (currentStatus->targetNumID != goalID)
    {
        return GALILEO_RETURN_CODE::INVALIDE_GOAL;
    }
    while (true)
    {
        auto previousStatus = *currentStatus;
        Sleep(500);
        auto afterStatus = *currentStatus;
        if (previousStatus.targetStatus != 0 && afterStatus.targetStatus == 0 &&
            afterStatus.targetNumID == goalID)
        {
            return GALILEO_RETURN_CODE::OK;
        }
        if (afterStatus.targetNumID != goalID)
        {
            return GALILEO_RETURN_CODE::GOAL_CANCELLED;
        }
    }
}

// Implementation of class ServerInfo
// ////////////////////////////////

ServerInfo::ServerInfo() {}

ServerInfo::ServerInfo(Json::Value serverInfoJson)
{
    ID = serverInfoJson["id"].asString();
    mac = serverInfoJson["mac"].asString();
    port = serverInfoJson["port"].asInt();
}

std::string ServerInfo::getMac()
{
    return mac;
}

void ServerInfo::setMac(std::string mac)
{
    this->mac = mac;
}

std::string ServerInfo::getPassword()
{
    return password;
}

void ServerInfo::setPassword(std::string password)
{
    this->password = password;
}

std::string ServerInfo::getIP()
{
    return ip;
}

void ServerInfo::setIP(std::string ip)
{
    this->ip = ip;
}

std::string ServerInfo::getID()
{
    return ID;
}

void ServerInfo::setID(std::string ID)
{
    this->ID = ID;
}

size_t ServerInfo::getTimestamp()
{
    return timestamp;
}

void ServerInfo::setTimestamp(size_t timestamp)
{
    this->timestamp = timestamp;
}

uint32_t ServerInfo::getPort()
{
    return port;
}

void ServerInfo::setPort(uint32_t port)
{
    this->port = port;
}

Json::Value ServerInfo::toJson() {
    Json::Value rootJsonValue;
    rootJsonValue["ID"] = ID;
    rootJsonValue["port"] = port;
    rootJsonValue["timestamp"] = timestamp;
    rootJsonValue["ip"] = ip;
    rootJsonValue["password"] = password;
    rootJsonValue["mac"] = mac;
    return rootJsonValue;
}

std::string ServerInfo::toJsonString() {
    Json::StreamWriterBuilder wbuilder;
    return Json::writeString(wbuilder, toJson());
}



// Implementation of class BroadcastReceiver
// ////////////////////////////////

BroadcastReceiver *BroadcastReceiver::instance = NULL;

BroadcastReceiver::BroadcastReceiver() : sdk(NULL)
{
// 初始化广播udp socket
#ifdef _WIN32

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::cout << "Failed. Error Code : " << WSAGetLastError() << std::endl;
        std::exception e("Failed create socket");
        throw e;
    }
    if ((serverSocket = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
    {
        std::cout << "Could not create socket : " << WSAGetLastError() << std::endl;
        std::exception e("Failed create socket");
        throw e;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(BROADCAST_PORT);
    if (bind(serverSocket, (struct sockaddr *)&server, sizeof(server)) ==
        SOCKET_ERROR)
    {
        std::cout << "Bind failed with error code : " << WSAGetLastError()
                  << std::endl;
        std::exception e("Failed create socket");
        throw e;
    }
    int timeout = 1000;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO,
                    (const char *)&timeout, sizeof(timeout)) < 0)
    {
        std::cout << "Set socket timeout failed" << std::endl;
    }
#else
    //create a UDP socket
    if ((serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        std::cout << "Failed create socket" << std::endl;
        std::runtime_error e("Failed create socket");
        throw e;
    }

    // zero out the structure
    memset((char *)&server, 0, sizeof(server));

    server.sin_family = AF_INET;
    server.sin_port = htons(BROADCAST_PORT);
    server.sin_addr.s_addr = htonl(INADDR_ANY);

    //bind socket to port
    if (bind(serverSocket, (struct sockaddr *)&server, sizeof(server)) == -1)
    {
        std::cout << "Failed create socket" << std::endl;
        std::runtime_error e("Failed create socket");
        throw e;
    }
    struct timeval tv;
    tv.tv_sec = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO,
                    (const char *)&tv, sizeof(struct timeval)) < 0)
    {
        std::cout << "Set socket timeout failed" << std::endl;
        int perrno = errno;
        printf("Error setsockopt: ERRNO = %s\n",strerror(perrno));
    }
#endif
    instance = this;
}

void BroadcastReceiver::SetSDK(GalileoSDK *sdk)
{
    this->sdk = sdk;
}

void BroadcastReceiver::StopAll()
{
    if (instance == NULL)
        return;
    instance->StopTask();
}

void BroadcastReceiver::StopTask()
{
    runningFlag = false;
    instance = NULL;
}

std::vector<ServerInfo> BroadcastReceiver::GetServers()
{
    if (instance == NULL)
    {
        return std::vector<ServerInfo>();
    }
    else
    {
        std::unique_lock<std::mutex> lock(instance->serversLock);
        return instance->serverList;
    }
}

void replaceAll(std::string &str,
                const std::string &from,
                const std::string &to)
{
    if (from.empty())
        return;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos)
    {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing
                                  // 'x' with 'yx'
    }
}

void BroadcastReceiver::Run()
{
    runningFlag = true;
    Json::CharReaderBuilder builder;
    Json::CharReader *reader = builder.newCharReader();
    instance = this;
    while (runningFlag)
    {
        ServerInfo *serverInfo = NULL;
        try
        {
            char buf[BUFLEN];
#ifdef _WIN32
            int slen, recv_len;
#else
            uint32_t slen, recv_len;
#endif // _WIN32

            
            slen = sizeof(si_other);
            fflush(stdout);
            memset(buf, '\0', BUFLEN);
            recv_len = recvfrom(serverSocket, buf, BUFLEN, 0,
                                (struct sockaddr *)&si_other, &slen);
            char ipStr[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(si_other.sin_addr), ipStr, INET_ADDRSTRLEN);
            std::string data = std::string(buf, 0, recv_len);
            if (!data.empty())
            {
                Json::Value serverInfoJson;
                std::string errors;
                bool parsingSuccessful = reader->parse(
                    data.c_str(), data.c_str() + data.size(), &serverInfoJson, &errors);
                if (!parsingSuccessful)
                {
                    std::cout << errors << std::endl;
                }
                else
                {
                    serverInfo = new ServerInfo(serverInfoJson);
                    std::string addrStr = std::string(ipStr);
                    replaceAll(addrStr, "/", "");
                    serverInfo->setIP(addrStr);
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cout << e.what() << std::endl;
        }

        std::vector<ServerInfo> result;

        std::unique_lock<std::mutex> lock(instance->serversLock);
        for (auto it = serverList.begin(); it < serverList.end(); it++)
        {
            if (Utils::GetCurrentTimestamp() - it->getTimestamp() > 10 * 1000)
            {
                std::cout << "Server: " << it->getID() << " offline" << std::endl;
                if (this->sdk != NULL)
                {
                    // 设置服务器下线回调
                    sdk->broadcastOfflineCallback(it->getID());
                }
            }
            else
            {
                result.push_back(*it);
            }
        }
        serverList = result;

        if (serverInfo == NULL)
        {
            continue;
        }

        // 更新服务器时间戳
        bool newServerFlag = true;
        for (auto it = serverList.begin(); it < serverList.end(); it++)
        {
            if (it->getID() == serverInfo->getID())
            {
                it->setTimestamp(Utils::GetCurrentTimestamp());
                it->setIP(serverInfo->getIP());
                it->setMac(serverInfo->getMac());
                newServerFlag = false;
            }
        }

        // 更新服务器列表
        if (newServerFlag)
        {
            serverInfo->setTimestamp(Utils::GetCurrentTimestamp());
            serverList.push_back(*serverInfo);
        }
        free(serverInfo);
    }
#ifdef _WIN32
    closesocket(serverSocket);
    WSACleanup();
#else
    close(serverSocket);
#endif
}

// Impletation of export c functions
// ////////////////////////////////

void* __stdcall CreateInstance() {
    GalileoSDK* instance = new GalileoSDK();
    return instance;
}

void __stdcall ReleaseInstance(void *instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    free(sdk);
}

GALILEO_RETURN_CODE Connect(void * instance, uint8_t * targetID, size_t length, 
    bool auto_connect, int timeout, 
    void(*OnConnect)(GALILEO_RETURN_CODE, uint8_t *, size_t),
    void(*OnDisconnect)(GALILEO_RETURN_CODE, uint8_t *, size_t))
{
    OnConnectCB = OnConnect;
    OnDisconnectCB = OnDisconnect;
    GalileoSDK* sdk = (GalileoSDK*)instance;
    std::string targetIDStr(targetID, targetID + length);

    return sdk->Connect(targetIDStr, auto_connect, timeout, [](GALILEO_RETURN_CODE status, std::string id) -> void {
        if (OnConnectCB != NULL)
            OnConnectCB(status, (uint8_t*)id.data(), id.length());
    }, [](GALILEO_RETURN_CODE status, std::string id) -> void {
        if (OnDisconnectCB != NULL)
            OnDisconnectCB(status, (uint8_t*)id.data(), id.length());
    });
}

void __stdcall GetServersOnline(void * instance, uint8_t* servers_json, size_t &length){
    GalileoSDK* sdk = (GalileoSDK*)instance;
    Json::Value servers(Json::arrayValue);
    auto serversOnline = sdk->GetServersOnline();
    for (auto it = serversOnline.begin(); it < serversOnline.end(); it++) {
        servers.append(it->toJson());
    }
    Json::StreamWriterBuilder wbuilder;
    auto serversStr = Json::writeString(wbuilder, servers);
    memcpy(servers_json, serversStr.data(), serversStr.size());
    length = serversStr.size();
}

void __stdcall GetCurrentServer(void * instance, uint8_t* servers_json, size_t &length) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    Json::Value currentServer;
    if (sdk->currentServer != NULL) {
        currentServer = sdk->currentServer->toJson();
    }
    Json::StreamWriterBuilder wbuilder;
    auto serversStr = Json::writeString(wbuilder, currentServer);
    memcpy(servers_json, serversStr.data(), serversStr.size());
    length = serversStr.size();
}

GALILEO_RETURN_CODE __stdcall PublishTest(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->PublishTest();
}

GALILEO_RETURN_CODE __stdcall SendCMD(void * instance, uint8_t* data, int length) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SendCMD(data, length);
}

GALILEO_RETURN_CODE __stdcall StartNav(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StartNav();
}

GALILEO_RETURN_CODE __stdcall StopNav(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StopNav();
}

GALILEO_RETURN_CODE __stdcall SetGoal(void * instance, int goalIndex) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SetGoal(goalIndex);
}

GALILEO_RETURN_CODE __stdcall PauseGoal(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->PauseGoal();
}

GALILEO_RETURN_CODE __stdcall ResumeGoal(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->ResumeGoal();
}

GALILEO_RETURN_CODE __stdcall CancelGoal(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->CancelGoal();
}

GALILEO_RETURN_CODE __stdcall InsertGoal(void * instance, float x, float y) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->InsertGoal(x, y);
}

GALILEO_RETURN_CODE __stdcall ResetGoal(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->ResetGoal();
}

GALILEO_RETURN_CODE __stdcall SetSpeed(void * instance, float vLinear, float vAngle) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SetSpeed(vLinear, vAngle);
}

GALILEO_RETURN_CODE __stdcall Shutdown(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->Shutdown();
}

GALILEO_RETURN_CODE __stdcall SetAngle(void * instance, uint8_t sign, uint8_t angle) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SetAngle(sign, angle);
}

GALILEO_RETURN_CODE __stdcall StartLoop(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StartLoop();
}

GALILEO_RETURN_CODE __stdcall StopLoop(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StopLoop();
}

GALILEO_RETURN_CODE __stdcall SetLoopWaitTime(void * instance, uint8_t time) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SetLoopWaitTime(time);
}

GALILEO_RETURN_CODE __stdcall StartMapping(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StartMapping();
}

GALILEO_RETURN_CODE __stdcall StopMapping(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StopMapping();
}

GALILEO_RETURN_CODE __stdcall SaveMap(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SaveMap();
}

GALILEO_RETURN_CODE __stdcall UpdateMap(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->UpdateMap();
}

GALILEO_RETURN_CODE __stdcall StartChargeLocal(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StartChargeLocal();
}

GALILEO_RETURN_CODE __stdcall stopChargeLocal(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->stopChargeLocal();
}

GALILEO_RETURN_CODE __stdcall SaveChargeBasePosition(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->SaveChargeBasePosition();
}

GALILEO_RETURN_CODE __stdcall StartCharge(void * instance, float x, float y) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StartCharge(x, y);
}

GALILEO_RETURN_CODE __stdcall StopCharge(void * instance) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->StopCharge();
}

GALILEO_RETURN_CODE __stdcall MoveTo(void * instance, float x, float y, uint8_t &goalNum) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->MoveTo(x, y, &goalNum);
}

GALILEO_RETURN_CODE __stdcall GetGoalNum(void * instance, uint8_t &goalNum) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->GetGoalNum(&goalNum);
}

GALILEO_RETURN_CODE __stdcall GetCurrentStatus(void * instance, uint8_t* status_json, size_t &length) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    Json::Value rootValue;
    galileo_serial_server::GalileoStatus status;
    GALILEO_RETURN_CODE res = sdk->GetCurrentStatus(&status);
    if (res == GALILEO_RETURN_CODE::OK) {
        rootValue = statusToJson(status);
    }
    Json::StreamWriterBuilder wbuilder;
    auto serversStr = Json::writeString(wbuilder, rootValue);
    memcpy(status_json, serversStr.data(), serversStr.size());
    length = serversStr.size();
    return res;
}

void __stdcall SetCurrentStatusCallback(void * instance, void(*callback)(
    GALILEO_RETURN_CODE, uint8_t* status_json, size_t length)) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    StatusCB = callback;
    sdk->SetCurrentStatusCallback([](GALILEO_RETURN_CODE res, galileo_serial_server::GalileoStatus status)->void {
        Json::Value rootValue = statusToJson(status);
        Json::StreamWriterBuilder wbuilder;
        std::string serversStr = Json::writeString(wbuilder, rootValue);
        StatusCB(res, (uint8_t*)serversStr.data(), serversStr.size());
    });
}

void __stdcall SetGoalReachedCallback(
    void * instance,
    void(*callback)(int goalID, uint8_t* status_json, size_t length)) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    ReachedCB = callback;
    sdk->SetGoalReachedCallback([](int goalID, galileo_serial_server::GalileoStatus status)->void {
        Json::Value rootValue = statusToJson(status);
        Json::StreamWriterBuilder wbuilder;
        std::string serversStr = Json::writeString(wbuilder, rootValue);
        ReachedCB(goalID, (uint8_t*)serversStr.data(), serversStr.size());
    });
}

GALILEO_RETURN_CODE __stdcall WaitForGoal(void * instance, int goalID) {
    GalileoSDK* sdk = (GalileoSDK*)instance;
    return sdk->WaitForGoal(goalID);
}

static Json::Value statusToJson(galileo_serial_server::GalileoStatus status) {
    Json::Value rootValue;
    rootValue["timestamp"] = status.header.stamp.toNSec() / 1000 / 1000;
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

} // namespace GalileoSDK
