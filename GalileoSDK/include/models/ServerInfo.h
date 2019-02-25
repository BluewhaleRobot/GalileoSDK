#pragma once
#ifndef __SERVER_INFO_H__
#define __SERVER_INFO_H__

#include "../Dll.h"
#include <iostream>
#include <json/json.h>
#include <string>

namespace GalileoSDK {
    class DLL_PUBLIC ServerInfo {
    public:
        ServerInfo();
        ServerInfo(Json::Value);
        std::string getMac();
        void setMac(std::string);
        std::string getPassword();
        void setPassword(std::string);
        std::string getIP();
        void setIP(std::string);
        std::string getID();
        void setID(std::string);
        size_t getTimestamp();
        void setTimestamp(size_t);
        uint32_t getPort();
        void setPort(uint32_t);

    private:
        std::string ID;
        uint32_t port;
        size_t timestamp;
        std::string ip = "";
        std::string password = "xiaoqiang";
        std::string mac;
    };
} // namespace GalileoSDK

#endif // !__SERVER_INFO_H__
