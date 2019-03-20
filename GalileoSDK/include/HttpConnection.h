#ifndef __HTTP_CONNECTION_H__
#define __HTTP_CONNECTION_H__

#include <string>
#include <iostream>
#include <sstream>
#ifdef _WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif
namespace GalileoSDK {
    class HttpConnection
    {
    public:
        HttpConnection();
        std::string postData(std::string host, std::string path, std::string post_content, int port);
        std::string getData(std::string host, std::string path, std::string get_content, int port);
    private:
        std::string socketHttp(std::string host, std::string request, int port);
    };
}
#endif // !__HTTP_CONNECTION_H__
