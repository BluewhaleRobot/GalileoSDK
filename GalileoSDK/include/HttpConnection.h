#ifndef __HTTP_CONNECTION_H__
#define __HTTP_CONNECTION_H__

#include <string>
#include <iostream>
#include <sstream>
#ifdef _WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#endif
#include "mutils.h"

namespace GalileoSDK
{
class HttpConnection
{
public:
    HttpConnection();
    std::string postData(std::string host, std::string path, std::string post_content, int port);
    std::string getData(std::string host, std::string path, std::string get_content, int port);
    std::string putData(std::string host, std::string path, std::string content, int port);
    std::string deleteData(std::string host, std::string path, std::string content, int port);
    int getStatusCode() {
        return status_code;
    }
private:
    std::string socketHttp(std::string host, std::string request, int port);
    int status_code;
};
} // namespace GalileoSDK
#endif // !__HTTP_CONNECTION_H__
