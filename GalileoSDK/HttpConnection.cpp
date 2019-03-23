#include "HttpConnection.h"


namespace GalileoSDK {
    HttpConnection::HttpConnection()
    {
#ifdef WIN32
        //此处一定要初始化一下，否则gethostbyname返回一直为空
        WSADATA wsa = { 0 };
        WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    }
    
    std::string HttpConnection::socketHttp(std::string host, std::string request, int port = 80)
    {
        int sockfd;
        struct sockaddr_in address;
        struct hostent *server;
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        server = gethostbyname(host.c_str());
        memcpy((char *)&address.sin_addr.s_addr, (char*)server->h_addr, server->h_length);
        if (-1 == connect(sockfd, (struct sockaddr *)&address, sizeof(address))) {
            return "";
        }
#ifdef WIN32
        send(sockfd, request.c_str(), request.size(), 0);
#else
        write(sockfd, request.c_str(), request.size());
#endif
        char *buf = new char[1024 * 1024];
        int offset = 0;
        int rc;
#ifdef WIN32
        while (rc = recv(sockfd, buf + offset, 1024, 0))
#else
        while (rc = read(sockfd, buf + offset, 1024))
#endif
        {
            offset += rc;
        }
#ifdef WIN32
        closesocket(sockfd);
#else
        close(sockfd);
#endif
        buf[offset] = 0;
        std::string res = std::string(buf);
        auto find_res = res.find("\r\n\r\n");
        auto body = res.substr(res.find("\r\n\r\n") + 4, res.size());
        delete buf;
        return body;
    }
    std::string HttpConnection::postData(std::string host, std::string path, std::string post_content, int port = 80)
    {
        //POST请求方式
        std::stringstream stream;
        stream << "POST " << path;
        stream << " HTTP/1.0\r\n";
        stream << "Host: " << host << "\r\n";
        stream << "User-Agent: Mozilla/5.0 (Windows; U; Windows NT 5.1; zh-CN; rv:1.9.2.3) Gecko/20100401 Firefox/3.6.3\r\n";
        stream << "Content-Type:application/json\r\n";
        stream << "Content-Length:" << post_content.length() << "\r\n";
        stream << "Connection:close\r\n\r\n";
        stream << post_content.c_str();

        return socketHttp(host, stream.str(), port);
    }
    std::string HttpConnection::getData(std::string host, std::string path, std::string get_content, int port = 80)
    {
        //GET请求方式
        std::stringstream stream;
        stream << "GET " << path << "?" << get_content;
        stream << " HTTP/1.0\r\n";
        stream << "Host: " << host << "\r\n";
        stream << "User-Agent: Mozilla/5.0 (Windows; U; Windows NT 5.1; zh-CN; rv:1.9.2.3) Gecko/20100401 Firefox/3.6.3\r\n";
        stream << "Connection:close\r\n\r\n";
        return socketHttp(host, stream.str(), port);
    }
}