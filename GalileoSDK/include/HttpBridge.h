#ifndef __HTTP_BRIDGE_H__
#define __HTTP_BRIDGE_H__

#include <string>

namespace GalileoSDK 
{

struct HttpBridgeRequest
{
	std::string uuid;
	std::string method;
	std::string body;
	std::string url;

	HttpBridgeRequest(std::string mmethod, std::string murl, std::string mbody)
	{
		uuid = Utils::GenHex(16);
		method = mmethod;
		url = murl;
		body = mbody;
	}

	HttpBridgeRequest(nlohmann::json j)
	{
		j.at("uuid").get_to(uuid);
		j.at("method").get_to(method);
		j.at("body").get_to(body);
		j.at("url").get_to(url);
	}

	nlohmann::json to_json()
	{
		return nlohmann::json{
			{"uuid", uuid},
			{"method", method},
			{"url", url},
			{"body", body}
		};
	}
};

struct HttpBridgeResponse
{
	std::string uuid;
	uint16_t status_code;
	std::string body;

	HttpBridgeResponse()
	{
		uuid = "";
		status_code = 0;
		body = "";
	}

	HttpBridgeResponse(nlohmann::json j)
	{
		j.at("uuid").get_to(uuid);
		j.at("status_code").get_to(status_code);
		j.at("body").get_to(body);
	}

	nlohmann::json to_json()
	{
		return nlohmann::json{
			{"uuid", uuid},
			{"status_code", status_code},
			{"body", body}
		};
	}
};

}

#endif // __HTTP_BRIDGE_H__
