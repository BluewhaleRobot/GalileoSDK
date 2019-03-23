#ifndef __GALILEO_RETURN_CODE_H__
#define __GALILEO_RETURN_CODE_H__

#include <string>
namespace GalileoSDK {
    enum GALILEO_RETURN_CODE
    {
        OK,
        NOT_CONNECTED,
        INVALIDE_STATE,
        NO_SERVER_FOUND,
        MULTI_SERVER_FOUND,
        NETWORK_ERROR,
        ALREADY_CONNECTED,
        TIMEOUT,
        SERVER_ERROR,
        GOAL_CANCELLED,
        INVALIDE_GOAL,
        INVALIDE_PARAMS
    };
    std::string GalileoReturnCodeToString(GALILEO_RETURN_CODE status);
}

#endif // !__GALILEO_RETURN_CODE_H__
