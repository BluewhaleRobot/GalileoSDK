#ifndef __IOT_WRAPPER_H__
#define __IOT_WRAPPER_H__
#if defined(__cplusplus)
#include <random>
#include <mutex>
#include <chrono>
#include <thread>
#ifdef WIN32
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
#endif // WIN32
extern "C" {
#endif
    uint32_t HAL_CPP_Random(uint32_t region);
    void *HAL_CPP_MutexCreate(void);
    void HAL_CPP_MutexDestroy(void *mutex);
    void HAL_CPP_MutexLock(void *mutex);
    void HAL_CPP_MutexUnlock(void *mutex);
    void HAL_CPP_SleepMs(uint32_t ms);
    uintptr_t HAL_CPP_TCP_Establish(const char *host, uint16_t port);
    int32_t HAL_CPP_TCP_Read(uintptr_t fd, char *buf, uint32_t len, uint32_t timeout_ms);
    int32_t HAL_CPP_TCP_Write(uintptr_t fd, const char *buf, uint32_t len, uint32_t timeout_ms);
    int HAL_CPP_TCP_Destroy(uintptr_t fd);
    void HAL_CPP_Srandom(uint32_t seed);
    uint64_t HAL_CPP_UptimeMs(void);
#if defined(__cplusplus)
}
#endif

#endif // !__WRAPPER_CPP_H__

