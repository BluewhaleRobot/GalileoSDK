#ifndef __WRAPPER_H__
#define __WRAPPER_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <memory.h>

    //#include <pthread.h>
    //#include <unistd.h>
    //#include <sys/prctl.h>
    //#include <sys/time.h>
    //#include <semaphore.h>
#include <errno.h>
#include <assert.h>
//#include <net/if.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <sys/ioctl.h>
//#include <sys/reboot.h>
//#include <sys/time.h>
#include <time.h>
#include <signal.h>

#include "infra_types.h"
#include "infra_defs.h"
#include "wrappers_defs.h"
//#include "stdarg.h"

void *HAL_Malloc(uint32_t size);
void HAL_Free(void *ptr);
void HAL_Printf(const char *fmt, ...);
int HAL_GetProductKey(char product_key[IOTX_PRODUCT_KEY_LEN + 1]);
int HAL_GetDeviceName(char device_name[IOTX_DEVICE_NAME_LEN + 1]);
int HAL_GetDeviceSecret(char device_secret[IOTX_DEVICE_SECRET_LEN]);
uint64_t HAL_UptimeMs(void);
int HAL_Snprintf(char *str, const int len, const char *fmt, ...);
#if defined(__cplusplus)
}
#endif

#endif // !__WRAPPER_H__
