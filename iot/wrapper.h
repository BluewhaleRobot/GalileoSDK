#ifndef __WRAPPER_H__
#define __WRAPPER_H__

#if defined(__cplusplus)
#include "iot_wrapper.h"
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <memory.h>
#include <errno.h>
#include <assert.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>

#include "infra_config.h"
#include "infra_types.h"
#include "infra_defs.h"
#include "wrappers_defs.h"
#ifndef __cplusplus
#include "iot_wrapper.h"
#endif

void *HAL_Malloc(uint32_t size);
void HAL_Free(void *ptr);
void HAL_Printf(const char *fmt, ...);
int HAL_GetProductKey(char product_key[IOTX_PRODUCT_KEY_LEN + 1]);
int HAL_GetDeviceName(char device_name[IOTX_DEVICE_NAME_LEN + 1]);
int HAL_GetDeviceSecret(char device_secret[IOTX_DEVICE_SECRET_LEN + 1]);

uint64_t HAL_UptimeMs(void);
int HAL_Snprintf(char *str, const int len, const char *fmt, ...);
void SetDeviceName(const char* device_name, int length);
void SetDeviceSecret(const char* device_secret, int length);
void SetFirewareVersion(const char* version, int length);
void SetProductKey(const char* product_key, int length);
#if defined(__cplusplus)
}
#endif

#endif // !__WRAPPER_H__
