/**
 * NOTE:
 *
 * HAL_TCP_xxx API reference implementation: wrappers/os/ubuntu/HAL_TCP_linux.c
 *
 */

#include "wrapper.h"

/**
 * @brief Deallocate memory block
 *
 * @param[in] ptr @n Pointer to a memory block previously allocated with platform_malloc.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_Free(void *ptr)
{
	free(ptr);
	return;
}

char my_device_name[IOTX_DEVICE_NAME_LEN + 1] = {0};
int device_name_length = 0;

/**
 * @brief Get device name from user's system persistent storage
 *
 * @param [ou] device_name: array to store device name, max length is IOTX_DEVICE_NAME_LEN
 * @return the actual length of device name
 */
int HAL_GetDeviceName(char device_name[IOTX_DEVICE_NAME_LEN + 1])
{
	int i = 0;
	for (i = 0; i < device_name_length;i++){
		device_name[i] = my_device_name[i];
	};
	return device_name_length;
}

void SetDeviceName(char* device_name, int length)
{
	device_name_length = length;
	int i = 0;
	for (i = 0; i < device_name_length; i++){
		my_device_name[i] = device_name[i];
	};
}


char my_device_secret[IOTX_DEVICE_SECRET_LEN + 1] = {0};
char device_secret_length = 0;

/**
 * @brief Get device secret from user's system persistent storage
 *
 * @param [ou] device_secret: array to store device secret, max length is IOTX_DEVICE_SECRET_LEN
 * @return the actual length of device secret
 */
int HAL_GetDeviceSecret(char device_secret[IOTX_DEVICE_SECRET_LEN + 1])
{
	int i = 0;
	for (i = 0; i < device_secret_length;i++){
		device_secret[i] = my_device_secret[i];
	};
	return (int)device_secret_length;
}

void SetDeviceSecret(char* device_secret, int length){
	device_secret_length = length;
	int i = 0;
	for (i = 0; i < device_secret_length;i++){
		my_device_secret[i] = device_secret[i];
	};
}

char fireware_version[IOTX_FIRMWARE_VER_LEN + 1] = {0};
int fireware_version_length = 0;
/**
 * @brief Get firmware version
 *
 * @param [ou] version: array to store firmware version, max length is IOTX_FIRMWARE_VER_LEN
 * @return the actual length of firmware version
 */
int HAL_GetFirmwareVersion(char *version)
{
	int i = 0;
	for (i = 0; i < fireware_version_length; i++){
		version[i] = fireware_version[i];
	};
	return fireware_version_length;
}

void SetFirewareVersion(char* version, int length){
	int i = 0;
	fireware_version_length = length;
	for (i = 0; i < fireware_version_length; i++){
		fireware_version[i] = version[i];
	}
}



char my_product_key[IOTX_PRODUCT_KEY_LEN + 1] = {0};
int product_key_length = 0;
/**
 * @brief Get product key from user's system persistent storage
 *
 * @param [ou] product_key: array to store product key, max length is IOTX_PRODUCT_KEY_LEN
 * @return  the actual length of product key
 */
int HAL_GetProductKey(char product_key[IOTX_PRODUCT_KEY_LEN + 1])
{
	int i = 0;
	for(i=0;i<product_key_length;i++){
		product_key[i] = my_product_key[i];
	}
	return product_key_length;
}

void SetProductKey(char* product_key, int length){
	int i = 0;
	product_key_length = length;
	for(i=0;i<product_key_length;i++){
		my_product_key[i] = product_key[i];
	}
}

char my_product_secret[IOTX_PRODUCT_SECRET_LEN + 1] = {0};
int product_secret_length = 0;

int HAL_GetProductSecret(char product_secret[IOTX_PRODUCT_SECRET_LEN + 1])
{
	int i = 0;
	for(i=0;i<product_secret_length;i++){
		product_secret[i] = my_product_secret[i];
	}
	return product_secret_length;
}

void SetProductSecret(char* product_secret, int length){
	int i = 0;
	product_secret_length = length;
	for(i=0;i<product_secret_length;i++){
		my_product_secret[i] = product_secret[i];
	}
}


/**
 * @brief Allocates a block of size bytes of memory, returning a pointer to the beginning of the block.
 *
 * @param [in] size @n specify block size in bytes.
 * @return A pointer to the beginning of the block.
 * @see None.
 * @note Block value is indeterminate.
 */
void *HAL_Malloc(uint32_t size)
{
	return malloc(size);
}


/**
 * @brief Create a mutex.
 *
 * @retval NULL : Initialize mutex failed.
 * @retval NOT_NULL : The mutex handle.
 * @see None.
 * @note None.
 */
void *HAL_MutexCreate(void)
{
	return (void*)1;
}


/**
 * @brief Destroy the specified mutex object, it will release related resource.
 *
 * @param [in] mutex @n The specified mutex.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_MutexDestroy(void *mutex)
{
	return;
}


/**
 * @brief Waits until the specified mutex is in the signaled state.
 *
 * @param [in] mutex @n the specified mutex.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_MutexLock(void *mutex)
{
	return;
}


/**
 * @brief Releases ownership of the specified mutex object..
 *
 * @param [in] mutex @n the specified mutex.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_MutexUnlock(void *mutex)
{
	return;
}


/**
 * @brief Writes formatted data to stream.
 *
 * @param [in] fmt: @n String that contains the text to be written, it can optionally contain embedded format specifiers
     that specifies how subsequent arguments are converted for output.
 * @param [in] ...: @n the variable argument list, for formatted and inserted in the resulting string replacing their respective specifiers.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_Printf(const char *fmt, ...)
{
	return;
}


uint32_t HAL_Random(uint32_t region)
{
	return (uint32_t)1;
}


    int HAL_SSLHooks_set(ssl_hooks_t *hooks)
{
	return (int)1;
}


    int32_t HAL_SSL_Destroy(uintptr_t handle)
{
	return (int32_t)1;
}


    uintptr_t HAL_SSL_Establish(const char *host, uint16_t port, const char *ca_crt, uint32_t ca_crt_len)
{
	return (uintptr_t)1;
}


    int HAL_SSL_Read(uintptr_t handle, char *buf, int len, int timeout_ms)
{
	return (int)1;
}


    int HAL_SSL_Write(uintptr_t handle, const char *buf, int len, int timeout_ms)
{
	return (int)1;
}


/**
 * @brief Sleep thread itself.
 *
 * @param [in] ms @n the time interval for which execution is to be suspended, in milliseconds.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_SleepMs(uint32_t ms)
{
	return;
}


/**
 * @brief Writes formatted data to string.
 *
 * @param [out] str: @n String that holds written text.
 * @param [in] len: @n Maximum length of character will be written
 * @param [in] fmt: @n Format that contains the text to be written, it can optionally contain embedded format specifiers
     that specifies how subsequent arguments are converted for output.
 * @param [in] ...: @n the variable argument list, for formatted and inserted in the resulting string replacing their respective specifiers.
 * @return bytes of character successfully written into string.
 * @see None.
 * @note None.
 */
int HAL_Snprintf(char *str, const int len, const char *fmt, ...)
{
	return (int)1;
}


void HAL_Srandom(uint32_t seed)
{
	return;
}


/**
 * @brief Retrieves the number of milliseconds that have elapsed since the system was boot.
 *
 * @return the number of milliseconds.
 * @see None.
 * @note None.
 */
uint64_t HAL_UptimeMs(void)
{
	return (uint64_t)1;
}


int HAL_Vsnprintf(char *str, const int len, const char *format, va_list ap)
{
	return (int)1;
}


