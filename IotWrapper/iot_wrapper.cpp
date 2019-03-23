#include "iot_wrapper.h"


uint32_t HAL_CPP_Random(uint32_t region)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dice(0, region);
    return dice(rng);
}

void *HAL_CPP_MutexCreate(void) {
    std::mutex *g_mutex = new std::mutex();
    return (void*)g_mutex;
}

void HAL_CPP_MutexDestroy(void *mutex) {
    if (mutex == NULL)
        return;
    std::mutex *g_mutex = (std::mutex *)mutex;
    delete g_mutex;
}

void HAL_CPP_MutexLock(void *mutex) {
    std::mutex *g_mutex = (std::mutex *)mutex;
    g_mutex->lock();
}

void HAL_CPP_MutexUnlock(void *mutex) {
    std::mutex *g_mutex = (std::mutex *)mutex;
    g_mutex->unlock();
}

void HAL_CPP_SleepMs(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

uintptr_t HAL_CPP_TCP_Establish(const char *host, uint16_t port) {
    int sockfd;
    struct sockaddr_in address;
    struct hostent *server;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    server = gethostbyname(host);
    memcpy((char *)&address.sin_addr.s_addr, (char*)server->h_addr, server->h_length);
    if (-1 == connect(sockfd, (struct sockaddr *)&address, sizeof(address))) {
        return (uintptr_t)(-1);
    }
    return sockfd;
}

int32_t HAL_CPP_TCP_Read(uintptr_t fd, char *buf, uint32_t len, uint32_t timeout_ms) {
    int offset = 0;
    int rc;
#ifdef WIN32
    rc = recv(fd, buf + offset, len, 0);
#else
    rc = read(fd, buf + offset, len, timeout_ms);
#endif
    return rc;
}

int32_t HAL_CPP_TCP_Write(uintptr_t fd, const char *buf, uint32_t len, uint32_t timeout_ms) {
#ifdef WIN32
    int32_t res = send(fd, buf, len, 0);
    return res;
#else
    return write(fd, buf, len, timeout_ms);
#endif
}

void HAL_CPP_Srandom(uint32_t seed) {
    std::srand(seed);
}

int HAL_CPP_TCP_Destroy(uintptr_t fd) {
#ifdef WIN32
    return closesocket(fd);
#else
    return close(fd);
#endif
}

LARGE_INTEGER
getFILETIMEoffset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

int
clock_gettime(int X, struct timeval *tv)
{
    LARGE_INTEGER           t;
    FILETIME            f;
    double                  microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 0;

    if (!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter) {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        }
        else {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) QueryPerformanceCounter(&t);
    else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (double)t.QuadPart / frequencyToMicroseconds;
    t.QuadPart = microseconds;
    tv->tv_sec = t.QuadPart / 1000000;
    tv->tv_usec = t.QuadPart % 1000000;
    return (0);
}

uint64_t HAL_CPP_UptimeMs() {
    uint64_t time_ms;
    struct timeval ts;

    clock_gettime(0, &ts);
    time_ms = ((uint64_t)ts.tv_sec * (uint64_t)1000) + (ts.tv_usec / 1000);

    return time_ms;
}