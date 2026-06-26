#pragma once

#if defined(_WIN32)

#include <chrono>
#include <thread>

inline int usleep(unsigned int microseconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
    return 0;
}

#else
#include_next <unistd.h>
#endif
