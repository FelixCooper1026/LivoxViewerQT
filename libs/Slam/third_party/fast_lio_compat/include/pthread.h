#pragma once

#if defined(_WIN32)

#include <cerrno>
#include <mutex>
#include <thread>

using pthread_t = std::thread*;
using pthread_attr_t = void;
using pthread_mutex_t = std::mutex*;

inline int pthread_mutex_init(pthread_mutex_t* mutex, const void*)
{
    *mutex = new std::mutex();
    return 0;
}

inline int pthread_mutex_destroy(pthread_mutex_t* mutex)
{
    delete *mutex;
    return 0;
}

inline int pthread_mutex_lock(pthread_mutex_t* mutex)
{
    (*mutex)->lock();
    return 0;
}

inline int pthread_mutex_unlock(pthread_mutex_t* mutex)
{
    (*mutex)->unlock();
    return 0;
}

inline int pthread_mutex_trylock(pthread_mutex_t* mutex)
{
    return (*mutex)->try_lock() ? 0 : EBUSY;
}

inline int pthread_create(pthread_t* thread, const pthread_attr_t*, void* (*startRoutine)(void*), void* argument)
{
    *thread = new std::thread([startRoutine, argument]() {
        startRoutine(argument);
    });
    return 0;
}

inline int pthread_join(pthread_t thread, void**)
{
    thread->join();
    delete thread;
    return 0;
}

#else
#include_next <pthread.h>
#endif
