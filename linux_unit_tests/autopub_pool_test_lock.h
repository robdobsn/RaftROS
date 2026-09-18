/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Host lock adapters for AutoPubPublisherPool tests
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

// std::mutex with the pool's lock contract (UINT32_MAX = wait forever).
// The bounded wait polls try_lock() rather than using std::timed_mutex:
// libtsan does not intercept pthread_mutex_clocklock, so try_lock_for()
// acquisitions are invisible to ThreadSanitizer and produce false reports.
// `failNextTimedLocks` makes that many bounded-timeout lock attempts fail, so
// single-threaded tests can exercise the producer's busy path deterministically
// (static because the pool owns its lock privately; leave at 0 when threaded).
struct AutoPubTestLock
{
    std::mutex mutex;
    static inline uint32_t failNextTimedLocks = 0;

    bool lock(uint32_t timeoutMs)
    {
        if (timeoutMs == UINT32_MAX)
        {
            mutex.lock();
            return true;
        }
        if (failNextTimedLocks > 0)
        {
            --failNextTimedLocks;
            return false;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        while (!mutex.try_lock())
        {
            if (std::chrono::steady_clock::now() >= deadline)
                return false;
            std::this_thread::yield();
        }
        return true;
    }

    void unlock() { mutex.unlock(); }
};
