/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROSBackendSelect - compile-time choice of exactly one ROS 2 transport backend
//
// RaftROS speaks ROS 2 either natively over RTPS/DDS or through a Zenoh router.
// The two share the auto-publish pipeline (device mapping, CDR serialisation,
// the publisher pool and the create/destroy/publish backend contract) but
// nothing below it, so a firmware image links one or the other - never both,
// and never a vtable to choose between them at runtime.
//
// Select with menuconfig (`RaftROS` menu) or in sdkconfig.defaults:
//
//     CONFIG_RAFTROS_BACKEND_RTPS=y      # default
//     CONFIG_RAFTROS_BACKEND_ZENOH=y
//
// Host builds have no Kconfig, so they default to RTPS unless
// RAFTROS_BACKEND_ZENOH is defined on the compiler command line.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Kconfig values reach a translation unit only through sdkconfig.h, which
// ESP-IDF does not force-include
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

// Kconfig (firmware) wins; a command-line define (host tests) is the fallback
#if defined(CONFIG_RAFTROS_BACKEND_ZENOH) || defined(CONFIG_RAFTROS_BACKEND_RTPS)
    #undef RAFTROS_BACKEND_ZENOH
    #undef RAFTROS_BACKEND_RTPS
    #if defined(CONFIG_RAFTROS_BACKEND_ZENOH)
        #define RAFTROS_BACKEND_ZENOH 1
        #define RAFTROS_BACKEND_RTPS  0
    #else
        #define RAFTROS_BACKEND_ZENOH 0
        #define RAFTROS_BACKEND_RTPS  1
    #endif
#elif defined(RAFTROS_BACKEND_ZENOH) && RAFTROS_BACKEND_ZENOH
    #undef RAFTROS_BACKEND_RTPS
    #define RAFTROS_BACKEND_RTPS 0
#else
    #undef RAFTROS_BACKEND_ZENOH
    #undef RAFTROS_BACKEND_RTPS
    #define RAFTROS_BACKEND_ZENOH 0
    #define RAFTROS_BACKEND_RTPS  1
#endif

#if (RAFTROS_BACKEND_RTPS + RAFTROS_BACKEND_ZENOH) != 1
    #error "RaftROS: exactly one transport backend must be selected"
#endif

/// @brief Name of the selected backend, for logs and status JSON
#if RAFTROS_BACKEND_ZENOH
    #define RAFTROS_BACKEND_NAME "zenoh"
#else
    #define RAFTROS_BACKEND_NAME "rtps"
#endif
