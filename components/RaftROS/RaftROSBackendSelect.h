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
//     CONFIG_RAFTROS_BACKEND_ZENOH=y        # default - Raft's own Zenoh implementation
//     CONFIG_RAFTROS_BACKEND_RTPS=y
//     CONFIG_RAFTROS_BACKEND_ZENOH_PICO=y   # experimental - the same ROS layer over zenoh-pico
//
// RAFTROS_BACKEND_ZENOH means Raft's own (clean-room) Zenoh session; the
// zenoh-pico build sets RAFTROS_BACKEND_ZENOH_PICO instead, so code written for
// one Zenoh implementation never compiles silently against the other.
//
// Host builds have no Kconfig, so they default to Zenoh unless
// RAFTROS_BACKEND_RTPS is defined on the compiler command line.
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
#if defined(CONFIG_RAFTROS_BACKEND_ZENOH) || defined(CONFIG_RAFTROS_BACKEND_RTPS) || \
    defined(CONFIG_RAFTROS_BACKEND_ZENOH_PICO)
    #undef RAFTROS_BACKEND_ZENOH
    #undef RAFTROS_BACKEND_RTPS
    #undef RAFTROS_BACKEND_ZENOH_PICO
    #if defined(CONFIG_RAFTROS_BACKEND_RTPS)
        #define RAFTROS_BACKEND_ZENOH      0
        #define RAFTROS_BACKEND_RTPS       1
        #define RAFTROS_BACKEND_ZENOH_PICO 0
    #elif defined(CONFIG_RAFTROS_BACKEND_ZENOH_PICO)
        #define RAFTROS_BACKEND_ZENOH      0
        #define RAFTROS_BACKEND_RTPS       0
        #define RAFTROS_BACKEND_ZENOH_PICO 1
    #else
        #define RAFTROS_BACKEND_ZENOH      1
        #define RAFTROS_BACKEND_RTPS       0
        #define RAFTROS_BACKEND_ZENOH_PICO 0
    #endif
#elif defined(RAFTROS_BACKEND_RTPS) && RAFTROS_BACKEND_RTPS
    #undef RAFTROS_BACKEND_ZENOH
    #define RAFTROS_BACKEND_ZENOH 0
#else
    #undef RAFTROS_BACKEND_ZENOH
    #undef RAFTROS_BACKEND_RTPS
    #define RAFTROS_BACKEND_ZENOH 1
    #define RAFTROS_BACKEND_RTPS  0
#endif

// Host builds never select zenoh-pico
#ifndef RAFTROS_BACKEND_ZENOH_PICO
    #define RAFTROS_BACKEND_ZENOH_PICO 0
#endif

#if (RAFTROS_BACKEND_RTPS + RAFTROS_BACKEND_ZENOH + RAFTROS_BACKEND_ZENOH_PICO) != 1
    #error "RaftROS: exactly one transport backend must be selected"
#endif

/// @brief Name of the selected backend, for logs and status JSON
#if RAFTROS_BACKEND_ZENOH
    #define RAFTROS_BACKEND_NAME "zenoh"
#elif RAFTROS_BACKEND_ZENOH_PICO
    #define RAFTROS_BACKEND_NAME "zenoh-pico"
#else
    #define RAFTROS_BACKEND_NAME "rtps"
#endif
