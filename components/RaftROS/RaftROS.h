/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework
//
// This header is the application-facing name: `RaftROS` is a SysMod registered
// as usual, whichever transport the image was built with -
//
//     raftCoreApp.registerSysMod("RaftROS", RaftROS::create, true);
//
// The transport is a build-time choice (see RaftROSBackendSelect.h), so this
// header just pulls in the one implementation that was selected.  The
// implementations share the auto-publish pipeline but declare different
// internals, and only one is ever compiled.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftROSBackendSelect.h"

#if RAFTROS_BACKEND_ZENOH
#include "Zenoh/RaftROSZenoh.h"
#elif RAFTROS_BACKEND_ZENOH_PICO
#include "ZenohPico/RaftROSZenohPico.h"
#else
#include "RTPS/RaftROSRTPS.h"
#endif
