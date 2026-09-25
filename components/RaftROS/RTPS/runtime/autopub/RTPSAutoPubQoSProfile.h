/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubQoSProfile - source-compatible aliases for the shared AutoPubQoSProfile
//
// The profiles moved to AutoPub/AutoPubQoSProfile.h so a non-RTPS backend can
// resolve the same semantic QoS from SysTypes overrides and device classes.
// The numeric reliability/durability values are the RTPS wire kinds, which this
// backend passes straight through.  These forwarders keep existing RTPS callers
// compiling unchanged (source compatibility only - not a precompiled ABI).
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AutoPub/AutoPubQoSProfile.h"

namespace RaftRuntime {
namespace RTPS {
namespace Runtime {
namespace AutoPub {

using RTPSAutoPubQoSProfileId = RaftRuntime::AutoPub::AutoPubQoSProfileId;
using RTPSAutoPubQoSProfile   = RaftRuntime::AutoPub::AutoPubQoSProfile;

static constexpr int32_t RTPS_AUTOPUB_RELIABILITY_BEST_EFFORT =
    RaftRuntime::AutoPub::AUTOPUB_RELIABILITY_BEST_EFFORT;
static constexpr int32_t RTPS_AUTOPUB_RELIABILITY_RELIABLE =
    RaftRuntime::AutoPub::AUTOPUB_RELIABILITY_RELIABLE;
static constexpr int32_t RTPS_AUTOPUB_DURABILITY_VOLATILE =
    RaftRuntime::AutoPub::AUTOPUB_DURABILITY_VOLATILE;
static constexpr int32_t RTPS_AUTOPUB_DURABILITY_TRANSIENT_LOCAL =
    RaftRuntime::AutoPub::AUTOPUB_DURABILITY_TRANSIENT_LOCAL;

inline RTPSAutoPubQoSProfile RTPSAutoPubQoSProfile_get(RTPSAutoPubQoSProfileId id)
{
    return RaftRuntime::AutoPub::AutoPubQoSProfile_get(id);
}

inline const char* RTPSAutoPubQoSProfile_name(RTPSAutoPubQoSProfileId id)
{
    return RaftRuntime::AutoPub::AutoPubQoSProfile_name(id);
}

inline bool RTPSAutoPubQoSProfile_parseName(const char* name, RTPSAutoPubQoSProfileId& outId)
{
    return RaftRuntime::AutoPub::AutoPubQoSProfile_parseName(name, outId);
}

inline RTPSAutoPubQoSProfileId RTPSAutoPubQoSProfile_defaultForClass(const char* clasCode)
{
    return RaftRuntime::AutoPub::AutoPubQoSProfile_defaultForClass(clasCode);
}

inline RTPSAutoPubQoSProfileId RTPSAutoPubQoSProfile_defaultForClasses(
        const char* const* clasArray, size_t clasCount, const char* deviceTypeName = nullptr)
{
    return RaftRuntime::AutoPub::AutoPubQoSProfile_defaultForClasses(
        clasArray, clasCount, deviceTypeName);
}

} // namespace AutoPub
} // namespace Runtime
} // namespace RTPS
} // namespace RaftRuntime
