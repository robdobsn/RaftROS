/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubTopicNaming - source-compatible aliases for the shared AutoPubTopicNaming
//
// The implementation moved to AutoPub/AutoPubTopicNaming.h so a non-RTPS backend
// can derive the same ROS 2 names.  These forwarders keep existing RTPS callers
// compiling unchanged (source compatibility only - not a precompiled ABI).
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AutoPub/AutoPubTopicNaming.h"

namespace RaftRuntime {
namespace RTPS {
namespace Runtime {
namespace AutoPub {

static constexpr const char* RTPS_AUTOPUB_FALLBACK_TYPE =
    RaftRuntime::AutoPub::AUTOPUB_FALLBACK_TYPE;

inline bool RTPSAutoPubTopicNaming_formatClassTopic(
        char* pOutBuf, size_t outBufLen, const char* pSlug,
        uint8_t busNum, uint32_t addr)
{
    return RaftRuntime::AutoPub::AutoPubTopicNaming_formatClassTopic(
        pOutBuf, outBufLen, pSlug, busNum, addr);
}

inline bool RTPSAutoPubTopicNaming_formatFallbackTopic(
        char* pOutBuf, size_t outBufLen, uint8_t busNum, uint32_t addr)
{
    return RaftRuntime::AutoPub::AutoPubTopicNaming_formatFallbackTopic(
        pOutBuf, outBufLen, busNum, addr);
}

inline bool RTPSAutoPubTopicNaming_formatFallbackType(char* pOutBuf, size_t outBufLen)
{
    return RaftRuntime::AutoPub::AutoPubTopicNaming_formatFallbackType(pOutBuf, outBufLen);
}

} // namespace AutoPub
} // namespace Runtime
} // namespace RTPS
} // namespace RaftRuntime
