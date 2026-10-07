/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubTopicNaming - transport-neutral ROS 2 topic / type name formatting for auto-published bus devices
//
// Phase 4 / Slice 4.3
//
// Header-only, no RaftCore / ESP-IDF dependencies so the pure string-formatting
// logic can be unit-tested in the Linux host harness without linking the full
// Raft firmware stack.  The per-device `clas`→message mapping that supersedes
// these fallbacks is introduced in Slice 4.4 (AutoPubClassMap).
//
// The fallback topic format used until the class map is in place is:
//     /raft/raw_<bus>_<addrHex>
//
// and the fallback type is:
//     std_msgs::msg::dds_::String_
//
// These are ROS topic names.  The "rt/" prefix DDS puts on the wire belongs to
// the RTPS backend, not here - a Zenoh image would reject it as not a ROS path.
// Type names
// use the "<pkg>::msg::dds_::<Type>_" form (note the trailing underscore).
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace RaftRuntime {
namespace AutoPub {

// ---------------------------------------------------------------------------
// Fallback ROS 2 type name used by Slice 4.3 before per-class mapping lands.
// std_msgs/String is a safe lowest-common-denominator: every ROS 2 install
// ships with it, and raw decoded device data can be serialized as JSON text.
// ---------------------------------------------------------------------------
static constexpr const char* AUTOPUB_FALLBACK_TYPE =
    "std_msgs::msg::dds_::String_";

/// @brief Format a ROS 2 topic name for a bus device given a message-class slug.
/// @param pOutBuf       output buffer (must be non-null)
/// @param outBufLen     size of output buffer in bytes (must be > 0)
/// @param pSlug         short message-class slug, e.g. "temperature" or "imu"
/// @param busNum        bus number (typically 1 for I2C-0)
/// @param address       device address (printed as lowercase 2-hex)
/// @return true on success, false if the output would not fit or inputs invalid
///
/// Format: `/raft/<slug>_<bus>_<addrHex>` - the ROS topic name, not a wire
/// name.  Each backend applies its own convention to it: RTPS prefixes `rt`
/// to get the DDS topic, Zenoh mangles it into a key expression.  The legacy fallback formatter
/// (`AutoPubTopicNaming_formatFallbackTopic`) is now a thin wrapper that
/// passes `"raw"` here, keeping the Slice 4.3 behaviour byte-for-byte.
inline bool AutoPubTopicNaming_formatClassTopic(
        char* pOutBuf, size_t outBufLen,
        const char* pSlug,
        uint8_t busNum, uint32_t address)
{
    if (!pOutBuf || outBufLen == 0 || !pSlug || !pSlug[0])
        return false;
    // The whole bus element address: for a device behind an I2C mux it
    // carries the slot in the upper bits (0x229 = address 0x29 on slot 2),
    // and two of the same sensor on different slots must get different topics
    const int written = std::snprintf(pOutBuf, outBufLen,
                                      "/raft/%s_%u_%02x",
                                      pSlug,
                                      (unsigned)busNum,
                                      (unsigned)address);
    if (written < 0)
    {
        pOutBuf[0] = '\0';
        return false;
    }
    if ((size_t)written >= outBufLen)
    {
        pOutBuf[0] = '\0';
        return false;
    }
    return true;
}

/// @brief Format the fallback ROS 2 topic name for a bus device.
/// @param pOutBuf       output buffer (must be non-null)
/// @param outBufLen     size of output buffer in bytes (must be > 0)
/// @param busNum        bus number (typically 1 for I2C-0)
/// @param address       device address (7-bit I2C address printed as 2-hex-lower)
/// @return true on success, false if the output would not fit or inputs invalid
inline bool AutoPubTopicNaming_formatFallbackTopic(
        char* pOutBuf, size_t outBufLen,
        uint8_t busNum, uint32_t address)
{
    return AutoPubTopicNaming_formatClassTopic(pOutBuf, outBufLen,
                                                   "raw", busNum, address);
}

/// @brief Copy the fallback ROS 2 type name into the supplied buffer.
/// @param pOutBuf       output buffer (must be non-null)
/// @param outBufLen     size of output buffer in bytes (must be > 0)
/// @return true on success, false if the output would not fit or inputs invalid
inline bool AutoPubTopicNaming_formatFallbackType(char* pOutBuf, size_t outBufLen)
{
    if (!pOutBuf || outBufLen == 0)
        return false;
    const int written = std::snprintf(pOutBuf, outBufLen,
                                      "%s", AUTOPUB_FALLBACK_TYPE);
    if (written < 0)
    {
        pOutBuf[0] = '\0';
        return false;
    }
    if ((size_t)written >= outBufLen)
    {
        pOutBuf[0] = '\0';
        return false;
    }
    return true;
}

} // namespace AutoPub
} // namespace RaftRuntime
