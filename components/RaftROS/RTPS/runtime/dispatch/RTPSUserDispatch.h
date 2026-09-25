#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS User Dispatch - pure helpers to decode user-topic DATA submessage payloads
// into application-friendly representations.
//
// Currently supports `std_msgs::msg::String_` (ROS 2 std_msgs/String), which is the
// default type used by the `rt/chatter` / `rt/chatter_in` topics in this codebase.
//
// No platform IO. No heap allocation. Wrappers choose when to invoke and how to deliver
// the decoded value to user code.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstring>

#include "CDRDecoder.h"
#include "AutoPub/AutoPubStringMessage.h"

namespace RaftRuntime::RTPS::Runtime::UserDispatch
{

// The decode itself is shared with the Zenoh build - an RTPS DATA payload and
// a Zenoh sample payload are the same CDR bytes - so these are aliases of
// AutoPub/AutoPubStringMessage.h and behave exactly as before.
using RTPSStdMsgsStringDecodeResult = RaftRuntime::AutoPub::AutoPubStringDecodeResult;

/// @brief Decode a `std_msgs::msg::String_` CDR-encapsulated payload
inline RTPSStdMsgsStringDecodeResult decodeStdMsgsString(
    const uint8_t* pPayload, uint32_t payloadLen,
    char* outBuf, uint32_t outBufLen)
{
    return RaftRuntime::AutoPub::AutoPubStringMessage_decode(pPayload, payloadLen, outBuf, outBufLen);
}

} // namespace RaftRuntime::RTPS::Runtime::UserDispatch
