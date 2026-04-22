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

namespace RaftRuntime::RTPS::Runtime::UserDispatch
{

// Outcome of decoding a DATA submessage payload as std_msgs/String.
// `textLen` excludes the trailing null; `outBuf` is always null-terminated on success.
struct RTPSStdMsgsStringDecodeResult
{
    bool success = false;
    uint32_t textLen = 0;
};

// Decode a `std_msgs::msg::String_` CDR-encapsulated payload.
//
// pPayload/payloadLen must cover the full DATA serialized payload, *including* the
// 4-byte CDR encapsulation header (scheme + options). Returns success=true iff the
// encapsulation header parses, the string length field fits, and the string content
// is fully present in the buffer.
//
// outBuf must be non-null; content will be truncated (but still null-terminated)
// if textLen+1 would exceed outBufLen.
inline RTPSStdMsgsStringDecodeResult decodeStdMsgsString(
    const uint8_t* pPayload, uint32_t payloadLen,
    char* outBuf, uint32_t outBufLen)
{
    RTPSStdMsgsStringDecodeResult res;
    if (!pPayload || !outBuf || outBufLen == 0)
        return res;

    CDRDecoder dec;
    dec.init(pPayload, payloadLen);
    if (!dec.readEncapsulationHeader())
        return res;

    uint32_t strLen = 0;
    if (!dec.readString(outBuf, outBufLen, strLen))
        return res;

    res.success = true;
    res.textLen = strLen;
    return res;
}

} // namespace RaftRuntime::RTPS::Runtime::UserDispatch
