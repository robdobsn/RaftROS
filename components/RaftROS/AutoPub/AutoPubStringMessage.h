/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubStringMessage - decode a std_msgs/String sample, whatever carried it
//
// Both transports deliver the same CDR bytes: an RTPS DATA submessage payload
// and a Zenoh sample payload are byte-identical for the same message, so the
// decode belongs above the transport.  RTPSUserDispatch keeps its own names as
// aliases of this.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>

#include "CDRDecoder.h"

namespace RaftRuntime::AutoPub
{

/// @brief Outcome of decoding a payload as std_msgs/String.  `textLen`
/// excludes the trailing NUL; the output is always NUL-terminated on success.
struct AutoPubStringDecodeResult
{
    bool success = false;
    uint32_t textLen = 0;
};

/// @brief Decode a `std_msgs::msg::String_` CDR payload
/// @param pPayload the full serialised payload, including the 4-byte CDR
///        encapsulation header (scheme + options)
/// @param outBuf non-null; content is truncated (but still NUL-terminated) if
///        it would not fit, and `textLen` still reports the source length
inline AutoPubStringDecodeResult AutoPubStringMessage_decode(
    const uint8_t* pPayload, uint32_t payloadLen,
    char* outBuf, uint32_t outBufLen)
{
    AutoPubStringDecodeResult result;
    if (!pPayload || !outBuf || outBufLen == 0)
        return result;

    CDRDecoder decoder;
    decoder.init(pPayload, payloadLen);
    if (!decoder.readEncapsulationHeader())
        return result;

    uint32_t strLen = 0;
    if (!decoder.readString(outBuf, outBufLen, strLen))
        return result;

    result.success = true;
    result.textLen = strLen;
    return result;
}

} // namespace RaftRuntime::AutoPub
