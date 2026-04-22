/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSSEDPPublicationParser — extract writerGUID + topicName from an SEDP DiscoveredWriterData
//
// A minimal zero-copy parser over the ParameterList payload of an SEDP
// BuiltinPublications DATA submessage.  Returns pointers+lengths into the caller-owned
// buffer for the topic string (not copied).
//
// Input: `pPayload` points to the start of the serialised payload INSIDE the DATA
// submessage, i.e. immediately after the 20-byte prefix
// (extraFlags + octetsToInlineQoS + readerId + writerId + seqNum), which includes
// the 4-byte CDR encapsulation header at its head.
//
// Shared-runtime layer: pure data / pure decisions; no sockets, no dynamic allocation,
// no Raft/ESP-IDF APIs.  Safe to include from unit tests.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

namespace RaftRuntime::RTPS::Runtime::Dispatch
{

struct RTPSParsedPublicationAnnounce
{
    bool hasWriterGuid = false;
    uint8_t writerGuid[16] = {0};   // guidPrefix(12) + writerEntityId(4)
    const char* topic = nullptr;    // pointer into source buffer (not null-safe if buffer freed)
    uint32_t topicLen = 0;          // length excluding trailing null
};

// Parse an SEDP BuiltinPublicationsData ParameterList payload (starting with the 4-byte
// CDR encapsulation header).  Returns true if at least PID_ENDPOINT_GUID was successfully
// extracted; topic is best-effort (may be null if PID_TOPIC_NAME was not present).
inline bool RTPSSEDPPublicationParser_parse(
    const uint8_t* pPayload,
    uint32_t payloadLen,
    RTPSParsedPublicationAnnounce& out)
{
    out = RTPSParsedPublicationAnnounce{};

    if (!pPayload || payloadLen < 8)
        return false;

    // Skip 4-byte CDR encapsulation header.
    const uint8_t* p = pPayload + 4;
    uint32_t remaining = payloadLen - 4;

    static constexpr uint16_t PID_SENTINEL       = 0x0001;
    static constexpr uint16_t PID_TOPIC_NAME     = 0x0005;
    static constexpr uint16_t PID_ENDPOINT_GUID  = 0x005A;

    uint32_t off = 0;
    while (off + 4 <= remaining)
    {
        const uint16_t pid  = static_cast<uint16_t>(p[off] | (p[off + 1] << 8));
        const uint16_t plen = static_cast<uint16_t>(p[off + 2] | (p[off + 3] << 8));
        off += 4;

        if (pid == PID_SENTINEL)
            break;
        if (static_cast<uint32_t>(off) + plen > remaining)
            break;

        const uint8_t* pVal = p + off;

        switch (pid)
        {
            case PID_ENDPOINT_GUID:
                if (plen >= 16)
                {
                    memcpy(out.writerGuid, pVal, 16);
                    out.hasWriterGuid = true;
                }
                break;

            case PID_TOPIC_NAME:
                // CDR string: uint32 LE length (including null) + bytes.
                if (plen >= 5)
                {
                    const uint32_t strLen =
                        static_cast<uint32_t>(pVal[0]) |
                        (static_cast<uint32_t>(pVal[1]) << 8) |
                        (static_cast<uint32_t>(pVal[2]) << 16) |
                        (static_cast<uint32_t>(pVal[3]) << 24);
                    // strLen includes trailing null; cap at plen - 4 for safety.
                    if (strLen > 0 && strLen <= (uint32_t)(plen - 4))
                    {
                        out.topic = reinterpret_cast<const char*>(pVal + 4);
                        out.topicLen = strLen - 1; // exclude null
                    }
                }
                break;

            default:
                break;
        }

        off += plen;
    }

    return out.hasWriterGuid;
}

} // namespace RaftRuntime::RTPS::Runtime::Dispatch
