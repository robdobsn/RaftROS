/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSSEDPSubscriptionParser — extract readerGUID + topicName + unicast locator port
// from an SEDP DiscoveredReaderData payload (writer = ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER).
//
// Mirrors RTPSSEDPPublicationParser_parse but additionally captures
// PID_UNICAST_LOCATOR (PID=0x002F).  This is required because rmw_fastrtps creates the
// `ros_discovery_info` subscription with
// `subscription_options.require_unique_network_flow_endpoints =
//  RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED`, which causes
// FastDDS to bind that reader to a UNIQUE UDP socket distinct from the participant
// default user-data port.  The reader's actual port is advertised in PID_UNICAST_LOCATOR;
// our DATA submessages addressed to that reader MUST use this port or the samples
// silently never reach the reader.
//
// Shared-runtime layer: pure data / pure decisions.  Safe to include from unit tests.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

namespace RaftRuntime::RTPS::Runtime::Dispatch
{

struct RTPSParsedSubscriptionAnnounce
{
    bool hasReaderGuid = false;
    uint8_t readerGuid[16] = {0};       // guidPrefix(12) + readerEntityId(4)
    const char* topic = nullptr;        // pointer into source buffer (not copied)
    uint32_t topicLen = 0;              // length excluding trailing null
    uint16_t unicastLocatorPort = 0;    // first PID_UNICAST_LOCATOR UDPv4 port (0 if absent)
};

inline bool RTPSSEDPSubscriptionParser_parseParameterList(
    const uint8_t* p,
    uint32_t remaining,
    RTPSParsedSubscriptionAnnounce& out)
{
    out = RTPSParsedSubscriptionAnnounce{};

    if (!p || remaining < 4)
        return false;

    static constexpr uint16_t PID_SENTINEL          = 0x0001;
    static constexpr uint16_t PID_TOPIC_NAME        = 0x0005;
    static constexpr uint16_t PID_UNICAST_LOCATOR   = 0x002F;
    static constexpr uint16_t PID_ENDPOINT_GUID     = 0x005A;
    static constexpr uint16_t PID_KEY_HASH          = 0x0070;

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
            case PID_KEY_HASH:
                if (plen >= 16)
                {
                    memcpy(out.readerGuid, pVal, 16);
                    out.hasReaderGuid = true;
                }
                break;

            case PID_TOPIC_NAME:
                if (plen >= 5)
                {
                    const uint32_t strLen =
                        static_cast<uint32_t>(pVal[0]) |
                        (static_cast<uint32_t>(pVal[1]) << 8) |
                        (static_cast<uint32_t>(pVal[2]) << 16) |
                        (static_cast<uint32_t>(pVal[3]) << 24);
                    if (strLen > 0 && strLen <= (uint32_t)(plen - 4))
                    {
                        out.topic = reinterpret_cast<const char*>(pVal + 4);
                        out.topicLen = strLen - 1; // exclude null
                    }
                }
                break;

            case PID_UNICAST_LOCATOR:
                // Locator (DDSI-RTPS §9.4.5.10): kind(int32 LE) + port(uint32 LE) + address(16)
                // For UDPv4 (kind=1) the port fits in the low 16 bits.
                // Take the FIRST locator only (typically the only entry for OPTIONALLY_REQUIRED
                // unique-flow endpoints).
                if (plen >= 24 && out.unicastLocatorPort == 0)
                {
                    const uint32_t port32 =
                        static_cast<uint32_t>(pVal[4]) |
                        (static_cast<uint32_t>(pVal[5]) << 8) |
                        (static_cast<uint32_t>(pVal[6]) << 16) |
                        (static_cast<uint32_t>(pVal[7]) << 24);
                    if (port32 != 0 && port32 <= 0xFFFF)
                        out.unicastLocatorPort = static_cast<uint16_t>(port32);
                }
                break;

            default:
                break;
        }

        off += plen;
    }

    return out.hasReaderGuid;
}

// Parse an SEDP BuiltinSubscriptionsData ParameterList payload (starting with the 4-byte
// CDR encapsulation header).  Returns true if the reader GUID was extracted from either
// PID_ENDPOINT_GUID or PID_KEY_HASH.
inline bool RTPSSEDPSubscriptionParser_parse(
    const uint8_t* pPayload,
    uint32_t payloadLen,
    RTPSParsedSubscriptionAnnounce& out)
{
    if (!pPayload || payloadLen < 8)
    {
        out = RTPSParsedSubscriptionAnnounce{};
        return false;
    }

    // Skip 4-byte CDR encapsulation header.
    return RTPSSEDPSubscriptionParser_parseParameterList(
        pPayload + 4, payloadLen - 4, out);
}

} // namespace RaftRuntime::RTPS::Runtime::Dispatch
