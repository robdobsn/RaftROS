/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SPDP Handler - Simple Participant Discovery Protocol message building
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SPDPHandler.h"
#include "RTPSMessage.h"
#include "RTPSParticipant.h"

SPDPHandler::SPDPHandler()
{
}

SPDPHandler::~SPDPHandler()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write ParameterList entry header (PID + length, both LE uint16)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SPDPHandler::writeParamHeader(uint8_t* pBuf, uint16_t pid, uint16_t length)
{
    pBuf[0] = pid & 0xFF;
    pBuf[1] = (pid >> 8) & 0xFF;
    pBuf[2] = length & 0xFF;
    pBuf[3] = (length >> 8) & 0xFF;
    return 4;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write a Locator parameter (PID header + 24-byte Locator_t)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SPDPHandler::writeLocatorParam(uint8_t* pBuf, uint16_t pid,
                                         uint32_t ipAddrNetOrder, uint16_t port)
{
    uint32_t pos = writeParamHeader(pBuf, pid, 24);
    RTPSParticipant::buildLocatorUDPv4(pBuf + pos, ipAddrNetOrder, port);
    return pos + 24;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build complete SPDP announcement RTPS message
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SPDPHandler::buildAnnouncementMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    uint32_t ipAddrNetOrder,
    uint32_t leaseDurationSec,
    uint64_t sequenceNumber)
{
    if (!pBuf || bufLen < 256)
        return 0;

    uint32_t pos = 0;

    // RTPS Header (20 bytes)
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_TS submessage (12 bytes)
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // Build SPDP payload (ParameterList) into a temp buffer
    uint8_t payload[256];
    uint32_t pp = 0;

    // CDR Encapsulation Header: PL_CDR_LE (0x00, 0x03 for parameter list LE)
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;

    // PID_PROTOCOL_VERSION (value: 2 bytes + 2 pad)
    pp += writeParamHeader(payload + pp, PID_PROTOCOL_VERSION, 4);
    payload[pp++] = RTPS_VERSION_MAJOR;
    payload[pp++] = RTPS_VERSION_MINOR;
    payload[pp++] = 0;
    payload[pp++] = 0;

    // PID_VENDORID (value: 2 bytes + 2 pad)
    pp += writeParamHeader(payload + pp, PID_VENDORID, 4);
    payload[pp++] = RTPS_VENDOR_ID[0];
    payload[pp++] = RTPS_VENDOR_ID[1];
    payload[pp++] = 0;
    payload[pp++] = 0;

    // PID_PARTICIPANT_GUID (value: 16 bytes)
    pp += writeParamHeader(payload + pp, PID_PARTICIPANT_GUID, 16);
    memcpy(payload + pp, participant.getParticipantGuid(), 16);
    pp += 16;

    // PID_BUILTIN_ENDPOINT_SET (value: 4 bytes)
    uint32_t endpointSet =
        DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER |
        DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR |
        DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER |
        DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR |
        DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER |
        DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR |
        BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER |
        BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER;
    pp += writeParamHeader(payload + pp, PID_BUILTIN_ENDPOINT_SET, 4);
    payload[pp++] = endpointSet & 0xFF;
    payload[pp++] = (endpointSet >> 8) & 0xFF;
    payload[pp++] = (endpointSet >> 16) & 0xFF;
    payload[pp++] = (endpointSet >> 24) & 0xFF;

    // PID_DEFAULT_UNICAST_LOCATOR (user data port)
    pp += writeLocatorParam(payload + pp, PID_DEFAULT_UNICAST_LOCATOR,
                            ipAddrNetOrder, participant.getUserUnicastPort());

    // PID_METATRAFFIC_UNICAST_LOCATOR (SEDP/metatraffic port)
    pp += writeLocatorParam(payload + pp, PID_METATRAFFIC_UNICAST_LOCATOR,
                            ipAddrNetOrder, participant.getMetatrafficUnicastPort());

    // PID_PARTICIPANT_LEASE_DURATION (value: 8 bytes = seconds + fraction)
    pp += writeParamHeader(payload + pp, PID_PARTICIPANT_LEASE_DURATION, 8);
    payload[pp++] = leaseDurationSec & 0xFF;
    payload[pp++] = (leaseDurationSec >> 8) & 0xFF;
    payload[pp++] = (leaseDurationSec >> 16) & 0xFF;
    payload[pp++] = (leaseDurationSec >> 24) & 0xFF;
    payload[pp++] = 0; payload[pp++] = 0;
    payload[pp++] = 0; payload[pp++] = 0;  // fraction = 0

    // PID_USER_DATA: key-value encoded "enclave=/;" required by ROS2 rmw_fastrtps
    {
        const char* userData = "enclave=/;";
        uint32_t udLen = (uint32_t)strlen(userData) + 1;  // include null terminator
        uint32_t udPadded = (udLen + 3) & ~3u;
        // PID_USER_DATA value is: uint32 length + octets + padding
        pp += writeParamHeader(payload + pp, PID_USER_DATA, (uint16_t)(4 + udPadded));
        payload[pp++] = udLen & 0xFF;
        payload[pp++] = (udLen >> 8) & 0xFF;
        payload[pp++] = (udLen >> 16) & 0xFF;
        payload[pp++] = (udLen >> 24) & 0xFF;
        memcpy(payload + pp, userData, udLen);
        pp += udLen;
        while (pp % 4 != 0) payload[pp++] = 0;
    }

    // PID_SENTINEL (terminates parameter list)
    pp += writeParamHeader(payload + pp, PID_SENTINEL, 0);

    // Write DATA submessage with the ParameterList payload
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_UNKNOWN,
        ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parse received SPDP message, extract participant info
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SPDPHandler::parseAnnouncementMessage(const uint8_t* pBuf, uint32_t bufLen,
                                            DiscoveredParticipant& outParticipant)
{
    outParticipant = DiscoveredParticipant{};

    // Parse RTPS header → extract guidPrefix
    uint32_t offset = RTPSMessage::parseHeader(pBuf, bufLen, outParticipant.guidPrefix);
    if (offset == 0)
        return false;

    // Walk submessages to find DATA
    while (offset < bufLen)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            pBuf + offset, bufLen - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0)
            break;

        if (submsgId == SUBMSG_DATA && contentLen >= 24)
        {
            // DATA content: extraFlags(2) + octetsToInlineQoS(2) +
            //   readerId(4) + writerId(4) + seqNum(8) = 20 bytes before payload
            // Then serialized data starts (CDR encaps header + ParameterList)
            const uint8_t* pPayload = pContent + 20;
            uint32_t payloadLen = contentLen - 20;

            if (payloadLen < 4)
            {
                offset += submsgSize;
                continue;
            }

            // Skip CDR encapsulation header (4 bytes)
            pPayload += 4;
            payloadLen -= 4;

            // Parse ParameterList entries
            uint32_t pOff = 0;
            while (pOff + 4 <= payloadLen)
            {
                uint16_t pid = pPayload[pOff] | (pPayload[pOff + 1] << 8);
                uint16_t plen = pPayload[pOff + 2] | (pPayload[pOff + 3] << 8);
                pOff += 4;

                if (pid == PID_SENTINEL)
                    break;
                if (pOff + plen > payloadLen)
                    break;

                const uint8_t* pVal = pPayload + pOff;

                switch (pid)
                {
                    case PID_PARTICIPANT_GUID:
                        if (plen >= 12)
                            memcpy(outParticipant.guidPrefix, pVal, 12);
                        break;

                    case PID_DEFAULT_UNICAST_LOCATOR:
                        if (plen >= 24)
                        {
                            outParticipant.userDataPort = pVal[4] | (pVal[5] << 8);
                            memcpy(&outParticipant.ipAddr, pVal + 20, 4);
                        }
                        break;

                    case PID_METATRAFFIC_UNICAST_LOCATOR:
                        if (plen >= 24)
                        {
                            outParticipant.metatrafficPort = pVal[4] | (pVal[5] << 8);
                            if (outParticipant.ipAddr == 0)
                                memcpy(&outParticipant.ipAddr, pVal + 20, 4);
                        }
                        break;

                    case PID_PARTICIPANT_LEASE_DURATION:
                        if (plen >= 4)
                        {
                            outParticipant.leaseDurationSec =
                                pVal[0] | (pVal[1] << 8) |
                                (pVal[2] << 16) | (pVal[3] << 24);
                        }
                        break;

                    default:
                        break;
                }
                pOff += plen;
            }

            outParticipant.valid = true;
            return true;
        }

        offset += submsgSize;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build ros_discovery_info CDR payload (ParticipantEntitiesInfo)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SPDPHandler::buildRosDiscoveryInfoPayload(
    uint8_t* pBuf, uint32_t bufLen,
    const uint8_t* participantGuid,
    const char* nodeName,
    const char* nodeNamespace)
{
    if (!pBuf || !participantGuid || !nodeName || !nodeNamespace)
        return 0;

    // Estimate: 4 encaps + 24 gid + 4 seq + (4+name+pad) + (4+ns+pad) + 4 + 4 = ~80+
    uint32_t nameLen = (uint32_t)strlen(nodeName) + 1;
    uint32_t nsLen = (uint32_t)strlen(nodeNamespace) + 1;
    uint32_t estimate = 4 + 24 + 4 + (4 + ((nameLen + 3) & ~3u)) +
                         (4 + ((nsLen + 3) & ~3u)) + 4 + 4;
    if (bufLen < estimate)
        return 0;

    uint32_t pos = 0;

    // CDR Encapsulation Header (LE, CDR1)
    pBuf[pos++] = 0x00;
    pBuf[pos++] = 0x01;  // CDR_LE
    pBuf[pos++] = 0x00;
    pBuf[pos++] = 0x00;

    // Gid: fixed uint8[24] (16 bytes GUID + 8 zeros)
    memcpy(pBuf + pos, participantGuid, 16);
    pos += 16;
    memset(pBuf + pos, 0, 8);
    pos += 8;

    // sequence<NodeEntitiesInfo> length = 1
    pBuf[pos++] = 1; pBuf[pos++] = 0; pBuf[pos++] = 0; pBuf[pos++] = 0;

    // NodeEntitiesInfo fields in CDR declaration order:
    // 1) string node_namespace
    pBuf[pos++] = nsLen & 0xFF;
    pBuf[pos++] = (nsLen >> 8) & 0xFF;
    pBuf[pos++] = (nsLen >> 16) & 0xFF;
    pBuf[pos++] = (nsLen >> 24) & 0xFF;
    memcpy(pBuf + pos, nodeNamespace, nsLen);
    pos += nsLen;
    while (pos % 4 != 0) pBuf[pos++] = 0;

    // 2) string node_name
    pBuf[pos++] = nameLen & 0xFF;
    pBuf[pos++] = (nameLen >> 8) & 0xFF;
    pBuf[pos++] = (nameLen >> 16) & 0xFF;
    pBuf[pos++] = (nameLen >> 24) & 0xFF;
    memcpy(pBuf + pos, nodeName, nameLen);
    pos += nameLen;
    while (pos % 4 != 0) pBuf[pos++] = 0;

    // sequence<Gid> reader_gid_seq = empty
    pBuf[pos++] = 0; pBuf[pos++] = 0; pBuf[pos++] = 0; pBuf[pos++] = 0;

    // sequence<Gid> writer_gid_seq = empty
    pBuf[pos++] = 0; pBuf[pos++] = 0; pBuf[pos++] = 0; pBuf[pos++] = 0;

    return pos;
}
