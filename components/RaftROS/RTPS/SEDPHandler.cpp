/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SEDP Handler - Simple Endpoint Discovery Protocol message building
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SEDPHandler.h"
#include "RTPSMessage.h"
#include "RTPSParticipant.h"

SEDPHandler::SEDPHandler()
{
}

SEDPHandler::~SEDPHandler()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write a CDR string into a ParameterList value: uint32 length + chars (incl null) + padding
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::writePLString(uint8_t* pBuf, const char* str)
{
    uint32_t slen = (uint32_t)strlen(str) + 1;  // include null terminator
    uint32_t pos = 0;
    // String length (LE uint32)
    pBuf[pos++] = slen & 0xFF;
    pBuf[pos++] = (slen >> 8) & 0xFF;
    pBuf[pos++] = (slen >> 16) & 0xFF;
    pBuf[pos++] = (slen >> 24) & 0xFF;
    // String data including null
    memcpy(pBuf + pos, str, slen);
    pos += slen;
    // Pad to 4-byte alignment
    while (pos % 4 != 0)
        pBuf[pos++] = 0;
    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers for writing LE values into buffer
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void putLE16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void putLE32(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build SEDP publication announcement RTPS message
// Announces a DataWriter endpoint to a remote participant
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildPublicationMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* writerEntityId,
    const char* topicName,
    const char* typeName,
    uint32_t reliabilityKind,
    uint32_t durabilityKind,
    uint64_t sequenceNumber,
    uint32_t ipAddrNetOrder,
    uint32_t heartbeatCount)
{
    if (!pBuf || bufLen < 400)
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST - set destination participant
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // Build SEDP publication ParameterList payload
    uint8_t payload[600];
    uint32_t pp = 0;

    // CDR Encapsulation: PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;  // PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;

    // PID_ENDPOINT_GUID (guidPrefix + entityId of the announced writer)
    putLE16(payload + pp, PID_ENDPOINT_GUID); pp += 2;
    putLE16(payload + pp, 16);                pp += 2;  // length = 16
    memcpy(payload + pp, participant.getGuidPrefix(), 12);
    pp += 12;
    memcpy(payload + pp, writerEntityId, 4);
    pp += 4;

    // PID_TOPIC_NAME (CDR string in parameter value)
    uint32_t topicStrLen = (uint32_t)strlen(topicName) + 1;
    uint32_t topicPadded = (topicStrLen + 3) & ~3u;
    putLE16(payload + pp, PID_TOPIC_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + topicPadded)); pp += 2;
    pp += writePLString(payload + pp, topicName);

    // PID_TYPE_NAME (CDR string in parameter value)
    uint32_t typeStrLen = (uint32_t)strlen(typeName) + 1;
    uint32_t typePadded = (typeStrLen + 3) & ~3u;
    putLE16(payload + pp, PID_TYPE_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + typePadded)); pp += 2;
    pp += writePLString(payload + pp, typeName);

    // PID_RELIABILITY (kind: uint32 + max_blocking_time: Duration_t = 8 bytes)
    putLE16(payload + pp, PID_RELIABILITY); pp += 2;
    putLE16(payload + pp, 12); pp += 2;  // length = 12
    putLE32(payload + pp, reliabilityKind); pp += 4;
    // max_blocking_time: 0.1s for RELIABLE
    putLE32(payload + pp, 0); pp += 4;   // seconds
    putLE32(payload + pp, 100000000); pp += 4; // fraction (~0.1s)

    // PID_DURABILITY (kind: uint32)
    putLE16(payload + pp, PID_DURABILITY); pp += 2;
    putLE16(payload + pp, 4); pp += 2;   // length = 4
    putLE32(payload + pp, durabilityKind); pp += 4;

    // PID_UNICAST_LOCATOR — tells remote where to send user data
    if (ipAddrNetOrder != 0)
    {
        putLE16(payload + pp, PID_UNICAST_LOCATOR); pp += 2;
        putLE16(payload + pp, 24); pp += 2;  // length = 24 (locator struct)
        RTPSParticipant::buildLocatorUDPv4(payload + pp, ipAddrNetOrder,
                                           participant.getUserUnicastPort());
        pp += 24;
    }

    // PID_PARTICIPANT_GUID
    putLE16(payload + pp, PID_PARTICIPANT_GUID); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, ENTITYID_PARTICIPANT, 4); pp += 4;

    // PID_SENTINEL
    putLE16(payload + pp, PID_SENTINEL); pp += 2;
    putLE16(payload + pp, 0); pp += 2;

    // Write DATA submessage: writerEntityId=SEDP_PUBLICATIONS_WRITER,
    // readerEntityId=SEDP_PUBLICATIONS_READER (remote)
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT for this SEDP writer (RELIABLE built-in endpoint)
    uint32_t hbCount = heartbeatCount > 0 ? heartbeatCount : (uint32_t)sequenceNumber;
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
        0, 1,  // firstSN = 1
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),  // lastSN
        hbCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build user DATA message (e.g., ros_discovery_info) with HEARTBEAT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildUserDataMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* writerEntityId,
    const uint8_t* pPayload, uint32_t payloadLen,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount,
    uint64_t firstSN)
{
    if (!pBuf || bufLen < (uint32_t)(80 + payloadLen))
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // DATA submessage with user payload
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_UNKNOWN,
        writerEntityId,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        pPayload, payloadLen);

    // HEARTBEAT (RELIABLE DataWriter)
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_UNKNOWN,
        writerEntityId,
        0, (uint32_t)(firstSN & 0xFFFFFFFF),
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        heartbeatCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build SEDP subscription announcement RTPS message
// Announces a DataReader endpoint to a remote participant
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildSubscriptionMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* readerEntityId,
    const char* topicName,
    const char* typeName,
    uint32_t reliabilityKind,
    uint32_t durabilityKind,
    uint64_t sequenceNumber,
    uint32_t ipAddrNetOrder,
    uint32_t heartbeatCount)
{
    if (!pBuf || bufLen < 300)
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // Build SEDP subscription ParameterList payload
    uint8_t payload[600];
    uint32_t pp = 0;

    // CDR Encapsulation: PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;

    // PID_ENDPOINT_GUID
    putLE16(payload + pp, PID_ENDPOINT_GUID); pp += 2;
    putLE16(payload + pp, 16);                pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12);
    pp += 12;
    memcpy(payload + pp, readerEntityId, 4);
    pp += 4;

    // PID_TOPIC_NAME
    uint32_t topicPadded = ((uint32_t)strlen(topicName) + 1 + 3) & ~3u;
    putLE16(payload + pp, PID_TOPIC_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + topicPadded)); pp += 2;
    pp += writePLString(payload + pp, topicName);

    // PID_TYPE_NAME
    uint32_t typePadded = ((uint32_t)strlen(typeName) + 1 + 3) & ~3u;
    putLE16(payload + pp, PID_TYPE_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + typePadded)); pp += 2;
    pp += writePLString(payload + pp, typeName);

    // PID_RELIABILITY
    putLE16(payload + pp, PID_RELIABILITY); pp += 2;
    putLE16(payload + pp, 12); pp += 2;
    putLE32(payload + pp, reliabilityKind); pp += 4;
    putLE32(payload + pp, 0); pp += 4;
    putLE32(payload + pp, 100000000); pp += 4;

    // PID_DURABILITY
    putLE16(payload + pp, PID_DURABILITY); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    putLE32(payload + pp, durabilityKind); pp += 4;

    // PID_UNICAST_LOCATOR — tells remote where to send user data
    if (ipAddrNetOrder != 0)
    {
        putLE16(payload + pp, PID_UNICAST_LOCATOR); pp += 2;
        putLE16(payload + pp, 24); pp += 2;
        RTPSParticipant::buildLocatorUDPv4(payload + pp, ipAddrNetOrder,
                                           participant.getUserUnicastPort());
        pp += 24;
    }

    // PID_PARTICIPANT_GUID
    putLE16(payload + pp, PID_PARTICIPANT_GUID); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, ENTITYID_PARTICIPANT, 4); pp += 4;

    // PID_SENTINEL
    putLE16(payload + pp, PID_SENTINEL); pp += 2;
    putLE16(payload + pp, 0); pp += 2;

    // Write DATA submessage via SEDP subscriptions writer
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT
    uint32_t hbCount = heartbeatCount > 0 ? heartbeatCount : (uint32_t)sequenceNumber;
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
        0, 1,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        hbCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build Participant Message Data (liveliness assertion)
// GuidPrefix + PARTICIPANT_MESSAGE_DATA_WRITER -> DATA + HB
// Payload: participantGuidPrefix(12) + kind(4) + data(4+0)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildParticipantMessageData(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount)
{
    if (!pBuf || bufLen < 120)
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // ParticipantMessageData payload:
    // CDR encapsulation (4 bytes) + participantGuidPrefix (12 bytes)
    // + kind (4 bytes: AUTOMATIC_LIVELINESS_UPDATE = {0,0,0,1})
    // + sequence of octets length (4 bytes: 0 = empty)
    uint8_t payload[24];
    uint32_t pp = 0;
    payload[pp++] = 0x00; payload[pp++] = 0x01;  // CDR_LE (plain CDR)
    payload[pp++] = 0x00; payload[pp++] = 0x00;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    // kind = AUTOMATIC_LIVELINESS_UPDATE
    payload[pp++] = 0x00; payload[pp++] = 0x00;
    payload[pp++] = 0x00; payload[pp++] = 0x01;
    // data length = 0
    payload[pp++] = 0x00; payload[pp++] = 0x00;
    payload[pp++] = 0x00; payload[pp++] = 0x00;

    // DATA submessage
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
        0, 1,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        heartbeatCount);

    return pos;
}
