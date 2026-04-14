/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Message - Wire format encoding/decoding
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RTPSMessage.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write RTPS header
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeHeader(uint8_t* pBuf, uint32_t bufLen, const uint8_t* guidPrefix)
{
    // RTPS header is 20 bytes (DDSI-RTPS §9.4.4)
    if (bufLen < 20)
        return 0;

    // Protocol ID: "RTPS"
    memcpy(pBuf, RTPS_PROTOCOL_ID, 4);

    // Version
    pBuf[4] = RTPS_VERSION_MAJOR;
    pBuf[5] = RTPS_VERSION_MINOR;

    // Vendor ID
    pBuf[6] = RTPS_VENDOR_ID[0];
    pBuf[7] = RTPS_VENDOR_ID[1];

    // GUID Prefix (12 bytes)
    memcpy(pBuf + 8, guidPrefix, 12);

    return 20;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write DATA submessage
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeDataSubmessage(uint8_t* pBuf, uint32_t bufLen,
                                           const uint8_t* writerGuid,
                                           uint64_t sequenceNumber,
                                           const uint8_t* pPayload, uint32_t payloadLen)
{
    // TODO: Implement DATA submessage encoding per DDSI-RTPS §9.4.5.3
    (void)pBuf;
    (void)bufLen;
    (void)writerGuid;
    (void)sequenceNumber;
    (void)pPayload;
    (void)payloadLen;
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parse RTPS header
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::parseHeader(const uint8_t* pBuf, uint32_t bufLen, uint8_t* guidPrefixOut)
{
    if (bufLen < 20)
        return 0;

    // Check protocol ID
    if (memcmp(pBuf, RTPS_PROTOCOL_ID, 4) != 0)
        return 0;

    // Extract GUID prefix
    if (guidPrefixOut)
        memcpy(guidPrefixOut, pBuf + 8, 12);

    return 20;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parse submessage
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::parseSubmessage(const uint8_t* pBuf, uint32_t bufLen,
                                       RTPSSubmessageId& submsgId,
                                       const uint8_t*& pPayload, uint32_t& payloadLen)
{
    // TODO: Implement submessage parsing per DDSI-RTPS §9.4.5
    (void)pBuf;
    (void)bufLen;
    submsgId = SUBMSG_DATA;
    pPayload = nullptr;
    payloadLen = 0;
    return 0;
}
