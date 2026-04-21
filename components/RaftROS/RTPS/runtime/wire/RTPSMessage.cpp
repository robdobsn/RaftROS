/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Message - Wire format encoding/decoding
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "runtime/wire/RTPSMessage.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write RTPS header (20 bytes)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeHeader(uint8_t* pBuf, uint32_t bufLen, const uint8_t* guidPrefix)
{
    if (bufLen < 20)
        return 0;
    memcpy(pBuf, RTPS_PROTOCOL_ID, 4);
    pBuf[4] = RTPS_VERSION_MAJOR;
    pBuf[5] = RTPS_VERSION_MINOR;
    pBuf[6] = RTPS_VENDOR_ID[0];
    pBuf[7] = RTPS_VENDOR_ID[1];
    memcpy(pBuf + 8, guidPrefix, 12);
    return 20;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write DATA submessage (DDSI-RTPS §8.3.7.2)
// Layout: submsg header(4) + extraFlags(2) + octetsToInlineQoS(2) +
//         readerEntityId(4) + writerEntityId(4) + writerSN(8) + payload
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeDataSubmessage(uint8_t* pBuf, uint32_t bufLen,
                                           const uint8_t* readerEntityId,
                                           const uint8_t* writerEntityId,
                                           int32_t seqNumHigh, uint32_t seqNumLow,
                                           const uint8_t* pPayload, uint32_t payloadLen)
{
    // Content size (after submessage header): extraFlags(2) + octetsToInlineQoS(2) +
    // readerId(4) + writerId(4) + seqNum(8) + payload
    uint32_t contentSize = 2 + 2 + 4 + 4 + 8 + payloadLen;
    uint32_t totalSize = 4 + contentSize; // submsg header + content
    if (bufLen < totalSize)
        return 0;

    uint32_t pos = 0;

    // Submessage header
    pBuf[pos++] = SUBMSG_DATA;                    // submessageId
    pBuf[pos++] = 0x05;                           // flags: E=1 (LE), D=1 (data present), Q=0
    writeLE16(pBuf + pos, (uint16_t)contentSize); // octetsToNextHeader
    pos += 2;

    // Extra flags
    writeLE16(pBuf + pos, 0);
    pos += 2;

    // Octets to inline QoS (offset from here to inline QoS or serialized data)
    // = readerId(4) + writerId(4) + seqNum(8) = 16
    writeLE16(pBuf + pos, 16);
    pos += 2;

    // Reader Entity ID
    memcpy(pBuf + pos, readerEntityId, 4);
    pos += 4;

    // Writer Entity ID
    memcpy(pBuf + pos, writerEntityId, 4);
    pos += 4;

    // Sequence Number (high, low) in LE
    writeLE32(pBuf + pos, (uint32_t)seqNumHigh);
    pos += 4;
    writeLE32(pBuf + pos, seqNumLow);
    pos += 4;

    // Serialized payload
    if (payloadLen > 0 && pPayload)
        memcpy(pBuf + pos, pPayload, payloadLen);
    pos += payloadLen;

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write INFO_TS submessage (12 bytes total)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeInfoTS(uint8_t* pBuf, uint32_t bufLen,
                                   int32_t seconds, uint32_t fraction)
{
    if (bufLen < 12)
        return 0;
    pBuf[0] = SUBMSG_INFO_TS;
    pBuf[1] = 0x01;                 // flags: E=1 (LE)
    writeLE16(pBuf + 2, 8);         // octetsToNextHeader = 8
    writeLE32(pBuf + 4, (uint32_t)seconds);
    writeLE32(pBuf + 8, fraction);
    return 12;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write INFO_DST submessage (16 bytes total)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeInfoDST(uint8_t* pBuf, uint32_t bufLen,
                                    const uint8_t* guidPrefix)
{
    if (bufLen < 16)
        return 0;
    pBuf[0] = SUBMSG_INFO_DST;
    pBuf[1] = 0x01;                 // flags: E=1 (LE)
    writeLE16(pBuf + 2, 12);        // octetsToNextHeader = 12
    memcpy(pBuf + 4, guidPrefix, 12);
    return 16;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write HEARTBEAT submessage (32 bytes total)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeHeartbeat(uint8_t* pBuf, uint32_t bufLen,
                                      const uint8_t* readerEntityId,
                                      const uint8_t* writerEntityId,
                                      int32_t firstSNHigh, uint32_t firstSNLow,
                                      int32_t lastSNHigh, uint32_t lastSNLow,
                                      uint32_t count)
{
    if (bufLen < 32)
        return 0;
    uint32_t pos = 0;
    pBuf[pos++] = SUBMSG_HEARTBEAT;
    pBuf[pos++] = 0x01;             // flags: E=1 (LE), F=0 (not final, ACKNACK expected)
    writeLE16(pBuf + pos, 28);      // octetsToNextHeader
    pos += 2;

    memcpy(pBuf + pos, readerEntityId, 4);  pos += 4;
    memcpy(pBuf + pos, writerEntityId, 4);  pos += 4;
    writeLE32(pBuf + pos, (uint32_t)firstSNHigh); pos += 4;
    writeLE32(pBuf + pos, firstSNLow); pos += 4;
    writeLE32(pBuf + pos, (uint32_t)lastSNHigh); pos += 4;
    writeLE32(pBuf + pos, lastSNLow); pos += 4;
    writeLE32(pBuf + pos, count); pos += 4;

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parse RTPS header
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::parseHeader(const uint8_t* pBuf, uint32_t bufLen, uint8_t* guidPrefixOut)
{
    if (bufLen < 20)
        return 0;
    if (memcmp(pBuf, RTPS_PROTOCOL_ID, 4) != 0)
        return 0;
    if (guidPrefixOut)
        memcpy(guidPrefixOut, pBuf + 8, 12);
    return 20;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parse submessage header at pBuf, returns total submessage size
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::parseSubmessage(const uint8_t* pBuf, uint32_t bufLen,
                                       RTPSSubmessageId& submsgId, uint8_t& flags,
                                       const uint8_t*& pContent, uint32_t& contentLen)
{
    if (bufLen < 4)
        return 0;
    submsgId = (RTPSSubmessageId)pBuf[0];
    flags = pBuf[1];
    // Endianness flag is bit 0
    bool le = (flags & 0x01) != 0;
    uint16_t octetsToNext = le ? readLE16(pBuf + 2) : ((pBuf[2] << 8) | pBuf[3]);
    if (4u + octetsToNext > bufLen)
        return 0;
    pContent = pBuf + 4;
    contentLen = octetsToNext;
    return 4 + octetsToNext;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write ACKNACK submessage (DDSI-RTPS §8.3.7.1)
// Layout: submsg header(4) + readerEntityId(4) + writerEntityId(4) +
//         readerSNState { bitmapBase(8), numBits(4), bitmap... } + count(4)
// With empty bitmap (numBits=0): 4+4+4+8+4+4 = 28 content bytes, 32 total
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RTPSMessage::writeAcknack(uint8_t* pBuf, uint32_t bufLen,
                                    const uint8_t* readerEntityId,
                                    const uint8_t* writerEntityId,
                                    int32_t bitmapBaseHigh, uint32_t bitmapBaseLow,
                                    uint32_t count)
{
    const uint32_t contentSize = 4 + 4 + 8 + 4 + 4;  // reader(4) + writer(4) + bitmapBase(8) + numBits(4) + count(4)
    const uint32_t totalSize = 4 + contentSize;       // submsg header + content
    if (bufLen < totalSize)
        return 0;

    uint32_t pos = 0;

    // Submessage header
    pBuf[pos++] = SUBMSG_ACKNACK;
    pBuf[pos++] = 0x01;  // flags: E=1 (LE), F=0 (not final — we want to keep receiving)
    writeLE16(pBuf + pos, (uint16_t)contentSize);
    pos += 2;

    // Reader Entity ID (our reader)
    memcpy(pBuf + pos, readerEntityId, 4);
    pos += 4;

    // Writer Entity ID (remote writer we're ACKing)
    memcpy(pBuf + pos, writerEntityId, 4);
    pos += 4;

    // SequenceNumberSet (bitmap base = lastSN+1, numBits=0 means "got everything")
    writeLE32(pBuf + pos, (uint32_t)bitmapBaseHigh);
    pos += 4;
    writeLE32(pBuf + pos, bitmapBaseLow);
    pos += 4;
    writeLE32(pBuf + pos, 0);  // numBits = 0 (empty bitmap = no NACKs)
    pos += 4;

    // Count
    writeLE32(pBuf + pos, count);
    pos += 4;

    return pos;
}
