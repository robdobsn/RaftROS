/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Message - Wire format encoding/decoding
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

// RTPS Protocol ID
static const uint8_t RTPS_PROTOCOL_ID[4] = {'R', 'T', 'P', 'S'};

// RTPS Version (2.2 for ROS 2 compatibility)
static const uint8_t RTPS_VERSION_MAJOR = 2;
static const uint8_t RTPS_VERSION_MINOR = 2;

// RTPS Vendor ID (custom – avoid 0x0000 which some implementations reject)
static const uint8_t RTPS_VENDOR_ID[2] = {0x01, 0x12};

// Submessage IDs (DDSI-RTPS §9.4.5)
enum RTPSSubmessageId : uint8_t
{
    SUBMSG_DATA      = 0x15,
    SUBMSG_HEARTBEAT = 0x07,
    SUBMSG_ACKNACK   = 0x06,
    SUBMSG_INFO_TS   = 0x09,
    SUBMSG_INFO_DST  = 0x0E,
};

class RTPSMessage
{
public:
    // Build RTPS message header (20 bytes)
    // Returns bytes written (20) or 0 on error
    static uint32_t writeHeader(uint8_t* pBuf, uint32_t bufLen, const uint8_t* guidPrefix);

    // Build DATA submessage (DDSI-RTPS §8.3.7.2)
    // Returns bytes written or 0 on error
    static uint32_t writeDataSubmessage(uint8_t* pBuf, uint32_t bufLen,
                                         const uint8_t* readerEntityId,
                                         const uint8_t* writerEntityId,
                                         int32_t seqNumHigh, uint32_t seqNumLow,
                                         const uint8_t* pPayload, uint32_t payloadLen);

    // Build DATA submessage with inline QoS containing a single PID_KEY_HASH.
    // Used for keyed user-data topics (e.g. ros_discovery_info / ParticipantEntitiesInfo)
    // where the reader requires the key hash in inline QoS to bind a sample to its
    // instance. `keyHash16` must point to a 16-byte key (commonly the participant
    // GUID for ParticipantEntitiesInfo). Returns bytes written or 0 on error.
    static uint32_t writeDataSubmessageWithKeyHash(uint8_t* pBuf, uint32_t bufLen,
                                                    const uint8_t* readerEntityId,
                                                    const uint8_t* writerEntityId,
                                                    int32_t seqNumHigh, uint32_t seqNumLow,
                                                    const uint8_t* keyHash16,
                                                    const uint8_t* pPayload, uint32_t payloadLen);

    // Build INFO_TS submessage (12 bytes)
    static uint32_t writeInfoTS(uint8_t* pBuf, uint32_t bufLen,
                                 int32_t seconds, uint32_t fraction);

    // Build INFO_DST submessage (16 bytes)
    static uint32_t writeInfoDST(uint8_t* pBuf, uint32_t bufLen,
                                  const uint8_t* guidPrefix);

    // Build HEARTBEAT submessage (32 bytes)
    static uint32_t writeHeartbeat(uint8_t* pBuf, uint32_t bufLen,
                                    const uint8_t* readerEntityId,
                                    const uint8_t* writerEntityId,
                                    int32_t firstSNHigh, uint32_t firstSNLow,
                                    int32_t lastSNHigh, uint32_t lastSNLow,
                                    uint32_t count);

    // Build ACKNACK submessage (28+4=32 bytes with empty bitmap)
    static uint32_t writeAcknack(uint8_t* pBuf, uint32_t bufLen,
                                  const uint8_t* readerEntityId,
                                  const uint8_t* writerEntityId,
                                  int32_t bitmapBaseHigh, uint32_t bitmapBaseLow,
                                  uint32_t count);

    // Build ACKNACK submessage with an explicit SequenceNumberSet bitmap (DDSI-RTPS §9.4.2.7).
    // `numBits` may be 0 (pure confirmation ACKNACK) or up to 256 per RTPS spec. The bitmap is
    // packed into ceil(numBits/32) little-endian 32-bit words. Bit i of `bitmapWords` corresponds
    // to SN (base + i); a set bit means NACK (missing), clear means ACK.
    // Returns bytes written or 0 on error (buf too small, numBits>256).
    static uint32_t writeAcknackWithBitmap(uint8_t* pBuf, uint32_t bufLen,
                                            const uint8_t* readerEntityId,
                                            const uint8_t* writerEntityId,
                                            int32_t bitmapBaseHigh, uint32_t bitmapBaseLow,
                                            uint32_t numBits,
                                            const uint32_t* bitmapWords,
                                            uint32_t count,
                                            bool finalFlag = false);

    // Parse RTPS message header, returns offset to first submessage or 0 on error
    static uint32_t parseHeader(const uint8_t* pBuf, uint32_t bufLen, uint8_t* guidPrefixOut);

    // Parse submessage header at offset, returns total submessage size or 0 on error
    static uint32_t parseSubmessage(const uint8_t* pBuf, uint32_t bufLen,
                                     RTPSSubmessageId& submsgId, uint8_t& flags,
                                     const uint8_t*& pContent, uint32_t& contentLen);

    // Read LE uint32 (also used externally for HEARTBEAT parsing)
    static uint32_t readLE32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24); }

private:
    // Write LE uint16
    static void writeLE16(uint8_t* p, uint16_t val) { p[0] = val & 0xFF; p[1] = (val >> 8) & 0xFF; }
    // Write LE uint32
    static void writeLE32(uint8_t* p, uint32_t val) {
        p[0] = val & 0xFF; p[1] = (val >> 8) & 0xFF;
        p[2] = (val >> 16) & 0xFF; p[3] = (val >> 24) & 0xFF;
    }
    // Read LE uint16
    static uint16_t readLE16(const uint8_t* p) { return p[0] | (p[1] << 8); }
};
