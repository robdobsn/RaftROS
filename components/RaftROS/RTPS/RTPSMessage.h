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
#include <vector>

// RTPS Protocol ID
static const uint8_t RTPS_PROTOCOL_ID[4] = {'R', 'T', 'P', 'S'};

// RTPS Version (2.2 for ROS 2 compatibility)
static const uint8_t RTPS_VERSION_MAJOR = 2;
static const uint8_t RTPS_VERSION_MINOR = 2;

// RTPS Vendor ID (OMG unregistered)
static const uint8_t RTPS_VENDOR_ID[2] = {0x00, 0x00};

// Submessage IDs (DDSI-RTPS §9.4.5)
enum RTPSSubmessageId : uint8_t
{
    SUBMSG_DATA = 0x15,
    SUBMSG_HEARTBEAT = 0x07,
    SUBMSG_ACKNACK = 0x06,
    SUBMSG_INFO_TS = 0x09,
    SUBMSG_INFO_DST = 0x0e,
};

class RTPSMessage
{
public:
    // Build RTPS message header (20 bytes)
    static uint32_t writeHeader(uint8_t* pBuf, uint32_t bufLen, const uint8_t* guidPrefix);

    // Build DATA submessage
    static uint32_t writeDataSubmessage(uint8_t* pBuf, uint32_t bufLen,
                                         const uint8_t* writerGuid,
                                         uint64_t sequenceNumber,
                                         const uint8_t* pPayload, uint32_t payloadLen);

    // Parse RTPS message header, returns offset to first submessage or 0 on error
    static uint32_t parseHeader(const uint8_t* pBuf, uint32_t bufLen, uint8_t* guidPrefixOut);

    // Parse next submessage, returns offset to next submessage or 0 if no more
    static uint32_t parseSubmessage(const uint8_t* pBuf, uint32_t bufLen,
                                     RTPSSubmessageId& submsgId,
                                     const uint8_t*& pPayload, uint32_t& payloadLen);
};
