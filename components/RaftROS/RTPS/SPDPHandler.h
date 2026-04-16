/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SPDP Handler - Simple Participant Discovery Protocol message building
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include "RTPSTypes.h"

class RTPSParticipant;

/// Discovered remote participant
struct DiscoveredParticipant
{
    uint8_t guidPrefix[12] = {};
    uint32_t ipAddr = 0;           // Network byte order
    uint16_t metatrafficPort = 0;
    uint16_t userDataPort = 0;
    uint32_t leaseDurationSec = 0;
    uint64_t discoveredTimeMs = 0;
    bool valid = false;
};

class SPDPHandler
{
public:
    SPDPHandler();
    ~SPDPHandler();

    /// Build a complete SPDP announcement RTPS message
    /// Returns bytes written, or 0 on error
    uint32_t buildAnnouncementMessage(
        uint8_t* pBuf, uint32_t bufLen,
        const RTPSParticipant& participant,
        uint32_t ipAddrNetOrder,
        uint32_t leaseDurationSec,
        uint64_t sequenceNumber);

    /// Parse a received SPDP message and extract participant info
    /// Returns true if valid
    bool parseAnnouncementMessage(const uint8_t* pBuf, uint32_t bufLen,
                                   DiscoveredParticipant& outParticipant);

    /// Build the ros_discovery_info CDR payload (ParticipantEntitiesInfo)
    /// Returns bytes written
    static uint32_t buildRosDiscoveryInfoPayload(
        uint8_t* pBuf, uint32_t bufLen,
        const uint8_t* participantGuid,
        const char* nodeName,
        const char* nodeNamespace);

private:
    /// Write a ParameterList entry header (PID + length)
    static uint32_t writeParamHeader(uint8_t* pBuf, uint16_t pid, uint16_t length);

    /// Write a Locator parameter (PID + 24-byte locator)
    static uint32_t writeLocatorParam(uint8_t* pBuf, uint16_t pid,
                                       uint32_t ipAddrNetOrder, uint16_t port);
};
