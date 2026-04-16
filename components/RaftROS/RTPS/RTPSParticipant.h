/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Participant - GUID generation, port calculation, locator helpers
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include "RTPSTypes.h"

class RTPSParticipant
{
public:
    RTPSParticipant();
    ~RTPSParticipant();

    // Initialize with domain ID, node name, and optional GUID prefix
    // If guidPrefix is nullptr, all zeros (caller should set via setGuidPrefixFromMAC)
    void init(uint32_t domainId, const char* nodeName,
              uint32_t participantId = 0, const uint8_t* guidPrefix = nullptr);

    // Set GUID prefix from 6-byte MAC address
    void setGuidPrefixFromMAC(const uint8_t* mac6);

    // Accessors
    const uint8_t* getGuidPrefix() const { return _guidPrefix; }
    const uint8_t* getParticipantGuid() const { return _participantGuid; }
    uint32_t getDomainId() const { return _domainId; }
    uint32_t getParticipantId() const { return _participantId; }
    const char* getNodeName() const { return _nodeName.c_str(); }

    // Port helpers
    uint16_t getSPDPMulticastPort() const { return rtpsDiscoveryMulticastPort(_domainId); }
    uint16_t getMetatrafficUnicastPort() const { return rtpsDiscoveryUnicastPort(_domainId, _participantId); }
    uint16_t getUserUnicastPort() const { return rtpsUserUnicastPort(_domainId, _participantId); }

    // Build a locator struct (24 bytes) for IPv4
    static void buildLocatorUDPv4(uint8_t* pOut, uint32_t ipAddrNetOrder, uint16_t port);

private:
    uint32_t _domainId = 0;
    uint32_t _participantId = 0;
    std::string _nodeName;
    uint8_t _guidPrefix[12] = {};
    uint8_t _participantGuid[16] = {};

    void updateParticipantGuid();
};
