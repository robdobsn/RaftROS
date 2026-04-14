/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Participant
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include "RaftArduino.h"

class RTPSParticipant
{
public:
    RTPSParticipant();
    ~RTPSParticipant();

    // Initialize participant with domain ID and node name
    void init(uint32_t domainId, const String& nodeName);

    // Get participant GUID (16 bytes: 12-byte prefix + 4-byte entityId)
    const uint8_t* getGUID() const { return _guid; }

    // Get domain ID
    uint32_t getDomainId() const { return _domainId; }

    // Get node name
    const String& getNodeName() const { return _nodeName; }

private:
    uint32_t _domainId = 0;
    String _nodeName;
    uint8_t _guid[16] = {};

    // Generate GUID from ESP32 MAC address
    void generateGUID();
};
