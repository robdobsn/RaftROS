/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SPDP Handler - Simple Participant Discovery Protocol
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>

class RTPSParticipant;

class SPDPHandler
{
public:
    SPDPHandler();
    ~SPDPHandler();

    // Initialize with participant reference
    void init(RTPSParticipant& participant);

    // Service SPDP (call from loop)
    void service();

    // Send participant announcement
    bool sendAnnouncement();

    // Process received SPDP message
    void processReceived(const uint8_t* pData, uint32_t dataLen);

private:
    RTPSParticipant* _pParticipant = nullptr;
    uint32_t _lastAnnouncementMs = 0;
    uint32_t _announcementIntervalMs = 30000; // Default 30s lease/3
};
