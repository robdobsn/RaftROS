/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SEDP Handler - Simple Endpoint Discovery Protocol
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>

class RTPSParticipant;

class SEDPHandler
{
public:
    SEDPHandler();
    ~SEDPHandler();

    // Initialize with participant reference
    void init(RTPSParticipant& participant);

    // Service SEDP (call from loop)
    void service();

    // Announce a DataWriter endpoint (publisher)
    bool announceWriter(const char* topicName, const char* typeName);

    // Announce a DataReader endpoint (subscriber)
    bool announceReader(const char* topicName, const char* typeName);

    // Remove an announced endpoint
    bool removeEndpoint(const char* topicName);

    // Process received SEDP message
    void processReceived(const uint8_t* pData, uint32_t dataLen);

private:
    RTPSParticipant* _pParticipant = nullptr;
};
