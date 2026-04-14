/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SEDP Handler - Simple Endpoint Discovery Protocol
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SEDPHandler.h"
#include "RTPSParticipant.h"

SEDPHandler::SEDPHandler()
{
}

SEDPHandler::~SEDPHandler()
{
}

void SEDPHandler::init(RTPSParticipant& participant)
{
    _pParticipant = &participant;
}

void SEDPHandler::service()
{
    // TODO: Exchange endpoint information with discovered participants
    // TODO: Match local writers with remote readers (and vice versa)
}

bool SEDPHandler::announceWriter(const char* topicName, const char* typeName)
{
    // TODO: Build and send PublicationBuiltinTopicData
    return false;
}

bool SEDPHandler::announceReader(const char* topicName, const char* typeName)
{
    // TODO: Build and send SubscriptionBuiltinTopicData
    return false;
}

bool SEDPHandler::removeEndpoint(const char* topicName)
{
    // TODO: Send dispose for the endpoint
    return false;
}

void SEDPHandler::processReceived(const uint8_t* pData, uint32_t dataLen)
{
    // TODO: Parse endpoint announcements from remote participants
    // TODO: Update matched endpoint state
}
