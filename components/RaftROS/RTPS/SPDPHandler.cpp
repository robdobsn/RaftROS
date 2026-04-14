/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SPDP Handler - Simple Participant Discovery Protocol
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SPDPHandler.h"
#include "RTPSParticipant.h"

SPDPHandler::SPDPHandler()
{
}

SPDPHandler::~SPDPHandler()
{
}

void SPDPHandler::init(RTPSParticipant& participant)
{
    _pParticipant = &participant;
}

void SPDPHandler::service()
{
    // TODO: Periodic SPDP announcements
    // TODO: Receive and parse remote participant announcements
    // TODO: Maintain participant table with lease expiry
}

bool SPDPHandler::sendAnnouncement()
{
    // TODO: Build SPDPdiscoveredParticipantData message
    // TODO: Send to well-known multicast group 239.255.0.1
    // TODO: Port = 7400 + 250 * domainId + 0 (for SPDP)
    return false;
}

void SPDPHandler::processReceived(const uint8_t* pData, uint32_t dataLen)
{
    // TODO: Parse RTPS message header
    // TODO: Extract participant GUID prefix
    // TODO: Extract unicast/multicast locators
    // TODO: Update participant table
}
