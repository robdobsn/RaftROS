/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Participant - GUID generation, port calculation, locator helpers
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "runtime/core/RTPSParticipant.h"

RTPSParticipant::RTPSParticipant()
{
}

RTPSParticipant::~RTPSParticipant()
{
}

void RTPSParticipant::init(uint32_t domainId, const char* nodeName,
                            uint32_t participantId, const uint8_t* guidPrefix)
{
    _domainId = domainId;
    _nodeName = nodeName ? nodeName : "raft_esp32";
    _participantId = participantId;

    if (guidPrefix)
        memcpy(_guidPrefix, guidPrefix, 12);
    else
        memset(_guidPrefix, 0, 12);

    updateParticipantGuid();
}

void RTPSParticipant::setGuidPrefixFromMAC(const uint8_t* mac6)
{
    // GUID prefix layout (12 bytes):
    // [0-1]  Vendor ID (unregistered)
    // [2-7]  MAC address (6 bytes, unique per chip)
    // [8-11] Participant discriminator
    _guidPrefix[0] = 0x01;
    _guidPrefix[1] = 0x0F;  // Vendor: unregistered, RaftROS marker
    memcpy(_guidPrefix + 2, mac6, 6);
    _guidPrefix[8] = (_participantId >> 24) & 0xFF;
    _guidPrefix[9] = (_participantId >> 16) & 0xFF;
    _guidPrefix[10] = (_participantId >> 8) & 0xFF;
    _guidPrefix[11] = _participantId & 0xFF;
    updateParticipantGuid();
}

void RTPSParticipant::updateParticipantGuid()
{
    memcpy(_participantGuid, _guidPrefix, 12);
    memcpy(_participantGuid + 12, ENTITYID_PARTICIPANT, 4);
}

void RTPSParticipant::buildLocatorUDPv4(uint8_t* pOut, uint32_t ipAddrNetOrder, uint16_t port)
{
    // Locator_t: kind(4) + port(4) + address(16) = 24 bytes (all LE in CDR)
    memset(pOut, 0, 24);
    // Kind: UDPv4 = 1 (LE)
    pOut[0] = LOCATOR_KIND_UDPv4; pOut[1] = 0; pOut[2] = 0; pOut[3] = 0;
    // Port (LE uint32)
    pOut[4] = port & 0xFF; pOut[5] = (port >> 8) & 0xFF; pOut[6] = 0; pOut[7] = 0;
    // Address: IPv4 mapped (12 zero bytes + 4 IPv4 octets in network byte order)
    memcpy(pOut + 20, &ipAddrNetOrder, 4);
}
