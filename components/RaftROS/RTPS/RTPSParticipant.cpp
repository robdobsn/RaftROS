/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Participant
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RTPSParticipant.h"

RTPSParticipant::RTPSParticipant()
{
}

RTPSParticipant::~RTPSParticipant()
{
}

void RTPSParticipant::init(uint32_t domainId, const String& nodeName)
{
    _domainId = domainId;
    _nodeName = nodeName;
    generateGUID();
}

void RTPSParticipant::generateGUID()
{
    // TODO: Generate GUID using ESP32 MAC address as prefix
    // GUID format (DDSI-RTPS §8.2.4):
    //   Bytes 0-11: GuidPrefix (unique per participant)
    //   Bytes 12-15: EntityId (0x000001c1 for participant)
    memset(_guid, 0, sizeof(_guid));
}
