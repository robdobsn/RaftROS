#include "RTPSParticipantLifecycle.h"

bool RTPSParticipantLifecycle_shouldActivate(bool isCurrentlyActive, uint32_t discoveredCount)
{
    return !isCurrentlyActive && (discoveredCount > 0);
}

void RTPSParticipantLifecycle_onActivated(uint32_t& lastWriterHeartbeatMs)
{
    lastWriterHeartbeatMs = 0;
}
