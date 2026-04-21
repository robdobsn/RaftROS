#include "RTPSParticipantLifecycle.h"

#include "runtime/discovery/RTPSDiscoveryRuntime.h"

bool RTPSParticipantLifecycle_shouldActivate(bool isCurrentlyActive, uint32_t discoveredCount)
{
    return RaftROS::RTPS::Runtime::DiscoveryRuntime::applyParticipantSetPolicy(
               isCurrentlyActive,
               discoveredCount,
               false,
               false)
        .shouldBeActive;
}

void RTPSParticipantLifecycle_onActivated(uint32_t& lastWriterHeartbeatMs)
{
    RaftROS::RTPS::Runtime::DiscoveryRuntime::onActivated(lastWriterHeartbeatMs);
}
