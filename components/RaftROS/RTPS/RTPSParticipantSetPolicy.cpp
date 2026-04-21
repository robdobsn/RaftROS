#include "RTPSParticipantSetPolicy.h"

#include "runtime/discovery/RTPSDiscoveryRuntime.h"

RTPSParticipantSetPolicyResult RTPSParticipantSetPolicy_apply(bool isCurrentlyActive,
                                                              uint32_t discoveredCount,
                                                              bool participantAdded,
                                                              bool forceImmediateOnAdd)
{
    return RaftROS::RTPS::Runtime::DiscoveryRuntime::applyParticipantSetPolicy(
        isCurrentlyActive,
        discoveredCount,
        participantAdded,
        forceImmediateOnAdd);
}
