#include "RTPSParticipantSetPolicy.h"

RTPSParticipantSetPolicyResult RTPSParticipantSetPolicy_apply(bool isCurrentlyActive,
                                                              uint32_t discoveredCount,
                                                              bool participantAdded,
                                                              bool forceImmediateOnAdd)
{
    RTPSParticipantSetPolicyResult result;
    result.shouldBeActive = isCurrentlyActive;

    if (!isCurrentlyActive && discoveredCount > 0)
    {
        result.shouldBeActive = true;
        result.triggerImmediateWriterHeartbeat = true;
        return result;
    }

    if (participantAdded && forceImmediateOnAdd)
    {
        result.triggerImmediateWriterHeartbeat = true;
    }

    return result;
}
