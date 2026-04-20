#pragma once

#include <cstdint>

struct RTPSParticipantSetPolicyResult
{
    bool shouldBeActive = false;
    bool triggerImmediateWriterHeartbeat = false;
};

// Apply participant-set policy for runtime state updates.
// - isCurrentlyActive: current runtime active state
// - discoveredCount: current discovered participant count
// - participantAdded: true when a new participant was added in this cycle
// - forceImmediateOnAdd: policy switch for triggering immediate writer heartbeat on add
RTPSParticipantSetPolicyResult RTPSParticipantSetPolicy_apply(bool isCurrentlyActive,
                                                              uint32_t discoveredCount,
                                                              bool participantAdded,
                                                              bool forceImmediateOnAdd);
