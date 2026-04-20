#pragma once

#include <stdint.h>

// Decide whether the runtime should transition to active state.
// Activation occurs only when currently inactive and there is at least one discovered participant.
bool RTPSParticipantLifecycle_shouldActivate(bool isCurrentlyActive, uint32_t discoveredCount);

// Apply the standard side effect when transitioning to active.
// Current policy: force immediate writer heartbeat on next scheduler check.
void RTPSParticipantLifecycle_onActivated(uint32_t& lastWriterHeartbeatMs);
