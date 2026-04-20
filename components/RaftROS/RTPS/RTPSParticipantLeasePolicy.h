#pragma once

#include <cstdint>

// Compute lease timeout in milliseconds with grace applied.
// Current policy: 2x lease duration, fallback to 240000 ms when lease is unknown.
uint32_t RTPSParticipantLeasePolicy_leaseTimeoutMs(uint32_t leaseDurationSec);

// Determine whether a discovered participant lease has expired.
bool RTPSParticipantLeasePolicy_isExpired(uint32_t nowMs,
                                          uint64_t discoveredTimeMs,
                                          uint32_t leaseDurationSec);
