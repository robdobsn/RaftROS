#include "RTPSParticipantLeasePolicy.h"

static constexpr uint32_t DEFAULT_LEASE_TIMEOUT_MS = 240000;

uint32_t RTPSParticipantLeasePolicy_leaseTimeoutMs(uint32_t leaseDurationSec)
{
    uint32_t leaseMs = leaseDurationSec * 1000 * 2;
    if (leaseMs == 0)
        leaseMs = DEFAULT_LEASE_TIMEOUT_MS;
    return leaseMs;
}

bool RTPSParticipantLeasePolicy_isExpired(uint32_t nowMs,
                                          uint64_t discoveredTimeMs,
                                          uint32_t leaseDurationSec)
{
    return (nowMs - discoveredTimeMs) > RTPSParticipantLeasePolicy_leaseTimeoutMs(leaseDurationSec);
}
