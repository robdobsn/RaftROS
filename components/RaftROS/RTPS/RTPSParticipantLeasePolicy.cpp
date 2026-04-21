#include "RTPSParticipantLeasePolicy.h"

#include "runtime/discovery/RTPSDiscoveryRuntime.h"

uint32_t RTPSParticipantLeasePolicy_leaseTimeoutMs(uint32_t leaseDurationSec)
{
    return RaftROS::RTPS::Runtime::DiscoveryRuntime::leaseTimeoutMs(leaseDurationSec);
}

bool RTPSParticipantLeasePolicy_isExpired(uint32_t nowMs,
                                          uint64_t discoveredTimeMs,
                                          uint32_t leaseDurationSec)
{
    return RaftROS::RTPS::Runtime::DiscoveryRuntime::isLeaseExpired(
        nowMs, discoveredTimeMs, leaseDurationSec);
}
