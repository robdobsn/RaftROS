#include "runtime/discovery/RTPSDiscoveryRuntime.h"

#include <arpa/inet.h>
#include <cstring>

namespace RaftROS::RTPS::Runtime::DiscoveryRuntime
{

static constexpr uint32_t DEFAULT_LEASE_TIMEOUT_MS = 240000;

MergeResult mergeParticipant(std::vector<DiscoveredParticipant>& discovered,
                             DiscoveredParticipant& remote,
                             uint64_t nowMs,
                             std::size_t maxCount)
{
    for (auto& dp : discovered)
    {
        if (std::memcmp(dp.guidPrefix, remote.guidPrefix, 12) == 0)
        {
            dp.discoveredTimeMs = nowMs;
            return MergeResult::RefreshedExisting;
        }
    }

    remote.discoveredTimeMs = nowMs;
    if (discovered.size() < maxCount)
    {
        discovered.push_back(remote);
        return MergeResult::AddedNew;
    }

    return MergeResult::CapacityFull;
}

RTPSParticipantSetPolicyResult applyParticipantSetPolicy(bool isCurrentlyActive,
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
        result.triggerImmediateWriterHeartbeat = true;

    return result;
}

uint32_t leaseTimeoutMs(uint32_t leaseDurationSec)
{
    uint32_t leaseMs = leaseDurationSec * 1000 * 2;
    if (leaseMs == 0)
        leaseMs = DEFAULT_LEASE_TIMEOUT_MS;
    return leaseMs;
}

bool isLeaseExpired(uint32_t nowMs,
                    uint64_t discoveredTimeMs,
                    uint32_t leaseDurationSec)
{
    return (nowMs - discoveredTimeMs) > leaseTimeoutMs(leaseDurationSec);
}

void onActivated(uint32_t& lastWriterHeartbeatMs)
{
    lastWriterHeartbeatMs = 0;
}

const DiscoveredParticipant* findByGuidPrefix(
    const std::vector<DiscoveredParticipant>& discovered,
    const uint8_t* guidPrefix)
{
    if (!guidPrefix)
        return nullptr;

    for (const auto& dp : discovered)
    {
        if (std::memcmp(dp.guidPrefix, guidPrefix, 12) == 0)
            return &dp;
    }
    return nullptr;
}

bool assignMetatrafficDestByIp(const std::vector<DiscoveredParticipant>& discovered,
                               const struct sockaddr_in& fromAddr,
                               struct sockaddr_in& outDest)
{
    outDest = fromAddr;
    for (const auto& dp : discovered)
    {
        if (dp.ipAddr == fromAddr.sin_addr.s_addr)
        {
            outDest.sin_port = htons(dp.metatrafficPort);
            return true;
        }
    }
    return false;
}

}
