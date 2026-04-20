#include "RTPSDiscoveryPolicy.h"

#include <cstring>

bool RTPSDiscoveryPolicy_refreshLeaseIfKnown(std::vector<DiscoveredParticipant>& discovered,
                                             const uint8_t* guidPrefix,
                                             uint64_t nowMs)
{
    for (auto& dp : discovered)
    {
        if (std::memcmp(dp.guidPrefix, guidPrefix, 12) == 0)
        {
            dp.discoveredTimeMs = nowMs;
            return true;
        }
    }
    return false;
}

bool RTPSDiscoveryPolicy_canAdd(std::size_t currentCount, std::size_t maxCount)
{
    return currentCount < maxCount;
}

void RTPSDiscoveryPolicy_prepareForAdd(DiscoveredParticipant& remote, uint64_t nowMs)
{
    remote.discoveredTimeMs = nowMs;
}

RTPSDiscoveryMergeResult RTPSDiscoveryPolicy_mergeParticipant(std::vector<DiscoveredParticipant>& discovered,
                                                              DiscoveredParticipant& remote,
                                                              uint64_t nowMs,
                                                              std::size_t maxCount)
{
    if (RTPSDiscoveryPolicy_refreshLeaseIfKnown(discovered, remote.guidPrefix, nowMs))
        return RTPSDiscoveryMergeResult::RefreshedExisting;

    RTPSDiscoveryPolicy_prepareForAdd(remote, nowMs);
    if (RTPSDiscoveryPolicy_canAdd(discovered.size(), maxCount))
    {
        discovered.push_back(remote);
        return RTPSDiscoveryMergeResult::AddedNew;
    }

    return RTPSDiscoveryMergeResult::CapacityFull;
}
