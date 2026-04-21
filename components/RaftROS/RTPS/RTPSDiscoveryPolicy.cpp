#include "RTPSDiscoveryPolicy.h"

#include <cstring>

#include "runtime/discovery/RTPSDiscoveryRuntime.h"

bool RTPSDiscoveryPolicy_refreshLeaseIfKnown(std::vector<DiscoveredParticipant>& discovered,
                                             const uint8_t* guidPrefix,
                                             uint64_t nowMs)
{
    if (!guidPrefix)
        return false;

    for (auto& dp : discovered)
    {
        if (memcmp(dp.guidPrefix, guidPrefix, 12) == 0)
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
    const RaftROS::RTPS::Runtime::DiscoveryRuntime::MergeResult result =
        RaftROS::RTPS::Runtime::DiscoveryRuntime::mergeParticipant(
            discovered, remote, nowMs, maxCount);
    switch (result)
    {
        case RaftROS::RTPS::Runtime::DiscoveryRuntime::MergeResult::RefreshedExisting:
            return RTPSDiscoveryMergeResult::RefreshedExisting;
        case RaftROS::RTPS::Runtime::DiscoveryRuntime::MergeResult::AddedNew:
            return RTPSDiscoveryMergeResult::AddedNew;
        default:
            return RTPSDiscoveryMergeResult::CapacityFull;
    }
}
