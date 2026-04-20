#include "RTPSDiscoveredParticipantLookup.h"

#include <string.h>
#include <arpa/inet.h>

const DiscoveredParticipant* RTPSDiscoveredParticipantLookup_findByGuidPrefix(
    const std::vector<DiscoveredParticipant>& discovered,
    const uint8_t* guidPrefix)
{
    if (!guidPrefix)
        return nullptr;

    for (const auto& dp : discovered)
    {
        if (memcmp(dp.guidPrefix, guidPrefix, 12) == 0)
            return &dp;
    }
    return nullptr;
}

bool RTPSDiscoveredParticipantLookup_assignMetatrafficDestByIp(
    const std::vector<DiscoveredParticipant>& discovered,
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
