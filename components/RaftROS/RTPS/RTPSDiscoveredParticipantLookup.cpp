#include "RTPSDiscoveredParticipantLookup.h"

#include "runtime/discovery/RTPSDiscoveryRuntime.h"

const DiscoveredParticipant* RTPSDiscoveredParticipantLookup_findByGuidPrefix(
    const std::vector<DiscoveredParticipant>& discovered,
    const uint8_t* guidPrefix)
{
    return RaftROS::RTPS::Runtime::DiscoveryRuntime::findByGuidPrefix(
        discovered, guidPrefix);
}

bool RTPSDiscoveredParticipantLookup_assignMetatrafficDestByIp(
    const std::vector<DiscoveredParticipant>& discovered,
    const struct sockaddr_in& fromAddr,
    struct sockaddr_in& outDest)
{
    return RaftROS::RTPS::Runtime::DiscoveryRuntime::assignMetatrafficDestByIp(
        discovered, fromAddr, outDest);
}
