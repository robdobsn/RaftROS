#pragma once

#include <cstddef>
#include <cstdint>
#ifdef ESP_PLATFORM
#include "lwip/sockets.h"
#include "lwip/inet.h"
#else
#include <netinet/in.h>
#endif
#include <vector>

#include "runtime/discovery/SPDPHandler.h"

struct RTPSParticipantSetPolicyResult
{
    bool shouldBeActive = false;
    bool triggerImmediateWriterHeartbeat = false;
};

namespace RaftRuntime::RTPS::Runtime::DiscoveryRuntime
{

enum class MergeResult : uint8_t
{
    RefreshedExisting = 0,
    AddedNew,
    CapacityFull,
};

MergeResult mergeParticipant(std::vector<DiscoveredParticipant>& discovered,
                             DiscoveredParticipant& remote,
                             uint64_t nowMs,
                             std::size_t maxCount);

RTPSParticipantSetPolicyResult applyParticipantSetPolicy(bool isCurrentlyActive,
                                                         uint32_t discoveredCount,
                                                         bool participantAdded,
                                                         bool forceImmediateOnAdd);

uint32_t leaseTimeoutMs(uint32_t leaseDurationSec);

bool isLeaseExpired(uint32_t nowMs,
                    uint64_t discoveredTimeMs,
                    uint32_t leaseDurationSec);

void onActivated(uint32_t& lastWriterHeartbeatMs);

const DiscoveredParticipant* findByGuidPrefix(
    const std::vector<DiscoveredParticipant>& discovered,
    const uint8_t* guidPrefix);

bool assignMetatrafficDestByIp(const std::vector<DiscoveredParticipant>& discovered,
                               const struct sockaddr_in& fromAddr,
                               struct sockaddr_in& outDest);

}
