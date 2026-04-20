#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "SPDPHandler.h"

// If participant already exists by guidPrefix, refresh discoveredTimeMs and return true.
bool RTPSDiscoveryPolicy_refreshLeaseIfKnown(std::vector<DiscoveredParticipant>& discovered,
                                             const uint8_t* guidPrefix,
                                             uint64_t nowMs);

// Check if a new participant can be added under max capacity.
bool RTPSDiscoveryPolicy_canAdd(std::size_t currentCount, std::size_t maxCount);

// Initialize add-time fields for a newly discovered participant.
void RTPSDiscoveryPolicy_prepareForAdd(DiscoveredParticipant& remote, uint64_t nowMs);

enum class RTPSDiscoveryMergeResult
{
    RefreshedExisting = 0,
    AddedNew,
    CapacityFull,
};

// Merge a discovered participant into the set:
// - refresh lease when known
// - add when new and capacity allows
// - report capacity full when new but not added
RTPSDiscoveryMergeResult RTPSDiscoveryPolicy_mergeParticipant(std::vector<DiscoveredParticipant>& discovered,
                                                              DiscoveredParticipant& remote,
                                                              uint64_t nowMs,
                                                              std::size_t maxCount);
