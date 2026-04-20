#pragma once

#include <stdint.h>
#include <netinet/in.h>
#include <vector>

#include "SPDPHandler.h"

const DiscoveredParticipant* RTPSDiscoveredParticipantLookup_findByGuidPrefix(
    const std::vector<DiscoveredParticipant>& discovered,
    const uint8_t* guidPrefix);

bool RTPSDiscoveredParticipantLookup_assignMetatrafficDestByIp(
    const std::vector<DiscoveredParticipant>& discovered,
    const struct sockaddr_in& fromAddr,
    struct sockaddr_in& outDest);
