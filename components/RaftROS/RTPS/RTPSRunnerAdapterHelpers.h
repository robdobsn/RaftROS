#pragma once

#include <stdint.h>
#include <vector>

#include "SPDPHandler.h"
#include "RTPSRxSubmessageRunner.h"

enum class RTPSRxAdapterReaderPolicy : uint8_t
{
    BuiltinEndpointMap = 0,
    UseHeartbeatReader,
};

enum class RTPSRxAdapterAckDestPolicy : uint8_t
{
    ReplyToSender = 0,
    RouteToDiscoveredMetatraffic,
};

struct RTPSRxRunnerAdapterBaseCtx
{
    const uint8_t* localGuidPrefix = nullptr;
    const std::vector<DiscoveredParticipant>* discovered = nullptr;
    int ackSendSock = -1;
    RTPSRxAdapterReaderPolicy readerPolicy = RTPSRxAdapterReaderPolicy::BuiltinEndpointMap;
    RTPSRxAdapterAckDestPolicy ackDestPolicy = RTPSRxAdapterAckDestPolicy::ReplyToSender;
};

void RTPSRunnerAdapter_applyRxBaseCallbacks(RTPSRxSubmessageRunnerCallbacks& callbacks);

struct RTPSAckNackRunnerAdapterBaseCtx
{
    const std::vector<DiscoveredParticipant>* discovered = nullptr;
    const DiscoveredParticipant* remote = nullptr;
};

bool RTPSRunnerAdapter_ackResolveRemote(void* userCtx, const uint8_t* srcGuidPrefix);
