#pragma once

#include <stdint.h>
#include "RTPSAckNack.h"

enum class RTPSAckNackRunnerAction
{
    RetransmitSedpRosDiscoveryPublication = 0,
    RetransmitSedpChatterPublication,
    RetransmitSedpRosDiscoverySubscription,
    RetransmitRosDiscoveryInfo,
    RetransmitChatterData,
};

struct RTPSAckNackRunnerOptions
{
    bool publicationsIncludesChatterAnnouncement = false;
    bool requirePublicationSeq2GateForRetransmit = false;
};

typedef void (*RTPSAckNackRunnerLogParsedFn)(
    void* userCtx,
    const RTPSAckNackFields& fields,
    RTPSAckNackWriterKind writerKind);

typedef bool (*RTPSAckNackRunnerResolveRemoteFn)(
    void* userCtx,
    const uint8_t* srcGuidPrefix);

typedef void (*RTPSAckNackRunnerUnknownRemoteFn)(void* userCtx);

typedef uint64_t (*RTPSAckNackRunnerGetChatterSeqFn)(void* userCtx);

typedef void (*RTPSAckNackRunnerExecuteActionFn)(
    void* userCtx,
    RTPSAckNackRunnerAction action,
    const RTPSAckNackFields& fields,
    RTPSAckNackWriterKind writerKind);

struct RTPSAckNackRunnerCallbacks
{
    RTPSAckNackRunnerLogParsedFn logParsed = nullptr;
    RTPSAckNackRunnerResolveRemoteFn resolveRemote = nullptr;
    RTPSAckNackRunnerUnknownRemoteFn unknownRemote = nullptr;
    RTPSAckNackRunnerGetChatterSeqFn getChatterSeq = nullptr;
    RTPSAckNackRunnerExecuteActionFn executeAction = nullptr;
};

void RTPSAckNackRunner_run(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    const RTPSAckNackRunnerOptions& options,
    const RTPSAckNackRunnerCallbacks& callbacks,
    void* userCtx);
