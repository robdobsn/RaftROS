#pragma once

#include <stdint.h>
#include "runtime/announce/RTPSInitialAnnouncePlan.h"

struct RTPSInitialAnnounceRunnerContext
{
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::EspStyle;
    RTPSInitialAnnounceCounterState seqCounters;
    uint32_t heartbeatCount = 0;
    uint32_t previousPayloadLen = 0;
};

typedef uint32_t (*RTPSInitialAnnounceBuildPayloadFn)(
    void* userCtx,
    const RTPSInitialAnnounceStep& step,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount,
    uint32_t previousPayloadLen);

typedef int (*RTPSInitialAnnounceSendPayloadFn)(
    void* userCtx,
    RTPSInitialAnnounceAction action,
    uint32_t payloadLen);

typedef void (*RTPSInitialAnnounceDebugPayloadFn)(
    void* userCtx,
    const RTPSInitialAnnounceStep& step,
    uint32_t payloadLen);

typedef void (*RTPSInitialAnnounceLogResultFn)(
    void* userCtx,
    const RTPSInitialAnnounceStep& step,
    const RTPSInitialAnnounceLogSpec& logSpec,
    const RTPSInitialAnnounceSendResultSpec& sendResult,
    int sentBytes,
    uint32_t payloadLen);

struct RTPSInitialAnnounceRunnerCallbacks
{
    RTPSInitialAnnounceBuildPayloadFn buildPayload = nullptr;
    RTPSInitialAnnounceSendPayloadFn sendPayload = nullptr;
    RTPSInitialAnnounceDebugPayloadFn debugPayload = nullptr;
    RTPSInitialAnnounceLogResultFn logResult = nullptr;
};

void RTPSInitialAnnounceRunner_run(
    const RTPSInitialAnnounceSequence& sequence,
    RTPSInitialAnnounceRunnerContext& runCtx,
    const RTPSInitialAnnounceRunnerCallbacks& callbacks,
    void* userCtx);