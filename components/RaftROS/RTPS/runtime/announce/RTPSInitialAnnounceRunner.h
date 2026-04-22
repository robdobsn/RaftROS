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

// Execute a single step of the announce sequence.  Returns true if the step index was
// within range (regardless of whether the step was skipped by pre/post-build evaluation),
// false if stepIdx >= sequence.numSteps.  Updates runCtx.heartbeatCount and
// runCtx.previousPayloadLen as appropriate so consecutive calls with incrementing stepIdx
// reproduce the behaviour of RTPSInitialAnnounceRunner_run.  Used by wrappers that want
// to spread the initial-announce burst across multiple scheduler ticks.
bool RTPSInitialAnnounceRunner_runStep(
    const RTPSInitialAnnounceSequence& sequence,
    uint8_t stepIdx,
    RTPSInitialAnnounceRunnerContext& runCtx,
    const RTPSInitialAnnounceRunnerCallbacks& callbacks,
    void* userCtx);