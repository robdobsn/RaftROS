#pragma once

#include <stdint.h>
#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

using RTPSAckNackFields =
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackFields;
using RTPSAckNackWriterKind =
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackWriterKind;
using RTPSAckNackRunnerAction =
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction;

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
    const RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionOptions& options,
    const RTPSAckNackRunnerCallbacks& callbacks,
    void* userCtx);