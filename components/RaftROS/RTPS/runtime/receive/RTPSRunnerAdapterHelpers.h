#pragma once

#include <stdint.h>
#include <vector>

#include "runtime/discovery/SPDPHandler.h"
#include "runtime/reliability/RTPSAckNackRunner.h"
#include "runtime/receive/RTPSRxSubmessageRunner.h"
#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

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

enum class RTPSAckNackRunnerSendChannel : uint8_t
{
    Metatraffic = 0,
    UserData,
};

typedef uint32_t (*RTPSAckNackRunnerBuildActionFn)(
    void* actionCtx,
    const uint8_t* srcGuidPrefix,
    uint64_t chatterSeq);

typedef void (*RTPSAckNackRunnerLogSendResultFn)(
    void* actionCtx,
    RTPSAckNackRunnerAction action,
    int sentBytes,
    uint32_t msgLen);

struct RTPSAckNackRunnerActionExecSpec
{
    RTPSAckNackRunnerBuildActionFn buildMessage = nullptr;
    RTPSAckNackRunnerSendChannel sendChannel = RTPSAckNackRunnerSendChannel::Metatraffic;
};

struct RTPSAckNackRunnerExecAdapterCtx
{
    RTPSAckNackRunnerAdapterBaseCtx base;
    int metatrafficSock = -1;
    int userDataSock = -1;
    const uint8_t* sendBuf = nullptr;
    uint32_t* heartbeatCount = nullptr;
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackMutationPolicy mutationPolicy;
    struct
    {
        bool dumpRosDiscoveryPayloadHex = false;
    } debugPolicy;
};

struct RTPSAckNackRunnerExecInitConfig
{
    const std::vector<DiscoveredParticipant>* discovered = nullptr;
    int metatrafficSock = -1;
    int userDataSock = -1;
    const uint8_t* sendBuf = nullptr;
    uint32_t* heartbeatCount = nullptr;
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackMutationPolicy mutationPolicy;
    bool dumpRosDiscoveryPayloadHex = false;
};

struct RTPSAckNackRunnerCallbackInitConfig
{
    RTPSAckNackRunnerLogParsedFn logParsed = nullptr;
    RTPSAckNackRunnerUnknownRemoteFn unknownRemote = nullptr;
    RTPSAckNackRunnerGetChatterSeqFn getChatterSeq = nullptr;
    RTPSAckNackRunnerExecuteActionFn executeAction = nullptr;
};

void RTPSRunnerAdapter_initAckExecContext(
    RTPSAckNackRunnerExecAdapterCtx& ctx,
    const RTPSAckNackRunnerExecInitConfig& initConfig);

void RTPSRunnerAdapter_initAckCallbacks(
    RTPSAckNackRunnerCallbacks& callbacks,
    const RTPSAckNackRunnerCallbackInitConfig& initConfig);

void RTPSRunnerAdapter_executeAckAction(
    RTPSAckNackRunnerExecAdapterCtx& adapterCtx,
    RTPSAckNackRunnerAction action,
    const uint8_t* srcGuidPrefix,
    uint64_t chatterSeq,
    const RTPSAckNackRunnerActionExecSpec& sedpRosDiscoveryPublicationSpec,
    const RTPSAckNackRunnerActionExecSpec& sedpChatterPublicationSpec,
    const RTPSAckNackRunnerActionExecSpec& sedpRosDiscoverySubscriptionSpec,
    const RTPSAckNackRunnerActionExecSpec& rosDiscoveryInfoSpec,
    const RTPSAckNackRunnerActionExecSpec& chatterDataSpec,
    void* actionCtx,
    RTPSAckNackRunnerLogSendResultFn logSendResult);

const char* RTPSRunnerAdapter_ackActionLogLabel(RTPSAckNackRunnerAction action);

const char* RTPSRunnerAdapter_ackActionResultLabel(RTPSAckNackRunnerAction action);

bool RTPSRunnerAdapter_shouldDumpAckPayload(
    RTPSAckNackRunnerAction action,
    const RTPSAckNackRunnerExecAdapterCtx& adapterCtx);

void RTPSRunnerAdapter_logHexPayload(const char* label,
                                     const uint8_t* pData,
                                     uint32_t dataLen);