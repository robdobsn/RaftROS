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

//
// Shared default ACKNACK action-execution context and helpers
//
// Wrappers set up a `RTPSAckActionStandardCtx` (via
// `RTPSRunnerAdapter_initStandardAckActionCtx`) and pass it as the user ctx to
// `RTPSAckNackRunner_run`. The default `executeAction` + pre-wired action
// specs then handle all five retransmit paths using shared code, with the
// wrapper only providing topic-payload builders and flavor/heartbeat policy.
//

class SEDPHandler;
class RTPSParticipant;

// Produce a user-data topic payload (e.g. chatter or ros_discovery_info) into
// the supplied buffer. Returns bytes written, or 0 on error.
typedef uint32_t (*RTPSAckActionBuildPayloadFn)(void* payloadCtx,
                                                uint8_t* buf,
                                                uint32_t bufLen);

// Whether to pass the live heartbeatCount (Linux) or a zero (ESP) when
// calling SEDPHandler::buildPublicationMessage / buildSubscriptionMessage
// during ACKNACK-driven SEDP retransmit. Existing wrappers differ on this.
enum class RTPSAckActionSedpBuildHeartbeatPolicy : uint8_t
{
    PassZero = 0,
    PassLiveHeartbeat,
};

struct RTPSAckActionStandardCtx
{
    RTPSAckNackRunnerExecAdapterCtx exec;
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionSedpSequenceContext
        sedpSequenceContext;
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionUserDataSequenceContext
        userDataSequenceContext;
    RTPSAckNackRunnerActionExecSpec sedpRosDiscoveryPublicationSpec;
    RTPSAckNackRunnerActionExecSpec sedpChatterPublicationSpec;
    RTPSAckNackRunnerActionExecSpec sedpRosDiscoverySubscriptionSpec;
    RTPSAckNackRunnerActionExecSpec rosDiscoveryInfoSpec;
    RTPSAckNackRunnerActionExecSpec chatterDataSpec;
    const uint8_t* srcGuidPrefix = nullptr;

    // Build environment (wrapper-owned, but policy/dispatch is shared)
    SEDPHandler* sedpHandler = nullptr;
    const RTPSParticipant* participant = nullptr;
    uint8_t* sendBufMutable = nullptr;
    uint32_t sendBufLen = 0;
    uint32_t myIpAddr = 0;
    RTPSAckActionSedpBuildHeartbeatPolicy sedpBuildHeartbeatPolicy =
        RTPSAckActionSedpBuildHeartbeatPolicy::PassZero;
    RTPSAckActionBuildPayloadFn buildRosDiscInfoPayload = nullptr;
    RTPSAckActionBuildPayloadFn buildChatterPayload = nullptr;
    void* payloadCtx = nullptr;
};

struct RTPSAckActionStandardInitConfig
{
    // exec init
    const std::vector<DiscoveredParticipant>* discovered = nullptr;
    int metatrafficSock = -1;
    int userDataSock = -1;
    uint8_t* sendBuf = nullptr;
    uint32_t sendBufLen = 0;
    uint32_t* heartbeatCount = nullptr;
    bool dumpRosDiscoveryPayloadHex = false;
    RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackRuntimeFlavor runtimeFlavor =
        RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackRuntimeFlavor::EspStyle;

    // Sequence values at time of ACKNACK handling
    uint64_t rosDiscoveryPublicationSeqNum = 0;
    uint64_t rosDiscoverySubscriptionSeqNum = 0;
    uint64_t chatterPublicationSeqNum = 0;
    uint64_t rosDiscoveryInfoSeqNum = 0;
    uint64_t chatterDataSeqNum = 0;

    // Build environment
    SEDPHandler* sedpHandler = nullptr;
    const RTPSParticipant* participant = nullptr;
    uint32_t myIpAddr = 0;
    RTPSAckActionSedpBuildHeartbeatPolicy sedpBuildHeartbeatPolicy =
        RTPSAckActionSedpBuildHeartbeatPolicy::PassZero;
    RTPSAckActionBuildPayloadFn buildRosDiscInfoPayload = nullptr;
    RTPSAckActionBuildPayloadFn buildChatterPayload = nullptr;
    void* payloadCtx = nullptr;
};

// Populate a standard ctx: sets exec adapter fields, sequence contexts (via
// flavor helpers), and wires all five action specs to default static builders.
// After this, the wrapper typically only sets `ctx.srcGuidPrefix`.
void RTPSRunnerAdapter_initStandardAckActionCtx(
    RTPSAckActionStandardCtx& ctx,
    const RTPSAckActionStandardInitConfig& config);

// Default executeAction callback for RTPSAckNackRunnerCallbacks.executeAction.
// Expects userCtx to be an RTPSAckActionStandardCtx*.
void RTPSRunnerAdapter_standardExecuteAction(
    void* userCtx,
    RTPSAckNackRunnerAction action,
    const RTPSAckNackFields& fields,
    RTPSAckNackWriterKind writerKind);

// Default getChatterSeq callback for RTPSAckNackRunnerCallbacks.getChatterSeq.
// Expects userCtx to be an RTPSAckActionStandardCtx*.
uint64_t RTPSRunnerAdapter_standardGetChatterSeq(void* userCtx);