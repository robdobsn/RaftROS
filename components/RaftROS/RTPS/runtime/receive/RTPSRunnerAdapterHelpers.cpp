#include "runtime/receive/RTPSRunnerAdapterHelpers.h"

#include <cstdio>
#include <sys/socket.h>

#include "runtime/discovery/RTPSDiscoveryRuntime.h"
#include "runtime/announce/SEDPHandler.h"
#include "runtime/core/RTPSParticipant.h"

#ifndef RAFTROS_ACK_HEX_DUMP_ENABLE
#define RAFTROS_ACK_HEX_DUMP_ENABLE 0
#endif

#ifndef RAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE
#define RAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE 0
#endif

void RTPSRunnerAdapter_applyRxBaseCallbacks(RTPSRxSubmessageRunnerCallbacks& callbacks)
{
    callbacks.getLocalGuidPrefix = [](void* userCtx) -> const uint8_t*
    {
        const RTPSRxRunnerAdapterBaseCtx* ctx =
            static_cast<const RTPSRxRunnerAdapterBaseCtx*>(userCtx);
        return ctx ? ctx->localGuidPrefix : nullptr;
    };

    callbacks.resolveReaderEID = [](void* userCtx,
                                    RTPSRxChannel,
                                    const uint8_t* writerEID,
                                    const uint8_t* fallbackReaderEID) -> const uint8_t*
    {
        const RTPSRxRunnerAdapterBaseCtx* ctx =
            static_cast<const RTPSRxRunnerAdapterBaseCtx*>(userCtx);
        if (!ctx)
            return fallbackReaderEID;
        if (ctx->readerPolicy == RTPSRxAdapterReaderPolicy::UseHeartbeatReader)
            return fallbackReaderEID;
        return RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::localReaderForRemoteWriter(
            writerEID,
            fallbackReaderEID);
    };

    callbacks.resolveAckDest = [](void* userCtx,
                                  RTPSRxChannel,
                                  const struct sockaddr_in& from,
                                  struct sockaddr_in& outDest) -> bool
    {
        const RTPSRxRunnerAdapterBaseCtx* ctx =
            static_cast<const RTPSRxRunnerAdapterBaseCtx*>(userCtx);
        if (!ctx)
        {
            outDest = from;
            return false;
        }

        if (ctx->ackDestPolicy == RTPSRxAdapterAckDestPolicy::RouteToDiscoveredMetatraffic)
        {
            if (ctx->discovered)
            {
                return RaftRuntime::RTPS::Runtime::DiscoveryRuntime::assignMetatrafficDestByIp(
                    *ctx->discovered, from, outDest);
            }
            outDest = from;
            return false;
        }

        outDest = from;
        return true;
    };

    callbacks.sendAck = [](void* userCtx,
                           const uint8_t* ackBuf,
                           uint32_t ackLen,
                           const struct sockaddr_in& destAddr) -> int
    {
        const RTPSRxRunnerAdapterBaseCtx* ctx =
            static_cast<const RTPSRxRunnerAdapterBaseCtx*>(userCtx);
        if (!ctx || ctx->ackSendSock < 0)
            return -1;
        return sendto(ctx->ackSendSock, ackBuf, ackLen, 0,
                      (const struct sockaddr*)&destAddr, sizeof(destAddr));
    };
}

bool RTPSRunnerAdapter_ackResolveRemote(void* userCtx, const uint8_t* srcGuidPrefix)
{
    RTPSAckNackRunnerAdapterBaseCtx* ctx =
        static_cast<RTPSAckNackRunnerAdapterBaseCtx*>(userCtx);
    if (!ctx || !ctx->discovered)
        return false;

    ctx->remote = RaftRuntime::RTPS::Runtime::DiscoveryRuntime::findByGuidPrefix(
        *ctx->discovered, srcGuidPrefix);
    return ctx->remote != nullptr;
}

void RTPSRunnerAdapter_initAckExecContext(
    RTPSAckNackRunnerExecAdapterCtx& ctx,
    const RTPSAckNackRunnerExecInitConfig& initConfig)
{
    ctx.base.discovered = initConfig.discovered;
    ctx.metatrafficSock = initConfig.metatrafficSock;
    ctx.userDataSock = initConfig.userDataSock;
    ctx.sendBuf = initConfig.sendBuf;
    ctx.heartbeatCount = initConfig.heartbeatCount;
    ctx.mutationPolicy = initConfig.mutationPolicy;
    ctx.debugPolicy.dumpRosDiscoveryPayloadHex = initConfig.dumpRosDiscoveryPayloadHex;
}

void RTPSRunnerAdapter_initAckCallbacks(
    RTPSAckNackRunnerCallbacks& callbacks,
    const RTPSAckNackRunnerCallbackInitConfig& initConfig)
{
    callbacks.logParsed = initConfig.logParsed;
    callbacks.resolveRemote = RTPSRunnerAdapter_ackResolveRemote;
    callbacks.unknownRemote = initConfig.unknownRemote;
    callbacks.getChatterSeq = initConfig.getChatterSeq;
    callbacks.executeAction = initConfig.executeAction;
}

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
    RTPSAckNackRunnerLogSendResultFn logSendResult)
{
    const DiscoveredParticipant* remote = adapterCtx.base.remote;
    if (!remote)
        return;

    const RTPSAckNackRunnerActionExecSpec* execSpec = nullptr;
    switch (action)
    {
        case RTPSAckNackRunnerAction::RetransmitSedpRosDiscoveryPublication:
            execSpec = &sedpRosDiscoveryPublicationSpec;
            break;
        case RTPSAckNackRunnerAction::RetransmitSedpChatterPublication:
            execSpec = &sedpChatterPublicationSpec;
            break;
        case RTPSAckNackRunnerAction::RetransmitSedpRosDiscoverySubscription:
            execSpec = &sedpRosDiscoverySubscriptionSpec;
            break;
        case RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo:
            execSpec = &rosDiscoveryInfoSpec;
            break;
        case RTPSAckNackRunnerAction::RetransmitChatterData:
            execSpec = &chatterDataSpec;
            break;
        default:
            return;
    }

    if (!execSpec || !execSpec->buildMessage)
        return;

    if (adapterCtx.heartbeatCount)
    {
        RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::applyAckActionHeartbeatMutation(
            action,
            adapterCtx.mutationPolicy,
            *adapterCtx.heartbeatCount);
    }

    const uint32_t msgLen = execSpec->buildMessage(actionCtx, srcGuidPrefix, chatterSeq);
    if ((msgLen == 0) || !adapterCtx.sendBuf)
        return;

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_addr.s_addr = remote->ipAddr;

    int sendSock = adapterCtx.metatrafficSock;
    if (execSpec->sendChannel == RTPSAckNackRunnerSendChannel::UserData)
    {
        sendSock = adapterCtx.userDataSock;
        dest.sin_port = htons(remote->userDataPort);
    }
    else
    {
        dest.sin_port = htons(remote->metatrafficPort);
    }

    int sent = -1;
    if (sendSock >= 0)
    {
        sent = sendto(sendSock,
                      adapterCtx.sendBuf,
                      msgLen,
                      0,
                      (struct sockaddr*)&dest,
                      sizeof(dest));
    }

    if (logSendResult)
        logSendResult(actionCtx, action, sent, msgLen);
}

const char* RTPSRunnerAdapter_ackActionLogLabel(RTPSAckNackRunnerAction action)
{
#if RAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE
    switch (action)
    {
        case RTPSAckNackRunnerAction::RetransmitSedpRosDiscoveryPublication:
            return "retransmit SEDP publication";
        case RTPSAckNackRunnerAction::RetransmitSedpChatterPublication:
            return "retransmit SEDP chatter publication";
        case RTPSAckNackRunnerAction::RetransmitSedpRosDiscoverySubscription:
            return "retransmit SEDP subscription";
        case RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo:
            return "retransmit ros_discovery_info";
        case RTPSAckNackRunnerAction::RetransmitChatterData:
            return "retransmit chatter data";
        default:
            return "unknown ACK action";
    }
#else
    (void)action;
    return "ACK action";
#endif
}

const char* RTPSRunnerAdapter_ackActionResultLabel(RTPSAckNackRunnerAction action)
{
#if RAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE
    switch (action)
    {
        case RTPSAckNackRunnerAction::RetransmitSedpRosDiscoveryPublication:
            return "SEDP publication retransmit";
        case RTPSAckNackRunnerAction::RetransmitSedpChatterPublication:
            return "SEDP chatter retransmit";
        case RTPSAckNackRunnerAction::RetransmitSedpRosDiscoverySubscription:
            return "SEDP subscription retransmit";
        case RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo:
            return "ros_discovery_info retransmit";
        case RTPSAckNackRunnerAction::RetransmitChatterData:
            return "chatter retransmit";
        default:
            return "ACK action send";
    }
#else
    (void)action;
    return "ACK send";
#endif
}

bool RTPSRunnerAdapter_shouldDumpAckPayload(
    RTPSAckNackRunnerAction action,
    const RTPSAckNackRunnerExecAdapterCtx& adapterCtx)
{
#if RAFTROS_ACK_HEX_DUMP_ENABLE
    return adapterCtx.debugPolicy.dumpRosDiscoveryPayloadHex &&
           (action == RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo);
#else
    (void)action;
    (void)adapterCtx;
    return false;
#endif
}

void RTPSRunnerAdapter_logHexPayload(const char* label,
                                     const uint8_t* pData,
                                     uint32_t dataLen)
{
#if RAFTROS_ACK_HEX_DUMP_ENABLE
    if (!pData || (dataLen == 0))
        return;

    std::fprintf(stderr, "%s (%u bytes):", label ? label : "PAYLOAD_HEX", dataLen);
    for (uint32_t i = 0; i < dataLen; i++)
        std::fprintf(stderr, " %02x", pData[i]);
    std::fprintf(stderr, "\n");
#else
    (void)label;
    (void)pData;
    (void)dataLen;
#endif
}

//
// Shared default ACKNACK action-execution helpers
//

namespace
{

uint32_t standardSedpHeartbeatForBuild(const RTPSAckActionStandardCtx& ctx)
{
    if (ctx.sedpBuildHeartbeatPolicy ==
        RTPSAckActionSedpBuildHeartbeatPolicy::PassLiveHeartbeat)
    {
        return ctx.exec.heartbeatCount ? *ctx.exec.heartbeatCount : 0;
    }
    return 0;
}

uint32_t standardBuildSedp(RTPSAckActionStandardCtx& ctx,
                           const uint8_t* destGuidPrefix,
                           RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction action)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

    RTPSAckActionSedpPlan plan;
    if (!getAckActionSedpPlan(action, ctx.sedpSequenceContext, plan))
        return 0;
    if (!ctx.sedpHandler || !ctx.participant || !ctx.sendBufMutable)
        return 0;

    const uint32_t hbForBuild = standardSedpHeartbeatForBuild(ctx);

    // Subscription action uses buildSubscriptionMessage; publications use
    // buildPublicationMessage. Profile lookup distinguishes them.
    RTPSAckActionSedpEndpointProfile subProfile;
    const bool isSubscription =
        getAckActionSedpSubscriptionProfile(action, subProfile);

    if (isSubscription)
    {
        return ctx.sedpHandler->buildSubscriptionMessage(
            ctx.sendBufMutable, ctx.sendBufLen,
            *ctx.participant, destGuidPrefix,
            plan.endpoint.entityId,
            plan.endpoint.topicName,
            plan.endpoint.typeName,
            plan.endpoint.reliabilityKind,
            plan.endpoint.durabilityKind,
            plan.sequenceNumber, ctx.myIpAddr, hbForBuild);
    }

    return ctx.sedpHandler->buildPublicationMessage(
        ctx.sendBufMutable, ctx.sendBufLen,
        *ctx.participant, destGuidPrefix,
        plan.endpoint.entityId,
        plan.endpoint.topicName,
        plan.endpoint.typeName,
        plan.endpoint.reliabilityKind,
        plan.endpoint.durabilityKind,
        plan.sequenceNumber, ctx.myIpAddr, hbForBuild);
}

uint32_t standardBuildUserData(RTPSAckActionStandardCtx& ctx,
                               const uint8_t* destGuidPrefix,
                               RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction action,
                               RTPSAckActionBuildPayloadFn payloadFn,
                               const char* hexDumpLabel)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

    if (!payloadFn || !ctx.sedpHandler || !ctx.participant || !ctx.sendBufMutable)
        return 0;

    uint8_t payloadBuf[256];
    const uint32_t payloadLen =
        payloadFn(ctx.payloadCtx, payloadBuf, sizeof(payloadBuf));
    if (payloadLen == 0)
        return 0;

    RTPSAckActionUserDataPlan plan;
    if (!getAckActionUserDataPlan(action, ctx.userDataSequenceContext, plan))
        return 0;

    if (RTPSRunnerAdapter_shouldDumpAckPayload(action, ctx.exec))
        RTPSRunnerAdapter_logHexPayload(hexDumpLabel, payloadBuf, payloadLen);

    const uint32_t hb = ctx.exec.heartbeatCount ? *ctx.exec.heartbeatCount : 0;

    // Keyed builtin topics (e.g. ros_discovery_info) require the inline-QoS
    // PID_KEY_HASH on every DATA emission.  The plan requests this for
    // retransmits to mirror what the periodic HB build path supplies.
    const uint8_t* keyHash16 =
        plan.useParticipantKeyHash ? ctx.participant->getParticipantGuid() : nullptr;

    if (plan.hasFirstSNOverride)
    {
        return ctx.sedpHandler->buildUserDataMessage(
            ctx.sendBufMutable, ctx.sendBufLen,
            *ctx.participant, destGuidPrefix,
            plan.writerEntityId,
            payloadBuf, payloadLen,
            plan.sequenceNumber, hb,
            plan.firstSN,
            keyHash16);
    }

    return ctx.sedpHandler->buildUserDataMessage(
        ctx.sendBufMutable, ctx.sendBufLen,
        *ctx.participant, destGuidPrefix,
        plan.writerEntityId,
        payloadBuf, payloadLen,
        plan.sequenceNumber, hb,
        /*firstSN=*/1,
        keyHash16);
}

uint32_t standardBuildSedpRosDiscoveryPublication(
    void* actionCtx, const uint8_t* srcGuidPrefix, uint64_t)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(actionCtx);
    return standardBuildSedp(*ctx, srcGuidPrefix,
        RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication);
}

uint32_t standardBuildSedpChatterPublication(
    void* actionCtx, const uint8_t* srcGuidPrefix, uint64_t)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(actionCtx);
    return standardBuildSedp(*ctx, srcGuidPrefix,
        RTPSAckNackDecisionAction::RetransmitSedpChatterPublication);
}

uint32_t standardBuildSedpRosDiscoverySubscription(
    void* actionCtx, const uint8_t* srcGuidPrefix, uint64_t)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(actionCtx);
    return standardBuildSedp(*ctx, srcGuidPrefix,
        RTPSAckNackDecisionAction::RetransmitSedpRosDiscoverySubscription);
}

uint32_t standardBuildRosDiscoveryInfo(
    void* actionCtx, const uint8_t* srcGuidPrefix, uint64_t)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(actionCtx);
    return standardBuildUserData(*ctx, srcGuidPrefix,
        RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo,
        ctx->buildRosDiscInfoPayload,
        "ROSDISC_PAYLOAD_HEX");
}

uint32_t standardBuildChatterData(
    void* actionCtx, const uint8_t* srcGuidPrefix, uint64_t chatterSeq)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(actionCtx);
    // Keep the sequence context in sync with the runner-supplied live seq.
    ctx->userDataSequenceContext.chatterDataSeqNum = chatterSeq;
    return standardBuildUserData(*ctx, srcGuidPrefix,
        RTPSAckNackDecisionAction::RetransmitChatterData,
        ctx->buildChatterPayload,
        "CHATTER_PAYLOAD_HEX");
}

}

void RTPSRunnerAdapter_initStandardAckActionCtx(
    RTPSAckActionStandardCtx& ctx,
    const RTPSAckActionStandardInitConfig& config)
{
    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

    RTPSAckNackRunnerExecInitConfig execInit;
    execInit.discovered = config.discovered;
    execInit.metatrafficSock = config.metatrafficSock;
    execInit.userDataSock = config.userDataSock;
    execInit.sendBuf = config.sendBuf;
    execInit.heartbeatCount = config.heartbeatCount;
    execInit.mutationPolicy =
        makeAckNackMutationPolicyForFlavor(config.runtimeFlavor);
    execInit.dumpRosDiscoveryPayloadHex = config.dumpRosDiscoveryPayloadHex;
    RTPSRunnerAdapter_initAckExecContext(ctx.exec, execInit);

    ctx.sedpSequenceContext = makeAckSedpSequenceContextForFlavor(
        config.runtimeFlavor,
        config.rosDiscoveryPublicationSeqNum,
        config.rosDiscoverySubscriptionSeqNum,
        config.chatterPublicationSeqNum);

    ctx.userDataSequenceContext = makeAckUserDataSequenceContextForFlavor(
        config.runtimeFlavor,
        config.rosDiscoveryInfoSeqNum,
        config.chatterDataSeqNum);

    ctx.sendBufMutable = config.sendBuf;
    ctx.sendBufLen = config.sendBufLen;
    ctx.sedpHandler = config.sedpHandler;
    ctx.participant = config.participant;
    ctx.myIpAddr = config.myIpAddr;
    ctx.sedpBuildHeartbeatPolicy = config.sedpBuildHeartbeatPolicy;
    ctx.buildRosDiscInfoPayload = config.buildRosDiscInfoPayload;
    ctx.buildChatterPayload = config.buildChatterPayload;
    ctx.payloadCtx = config.payloadCtx;

    ctx.sedpRosDiscoveryPublicationSpec.sendChannel =
        RTPSAckNackRunnerSendChannel::Metatraffic;
    ctx.sedpRosDiscoveryPublicationSpec.buildMessage =
        &standardBuildSedpRosDiscoveryPublication;

    ctx.sedpChatterPublicationSpec.sendChannel =
        RTPSAckNackRunnerSendChannel::Metatraffic;
    ctx.sedpChatterPublicationSpec.buildMessage =
        &standardBuildSedpChatterPublication;

    ctx.sedpRosDiscoverySubscriptionSpec.sendChannel =
        RTPSAckNackRunnerSendChannel::Metatraffic;
    ctx.sedpRosDiscoverySubscriptionSpec.buildMessage =
        &standardBuildSedpRosDiscoverySubscription;

    ctx.rosDiscoveryInfoSpec.sendChannel =
        RTPSAckNackRunnerSendChannel::UserData;
    ctx.rosDiscoveryInfoSpec.buildMessage = &standardBuildRosDiscoveryInfo;

    ctx.chatterDataSpec.sendChannel =
        RTPSAckNackRunnerSendChannel::UserData;
    ctx.chatterDataSpec.buildMessage = &standardBuildChatterData;
}

uint64_t RTPSRunnerAdapter_standardGetChatterSeq(void* userCtx)
{
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(userCtx);
    if (!ctx)
        return 0;
    return ctx->userDataSequenceContext.chatterDataSeqNum;
}

void RTPSRunnerAdapter_standardExecuteAction(
    void* userCtx,
    RTPSAckNackRunnerAction action,
    const RTPSAckNackFields&,
    RTPSAckNackWriterKind)
{
    auto* ctx = static_cast<RTPSAckActionStandardCtx*>(userCtx);
    if (!ctx)
        return;

    const uint64_t chatterSeq = ctx->userDataSequenceContext.chatterDataSeqNum;

    RTPSRunnerAdapter_executeAckAction(
        ctx->exec,
        action,
        ctx->srcGuidPrefix,
        chatterSeq,
        ctx->sedpRosDiscoveryPublicationSpec,
        ctx->sedpChatterPublicationSpec,
        ctx->sedpRosDiscoverySubscriptionSpec,
        ctx->rosDiscoveryInfoSpec,
        ctx->chatterDataSpec,
        ctx,
        nullptr);
}