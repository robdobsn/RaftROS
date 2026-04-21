#include "runtime/receive/RTPSRunnerAdapterHelpers.h"

#include <cstdio>
#include <sys/socket.h>

#include "runtime/discovery/RTPSDiscoveryRuntime.h"

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
        return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::localReaderForRemoteWriter(
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
                return RaftROS::RTPS::Runtime::DiscoveryRuntime::assignMetatrafficDestByIp(
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

    ctx->remote = RaftROS::RTPS::Runtime::DiscoveryRuntime::findByGuidPrefix(
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
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::applyAckActionHeartbeatMutation(
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