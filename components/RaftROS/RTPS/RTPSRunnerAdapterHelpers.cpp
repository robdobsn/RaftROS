#include "RTPSRunnerAdapterHelpers.h"

#include <sys/socket.h>

#include "RTPSBuiltinEndpointMap.h"
#include "RTPSDiscoveredParticipantLookup.h"

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
        return RTPSBuiltinEndpointMap_localReaderForRemoteWriter(writerEID, fallbackReaderEID);
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
                return RTPSDiscoveredParticipantLookup_assignMetatrafficDestByIp(
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

    ctx->remote = RTPSDiscoveredParticipantLookup_findByGuidPrefix(
        *ctx->discovered, srcGuidPrefix);
    return ctx->remote != nullptr;
}
