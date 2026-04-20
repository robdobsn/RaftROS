#pragma once

#include <stdint.h>
#include <netinet/in.h>

#include "RTPSMessage.h"

enum class RTPSRxChannel
{
    Metatraffic = 0,
    UserData,
};

struct RTPSRxSubmessageRunnerCallbacks
{
    const uint8_t* (*getLocalGuidPrefix)(void* userCtx) = nullptr;
    const uint8_t* (*resolveReaderEID)(void* userCtx,
                                       RTPSRxChannel channel,
                                       const uint8_t* remoteWriterEID,
                                       const uint8_t* fallbackReaderEID) = nullptr;
    bool (*resolveAckDest)(void* userCtx,
                           RTPSRxChannel channel,
                           const struct sockaddr_in& fromAddr,
                           struct sockaddr_in& outDest) = nullptr;
    int (*sendAck)(void* userCtx,
                   const uint8_t* ackBuf,
                   uint32_t ackLen,
                   const struct sockaddr_in& destAddr) = nullptr;

    void (*onInvalidHeader)(void* userCtx, RTPSRxChannel channel) = nullptr;
    void (*onHeartbeat)(void* userCtx,
                        RTPSRxChannel channel,
                        const uint8_t* writerEID,
                        uint32_t lastSNLow,
                        bool responded,
                        int sentBytes) = nullptr;
    void (*onData)(void* userCtx,
                   RTPSRxChannel channel,
                   const uint8_t* packet,
                   uint32_t packetLen,
                   const uint8_t* srcGuidPrefix,
                   const struct sockaddr_in& fromAddr,
                   const uint8_t* pContent,
                   uint32_t contentLen) = nullptr;
    void (*onAckNack)(void* userCtx,
                      RTPSRxChannel channel,
                      const uint8_t* srcGuidPrefix,
                      const uint8_t* pContent,
                      uint32_t contentLen,
                      const struct sockaddr_in& fromAddr) = nullptr;
    void (*onOther)(void* userCtx,
                    RTPSRxChannel channel,
                    RTPSSubmessageId submsgId,
                    uint8_t flags,
                    uint32_t contentLen) = nullptr;
};

bool RTPSRxSubmessageRunner_run(
    const uint8_t* packet,
    uint32_t packetLen,
    const struct sockaddr_in& fromAddr,
    RTPSRxChannel channel,
    uint32_t& acknackCount,
    const RTPSRxSubmessageRunnerCallbacks& callbacks,
    void* userCtx);
