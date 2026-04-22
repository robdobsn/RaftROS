#pragma once

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "lwip/sockets.h"
#else
#include <netinet/in.h>
#endif

#include "runtime/wire/RTPSMessage.h"
#include "runtime/reliability/RTPSReaderRuntime.h"

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

    // Opt-in reader-runtime delegation. When `resolveReaderWriterState` is non-null,
    // incoming HEARTBEATs for the reader's matched writers are routed through the
    // shared RTPSReaderRunner decision path (bitmap-capable ACKNACK). When null,
    // the legacy inline ACKNACK path is used for backwards compatibility.
    //
    // `resolveReaderWriterState` returns a pointer to wrapper-owned reader state
    // for the (srcGuidPrefix, writerEID) pair, or nullptr to fall back to the
    // legacy path for this HEARTBEAT.
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderWriterState*
        (*resolveReaderWriterState)(void* userCtx,
                                     RTPSRxChannel channel,
                                     const uint8_t* srcGuidPrefix,
                                     const uint8_t* writerEID) = nullptr;
};

bool RTPSRxSubmessageRunner_run(
    const uint8_t* packet,
    uint32_t packetLen,
    const struct sockaddr_in& fromAddr,
    RTPSRxChannel channel,
    uint32_t& acknackCount,
    const RTPSRxSubmessageRunnerCallbacks& callbacks,
    void* userCtx);