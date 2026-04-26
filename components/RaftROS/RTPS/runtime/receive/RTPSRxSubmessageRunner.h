#pragma once

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "lwip/sockets.h"
#else
#include <netinet/in.h>
#endif

#include "runtime/wire/RTPSMessage.h"
#include "runtime/reliability/RTPSReaderRuntime.h"

// Given the content of a DATA submessage (pointer/length starting after the 4-byte
// submessage header) and the DATA flags byte (bit 1 Q = inlineQoS present), return
// the pointer/length of the serialized payload (skipping the 20-byte fixed prefix
// and, when present, the inline-QoS ParameterList terminated by PID_SENTINEL).
// On malformed input, returns {nullptr, 0}.
static inline void RTPSData_getSerializedPayload(
    const uint8_t* pContent, uint32_t contentLen, uint8_t flags,
    const uint8_t*& payloadOut, uint32_t& payloadLenOut)
{
    payloadOut = nullptr;
    payloadLenOut = 0;
    if (!pContent || contentLen < 20)
        return;
    uint32_t off = 20;
    if (flags & 0x02) // Q: inline QoS ParameterList present
    {
        while (off + 4 <= contentLen)
        {
            const uint16_t pid  = (uint16_t)(pContent[off] | (pContent[off + 1] << 8));
            const uint16_t plen = (uint16_t)(pContent[off + 2] | (pContent[off + 3] << 8));
            off += 4;
            if (pid == 0x0001) // PID_SENTINEL
                break;
            if ((uint32_t)off + plen > contentLen)
                return;
            off += plen;
        }
    }
    if (off > contentLen)
        return;
    payloadOut = pContent + off;
    payloadLenOut = contentLen - off;
}

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
    void (*onHeartbeatDecision)(void* userCtx,
                                RTPSRxChannel channel,
                                const uint8_t* writerEID,
                                const RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatFields& fields,
                                const RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatDecision& decision,
                                bool responded,
                                int sentBytes) = nullptr;
    void (*onAckNackBuilt)(void* userCtx,
                           RTPSRxChannel channel,
                           const uint8_t* readerEID,
                           const uint8_t* writerEID,
                           const RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatDecision& decision,
                           const uint8_t* ackBuf,
                           uint32_t ackLen,
                           const struct sockaddr_in& fromAddr,
                           const struct sockaddr_in& destAddr) = nullptr;
    void (*onData)(void* userCtx,
                   RTPSRxChannel channel,
                   const uint8_t* packet,
                   uint32_t packetLen,
                   const uint8_t* srcGuidPrefix,
                   const struct sockaddr_in& fromAddr,
                   const uint8_t* pContent,
                   uint32_t contentLen,
                   uint8_t flags) = nullptr;
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