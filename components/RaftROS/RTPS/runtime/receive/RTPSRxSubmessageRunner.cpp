#include "runtime/receive/RTPSRxSubmessageRunner.h"

#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"
#include "runtime/reliability/RTPSReaderRunner.h"

namespace
{

// Build + send an ACKNACK using the bitmap-capable wire builder, driven by the
// pure RTPSReaderRunner heartbeat decision. Returns the number of bytes sent
// (>= 0) on success, or -1 if IO failed. `acknackCountOut` receives the
// outgoing ACKNACK count stamped on the wire (for telemetry).
int emitReaderRunnerAckNack(
    const RTPSRxSubmessageRunnerCallbacks& callbacks,
    void* userCtx,
    RTPSRxChannel channel,
    const uint8_t* srcGuidPrefix,
    const struct sockaddr_in& fromAddr,
    const RTPSReaderHeartbeatParsed& parsed,
    const RTPSReaderHeartbeatDecision& decision,
    uint32_t& acknackCountOut)
{
    if (!callbacks.getLocalGuidPrefix || !callbacks.resolveReaderEID ||
        !callbacks.resolveAckDest || !callbacks.sendAck)
        return -1;

    const uint8_t* ourReaderEID = callbacks.resolveReaderEID(
        userCtx, channel, parsed.writerEID, parsed.readerEID);
    if (!ourReaderEID)
        return -1;

    struct sockaddr_in ackDest = {};
    if (!callbacks.resolveAckDest(userCtx, channel, fromAddr, ackDest))
        return -1;

    uint8_t ackBuf[96];
    uint32_t pos = 0;
    pos += RTPSMessage::writeHeader(
        ackBuf + pos, sizeof(ackBuf) - pos, callbacks.getLocalGuidPrefix(userCtx));
    pos += RTPSMessage::writeInfoDST(
        ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);

    // Pack the decision's bitmap (uint64_t) into up to two LE32 words.
    // decision.ackNackNumBits is already capped at the reader runtime MAX_BITMAP_BITS (<=64).
    uint32_t bitmapWords[2] = { 0u, 0u };
    if (decision.ackNackNumBits > 0)
    {
        bitmapWords[0] = (uint32_t)(decision.ackNackBitmap & 0xFFFFFFFFu);
        bitmapWords[1] = (uint32_t)((decision.ackNackBitmap >> 32) & 0xFFFFFFFFu);
    }

    acknackCountOut = decision.ackNackCount;
    const uint32_t baseHigh = (uint32_t)(decision.ackNackBase >> 32);
    const uint32_t baseLow  = (uint32_t)(decision.ackNackBase & 0xFFFFFFFFu);
    const uint32_t ackLen = RTPSMessage::writeAcknackWithBitmap(
        ackBuf + pos, sizeof(ackBuf) - pos,
        ourReaderEID, parsed.writerEID,
        (int32_t)baseHigh, baseLow,
        decision.ackNackNumBits, bitmapWords,
        decision.ackNackCount,
        /*finalFlag*/ false);
    if (ackLen == 0)
        return -1;
    pos += ackLen;

    return callbacks.sendAck(userCtx, ackBuf, pos, ackDest);
}

} // anonymous namespace

bool RTPSRxSubmessageRunner_run(
    const uint8_t* packet,
    uint32_t packetLen,
    const struct sockaddr_in& fromAddr,
    RTPSRxChannel channel,
    uint32_t& acknackCount,
    const RTPSRxSubmessageRunnerCallbacks& callbacks,
    void* userCtx)
{
    if (!packet || packetLen == 0)
        return false;

    uint8_t srcGuidPrefix[12];
    uint32_t hdrLen = RTPSMessage::parseHeader(packet, packetLen, srcGuidPrefix);
    if (hdrLen == 0)
    {
        if (callbacks.onInvalidHeader)
            callbacks.onInvalidHeader(userCtx, channel);
        return false;
    }

    uint32_t offset = hdrLen;
    while (offset < packetLen)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            packet + offset, packetLen - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0)
            break;

        if ((submsgId == SUBMSG_HEARTBEAT) && (contentLen >= 28))
        {
            const uint8_t* fallbackReaderEID = pContent;
            const uint8_t* writerEID = pContent + 4;
            uint32_t lastSNHigh = RTPSMessage::readLE32(pContent + 20);
            uint32_t lastSNLow = RTPSMessage::readLE32(pContent + 24);

            // Opt-in: delegate to RTPSReaderRunner when the wrapper has
            // registered a reader-state lookup for this (src, writer).
            RaftRuntime::RTPS::Runtime::Reader::RTPSReaderWriterState* readerState = nullptr;
            if (callbacks.resolveReaderWriterState)
                readerState = callbacks.resolveReaderWriterState(
                    userCtx, channel, srcGuidPrefix, writerEID);

            if (readerState)
            {
                RTPSReaderHeartbeatParsed parsed;
                if (RTPSReaderRunner_parseHeartbeat(pContent, contentLen, flags, parsed))
                {
                    const auto decision =
                        RaftRuntime::RTPS::Runtime::Reader::evaluateIncomingHeartbeatDecision(
                            *readerState, parsed.fields);

                    bool responded = false;
                    int sent = -1;
                    if (decision.sendAckNack)
                    {
                        uint32_t stampedCount = 0;
                        sent = emitReaderRunnerAckNack(
                            callbacks, userCtx, channel, srcGuidPrefix, fromAddr,
                            parsed, decision, stampedCount);
                        if (sent >= 0)
                        {
                            acknackCount = stampedCount;
                            responded = true;
                        }
                    }

                    RaftRuntime::RTPS::Runtime::Reader::applyHeartbeatProcessedToReaderState(
                        *readerState, parsed.fields, responded);

                    if (callbacks.onHeartbeat)
                        callbacks.onHeartbeat(userCtx, channel, writerEID, lastSNLow, responded, sent);

                    offset += submsgSize;
                    continue;
                }
                // Fall through to legacy path on parse failure.
            }

            bool responded = false;
            int sent = -1;
            if (RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::shouldRespondToHeartbeat(flags) &&
                callbacks.getLocalGuidPrefix && callbacks.resolveReaderEID &&
                callbacks.resolveAckDest && callbacks.sendAck)
            {
                const uint8_t* ourReaderEID = callbacks.resolveReaderEID(
                    userCtx, channel, writerEID, fallbackReaderEID);
                struct sockaddr_in ackDest = {};
                if (ourReaderEID && callbacks.resolveAckDest(userCtx, channel, fromAddr, ackDest))
                {
                    uint8_t ackBuf[80];
                    uint32_t pos = 0;
                    pos += RTPSMessage::writeHeader(
                        ackBuf + pos, sizeof(ackBuf) - pos, callbacks.getLocalGuidPrefix(userCtx));
                    pos += RTPSMessage::writeInfoDST(
                        ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);

                    uint32_t ackBaseLow = lastSNLow + 1;
                    uint32_t ackBaseHigh = lastSNHigh + (ackBaseLow == 0 ? 1 : 0);
                    acknackCount++;
                    pos += RTPSMessage::writeAcknack(
                        ackBuf + pos, sizeof(ackBuf) - pos,
                        ourReaderEID, writerEID,
                        (int32_t)ackBaseHigh, ackBaseLow, acknackCount);

                    sent = callbacks.sendAck(userCtx, ackBuf, pos, ackDest);
                    responded = true;
                }
            }

            if (callbacks.onHeartbeat)
                callbacks.onHeartbeat(userCtx, channel, writerEID, lastSNLow, responded, sent);
        }
        else if ((submsgId == SUBMSG_DATA) && (contentLen >= 24))
        {
            if (callbacks.onData)
                callbacks.onData(userCtx, channel, packet, packetLen, srcGuidPrefix, fromAddr, pContent, contentLen);
        }
        else if ((submsgId == SUBMSG_ACKNACK) && (contentLen >= 24))
        {
            if (callbacks.onAckNack)
                callbacks.onAckNack(userCtx, channel, srcGuidPrefix, pContent, contentLen, fromAddr);
        }
        else
        {
            if (callbacks.onOther)
                callbacks.onOther(userCtx, channel, submsgId, flags, contentLen);
        }

        offset += submsgSize;
    }

    return true;
}