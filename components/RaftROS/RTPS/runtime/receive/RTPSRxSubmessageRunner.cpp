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

    // Pack the decision's bitmap (uint64_t) into up to two LE32 wire words.
    //
    // INTERNAL representation (RTPSReaderRuntime): bit i of ackNackBitmap (LSB-first,
    //   i.e. bit 0 = 1<<0) represents SN (ackNackBase + i).
    //
    // RTPS WIRE encoding (§9.4.2.7): bit i maps to bit (31 - i%32) of word (i/32),
    //   i.e. SN ackNackBase is the MSB of bitmapWords[0].  FastDDS serialises this as:
    //     bitmapWords[j] |= (1u << (31 - i%32))  for each set bit i in word j.
    //
    // Translation: reverse the bit order within each 32-bit chunk before writing.
    auto bitrev32 = [](uint32_t x) -> uint32_t {
        x = ((x >>  1) & 0x55555555u) | ((x <<  1) & 0xAAAAAAAAu);
        x = ((x >>  2) & 0x33333333u) | ((x <<  2) & 0xCCCCCCCCu);
        x = ((x >>  4) & 0x0F0F0F0Fu) | ((x <<  4) & 0xF0F0F0F0u);
        x = ((x >>  8) & 0x00FF00FFu) | ((x <<  8) & 0xFF00FF00u);
        x = ( x >> 16              ) | ( x << 16              );
        return x;
    };
    uint32_t bitmapWords[2] = { 0u, 0u };
    if (decision.ackNackNumBits > 0)
    {
        bitmapWords[0] = bitrev32((uint32_t)(decision.ackNackBitmap & 0xFFFFFFFFu));
        bitmapWords[1] = bitrev32((uint32_t)((decision.ackNackBitmap >> 32) & 0xFFFFFFFFu));
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

    if (callbacks.onAckNackBuilt)
    {
        callbacks.onAckNackBuilt(
            userCtx, channel, ourReaderEID, parsed.writerEID, decision,
            ackBuf, pos, fromAddr, ackDest);
    }
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

                    if (callbacks.onHeartbeatDecision)
                    {
                        callbacks.onHeartbeatDecision(
                            userCtx, channel, writerEID, parsed.fields, decision, responded, sent);
                    }
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
            RaftRuntime::RTPS::Runtime::Reader::RTPSReaderWriterState* readerState = nullptr;
            const uint8_t* writerEID = pContent + 8;
            if (callbacks.resolveReaderWriterState)
                readerState = callbacks.resolveReaderWriterState(
                    userCtx, channel, srcGuidPrefix, writerEID);
            if (readerState)
            {
                RTPSReaderDataParsed parsed;
                if (RTPSReaderRunner_parseData(pContent, contentLen, flags, parsed))
                {
                    // Key-only dispose/unregister DATA can arrive long after
                    // the live endpoint ADD samples. Do not advance the reader
                    // reliability window for those no-payload samples, or the
                    // next HEARTBEAT ACKNACK will start near the dispose SN and
                    // never request the earlier ADD payloads we actually need.
                    if (parsed.payload && parsed.payloadLen > 0)
                    {
                        const auto decision =
                            RaftRuntime::RTPS::Runtime::Reader::evaluateIncomingDataDecision(
                                *readerState, parsed.fields);
                        if (decision == RaftRuntime::RTPS::Runtime::Reader::RTPSReaderDataAction::Accept)
                        {
                            RaftRuntime::RTPS::Runtime::Reader::applyAcceptedDataToReaderState(
                                *readerState, parsed.fields.writerSeqNum);
                        }
                    }
                }
            }
            if (callbacks.onData)
                callbacks.onData(userCtx, channel, packet, packetLen, srcGuidPrefix, fromAddr, pContent, contentLen, flags);
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