/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Reader Runtime - pure reliability decisions for the reader (subscriber) side.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "runtime/reliability/RTPSReaderRuntime.h"

namespace RaftRuntime::RTPS::Runtime::Reader
{

RTPSReaderDataAction evaluateIncomingDataDecision(
    const RTPSReaderWriterState& state,
    const RTPSReaderDataFields& data)
{
    // Any SN at or below highestContiguousSeq has already been delivered -> dedup.
    if (data.writerSeqNum <= state.highestContiguousSeq)
        return RTPSReaderDataAction::Dedup;

    // Check the bitmap for SNs strictly above highestContiguousSeq.
    const uint64_t offset = data.writerSeqNum - state.highestContiguousSeq - 1;
    if (offset < RTPS_READER_MAX_BITMAP_BITS)
    {
        const uint64_t mask = (uint64_t)1 << offset;
        if ((state.receivedBitmap & mask) != 0)
            return RTPSReaderDataAction::Dedup;
        return RTPSReaderDataAction::Accept;
    }

    // SN is beyond our tracked window. Still accept; caller is expected to
    // advance state (which will shift the window).
    return RTPSReaderDataAction::Accept;
}

void applyAcceptedDataToReaderState(
    RTPSReaderWriterState& state,
    uint64_t writerSeqNum)
{
    if (writerSeqNum <= state.highestContiguousSeq)
        return;

    if (writerSeqNum > state.highestSeenSeq)
        state.highestSeenSeq = writerSeqNum;

    // Fast path: exactly the next-expected SN.
    if (writerSeqNum == state.highestContiguousSeq + 1)
    {
        state.highestContiguousSeq = writerSeqNum;
        // Shift bitmap down by 1 and consume any leading run of received SNs.
        state.receivedBitmap >>= 1;
        while ((state.receivedBitmap & 1ULL) != 0)
        {
            state.highestContiguousSeq += 1;
            state.receivedBitmap >>= 1;
        }
        return;
    }

    const uint64_t offset = writerSeqNum - state.highestContiguousSeq - 1;
    if (offset < RTPS_READER_MAX_BITMAP_BITS)
    {
        state.receivedBitmap |= ((uint64_t)1 << offset);
        return;
    }

    // Outside current window: shift window so this SN becomes tracked.
    // Keep logic simple - drop older tracked bits that fall off the top.
    const uint64_t desiredBase = writerSeqNum - (RTPS_READER_MAX_BITMAP_BITS - 1);
    const uint64_t shift = desiredBase - (state.highestContiguousSeq + 1);
    if (shift >= RTPS_READER_MAX_BITMAP_BITS)
        state.receivedBitmap = 0;
    else
        state.receivedBitmap >>= shift;
    state.highestContiguousSeq = desiredBase - 1;
    const uint64_t newOffset = writerSeqNum - state.highestContiguousSeq - 1;
    state.receivedBitmap |= ((uint64_t)1 << newOffset);
}

RTPSReaderHeartbeatDecision evaluateIncomingHeartbeatDecision(
    const RTPSReaderWriterState& state,
    const RTPSReaderHeartbeatFields& hb)
{
    RTPSReaderHeartbeatDecision decision;
    decision.ackNackCount = state.outgoingAckNackCount + 1;

    // Best-effort readers never ACKNACK.
    if (state.reliabilityKind != RTPSReaderReliabilityKind::Reliable)
        return decision;

    // Liveliness-only heartbeat carries no SN semantics; no ACKNACK needed.
    if (hb.livelinessFlag)
        return decision;

    // Duplicate heartbeat (same count) with FINAL flag -> writer does not need reply.
    const bool heartbeatCountAdvanced =
        !state.hasSeenHeartbeat || (hb.count != state.lastHeartbeatCount);
    if (hb.finalFlag && !heartbeatCountAdvanced)
        return decision;

    // Compute ACKNACK base and bitmap.
    // Base = first SN we are still missing. In RTPS this is highestContiguousSeq + 1,
    // clamped to hb.firstSN (never ACK a SN below the writer's available range).
    uint64_t base = state.highestContiguousSeq + 1;
    if (base < hb.firstSN)
        base = hb.firstSN;

    // If the writer is far beyond our tracked window, request the most recent
    // window rather than repeatedly NACKing old samples that may have already
    // fallen out of the writer history. This is important for long-lived SEDP
    // writers, whose sequence numbers can be very high before this reader joins.
    if (hb.lastSN >= base + RTPS_READER_MAX_BITMAP_BITS)
        base = hb.lastSN - (RTPS_READER_MAX_BITMAP_BITS - 1);

    // If we are fully caught up (base > hb.lastSN), send a "nothing missing" ACKNACK
    // with numBits=0 at base = hb.lastSN + 1 ONLY when the writer did not set FINAL
    // (writer is explicitly asking for confirmation).
    if (base > hb.lastSN)
    {
        if (hb.finalFlag)
            return decision;
        decision.sendAckNack = true;
        decision.ackNackBase = hb.lastSN + 1;
        decision.ackNackNumBits = 0;
        decision.ackNackBitmap = 0;
        return decision;
    }

    // Otherwise build a bitmap over [base, min(hb.lastSN, base + MAX - 1)].
    const uint64_t windowEnd =
        (hb.lastSN < base + RTPS_READER_MAX_BITMAP_BITS - 1)
            ? hb.lastSN
            : base + RTPS_READER_MAX_BITMAP_BITS - 1;
    const uint32_t numBits = (uint32_t)(windowEnd - base + 1);

    // RTPS §9.4.5.2: bit i in the readerSNState bitmap = 1 means "reader has NOT yet
    // received SN (base + i) — please retransmit".  Bit = 0 means "already received".
    // reader state stores received flags for SNs > highestContiguousSeq; project onto
    // [base, windowEnd] as the logical NOT of those flags.
    uint64_t bitmap = 0;
    for (uint32_t i = 0; i < numBits; ++i)
    {
        const uint64_t sn = base + i;
        const uint64_t offset = sn - state.highestContiguousSeq - 1;
        bool received = false;
        if (offset < RTPS_READER_MAX_BITMAP_BITS)
        {
            received = (state.receivedBitmap & ((uint64_t)1 << offset)) != 0;
        }
        if (!received)
            bitmap |= ((uint64_t)1 << i);
    }

    decision.sendAckNack = true;
    decision.ackNackBase = base;
    decision.ackNackNumBits = numBits;
    decision.ackNackBitmap = bitmap;
    return decision;
}

void applyHeartbeatProcessedToReaderState(
    RTPSReaderWriterState& state,
    const RTPSReaderHeartbeatFields& hb,
    bool ackNackSent)
{
    state.lastHeartbeatCount = hb.count;
    state.hasSeenHeartbeat = true;
    if (hb.lastSN > state.highestSeenSeq)
        state.highestSeenSeq = hb.lastSN;
    if (ackNackSent)
        state.outgoingAckNackCount += 1;
}

}
