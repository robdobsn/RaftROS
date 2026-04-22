#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Reader Runtime - pure reliability decisions for the reader (subscriber) side.
//
// Mirrors the shape of RTPSReliabilityAndWriterStateRuntime but for incoming DATA /
// HEARTBEAT / GAP traffic. This module owns:
//   - Per-remote-writer reader state (seen SNs, last heartbeat count, missing window)
//   - Pure decision helpers:
//       * evaluateIncomingDataDecision  - accept / drop / dedup incoming DATA sample
//       * evaluateIncomingHeartbeatDecision - should we emit ACKNACK, and with what base/bitmap
//
// No platform IO. No serialization. Wrappers translate decisions into socket sends.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdint>

namespace RaftRuntime::RTPS::Runtime::Reader
{

// Max number of outstanding missing SNs we track per remote writer.
// Matches typical RTPS ACKNACK bitmap width; kept small for ESP32 RAM.
static constexpr uint32_t RTPS_READER_MAX_BITMAP_BITS = 64;

enum class RTPSReaderReliabilityKind : uint8_t
{
    BestEffort = 0,
    Reliable,
};

// Per-remote-writer reliability state held by the reader.
// One instance per (remote participant, remote writer entity id) pair.
struct RTPSReaderWriterState
{
    // Reliability policy negotiated for this match
    RTPSReaderReliabilityKind reliabilityKind = RTPSReaderReliabilityKind::Reliable;

    // Highest writer sequence number delivered to user dispatch.
    // Everything <= this SN is considered fully received.
    uint64_t highestContiguousSeq = 0;

    // Highest writer SN we have *seen at all* (possibly with gaps beneath).
    // Used to derive the ACKNACK bitmap base and width.
    uint64_t highestSeenSeq = 0;

    // Bitmap of received SNs strictly above highestContiguousSeq.
    // Bit i corresponds to SN (highestContiguousSeq + 1 + i).
    // Length capped at RTPS_READER_MAX_BITMAP_BITS.
    uint64_t receivedBitmap = 0;

    // Last heartbeat count seen from the writer, for dedup.
    // RTPS HEARTBEAT submessages carry a monotonically increasing count;
    // readers only need to respond with ACKNACK when the count advances
    // (or when the FINAL flag is clear).
    uint32_t lastHeartbeatCount = 0;
    bool hasSeenHeartbeat = false;

    // Monotonic count we put into outgoing ACKNACK submessages.
    uint32_t outgoingAckNackCount = 0;
};

// Decision returned by evaluateIncomingDataDecision.
enum class RTPSReaderDataAction : uint8_t
{
    Accept = 0,  // Deliver payload to user dispatch; update state
    Dedup,       // Already seen this SN; drop silently
    Drop,        // Out-of-window or invalid; drop
};

struct RTPSReaderDataFields
{
    uint64_t writerSeqNum = 0;
};

// Pure decision: should this incoming DATA sample be delivered, deduped, or dropped?
// Does not mutate state; caller must call applyAcceptedDataToReaderState(...) on Accept.
RTPSReaderDataAction evaluateIncomingDataDecision(
    const RTPSReaderWriterState& state,
    const RTPSReaderDataFields& data);

// Mutates state to record that a DATA sample with `writerSeqNum` has been accepted.
// Advances highestContiguousSeq across any newly-contiguous range.
void applyAcceptedDataToReaderState(
    RTPSReaderWriterState& state,
    uint64_t writerSeqNum);

struct RTPSReaderHeartbeatFields
{
    uint64_t firstSN = 0;
    uint64_t lastSN = 0;
    uint32_t count = 0;
    bool finalFlag = false;      // If set, writer does not require a reply
    bool livelinessFlag = false; // If set, HB is liveliness-only
};

// Decision returned by evaluateIncomingHeartbeatDecision.
struct RTPSReaderHeartbeatDecision
{
    bool sendAckNack = false;
    // ACKNACK bitmap base = first SN we are *missing* (== highestContiguousSeq + 1).
    uint64_t ackNackBase = 0;
    // Bitmap width (number of bits after base) and contents.
    uint32_t ackNackNumBits = 0;
    uint64_t ackNackBitmap = 0;
    // Count to put into outgoing ACKNACK submessage (state.outgoingAckNackCount + 1).
    uint32_t ackNackCount = 0;
};

// Pure decision: given incoming HEARTBEAT and current reader state,
// compute whether to emit ACKNACK and with what contents.
// Does not mutate state; caller calls applyEmittedAckNackToReaderState(...) after sending.
RTPSReaderHeartbeatDecision evaluateIncomingHeartbeatDecision(
    const RTPSReaderWriterState& state,
    const RTPSReaderHeartbeatFields& hb);

// Mutates state to record that a HEARTBEAT has been processed and (optionally) an
// ACKNACK was emitted. Always updates lastHeartbeatCount; increments
// outgoingAckNackCount if the decision asked for an ACKNACK send.
void applyHeartbeatProcessedToReaderState(
    RTPSReaderWriterState& state,
    const RTPSReaderHeartbeatFields& hb,
    bool ackNackSent);

}
