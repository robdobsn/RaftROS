#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Reader Runner - orchestration layer around the pure reader-runtime decisions.
//
// Mirrors the RTPSAckNackRunner pattern on the writer side:
//   - Parses submessage content into structured fields
//   - Calls the pure decision helpers in RTPSReaderRuntime
//   - Invokes wrapper callbacks to do IO (send ACKNACK, dispatch decoded payload)
//
// No sockets, no CDR decode, no per-wrapper state — all delegated via callbacks.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

#include "runtime/reliability/RTPSReaderRuntime.h"

using RTPSReaderWriterState =
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderWriterState;
using RTPSReaderDataAction =
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderDataAction;
using RTPSReaderDataFields =
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderDataFields;
using RTPSReaderHeartbeatFields =
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatFields;
using RTPSReaderHeartbeatDecision =
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatDecision;

// Submessage fields as they appear on the wire. Entity IDs are 4-byte pointers
// into the incoming packet buffer (valid only for the duration of the callback).
struct RTPSReaderHeartbeatParsed
{
    const uint8_t* readerEID = nullptr; // ENTITYID_UNKNOWN means "any reader"
    const uint8_t* writerEID = nullptr;
    uint8_t flags = 0;                  // Raw submessage flags byte
    RTPSReaderHeartbeatFields fields;
};

struct RTPSReaderDataParsed
{
    const uint8_t* readerEID = nullptr;
    const uint8_t* writerEID = nullptr;
    uint8_t flags = 0;                  // Raw submessage flags byte (D/K/Q flags live here)
    RTPSReaderDataFields fields;
    const uint8_t* payload = nullptr;   // May be null if inline-QoS-only
    uint32_t payloadLen = 0;
};

// Pure parsers. Return true on success; on failure `out` is untouched.
bool RTPSReaderRunner_parseHeartbeat(
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    RTPSReaderHeartbeatParsed& out);

bool RTPSReaderRunner_parseData(
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    RTPSReaderDataParsed& out);

// Callback invoked to look up (or lazily create) the reader state for the
// (srcGuidPrefix, writerEID) pair. Return nullptr to drop the submessage.
typedef RTPSReaderWriterState* (*RTPSReaderRunnerResolveStateFn)(
    void* userCtx,
    const uint8_t* srcGuidPrefix,
    const uint8_t* writerEID);

// Callback to emit an ACKNACK submessage. Implementation owns wire assembly and socket IO.
typedef void (*RTPSReaderRunnerSendAckNackFn)(
    void* userCtx,
    const uint8_t* srcGuidPrefix,
    const uint8_t* readerEID,
    const uint8_t* writerEID,
    const RTPSReaderHeartbeatDecision& decision);

// Callback to deliver an accepted DATA sample to user dispatch (CDR decode + topic dispatch).
typedef void (*RTPSReaderRunnerDispatchDataFn)(
    void* userCtx,
    const uint8_t* srcGuidPrefix,
    const RTPSReaderDataParsed& parsed);

// Optional observers (logging, metrics, tests).
typedef void (*RTPSReaderRunnerOnHeartbeatFn)(
    void* userCtx,
    const RTPSReaderHeartbeatParsed& parsed,
    const RTPSReaderHeartbeatDecision& decision);

typedef void (*RTPSReaderRunnerOnDataFn)(
    void* userCtx,
    const RTPSReaderDataParsed& parsed,
    RTPSReaderDataAction action);

struct RTPSReaderRunnerCallbacks
{
    RTPSReaderRunnerResolveStateFn resolveState = nullptr;
    RTPSReaderRunnerSendAckNackFn sendAckNack = nullptr;
    RTPSReaderRunnerDispatchDataFn dispatchData = nullptr;
    RTPSReaderRunnerOnHeartbeatFn onHeartbeatParsed = nullptr;
    RTPSReaderRunnerOnDataFn onDataDecision = nullptr;
};

// Orchestrators. `pContent`/`contentLen` are the submessage content (after the
// 4-byte submessage header), exactly as produced by RTPSMessage::parseSubmessage.
// `flags` is the raw submessage flags byte.
void RTPSReaderRunner_onHeartbeat(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    const RTPSReaderRunnerCallbacks& callbacks,
    void* userCtx);

void RTPSReaderRunner_onData(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    const RTPSReaderRunnerCallbacks& callbacks,
    void* userCtx);
