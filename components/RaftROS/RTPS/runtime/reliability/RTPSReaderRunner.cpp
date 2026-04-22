/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Reader Runner - orchestration around pure reader-runtime decisions.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "runtime/reliability/RTPSReaderRunner.h"

namespace
{

inline uint32_t readLE32_local(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

inline uint16_t readLE16_local(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// RTPS HEARTBEAT submessage content layout (DDSI-RTPS §8.3.7.5):
//   readerEntityId (4) + writerEntityId (4) + firstSN (8) + lastSN (8) + count (4) = 28 bytes
static constexpr uint32_t HEARTBEAT_CONTENT_MIN_LEN = 28;

// HEARTBEAT flags (bit 0 is endianness):
static constexpr uint8_t HEARTBEAT_FLAG_FINAL      = 0x02; // F
static constexpr uint8_t HEARTBEAT_FLAG_LIVELINESS = 0x04; // L

// RTPS DATA submessage content layout (DDSI-RTPS §8.3.7.2):
//   extraFlags (2) + octetsToInlineQos (2) + readerEntityId (4) + writerEntityId (4)
//   + writerSN (8) + [inlineQos ...] + [serializedPayload ...]
static constexpr uint32_t DATA_CONTENT_HEADER_LEN = 20;

// DATA flags (bit 0 is endianness):
static constexpr uint8_t DATA_FLAG_INLINE_QOS = 0x02; // Q
static constexpr uint8_t DATA_FLAG_DATA       = 0x04; // D (payload present)
static constexpr uint8_t DATA_FLAG_KEY        = 0x08; // K (dispose/unregister)

} // anonymous namespace

bool RTPSReaderRunner_parseHeartbeat(
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    RTPSReaderHeartbeatParsed& out)
{
    if (!pContent || contentLen < HEARTBEAT_CONTENT_MIN_LEN)
        return false;

    out.readerEID = pContent + 0;
    out.writerEID = pContent + 4;
    out.flags = flags;

    const uint32_t firstSNHigh = readLE32_local(pContent + 8);
    const uint32_t firstSNLow  = readLE32_local(pContent + 12);
    const uint32_t lastSNHigh  = readLE32_local(pContent + 16);
    const uint32_t lastSNLow   = readLE32_local(pContent + 20);
    const uint32_t count       = readLE32_local(pContent + 24);

    out.fields.firstSN = ((uint64_t)firstSNHigh << 32) | (uint64_t)firstSNLow;
    out.fields.lastSN  = ((uint64_t)lastSNHigh  << 32) | (uint64_t)lastSNLow;
    out.fields.count   = count;
    out.fields.finalFlag      = (flags & HEARTBEAT_FLAG_FINAL) != 0;
    out.fields.livelinessFlag = (flags & HEARTBEAT_FLAG_LIVELINESS) != 0;

    return true;
}

bool RTPSReaderRunner_parseData(
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    RTPSReaderDataParsed& out)
{
    if (!pContent || contentLen < DATA_CONTENT_HEADER_LEN)
        return false;

    const uint16_t octetsToInlineQos = readLE16_local(pContent + 2);

    out.readerEID = pContent + 4;
    out.writerEID = pContent + 8;
    out.flags = flags;

    const uint32_t writerSNHigh = readLE32_local(pContent + 12);
    const uint32_t writerSNLow  = readLE32_local(pContent + 16);
    out.fields.writerSeqNum = ((uint64_t)writerSNHigh << 32) | (uint64_t)writerSNLow;

    // Locate payload start. octetsToInlineQos is measured from the start of
    // readerEntityId (content offset 4) to the start of inlineQos or payload.
    // i.e. inlineQos/payload begins at content offset (4 + octetsToInlineQos).
    uint32_t pos = 4u + (uint32_t)octetsToInlineQos;
    if (pos > contentLen)
        return false;

    // Skip inline QoS if present. Inline QoS is a ParameterList terminated by
    // PID_SENTINEL (0x0001, length 0). We scan until sentinel or end-of-content.
    if (flags & DATA_FLAG_INLINE_QOS)
    {
        while (pos + 4 <= contentLen)
        {
            const uint16_t pid    = readLE16_local(pContent + pos);
            const uint16_t pidLen = readLE16_local(pContent + pos + 2);
            pos += 4;
            if (pid == 0x0001) // PID_SENTINEL
                break;
            if ((uint64_t)pos + pidLen > contentLen)
                return false;
            pos += pidLen;
        }
    }

    if (flags & DATA_FLAG_DATA)
    {
        if (pos > contentLen)
            return false;
        out.payload = pContent + pos;
        out.payloadLen = contentLen - pos;
    }
    else
    {
        out.payload = nullptr;
        out.payloadLen = 0;
    }

    return true;
}

void RTPSReaderRunner_onHeartbeat(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    const RTPSReaderRunnerCallbacks& callbacks,
    void* userCtx)
{
    RTPSReaderHeartbeatParsed parsed;
    if (!RTPSReaderRunner_parseHeartbeat(pContent, contentLen, flags, parsed))
        return;

    if (!callbacks.resolveState)
        return;

    RTPSReaderWriterState* state = callbacks.resolveState(userCtx, srcGuidPrefix, parsed.writerEID);
    if (!state)
        return;

    const RTPSReaderHeartbeatDecision decision =
        RaftRuntime::RTPS::Runtime::Reader::evaluateIncomingHeartbeatDecision(
            *state, parsed.fields);

    if (callbacks.onHeartbeatParsed)
        callbacks.onHeartbeatParsed(userCtx, parsed, decision);

    bool sent = false;
    if (decision.sendAckNack && callbacks.sendAckNack)
    {
        callbacks.sendAckNack(userCtx, srcGuidPrefix, parsed.readerEID, parsed.writerEID, decision);
        sent = true;
    }

    RaftRuntime::RTPS::Runtime::Reader::applyHeartbeatProcessedToReaderState(
        *state, parsed.fields, sent);
}

void RTPSReaderRunner_onData(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    uint8_t flags,
    const RTPSReaderRunnerCallbacks& callbacks,
    void* userCtx)
{
    RTPSReaderDataParsed parsed;
    if (!RTPSReaderRunner_parseData(pContent, contentLen, flags, parsed))
        return;

    if (!callbacks.resolveState)
        return;

    RTPSReaderWriterState* state = callbacks.resolveState(userCtx, srcGuidPrefix, parsed.writerEID);
    if (!state)
        return;

    const RTPSReaderDataAction action =
        RaftRuntime::RTPS::Runtime::Reader::evaluateIncomingDataDecision(
            *state, parsed.fields);

    if (callbacks.onDataDecision)
        callbacks.onDataDecision(userCtx, parsed, action);

    if (action == RTPSReaderDataAction::Accept)
    {
        RaftRuntime::RTPS::Runtime::Reader::applyAcceptedDataToReaderState(
            *state, parsed.fields.writerSeqNum);

        if (callbacks.dispatchData)
            callbacks.dispatchData(userCtx, srcGuidPrefix, parsed);
    }
}
