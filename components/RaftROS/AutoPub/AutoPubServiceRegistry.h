/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubServiceRegistry - the transport-neutral half of serving ROS 2 services
//
// A SysMod hands this registry each request the transport delivers and asks
// it, once per loop pass, what to send next.  Everything the transport does
// not decide lives here: which service a request is for, whether the handler
// answers now or later, how many handlers run in one pass, how long a deferred
// request may wait, and what the reply is.  The transport supplies the wire:
// on Zenoh a RESPONSE then a RESPONSE_FINAL, addressed by request id and key.
//
// Why it is shaped like this - the two ways a service can break the Raft
// main-loop contract:
//
//   * A handler that blocks.  A publish is driven by us, on our schedule; a
//     request is driven by a client, on theirs, and the handler runs on the
//     loop task.  So a handler either answers from state it already holds and
//     returns Replied, or returns Deferred and the application completes the
//     request later, from the loop, when the answer exists.  Nothing here
//     waits.
//
//   * A client that floods.  Every reply is a datagram (~0.8 ms of loop time
//     on the ESP32-S3).  Requests are held in a small table; a request that
//     arrives when the table is full is refused with an error at once, and at
//     most `dispatchBudget` handlers run per pass.  The loop's cost per pass is
//     bounded whatever a client does.
//
// No RaftCore dependency: fixed buffers, std::function handlers, a clock the
// caller passes in - so it is tested on the host like the pool and the codec.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>

#include "AutoPubServiceCodec.h"

namespace RaftRuntime::AutoPub
{

static constexpr uint32_t AUTOPUB_SERVICE_NAME_MAX = 64;
static constexpr uint32_t AUTOPUB_SERVICE_ATTACHMENT_SIZE = 33;

/// @brief What a handler decided
enum class AutoPubServiceOutcome : uint8_t
{
    Replied,        ///< The reply is filled in; send it
    Deferred,       ///< Keep the request; the application will complete() it
    Refused,        ///< Send an error with the reason
};

/// @brief One request, as a handler sees it
struct AutoPubServiceRequest
{
    uint8_t slot = 0;                       ///< Which service
    uint32_t token = 0;                     ///< Handle for complete() when deferred
    AutoPubServiceRequestFields fields;     ///< Decoded request
};

/// @brief The reply a handler (or a later complete()) fills in
struct AutoPubServiceReply
{
    AutoPubServiceResponseFields fields;
    const char* reason = "";                ///< For Refused
};

using AutoPubServiceHandler = std::function<AutoPubServiceOutcome(const AutoPubServiceRequest&, AutoPubServiceReply&)>;

/// @brief What the transport should send next, if anything
struct AutoPubServiceSend
{
    enum class Kind : uint8_t { None, Response, Error, Final };
    Kind kind = Kind::None;
    uint8_t slot = 0;
    uint32_t requestId = 0;
    const uint8_t* payload = nullptr;       ///< Response: encoded CDR
    uint32_t payloadLen = 0;
    const uint8_t* attachment = nullptr;    ///< Response: 33 bytes, request sequence echoed
    const char* reason = "";                ///< Error
};

/// @tparam CAPACITY services this device can hold
/// @tparam INFLIGHT requests held at once, across all services
/// @tparam REPLY_MAX bytes of encoded response kept per in-flight request
/// @tparam REQUEST_MAX bytes of a Raw request kept until its handler runs
template <uint8_t CAPACITY = 4, uint8_t INFLIGHT = 4, uint32_t REPLY_MAX = 256, uint32_t REQUEST_MAX = 64>
class AutoPubServiceRegistry
{
public:
    static constexpr uint8_t INVALID_SLOT = 0xFF;
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 5000;

    /// @brief Register a service.  Fails if the table is full, the type is
    /// not one the codec knows, or the name does not fit.
    /// @return slot, or INVALID_SLOT
    uint8_t add(const char* rosName, AutoPubServiceKind kind, AutoPubServiceHandler handler)
    {
        if (!rosName || !*rosName || kind == AutoPubServiceKind::Unknown || !handler ||
            std::strlen(rosName) >= AUTOPUB_SERVICE_NAME_MAX)
            return INVALID_SLOT;
        for (uint8_t slot = 0; slot < CAPACITY; ++slot)
        {
            if (_services[slot].inUse && std::strcmp(_services[slot].rosName, rosName) == 0)
                return INVALID_SLOT;                    // one server per name
        }
        for (uint8_t slot = 0; slot < CAPACITY; ++slot)
        {
            Service& service = _services[slot];
            if (service.inUse)
                continue;
            std::strcpy(service.rosName, rosName);
            service.kind = kind;
            service.handler = std::move(handler);
            service.inUse = true;
            return slot;
        }
        return INVALID_SLOT;
    }

    /// @brief Remove a service; requests in flight for it are dropped
    void remove(uint8_t slot)
    {
        if (slot >= CAPACITY || !_services[slot].inUse)
            return;
        for (auto& entry : _inflight)
            if (entry.state != Entry::State::Free && entry.slot == slot)
                entry.state = Entry::State::Free;
        _services[slot] = Service();
    }

    /// @brief Accept a request the transport delivered.  Copies what the
    /// reply will need (the request id and the client's attachment) and the
    /// decoded fields; the transport's buffer can be reused as soon as this
    /// returns.  A full table or an undecodable payload gets an error reply
    /// queued in the same call - the client hears back either way.
    /// @param slot the service (the transport resolved it from the key)
    /// @return true if the request was queued for a handler
    bool accept(uint8_t slot, uint32_t requestId, const uint8_t* payload, uint32_t payloadLen,
                const uint8_t* attachment, uint32_t attachmentLen, uint32_t clientTimeoutMs, uint64_t nowMs)
    {
        if (slot >= CAPACITY || !_services[slot].inUse)
            return false;
        Entry* entry = freeEntry();
        if (!entry)
        {
            ++_refusedBusy;
            // Refuse through a spare that is reserved for exactly this
            entry = &_busyReply;
            if (entry->state != Entry::State::Free)
                return false;                           // already refusing one; drop
            begin(*entry, slot, requestId, attachment, attachmentLen, nowMs, clientTimeoutMs);
            entry->reason = "busy";
            entry->state = Entry::State::SendError;
            return false;
        }
        begin(*entry, slot, requestId, attachment, attachmentLen, nowMs, clientTimeoutMs);
        // A Raw request is kept whole for its handler; one that does not fit
        // is refused here, not truncated
        const bool rawFits = _services[slot].kind != AutoPubServiceKind::Raw ||
                             (payload && payloadLen > 0 && payloadLen <= REQUEST_MAX);
        if (!rawFits || !AutoPubServiceCodec_decodeRequest(_services[slot].kind, payload, payloadLen, entry->fields))
        {
            ++_refusedBad;
            entry->reason = "bad request";
            entry->state = Entry::State::SendError;
            return false;
        }
        entry->requestLen = 0;
        if (_services[slot].kind == AutoPubServiceKind::Raw)
        {
            std::memcpy(entry->request, payload, payloadLen);
            entry->requestLen = payloadLen;
        }
        entry->state = Entry::State::Queued;
        ++_accepted;
        return true;
    }

    /// @brief Run handlers for queued requests, at most `dispatchBudget` of
    /// them, and time out deferred ones.  Once per loop pass.
    void service(uint64_t nowMs, uint8_t dispatchBudget)
    {
        for (auto& entry : _inflight)
        {
            if (entry.state == Entry::State::Deferred && nowMs >= entry.deadlineMs)
            {
                ++_timedOut;
                entry.reason = "timeout";
                entry.state = Entry::State::SendError;
            }
        }
        uint8_t dispatched = 0;
        for (auto& entry : _inflight)
        {
            if (dispatched >= dispatchBudget)
                break;
            if (entry.state != Entry::State::Queued)
                continue;
            ++dispatched;
            AutoPubServiceRequest request;
            request.slot = entry.slot;
            request.token = entry.token;
            request.fields = entry.fields;
            request.fields.raw = entry.request;
            request.fields.rawLen = entry.requestLen;
            AutoPubServiceReply reply;
            const AutoPubServiceOutcome outcome = _services[entry.slot].handler(request, reply);
            switch (outcome)
            {
                case AutoPubServiceOutcome::Replied:
                    finish(entry, reply);
                    break;
                case AutoPubServiceOutcome::Deferred:
                    ++_deferred;
                    entry.state = Entry::State::Deferred;
                    break;
                case AutoPubServiceOutcome::Refused:
                default:
                    ++_refusedByHandler;
                    entry.reason = reply.reason ? reply.reason : "refused";
                    entry.state = Entry::State::SendError;
                    break;
            }
        }
    }

    /// @brief Complete a deferred request (loop task only)
    /// @return false if the token is unknown - already timed out, or never deferred
    bool complete(uint32_t token, const AutoPubServiceReply& reply)
    {
        for (auto& entry : _inflight)
        {
            if (entry.state == Entry::State::Deferred && entry.token == token)
            {
                finish(entry, reply);
                return true;
            }
        }
        return false;
    }

    /// @brief What to send next.  The transport sends it and calls sent().
    /// A response is followed by its final on the next call, so a reply takes
    /// two passes - one message per pass, like everything else.
    AutoPubServiceSend next() const
    {
        AutoPubServiceSend send;
        const Entry* entry = sending();
        if (!entry)
            return send;
        send.slot = entry->slot;
        send.requestId = entry->requestId;
        switch (entry->state)
        {
            case Entry::State::SendResponse:
                send.kind = AutoPubServiceSend::Kind::Response;
                send.payload = entry->reply;
                send.payloadLen = entry->replyLen;
                send.attachment = entry->attachment;
                break;
            case Entry::State::SendError:
                send.kind = AutoPubServiceSend::Kind::Error;
                send.reason = entry->reason;
                break;
            case Entry::State::SendFinal:
                send.kind = AutoPubServiceSend::Kind::Final;
                break;
            default:
                break;
        }
        return send;
    }

    /// @brief The transport sent what next() described
    void sent()
    {
        Entry* entry = sending();
        if (!entry)
            return;
        if (entry->state == Entry::State::SendResponse || entry->state == Entry::State::SendError)
            entry->state = Entry::State::SendFinal;
        else if (entry->state == Entry::State::SendFinal)
        {
            ++_completed;
            entry->state = Entry::State::Free;
        }
    }

    // Lookups and diagnostics
    uint8_t capacity() const { return CAPACITY; }
    bool isInUse(uint8_t slot) const { return slot < CAPACITY && _services[slot].inUse; }
    const char* rosName(uint8_t slot) const { return isInUse(slot) ? _services[slot].rosName : nullptr; }
    AutoPubServiceKind kind(uint8_t slot) const { return isInUse(slot) ? _services[slot].kind : AutoPubServiceKind::Unknown; }
    uint8_t inUseCount() const
    {
        uint8_t n = 0;
        for (const auto& s : _services) n += s.inUse ? 1 : 0;
        return n;
    }
    uint8_t inflightCount() const
    {
        uint8_t n = 0;
        for (const auto& e : _inflight) n += e.state != Entry::State::Free ? 1 : 0;
        return n;
    }
    struct Stats
    {
        uint32_t accepted = 0, completed = 0, deferred = 0, timedOut = 0;
        uint32_t refusedBusy = 0, refusedBad = 0, refusedByHandler = 0;
    };
    Stats stats() const { return {_accepted, _completed, _deferred, _timedOut, _refusedBusy, _refusedBad, _refusedByHandler}; }

private:
    struct Service
    {
        bool inUse = false;
        char rosName[AUTOPUB_SERVICE_NAME_MAX] = {};
        AutoPubServiceKind kind = AutoPubServiceKind::Unknown;
        AutoPubServiceHandler handler;
    };
    struct Entry
    {
        enum class State : uint8_t { Free, Queued, Deferred, SendResponse, SendError, SendFinal };
        State state = State::Free;
        uint8_t slot = 0;
        uint32_t token = 0;
        uint32_t requestId = 0;
        uint64_t deadlineMs = 0;
        AutoPubServiceRequestFields fields;
        uint8_t attachment[AUTOPUB_SERVICE_ATTACHMENT_SIZE] = {};
        uint8_t request[REQUEST_MAX] = {};      ///< Raw kind: the request CDR until dispatch
        uint32_t requestLen = 0;
        uint8_t reply[REPLY_MAX] = {};
        uint32_t replyLen = 0;
        const char* reason = "";
    };

    Entry* freeEntry()
    {
        for (auto& entry : _inflight)
            if (entry.state == Entry::State::Free)
                return &entry;
        return nullptr;
    }
    void begin(Entry& entry, uint8_t slot, uint32_t requestId, const uint8_t* attachment, uint32_t attachmentLen,
               uint64_t nowMs, uint32_t clientTimeoutMs)
    {
        entry.slot = slot;
        entry.requestId = requestId;
        entry.token = ++_nextToken;
        const uint32_t timeout = clientTimeoutMs ? (clientTimeoutMs < DEFAULT_TIMEOUT_MS ? clientTimeoutMs : DEFAULT_TIMEOUT_MS)
                                                 : DEFAULT_TIMEOUT_MS;
        entry.deadlineMs = nowMs + timeout;
        std::memset(entry.attachment, 0, sizeof(entry.attachment));
        if (attachment && attachmentLen == AUTOPUB_SERVICE_ATTACHMENT_SIZE)
            std::memcpy(entry.attachment, attachment, AUTOPUB_SERVICE_ATTACHMENT_SIZE);
        entry.replyLen = 0;
        entry.reason = "";
    }
    void finish(Entry& entry, const AutoPubServiceReply& reply)
    {
        entry.replyLen = AutoPubServiceCodec_encodeResponse(_services[entry.slot].kind, reply.fields,
                                                            entry.reply, REPLY_MAX);
        if (entry.replyLen == 0)
        {
            entry.reason = "reply too large";
            entry.state = Entry::State::SendError;
            return;
        }
        entry.state = Entry::State::SendResponse;
    }
    /// @brief The entry with something to send: the busy refusal first (it
    /// holds no table slot), then the oldest-staged reply
    const Entry* sending() const
    {
        if (_busyReply.state == Entry::State::SendError || _busyReply.state == Entry::State::SendFinal)
            return &_busyReply;
        for (const auto& entry : _inflight)
            if (entry.state == Entry::State::SendResponse || entry.state == Entry::State::SendError ||
                entry.state == Entry::State::SendFinal)
                return &entry;
        return nullptr;
    }
    Entry* sending() { return const_cast<Entry*>(static_cast<const AutoPubServiceRegistry*>(this)->sending()); }

    Service _services[CAPACITY];
    Entry _inflight[INFLIGHT];
    Entry _busyReply;                       ///< Spare for refusing when the table is full
    uint32_t _nextToken = 0;
    uint32_t _accepted = 0, _completed = 0, _deferred = 0, _timedOut = 0;
    uint32_t _refusedBusy = 0, _refusedBad = 0, _refusedByHandler = 0;
};

} // namespace RaftRuntime::AutoPub
