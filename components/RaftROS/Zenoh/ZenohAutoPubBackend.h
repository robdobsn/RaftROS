///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ZenohAutoPubBackend - Zenoh implementation of the auto-publish backend contract
//
// Implements the same three operations as RTPSAutoPubBackend (createPublisher,
// destroyPublisher, publish) over a ZenohTCPSession, so the shared AutoPub
// layer is unchanged: it hands over an AutoPubEndpointDesc ("publish this ROS
// topic/type with this QoS") and later a serialised sample, and never sees
// key expressions, liveliness tokens or sockets.  Exactly one backend is
// linked into a firmware image - selected at compile time, no vtable.
//
// Differences from RTPS that the contract has to absorb:
//
//  * Discovery is not per-peer.  RTPS announces each writer to each discovered
//    participant with SEDP; Zenoh declares one liveliness token per endpoint to
//    the router, which fans it out.  So there is no announceToPeer() /
//    disposeAtPeer() here - createPublisher() and destroyPublisher() carry the
//    whole visibility lifecycle.
//
//  * The session carries one outbound message at a time, and encoding a
//    declaration must not block the loop.  createPublisher() therefore only
//    *stages* a declaration (the key, token and GID are built up front, so
//    failure is reported immediately); service() sends staged work, one message
//    per call.  A slot is not publishable until its token has gone out.
//
//  * A dropped session invalidates every declaration the router held, so
//    service() re-stages all live slots when the session leaves Established.
//    Sequence numbers and entity ids are *not* reset - entity ids are never
//    reused, which keeps a re-declared endpoint distinguishable from the old.
//
// Rob Dobson 2026
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "ZenohNetworkMessage.h"
#include "ZenohROSCodec.h"
#include "ZenohROSIdentity.h"
#include "ZenohTCPSession.h"
#include "AutoPub/AutoPubClassMap.h"
#include "AutoPub/AutoPubEndpointDesc.h"
#include "AutoPub/AutoPubQoSProfile.h"
#include "AutoPub/AutoPubSampleRunner.h"

namespace RaftRuntime::Zenoh
{

/// @brief Slots, and the buffers each slot holds.  A slot keeps its key
/// expression and liveliness token for its whole life: the key is needed on
/// every publish and the token on every re-declare and interest reply, and
/// neither can be rebuilt without re-entering the format path.
/// Cost is CAPACITY * (KEY_MAX + TOKEN_MAX) bytes of the owning object.
static constexpr uint8_t ZENOH_AUTOPUB_CAPACITY = 16;
static constexpr size_t ZENOH_AUTOPUB_KEY_MAX = 256;
static constexpr size_t ZENOH_AUTOPUB_TOKEN_MAX = 448;

/// @brief Token id 1 is the node liveliness token, declared by the SysMod for
/// the node itself.  Endpoint tokens start above it, one fixed id per slot, so
/// an undeclare needs nothing but the slot index.
static constexpr uint32_t ZENOH_AUTOPUB_FIRST_TOKEN_ID = 2;

/// @brief Session and node identity the backend borrows from the SysMod.  All
/// are owned by the SysMod and must outlive the backend; the string fields are
/// the same node identity used for the node's own liveliness token, so every
/// endpoint token agrees with it.
struct ZenohAutoPubBackendDeps
{
    ZenohTCPSession* session = nullptr;     ///< Read every service/publish: may be re-started on link loss
    uint8_t* sendBuf = nullptr;             ///< Scratch for one network message (loop task only)
    uint32_t sendBufLen = 0;
    uint32_t domainId = 0;
    const char* sessionId = nullptr;        ///< Lower-case hex of the session identity, no leading zeros
    uint64_t nodeId = 0;
    const char* enclave = "/";
    const char* nodeNamespace = "/";
    const char* nodeName = nullptr;

    bool isValid() const
    {
        return session && sendBuf && sendBufLen >= 64 && sessionId && nodeName &&
               enclave && nodeNamespace;
    }
};

class ZenohAutoPubBackend
{
public:
    static constexpr uint8_t INVALID_SLOT = 0xFF;

    /// @brief Bind to the SysMod's session and node identity (loop task, once at setup)
    void setup(const ZenohAutoPubBackendDeps& deps) { _deps = deps; }

    /// @brief Create a publisher for the described endpoint.  Builds the key
    /// expression, liveliness token and publisher GID immediately so a bad
    /// descriptor fails here rather than silently later, then stages the
    /// declaration for service() to send.
    /// @return slot index, or INVALID_SLOT if the backend is unconfigured, the
    ///         registry is full, or the descriptor cannot be expressed as a
    ///         Zenoh endpoint.  The slot is reused after destroyPublisher().
    uint8_t createPublisher(const RaftRuntime::AutoPub::AutoPubEndpointDesc& desc)
    {
        if (!_deps.isValid() || !desc.isValid())
            return INVALID_SLOT;
        const char* typeHash = RaftRuntime::AutoPub::AutoPubClassMap_typeHash(desc.msgKind);
        if (!typeHash)
            return INVALID_SLOT;
        uint8_t slot = INVALID_SLOT;
        for (uint8_t index = 0; index < ZENOH_AUTOPUB_CAPACITY; ++index)
        {
            if (_entries[index].state == Entry::State::Free)
            {
                slot = index;
                break;
            }
        }
        if (slot == INVALID_SLOT)
            return INVALID_SLOT;

        Entry& entry = _entries[slot];
        const uint64_t entityId = _nextEntityId;
        const ZenohROSCodec::NodeIdentity node = nodeIdentity();
        const ZenohROSCodec::Endpoint endpoint{entityId, ZenohROSCodec::EndpointKind::Publisher,
            desc.topic, desc.type, typeHash, qosFor(desc.qosProfileId)};
        if (!ZenohROSCodec::formatTopicKey(entry.key, sizeof(entry.key), node.domainId,
                                           endpoint.topic, endpoint.wireType, endpoint.typeHash) ||
            !ZenohROSCodec::formatEndpointToken(entry.token, sizeof(entry.token), node, endpoint) ||
            !ZenohROSIdentity::deriveEndpointGid(node, endpoint, entry.gid))
        {
            entry.key[0] = '\0';
            entry.token[0] = '\0';
            return INVALID_SLOT;
        }
        entry.entityId = entityId;
        entry.sequence = 0;
        entry.state = Entry::State::PendingDeclare;
        ++_nextEntityId;
        return slot;
    }

    /// @brief Release a publisher's slot.  Safe to call with INVALID_SLOT.
    /// A slot whose token was never sent frees immediately; a declared one
    /// stays occupied until service() has sent the undeclare, so a subscriber
    /// never sees the endpoint linger for the token lease.
    void destroyPublisher(uint8_t slot)
    {
        if (slot >= ZENOH_AUTOPUB_CAPACITY)
            return;
        Entry& entry = _entries[slot];
        if (entry.state == Entry::State::Declared)
            entry.state = Entry::State::PendingUndeclare;
        else if (entry.state == Entry::State::PendingDeclare)
            release(entry);
    }

    /// @brief Publish one serialised sample on a declared endpoint.  One
    /// sequence number per accepted sample; a sample that is not sent does not
    /// consume one, so a subscriber sees no gap when the link stalls.
    /// `outPeersSent` is 0 or 1 - the router is the only peer, and it fans out.
    RaftRuntime::AutoPub::AutoPubPublishResult publish(
            uint8_t slot, const uint8_t* payload, uint32_t length,
            uint64_t* outSequence = nullptr, uint32_t* outPeersSent = nullptr)
    {
        using RaftRuntime::AutoPub::AutoPubPublishResult;
        if (outSequence)
            *outSequence = 0;
        if (outPeersSent)
            *outPeersSent = 0;
        if (!_deps.isValid() || slot >= ZENOH_AUTOPUB_CAPACITY || !payload || length == 0)
            return AutoPubPublishResult::InvalidHandle;
        Entry& entry = _entries[slot];
        if (entry.state == Entry::State::Free)
            return AutoPubPublishResult::InvalidHandle;
        if (length > ZenohNetworkMessage::MAX_PAYLOAD_SIZE)
            return AutoPubPublishResult::Oversized;
        if (_deps.session->state() != ZenohTCPSession::State::Established)
            return AutoPubPublishResult::Disconnected;
        // Not yet visible to subscribers (or being torn down): drop rather than
        // publish on a key nobody is matching.  Transient - service() clears it.
        if (entry.state != Entry::State::Declared || _deps.session->outputSize() != 0)
            return AutoPubPublishResult::QueueFull;

        uint8_t attachment[ZenohROSCodec::ATTACHMENT_SIZE];
        const ZenohROSCodec::Attachment metadata{static_cast<int64_t>(entry.sequence + 1),
            static_cast<int64_t>(_nowMs) * 1000000, entry.gid};
        if (!ZenohROSCodec::encodeAttachment(attachment, sizeof(attachment), metadata))
            return AutoPubPublishResult::SendFailed;
        const size_t msgLen = ZenohNetworkMessage::put(_deps.sendBuf, _deps.sendBufLen, entry.key,
                                                       payload, length, attachment, sizeof(attachment));
        if (msgLen == 0)
            return AutoPubPublishResult::Oversized;
        if (!_deps.session->sendNetworkMessage(_deps.sendBuf, msgLen, _nowMs))
            return AutoPubPublishResult::QueueFull;
        ++entry.sequence;
        ++_published;
        if (outSequence)
            *outSequence = entry.sequence;
        if (outPeersSent)
            *outPeersSent = 1;
        return AutoPubPublishResult::Accepted;
    }

    /// @brief Send at most one staged declaration or undeclaration.  Call once
    /// per loop from the task that owns the session: the session carries one
    /// outbound message at a time, so staged work is sent round-robin from a
    /// rotating cursor and no slot starves behind a busy one.
    /// Also supplies the clock used for sample source timestamps, which are
    /// therefore accurate to one loop period.
    /// @return true if a message was sent
    bool service(uint64_t nowMs)
    {
        _nowMs = nowMs;
        if (!_deps.isValid())
            return false;
        const auto state = _deps.session->state();
        if (state != ZenohTCPSession::State::Established)
        {
            // The router drops every declaration when the session goes: stage
            // the live ones again, and let the dying ones go without an
            // undeclare nobody would receive.
            if (_sessionWasEstablished)
            {
                for (auto& entry : _entries)
                {
                    if (entry.state == Entry::State::Declared)
                    {
                        entry.state = Entry::State::PendingDeclare;
                        ++_redeclares;
                    }
                    else if (entry.state == Entry::State::PendingUndeclare)
                        release(entry);
                }
                _sessionWasEstablished = false;
            }
            return false;
        }
        _sessionWasEstablished = true;
        if (_deps.session->outputSize() != 0)
            return false;
        for (uint8_t offset = 0; offset < ZENOH_AUTOPUB_CAPACITY; ++offset)
        {
            const uint8_t slot = static_cast<uint8_t>((_cursor + offset) % ZENOH_AUTOPUB_CAPACITY);
            Entry& entry = _entries[slot];
            if (entry.state != Entry::State::PendingDeclare && entry.state != Entry::State::PendingUndeclare)
                continue;
            const bool declaring = entry.state == Entry::State::PendingDeclare;
            const size_t msgLen = declaring ?
                ZenohNetworkMessage::declareToken(_deps.sendBuf, _deps.sendBufLen, tokenId(slot), entry.token) :
                ZenohNetworkMessage::undeclareToken(_deps.sendBuf, _deps.sendBufLen, tokenId(slot));
            if (msgLen == 0)
            {
                // Unsendable token: free the slot rather than retry forever
                release(entry);
                ++_declareFailures;
                continue;
            }
            if (!_deps.session->sendNetworkMessage(_deps.sendBuf, msgLen, nowMs))
                return false;
            if (declaring)
                entry.state = Entry::State::Declared;
            else
                release(entry);
            _cursor = static_cast<uint8_t>((slot + 1) % ZENOH_AUTOPUB_CAPACITY);
            return true;
        }
        return false;
    }

    /// @brief Whether a slot's token has been sent, i.e. the endpoint is
    /// visible to subscribers and publishable
    bool isDeclared(uint8_t slot) const
    {
        return slot < ZENOH_AUTOPUB_CAPACITY && _entries[slot].state == Entry::State::Declared;
    }

    /// @brief Liveliness token id for a slot - fixed for the slot's life
    static uint32_t tokenId(uint8_t slot) { return ZENOH_AUTOPUB_FIRST_TOKEN_ID + slot; }

    /// @brief Liveliness token key for a slot, for the SysMod's interest
    /// replies (a router asking what this node holds must be told about
    /// endpoint tokens as well as the node token).
    /// @return NUL-terminated token, or nullptr if the slot holds none
    const char* tokenKeyForSlot(uint8_t slot) const
    {
        if (!isDeclared(slot))
            return nullptr;
        return _entries[slot].token;
    }

    /// @brief Key expression a slot publishes on (diagnostics / tests)
    const char* topicKeyForSlot(uint8_t slot) const
    {
        if (slot >= ZENOH_AUTOPUB_CAPACITY || _entries[slot].state == Entry::State::Free)
            return nullptr;
        return _entries[slot].key;
    }

    /// @brief Publisher GID a slot puts in every sample attachment
    const ZenohROSIdentity::Gid* gidForSlot(uint8_t slot) const
    {
        if (slot >= ZENOH_AUTOPUB_CAPACITY || _entries[slot].state == Entry::State::Free)
            return nullptr;
        return &_entries[slot].gid;
    }

    uint8_t capacity() const { return ZENOH_AUTOPUB_CAPACITY; }

    /// @brief Slots holding an endpoint, declared or not
    uint8_t inUseCount() const
    {
        uint8_t count = 0;
        for (const auto& entry : _entries)
            count += entry.state != Entry::State::Free ? 1 : 0;
        return count;
    }

    /// @brief Slots waiting for service() to send a declare or undeclare
    uint8_t pendingCount() const
    {
        uint8_t count = 0;
        for (const auto& entry : _entries)
            count += (entry.state == Entry::State::PendingDeclare ||
                      entry.state == Entry::State::PendingUndeclare) ? 1 : 0;
        return count;
    }

    struct Stats
    {
        uint32_t published = 0;         ///< Samples accepted by the session
        uint32_t redeclares = 0;        ///< Endpoints re-staged after a session drop
        uint32_t declareFailures = 0;   ///< Slots dropped because their token would not encode
    };
    Stats stats() const { return {_published, _redeclares, _declareFailures}; }

private:
    struct Entry
    {
        enum class State : uint8_t { Free, PendingDeclare, Declared, PendingUndeclare };
        State state = State::Free;
        uint64_t entityId = 0;
        uint64_t sequence = 0;
        ZenohROSIdentity::Gid gid{};
        char key[ZENOH_AUTOPUB_KEY_MAX] = {};
        char token[ZENOH_AUTOPUB_TOKEN_MAX] = {};
    };

    static void release(Entry& entry)
    {
        entry.state = Entry::State::Free;
        entry.key[0] = '\0';
        entry.token[0] = '\0';
        entry.sequence = 0;
    }

    ZenohROSCodec::NodeIdentity nodeIdentity() const
    {
        return {_deps.domainId, _deps.sessionId, _deps.nodeId, _deps.enclave,
                _deps.nodeNamespace, _deps.nodeName};
    }

    static ZenohROSCodec::QoS qosFor(RaftRuntime::AutoPub::AutoPubQoSProfileId profileId)
    {
        const auto profile = RaftRuntime::AutoPub::AutoPubQoSProfile_get(profileId);
        ZenohROSCodec::QoS qos;
        qos.reliability = profile.reliability == RaftRuntime::AutoPub::AUTOPUB_RELIABILITY_RELIABLE ?
            ZenohROSCodec::Reliability::Reliable : ZenohROSCodec::Reliability::BestEffort;
        qos.durability = profile.durability == RaftRuntime::AutoPub::AUTOPUB_DURABILITY_TRANSIENT_LOCAL ?
            ZenohROSCodec::Durability::TransientLocal : ZenohROSCodec::Durability::Volatile;
        qos.depth = profile.historyDepth;
        return qos;
    }

    ZenohAutoPubBackendDeps _deps;
    Entry _entries[ZENOH_AUTOPUB_CAPACITY];
    uint64_t _nextEntityId = ZENOH_AUTOPUB_FIRST_TOKEN_ID;
    uint64_t _nowMs = 0;
    uint8_t _cursor = 0;
    bool _sessionWasEstablished = false;
    uint32_t _published = 0;
    uint32_t _redeclares = 0;
    uint32_t _declareFailures = 0;
};

} // namespace RaftRuntime::Zenoh
