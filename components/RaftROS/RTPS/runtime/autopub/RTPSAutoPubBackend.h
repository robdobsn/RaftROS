/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubBackend - RTPS implementation of the auto-publish backend contract
//
// Owns everything transport-specific about an auto-published endpoint: the
// dynamic writer registry (slots, entity ids, sequence numbers) and the send
// path to discovered participants.  The shared layer hands it an
// AutoPubEndpointDesc ("publish this ROS topic/type with this QoS") and later
// a serialised sample; it never sees entity ids, GUIDs or sockets.
//
// A Zenoh backend implements the same three operations (createPublisher,
// destroyPublisher, publish) and is selected at compile time - there is no
// vtable and only one backend is linked into a firmware image.
//
// SEDP announce/dispose still runs in the SysMod (it shares the participant's
// discovery machinery with /chatter and ros_discovery_info); the backend
// exposes entityIdForSlot() for it.  Moving announce behind the interface is
// the next step.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "AutoPub/AutoPubEndpointDesc.h"
#include "AutoPub/AutoPubSampleRunner.h"
#include "RTPSAutoPubLifecycle.h"
#include "RTPSAutoPubSampleEmitter.h"
#include "runtime/announce/SEDPHandler.h"
#include "runtime/core/RTPSParticipant.h"
#include "runtime/discovery/SPDPHandler.h"

namespace RaftRuntime {
namespace RTPS {
namespace Runtime {
namespace AutoPub {

/// @brief RTPS runtime objects the backend borrows from the SysMod.  All are
/// owned by the SysMod and must outlive the backend.
struct RTPSAutoPubBackendDeps
{
    RTPSParticipant* participant = nullptr;
    SEDPHandler* sedpHandler = nullptr;
    std::vector<DiscoveredParticipant>* discovered = nullptr;
    const int* userDataSock = nullptr;      ///< Read each publish: sockets are recreated on reconnect
    uint8_t* sendBuf = nullptr;             ///< Scratch for one datagram (loop task only)
    uint32_t sendBufLen = 0;
    const int* metatrafficSock = nullptr;   ///< Discovery (SEDP announce/dispose)
    uint8_t* metaSendBuf = nullptr;         ///< Scratch for one discovery datagram
    uint32_t metaSendBufLen = 0;
    const uint32_t* myIpAddr = nullptr;     ///< Announced in SEDP locators

    bool isValid() const
    {
        return participant && sedpHandler && discovered && userDataSock && sendBuf && (sendBufLen > 0);
    }

    bool isDiscoveryValid() const
    {
        return isValid() && metatrafficSock && metaSendBuf && (metaSendBufLen > 0) && myIpAddr;
    }
};

class RTPSAutoPubBackend
{
public:
    static constexpr uint8_t INVALID_SLOT = 0xFF;

    /// @brief Bind to the SysMod's RTPS objects (loop task, once at setup)
    void setup(const RTPSAutoPubBackendDeps& deps) { _deps = deps; }

    /// @brief Create a publisher for the described endpoint.
    /// @return slot index, or INVALID_SLOT if the registry is full or the
    ///         descriptor is unusable.  The slot is the opaque reference the
    ///         caller keeps; it is reused after destroyPublisher().
    uint8_t createPublisher(const RaftRuntime::AutoPub::AutoPubEndpointDesc& desc)
    {
        if (!desc.isValid())
            return INVALID_SLOT;
        const RTPSDynamicWriterKey key{desc.deviceId.busNum, desc.deviceId.address, desc.deviceId.subIndex};
        uint8_t entityId[4] = {0};
        const int slot = _lifecycle.attach(key, desc.topic, desc.type, entityId,
                                           static_cast<uint8_t>(desc.qosProfileId));
        return (slot < 0) ? INVALID_SLOT : (uint8_t)slot;
    }

    /// @brief Release a publisher's slot.  Safe to call with INVALID_SLOT.
    void destroyPublisher(uint8_t slot)
    {
        if (slot != INVALID_SLOT)
            _lifecycle.detachSlot(slot);
    }

    /// @brief Publish one serialised sample to every discovered participant.
    /// Best-effort (VOLATILE) fan-out: one sequence number per non-empty
    /// sample even with no peers, as the RTPS writer semantics require.
    RaftRuntime::AutoPub::AutoPubPublishResult publish(
            uint8_t slot, const uint8_t* payload, uint32_t length,
            uint64_t* outSequence = nullptr, uint32_t* outPeersSent = nullptr)
    {
        using RaftRuntime::AutoPub::AutoPubPublishResult;
        if (outSequence)
            *outSequence = 0;
        if (outPeersSent)
            *outPeersSent = 0;
        if (!_deps.isValid() || (slot == INVALID_SLOT))
            return AutoPubPublishResult::InvalidHandle;

        const RTPSAutoPubSampleEmission emission = RTPSAutoPubSampleEmitter_emit(
            _lifecycle.getMutable(slot), payload, length, *_deps.discovered,
            [this](const DiscoveredParticipant& remote, const RTPSDynamicWriterEntry& entry,
                   const uint8_t* cdr, uint32_t cdrLength, uint64_t sequence) {
                const uint32_t msgLen = _deps.sedpHandler->buildUserDataMessage(
                    _deps.sendBuf, _deps.sendBufLen,
                    *_deps.participant, remote.guidPrefix,
                    entry.entityId, cdr, cdrLength,
                    sequence, /*heartbeatCount=*/0,
                    /*firstSN=*/sequence);
                if (msgLen == 0)
                    return AutoPubPublishResult::Oversized;
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote.userDataPort);
                dest.sin_addr.s_addr = remote.ipAddr;
                const int sent = sendto(*_deps.userDataSock, _deps.sendBuf, msgLen, 0,
                                        (struct sockaddr*)&dest, sizeof(dest));
                return sent > 0 ? AutoPubPublishResult::Accepted : AutoPubPublishResult::SendFailed;
            });
        if (outSequence)
            *outSequence = emission.sequence;
        if (outPeersSent)
            *outPeersSent = emission.peersSent;
        return emission.result;
    }

    /// @brief Announce one endpoint to one discovered participant (SEDP
    /// publication DATA).  `heartbeatLastSN` is the participant-wide SEDP
    /// publications high-water mark, which the SysMod owns because it spans
    /// every announced endpoint, not just auto-published ones.
    bool announceToPeer(uint8_t slot, const DiscoveredParticipant& remote, uint64_t heartbeatLastSN)
    {
        if (!_deps.isDiscoveryValid())
            return false;
        const auto* pEntry = _lifecycle.get(slot);
        if (!pEntry || !pEntry->inUse || !pEntry->topic || !pEntry->type)
            return false;

        // The SEDP publications sequence is a fixed unique value per slot,
        // assigned at allocate(): re-announces reuse it so a remote treats
        // them as retransmits rather than new samples.
        const auto qos = RaftRuntime::AutoPub::AutoPubQoSProfile_get(
            static_cast<RaftRuntime::AutoPub::AutoPubQoSProfileId>(pEntry->qosProfileId));
        const uint32_t payloadLen = _deps.sedpHandler->buildPublicationMessage(
            _deps.metaSendBuf, _deps.metaSendBufLen,
            *_deps.participant, remote.guidPrefix,
            pEntry->entityId, pEntry->topic, pEntry->type,
            qos.reliability, qos.durability,
            pEntry->sedpSeqNum, *_deps.myIpAddr, /*heartbeatCount*/ 0,
            heartbeatLastSN);
        if (payloadLen == 0)
            return false;
        sendToPeerMetatraffic(remote, payloadLen);
        return true;
    }

    /// @brief Dispose one endpoint at one participant so it drops the writer
    /// promptly instead of waiting for the participant lease to expire.
    /// Must run before the slot is released - the live entity id is needed.
    bool disposeAtPeer(uint8_t slot, const DiscoveredParticipant& remote, uint64_t disposeSeq)
    {
        if (!_deps.isDiscoveryValid())
            return false;
        const auto* pEntry = _lifecycle.get(slot);
        if (!pEntry || !pEntry->inUse)
            return false;
        const uint32_t msgLen = _deps.sedpHandler->buildPublicationDisposeMessage(
            _deps.metaSendBuf, _deps.metaSendBufLen,
            *_deps.participant, remote.guidPrefix,
            pEntry->entityId, disposeSeq);
        if (msgLen == 0)
            return false;
        return sendToPeerMetatraffic(remote, msgLen) > 0;
    }

    /// @brief Entity id for a slot, for the SysMod's SEDP announce/dispose
    const uint8_t* entityIdForSlot(uint8_t slot) const
    {
        const auto* entry = _lifecycle.get(slot);
        return (entry && entry->inUse) ? entry->entityId : nullptr;
    }

    uint8_t capacity() const { return _lifecycle.capacity(); }
    uint8_t inUseCount() const { return _lifecycle.inUseCount(); }

    /// @brief Direct access for the SEDP announce paths that still live in the
    /// SysMod (topic/type strings, entity ids, heartbeat state)
    RTPSAutoPubLifecycle& lifecycle() { return _lifecycle; }
    const RTPSAutoPubLifecycle& lifecycle() const { return _lifecycle; }

private:
    int sendToPeerMetatraffic(const DiscoveredParticipant& remote, uint32_t len)
    {
        struct sockaddr_in dest = {};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(remote.metatrafficPort);
        dest.sin_addr.s_addr = remote.ipAddr;
        return sendto(*_deps.metatrafficSock, _deps.metaSendBuf, len, 0,
                      (struct sockaddr*)&dest, sizeof(dest));
    }

    RTPSAutoPubLifecycle _lifecycle;
    RTPSAutoPubBackendDeps _deps;
};

} // namespace AutoPub
} // namespace Runtime
} // namespace RTPS
} // namespace RaftRuntime
