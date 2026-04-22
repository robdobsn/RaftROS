/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftSysMod.h"
#include "runtime/core/RTPSParticipant.h"
#include "runtime/discovery/SPDPHandler.h"
#include "runtime/announce/SEDPHandler.h"
#include "runtime/announce/RTPSInitialAnnouncePlan.h"
#include "runtime/announce/RTPSInitialAnnounceRunner.h"
#include "runtime/announce/RTPSWriterHeartbeatRunner.h"
#include "runtime/reliability/RTPSReaderStateMap.h"
#include "runtime/dispatch/RTPSSubscriptionRegistry.h"
#include "runtime/dispatch/RTPSRemotePublicationMap.h"
#include "runtime/dispatch/RTPSSEDPPublicationParser.h"
#include <functional>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>

class APISourceInfo;

class RaftROS : public RaftSysMod
{
public:
    RaftROS(const char* pModuleName, RaftJsonIF& sysConfig);
    virtual ~RaftROS();

    // SysMod lifecycle
    void setup() override;
    void loop() override;
    void addRestAPIEndpoints(RestAPIEndpointManager& endpointManager) override;
    String getStatusJSON() const override;

    // Signature: (writerEID[4], srcGuidPrefix[12], text, textLen)
    using StringMessageHandler = std::function<void(const uint8_t*, const uint8_t*, const char*, uint32_t)>;

    // Register the subscription topic + type + handler for the DEFAULT std_msgs/String
    // subscription slot (slot 0 - the `rt/chatter_in` reader on ENTITYID_CHATTER_READER).
    // The topic / type name are sent in the SEDP subscription advertisement so ROS 2 publishers
    // will route matching traffic to our reader.  Pass an empty std::function to clear.
    void setStringSubscription(const char* topic, const char* type, StringMessageHandler handler)
    {
        if (topic && *topic)
            _subscriptionTopic = topic;
        if (type && *type)
            _subscriptionType = type;
        // Mirror slot 0 in the registry so N-ary announce/HB loops see a consistent view.
        if (_subscriptionRegistry.count > 0)
            _subscriptionRegistry.setTopic(0, _subscriptionTopic.c_str(), _subscriptionType.c_str());
        _stringMessageHandler = std::move(handler);
    }

    // Register an ADDITIONAL std_msgs/String subscription.  Returns the slot index assigned
    // (>= 1), or -1 if the registry is full (capacity = RTPS_SUBSCRIPTION_REGISTRY_CAPACITY).
    //
    // The slot gets a fresh RTPS reader entity ID (deterministic per slot) and is advertised
    // via SEDP on the next initial announce to any discovered participant, and on every
    // periodic heartbeat retransmit thereafter.  Reader GIDs for all slots are also included
    // in the `ros_discovery_info` payload.
    //
    // NOTE: The string/type pointers must remain valid for the lifetime of this RaftROS
    // instance (they are typically string literals).  The registry does not copy strings.
    //
    // Per-topic dispatch: when a matching remote SEDP publication is observed, the
    // (remote writerGuid -> slot) correlation is recorded in `_remotePubMap` and
    // subsequent user-data DATA on that writer is routed to the slot's `handler`.
    // Messages arriving before the SEDP pub has been parsed fall through to the legacy
    // `_stringMessageHandler` so single-subscription applications keep working.
    int addStringSubscription(const char* topic, const char* type, StringMessageHandler handler = {})
    {
        // Check for an existing registry entry with this topic (e.g. the slot-0 entry
        // pre-populated by RaftROS::setup() for the default subscription).  Reuse it
        // rather than appending a duplicate, so the entity-ID and SEDP announce remain
        // consistent and the handler lands in the correct slot.
        for (uint8_t i = 0; i < _subscriptionRegistry.count; i++)
        {
            if (_subscriptionRegistry.entries[i].topic &&
                strcmp(_subscriptionRegistry.entries[i].topic, topic) == 0)
            {
                if (handler)
                    _extraSubscriptionHandlers[i] = std::move(handler);
                return i;
            }
        }
        const int slot = _subscriptionRegistry.add(topic, type);
        if (slot < 0)
            return -1;
        if (handler)
            _extraSubscriptionHandlers[slot] = std::move(handler);
        return slot;
    }

    // Register a handler invoked when a std_msgs/String arrives on the default /chatter_in topic
    // ("rt/chatter_in", std_msgs::msg::dds_::String_).  Pass an empty std::function to clear.
    void setStringMessageHandler(StringMessageHandler handler)
    {
        _stringMessageHandler = std::move(handler);
    }

    // Factory for SysMod registration
    static RaftSysMod* create(const char* pModuleName, RaftJsonIF& sysConfig)
    {
        return new RaftROS(pModuleName, sysConfig);
    }

private:
    // Configuration
    bool _isEnabled = false;
    uint32_t _domainId = 0;
    String _nodeName;
    String _nodeNamespace;
    uint32_t _leaseDurationSec = 120;
    uint32_t _spdpIntervalMs = 30000;

    // Protocol handlers (platform-independent)
    RTPSParticipant _participant;
    SPDPHandler _spdpHandler;
    SEDPHandler _sedpHandler;

    // UDP sockets (-1 = not created)
    int _spdpSock = -1;          // Multicast for SPDP (port 7400)
    int _metatrafficSock = -1;   // Unicast for SEDP  (port 7410)
    int _userDataSock = -1;      // Unicast for data   (port 7411)

    // Our IP (network byte order)
    uint32_t _myIpAddr = 0;

    // Connection state
    enum class ConnState { DISCONNECTED, ANNOUNCING, ACTIVE };
    ConnState _connState = ConnState::DISCONNECTED;

    // Sequence numbers and counters
    uint32_t _lastSpdpSendMs = 0;
    uint32_t _lastWriterHbMs = 0;
    static const uint32_t WRITER_HB_INTERVAL_MS = 1000;
    uint64_t _spdpSeqNum = 0;
    uint64_t _sedpSeqNum = 1;     // seq 1 on SEDP pubs writer = ros_disc_info publication
    uint64_t _sedpSubSeqNum = 1;  // always 1 (single SEDP sub, never changes)
    uint64_t _rosDiscSeqNum = 1;  // always 1 (single ros_discovery_info sample)
    uint64_t _livelinessSeqNum = 0; // incremented each liveliness send
    uint32_t _heartbeatCount = 0;
    uint32_t _acknackCount = 0;

    // Per-(remote participant, writer entity id) reader state for reliable delivery bookkeeping.
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderStateMap _readerStateMap;

    // Optional user-topic std_msgs/String dispatch handler; invoked by recvUserData.
    StringMessageHandler _stringMessageHandler;

    // N-ary subscription registry.  Slot 0 is pre-populated at construction with the
    // default ENTITYID_CHATTER_READER / rt/chatter_in entry.  Slots 1..N are appended
    // via addStringSubscription().  Iterated by the announce/HB paths and by
    // buildRosDiscInfoWithGids() to include every reader's GID.
    RaftRuntime::RTPS::Runtime::Dispatch::RTPSSubscriptionRegistry _subscriptionRegistry;

    // Per-remote-writer -> subscription-slot mapping.  Populated lazily when an SEDP
    // BuiltinPublications DATA message is received whose advertised topic matches one
    // of our registry entries.  Consulted by recvUserData() to route incoming user-data
    // DATA submessages to the correct per-slot handler.
    RaftRuntime::RTPS::Runtime::Dispatch::RTPSRemotePublicationMap _remotePubMap;

    // Per-extra-slot handlers (slot 0 uses _stringMessageHandler).  Invoked by
    // recvUserData() when the remote writer was previously correlated to this slot
    // via _remotePubMap.
    StringMessageHandler _extraSubscriptionHandlers[8];

    // Per-extra-slot SEDP subscription sequence number.
    // All SEDP subscription DATA shares the same writer (000004C2) so the sequence
    // numbers must be strictly monotonically increasing across all slots.
    // Slot 0 (chatter_in) is announced via the main sequence at seq _sedpSubSeqNum+1 = 2.
    // Slot N uses seq _sedpSubSeqNum+1+N so that slot 1 = 3, slot 2 = 4, etc.
    uint64_t _extraSubscriptionSeqNums[8] = {2, 3, 4, 5, 6, 7, 8, 9};

    // Configurable subscription topic/type (used for SEDP advertisement + dispatch).
    // Defaults to the ROS 2 /chatter_in topic; replaced by setStringSubscription().
    String _subscriptionTopic = "rt/chatter_in";
    String _subscriptionType  = "std_msgs::msg::dds_::String_";

    // Chatter topic (Phase 2) - sequence numbers and timing
    uint64_t _chatterSeqNum = 0;       // increments each publish
    uint64_t _chatterSedpSeqNum = 2;   // seq 2 on SEDP pubs writer = chatter publication
    uint32_t _lastChatterSendMs = 0;
    static const uint32_t CHATTER_PUBLISH_INTERVAL_MS = 1000;
    uint32_t _chatterMsgIndex = 0;     // counter for message content

    // Discovered remote participants
    std::vector<DiscoveredParticipant> _discovered;
    static const uint32_t MAX_DISCOVERED = 8;

    // Pending initial-announce entries.  When a new participant is discovered, instead
    // of firing the whole SEDP/liveliness/ros_discovery burst synchronously (which has
    // been observed to consume ~30 ms in a single loop() tick on ESP32-S3), we enqueue
    // a PendingAnnounce and emit one step per loop() iteration from drainPendingAnnounces().
    // Once the main sequence is exhausted we iterate the extra-subscription slots 1..N
    // one per tick.  The entry is removed when both phases complete.
    struct PendingAnnounce
    {
        DiscoveredParticipant remote;               // snapshot (value, not pointer)
        struct sockaddr_in senderAddr = {};         // snapshot of sender for initial unicast
        RTPSInitialAnnounceSequence sequence;       // built once at enqueue time
        RTPSInitialAnnounceRunnerContext runCtx;    // carries seq counters + previousPayloadLen
        uint8_t stepIdx = 0;                        // next main-sequence step to emit
        uint8_t extraSubSlot = 1;                   // next extra-sub registry slot to announce
        bool mainPhaseDone = false;                 // true once stepIdx reached numSteps
    };
    static const uint32_t MAX_PENDING_ANNOUNCES = 8;
    std::vector<PendingAnnounce> _pendingAnnounces;

    // Periodic writer-heartbeat pass state.  Instead of bursting ~14 sendto()'s in a
    // single loop() tick (2 peers x ~7 actions), a pass is kicked off once per
    // WRITER_HB_INTERVAL_MS and then drained one (peer, step) per loop() iteration.
    struct WriterHbPassState
    {
        bool active = false;
        RTPSWriterHeartbeatCounterState counters;   // live across the pass
        RTPSWriterHeartbeatSequence sequence;       // built once at pass start
        uint32_t peerIdx = 0;                       // index into _discovered
        uint8_t stepIdx = 0;                        // within main sequence for current peer
        uint8_t extraSubSlot = 1;                   // extra-sub slot (1..N) for current peer
        bool mainPhaseDoneForPeer = false;          // true once stepIdx reached numSteps
    };
    WriterHbPassState _hbPass;

    // Diagnostic tracking (for detecting silent list/state transitions)
    std::size_t _lastLoggedDiscoveredCount = 0;
    uint32_t _lastDiscoveredHealthLogMs = 0;
    static const uint32_t DISCOVERED_HEALTH_LOG_INTERVAL_MS = 5000;

    // Buffers for UDP I/O
    uint8_t _sendBuf[1024] = {};
    uint8_t _recvBuf[2048] = {};

    // Networking helpers
    uint32_t getLocalIP();
    bool createSockets();
    void closeSockets();
    void sendSPDP();
    void recvSPDP();
    void recvMetatraffic();
    void recvUserData();
    void startWriterHeartbeatPass();
    void stepWriterHeartbeatPass();
    void handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr);
    void drainPendingAnnounces();
    void purgeStaleParticipants();
    void processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr);
    void handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen,
                       const struct sockaddr_in& fromAddr);

    // Chatter topic helpers
    void publishChatter();
    uint32_t buildChatterPayload(uint8_t* pBuf, uint32_t bufLen, const char* message);

    // Helper to build ros_discovery_info payload with our writer GIDs
    uint32_t buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen);

    // REST API handler
    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);

    static constexpr const char* MODULE_PREFIX = "RaftROS";

};
