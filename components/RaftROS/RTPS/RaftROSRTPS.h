/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS (RTPS build) - native ROS 2 node speaking RTPS/DDS over UDP
//
// Selected by CONFIG_RAFTROS_BACKEND_RTPS (the default).  Applications include
// RaftROS.h, not this header.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftROSBackendSelect.h"
#if !RAFTROS_BACKEND_RTPS
    #error "RaftROSRTPS.h is the RTPS build's SysMod - include RaftROS.h, which selects the backend"
#endif

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
#include "runtime/dispatch/RTPSSEDPSubscriptionParser.h"
#include "runtime/autopub/RTPSAutoPubBackend.h"
#include "AutoPub/AutoPubAttachPlan.h"
#include "runtime/autopub/RTPSAutoPubClassMap.h"
#include "runtime/autopub/RTPSAutoPubQoSProfile.h"
#include "AutoPub/AutoPubPublisherPool.h"
#include "RaftThreading.h"
#include "RaftDeviceConsts.h"
#include "DeviceTypeRecord.h"
#include "RaftBusDevicesIF.h"
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
    uint64_t _rosDiscSeqNum = 1;  // bumped on autopub attach/detach (writer keeps only latest sample, firstSN==lastSN==seq)
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
        uint8_t extraPubSlot = 0;                   // next autopub writer-registry slot to announce
        bool mainPhaseDone = false;                 // true once stepIdx reached numSteps
        bool extraSubPhaseDone = false;             // true once extraSubSlot reached count
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
        uint8_t extraPubSlot = 0;                   // autopub-writer slot for current peer
        bool mainPhaseDoneForPeer = false;          // true once stepIdx reached numSteps
        bool extraSubPhaseDoneForPeer = false;      // true once extraSubSlot reached count
    };
    WriterHbPassState _hbPass;

    // Diagnostic tracking (for detecting silent list/state transitions)
    std::size_t _lastLoggedDiscoveredCount = 0;
    uint32_t _lastDiscoveredHealthLogMs = 0;
    static const uint32_t DISCOVERED_HEALTH_LOG_INTERVAL_MS = 5000;

    // Buffers for UDP I/O
    uint8_t _sendBuf[1024] = {};
    uint8_t _recvBuf[2048] = {};

    // Send buffer for auto-published user data.  Used only on the loop task
    // (autoPubDrainSamples), so no locking is needed; kept separate from
    // `_sendBuf` so a sample send never clobbers a partially built SEDP message.
    uint8_t _autoPubSendBuf[1024] = {};

    // Networking helpers
    uint32_t getLocalIP();
    bool createSockets();
    void closeSockets();
    void sendSPDP();
    void recvSPDP();
    void recvMetatraffic();
    void processMetatrafficPacket(const uint8_t* packet, uint32_t packetLen, const struct sockaddr_in& fromAddr);
    void recvUserData();
    void startWriterHeartbeatPass();
    void stepWriterHeartbeatPass();
    void handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr);
    void drainPendingAnnounces();
    void purgeStaleParticipants();
    void processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr);
    void handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen,
                       const struct sockaddr_in& fromAddr);

    // Phase 4 / Slice 4.6 — emit an SEDP PublicationBuiltinTopic DATA(w) for
    // the dynamic writer in the given autopub registry slot to the specified
    // peer.  Returns true if a packet was built and sent.  Advances the slot's
    // `sedpSeqNum` exactly once on success.
    bool emitAutoPubSedpAnnounce(const DiscoveredParticipant& remote, uint8_t slot);

    // Chatter topic helpers
    void publishChatter();
    uint32_t buildChatterPayload(uint8_t* pBuf, uint32_t bufLen, const char* message);

    // Helper to build ros_discovery_info payload with our writer GIDs
    uint32_t buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen);

    // Compute the current high-water mark sequence number across every endpoint
    // advertised on the SEDP publications / subscriptions writers.  Each builtin
    // SEDP writer is a single RTPS endpoint with a monotonic seq stream — its
    // HEARTBEAT lastSN MUST equal that high-water mark.  Without these helpers
    // each per-endpoint announce naively published HB(lastSN=<endpoint seq>),
    // which regressed every time a lower-numbered endpoint was re-announced and
    // caused FastDDS readers to silently un-match the writer (root cause of
    // `_NODE_NAME_UNKNOWN_` in `ros2 topic info -v`).
    uint64_t computeSedpPubHeartbeatLastSN() const;
    uint64_t computeSedpSubHeartbeatLastSN() const;

    // -----------------------------------------------------------------
    // Phase 4 / Z2 — per-bus-device auto-publishing plumbing
    //
    // Flow (status callbacks and loop() run on the SysMod loop task; data
    // callbacks run on bus worker tasks):
    //   1. setup() subscribes to DeviceManager::registerForDeviceStatusChange.
    //   2. On a device going online the status callback allocates writer
    //      slot(s) in _autoPubLifecycle, builds a DynamicWriterCtx, acquires a
    //      generation-safe handle from _autoPubPool and registers the data
    //      callback with that handle (not the ctx pointer) as callback info.
    //   3. The data callback (bus task) decodes under the pool lock and stores
    //      only the latest record in the handle's mailbox — no network I/O.
    //   4. loop() drains the mailboxes, serialises and emits RTPS samples.
    //   5. On offline / pending-deletion the handle is released (invalidating
    //      any in-flight or stale callback), writers are disposed and the ctx
    //      is freed.  DeviceManager's unregister does not reach the bus-level
    //      registration, so the handle generation is what makes this safe.
    // -----------------------------------------------------------------
    struct DynamicWriterCtx
    {
        RaftROS* pOwner = nullptr;
        RaftDeviceID deviceID;
        uint16_t deviceTypeIndex = DEVICE_TYPE_INDEX_INVALID;
        uint8_t slot = 0xFF;
        RaftRuntime::AutoPub::AutoPubPublisherHandle handle;

        // Cached decode metadata
        DeviceTypeRecordDecodeFn decodeFn = nullptr;
        const AttrFieldDesc* pFieldDescs = nullptr;
        uint16_t fieldCount = 0;
        uint16_t structSize = 0;
        uint16_t pollDataSizeBytes = 0;
        RaftBusDeviceDecodeState decodeState;

        // Pre-allocated decode buffer (heap; bus callback runs on bus task
        // which has limited stack budget).  Touched only under the pool lock.
        uint8_t* pDecodeBuf = nullptr;
        uint32_t decodeBufSize = 0;
        uint16_t maxDecodeRecords = 0;

        // Slice 4.5: ROS 2 message kind cached at attach time so the hot
        // path does not re-run the class-map lookup.
        RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubMsgKind msgKind =
            RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubMsgKind::Unknown;

        // Slice 4.10 — composite secondary writer (e.g. AHT20 publishes
        // Temperature on the primary slot and RelativeHumidity here).
        // `secondarySlot == 0xFF` means "no secondary".  When present, the
        // secondary writer shares the decoded record with the primary but has
        // its own registry slot (different entityId + topic).
        uint8_t secondarySlot = 0xFF;
        RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubMsgKind secondaryMsgKind =
            RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubMsgKind::Unknown;

        // Diagnostics: data callbacks received (incremented under the pool
        // lock; read on the loop task only after the handle is released).
        uint32_t sampleCount = 0;

        // Diagnostics: histogram of gaps between consecutive data callbacks
        // (<50, <150, <250, <400, >=400 ms).  Written by the bus task under
        // the pool lock; atomics so the loop task can read them for logging.
        static constexpr uint8_t CB_GAP_BUCKETS = 5;
        int64_t lastCallbackUs = 0;
        std::atomic<uint32_t> cbGapHist[CB_GAP_BUCKETS] = {};

        ~DynamicWriterCtx() { delete[] pDecodeBuf; }
    };

    // Owning pointer array — index matches lifecycle slot.  Nullptr entries
    // indicate a free slot.  Capacity matches the lifecycle registry.
    DynamicWriterCtx* _autoPubCtxs[
        RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY] = {};

    // Bus-task → loop-task handoff.  One entry per attached device (composites
    // share one), so capacity never exceeds the writer registry.  64 bytes
    // covers every generated poll record (largest is 36 bytes); attach rejects
    // larger records rather than truncating them.
    struct AutoPubRaftMutexLock
    {
        RaftMutex mutex;
        AutoPubRaftMutexLock() { RaftMutex_init(mutex); }
        ~AutoPubRaftMutexLock() { RaftMutex_destroy(mutex); }
        AutoPubRaftMutexLock(const AutoPubRaftMutexLock&) = delete;
        AutoPubRaftMutexLock& operator=(const AutoPubRaftMutexLock&) = delete;
        bool lock(uint32_t timeoutMs) { return RaftMutex_lock(mutex, timeoutMs); }
        void unlock() { RaftMutex_unlock(mutex); }
    };
    static constexpr uint32_t AUTOPUB_MAILBOX_RECORD_SIZE = 64;
    static constexpr uint32_t AUTOPUB_PRODUCER_LOCK_TIMEOUT_MS = 2;
    RaftRuntime::AutoPub::AutoPubPublisherPool<
        AutoPubRaftMutexLock,
        RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY,
        AUTOPUB_MAILBOX_RECORD_SIZE> _autoPubPool;

    // CDR payload buffers shared by all writers: serialisation now happens
    // only on the loop task, one sample at a time.  Sized for the largest
    // message the serialiser emits (sensor_msgs/Imu ≈ 320 B + frame_id margin).
    static constexpr uint32_t AUTOPUB_CDR_BUF_SIZE = 512;
    uint8_t _autoPubCDRBufs[2][AUTOPUB_CDR_BUF_SIZE] = {};

    // Compile-selected auto-publish backend: owns endpoint creation and the
    // send path for whichever transport this image is built with.  The shared
    // layer hands it AutoPubEndpointDesc descriptors and serialised samples.
    RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubBackend _autoPubBackend;

    // Track whether we registered a status-change callback with DeviceManager.
    bool _autoPubStatusCBRegistered = false;

    // Slice 4.11 — SysTypes-driven QoS overrides, parsed once at setup().
    //   `_qosAliasOverrides`   : matched against the per-device topic alias
    //                            (last path segment, e.g. "imu_1_6a").
    //   `_qosClassOverrides`   : matched against the device `clas[]` entries
    //                            (e.g. "ACC", "TEMP").
    // Resolution order per design §7.2:
    //   per-device alias → per-class override → built-in default.
    struct QoSOverride
    {
        String key;
        RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubQoSProfileId id =
            RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubQoSProfileId::FallbackString;
    };
    std::vector<QoSOverride> _qosAliasOverrides;
    std::vector<QoSOverride> _qosClassOverrides;

    // Parse the SysTypes `qosProfiles` block (called from setup()).
    void autoPubParseQoSOverrides();

    // Resolve a profile id for a device using (alias, clas[]) — returns the
    // final profile id after applying overrides.  `pTopicAlias` is the short
    // per-device suffix (e.g. "imu_1_6a"); `pClasArray` is the device's class
    // list; `pDeviceTypeName` is optional.
    RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubQoSProfileId
        autoPubResolveQoSProfileId(
            const char* pTopicAlias,
            const char* const* pClasArray, size_t clasCount,
            const char* pDeviceTypeName) const;

    // Per-device status callback (loop task) and data callback (bus task).
    void autoPubOnDeviceStatusChange(RaftDevice& device, const BusAddrStatus& addrStatus);
    void autoPubOnDeviceData(uint16_t deviceTypeIdx, std::vector<uint8_t> data, const void* pCallbackInfo);
    bool autoPubAttachDevice(RaftDevice& device, const BusAddrStatus& addrStatus);
    void autoPubDetachDevice(RaftDevice& device, const BusAddrStatus& addrStatus);

    // Serialise and emit every pending mailbox sample (loop task).
    void autoPubDrainSamples();

    // Diagnostics: gaps between drain passes (loop task only), reset each
    // periodic status log.
    int64_t _autoPubLastDrainUs = 0;
    uint32_t _autoPubDrainGapMaxUs = 0;
    uint32_t _autoPubDrainGapsOver150ms = 0;

    // REST API handler
    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);

    static constexpr const char* MODULE_PREFIX = "RaftROS";

};
