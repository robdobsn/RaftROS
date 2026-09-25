/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS (Zenoh build) - ROS 2 node reaching the graph through a Zenoh router
//
// Selected by CONFIG_RAFTROS_BACKEND_ZENOH.  Applications include RaftROS.h,
// not this header, and register the SysMod under the same name as the RTPS
// build - the transport is a build-time choice, not an API change.
//
// Where the RTPS build owns three UDP sockets, SPDP/SEDP discovery and a
// participant registry, this build owns exactly one TCP session to a router.
// The router carries discovery, so a declaration is made once and fanned out
// rather than announced per peer.
//
// Loop discipline: every socket is non-blocking and the loop does bounded work
// per pass (one receive, one outbound message), so the SysMod stays inside the
// Raft main-loop budget of 10 ms average / 50 ms worst case.
//
// DeviceManager auto-publish works exactly as it does on the RTPS build - the
// pipeline above the transport is shared - and so do std_msgs/String
// subscriptions, with the same application API.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftROSBackendSelect.h"
#if !RAFTROS_BACKEND_ZENOH
    #error "RaftROSZenoh.h is the Zenoh build's SysMod - include RaftROS.h, which selects the backend"
#endif

#include "RaftSysMod.h"
#include "AutoPub/AutoPubDeviceSource.h"
#include "AutoPub/AutoPubStringMessage.h"
#include "ZenohAutoPubBackend.h"
#include "ZenohInterestMatch.h"
#include "ZenohROSCodec.h"
#include "ZenohTCPSession.h"
#include <array>
#include <functional>

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

    /// @brief Called when a std_msgs/String arrives on a subscribed topic.
    /// Signature matches the RTPS build so applications are portable:
    /// (writerEID[4], publisherGuid[12], text, textLen).  Zenoh identifies a
    /// publisher by the 16-byte GID in the sample, split the same way a DDS
    /// GUID is - the last four bytes as the entity id, the first twelve as the
    /// participant prefix.
    using StringMessageHandler = std::function<void(const uint8_t*, const uint8_t*, const char*, uint32_t)>;

    /// @brief Subscribe to a std_msgs/String topic.
    /// @param topic either a ROS name ("/chatter_in") or the DDS form the RTPS
    ///        build takes ("rt/chatter_in") - both name the same ROS topic
    /// @param type the ROS 2 wire type name, e.g. "std_msgs::msg::dds_::String_"
    /// @return the slot index, or -1 if the table is full or the type is one
    ///         this build has no ROS type hash for (a Zenoh key needs it)
    int addStringSubscription(const char* topic, const char* type, StringMessageHandler handler = {});

    /// @brief Set the default subscription (slot 0), replacing any topic set before
    void setStringSubscription(const char* topic, const char* type, StringMessageHandler handler)
    {
        _defaultHandler = std::move(handler);
        addStringSubscription(topic, type, _defaultHandler);
    }

    /// @brief Handler for any subscribed topic that has no handler of its own
    void setStringMessageHandler(StringMessageHandler handler)
    {
        _defaultHandler = std::move(handler);
    }

    /// @brief SysMod factory, registered by the application as "RaftROS"
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
    String _routerHost;
    uint32_t _routerPort = 7447;

    /// @brief Where the session is.  The router is a single peer, so this is a
    /// plain connect/handshake/serve progression rather than a discovery state.
    enum class ConnState : uint8_t
    {
        DISCONNECTED,   ///< No socket; waiting for an IP address or a retry delay
        CONNECTING,     ///< TCP connect issued, waiting for it to complete
        HANDSHAKE,      ///< Session started, exchanging INIT/OPEN with the router
        READY,          ///< Session established; declarations and samples flow
    };
    ConnState _connState = ConnState::DISCONNECTED;
    int _sock = -1;
    uint32_t _lastConnectAttemptMs = 0;
    uint32_t _connectStartedMs = 0;
    uint32_t _reconnectDelayMs = RECONNECT_DELAY_MIN_MS;
    static const uint32_t RECONNECT_DELAY_MIN_MS = 1000;
    static const uint32_t RECONNECT_DELAY_MAX_MS = 30000;
    static const uint32_t CONNECT_TIMEOUT_MS = 10000;

    // Session, identity and the node's own liveliness token
    RaftRuntime::Zenoh::ZenohTCPSession _session;
    std::array<uint8_t, 16> _identity{};
    char _sessionIdStr[33] = {};
    char _nodeToken[RaftRuntime::Zenoh::ZENOH_AUTOPUB_TOKEN_MAX] = {};

    /// @brief The node token is declared once per session, before anything
    /// else: a router that has not seen it attributes our endpoints to no node.
    enum class NodeTokenState : uint8_t { PENDING, DECLARED };
    NodeTokenState _nodeTokenState = NodeTokenState::PENDING;
    static const uint32_t NODE_TOKEN_ID = 1;

    // Auto-publish backend (endpoint declarations and samples) and the shared
    // pipeline that feeds it from DeviceManager.  Zenoh needs no hooks around
    // an endpoint's life: a declaration is made once and the router fans it
    // out, so there is no per-peer announce or dispose to do.
    RaftRuntime::Zenoh::ZenohAutoPubBackend _autoPubBackend;
    RaftRuntime::AutoPub::AutoPubDeviceSource<
        RaftRuntime::Zenoh::ZenohAutoPubBackend,
        RaftRuntime::Zenoh::ZENOH_AUTOPUB_CAPACITY> _autoPubSource;

    /// @brief A 1 Hz std_msgs/String on /chatter, as the RTPS build publishes,
    /// so there is something to `ros2 topic echo` with no sensor attached.
    /// Goes through the same backend as a device endpoint - a publisher is a
    /// publisher, whether a bus device or this feeds it.
    bool _chatterEnabled = true;
    uint8_t _chatterSlot = RaftRuntime::Zenoh::ZenohAutoPubBackend::INVALID_SLOT;
    uint32_t _lastChatterSendMs = 0;
    uint32_t _chatterMsgIndex = 0;
    static const uint32_t CHATTER_PUBLISH_INTERVAL_MS = 1000;
    void publishChatter(uint32_t nowMs);

    /// @brief One subscribed topic.  Like a published endpoint it is staged
    /// first and declared when the session can take it, because a declaration
    /// must not block the loop and the session carries one message at a time.
    struct Subscription
    {
        enum class State : uint8_t { FREE, PENDING_DECLARE, DECLARED };
        State state = State::FREE;
        char rosTopic[RaftRuntime::AutoPub::AUTOPUB_TOPIC_MAX_LEN] = {};
        char type[RaftRuntime::AutoPub::AUTOPUB_TYPE_MAX_LEN] = {};
        char key[RaftRuntime::Zenoh::ZENOH_AUTOPUB_KEY_MAX] = {};
        char token[RaftRuntime::Zenoh::ZENOH_AUTOPUB_TOKEN_MAX] = {};
        bool tokenDeclared = false;
        uint64_t entityId = 0;
        uint32_t received = 0;
        StringMessageHandler handler;
    };
    static const uint8_t MAX_SUBSCRIPTIONS = 4;
    /// @brief Subscriber and token ids live in separate id spaces, and are kept
    /// clear of the publisher token ids either way
    static const uint32_t SUBSCRIBER_ID_BASE = 100;
    static const uint32_t SUBSCRIBER_TOKEN_ID_BASE = 200;
    Subscription _subscriptions[MAX_SUBSCRIPTIONS];
    /// @brief Entity ids for subscriptions come from a range the publisher
    /// entity ids never reach, so the two are never confused in the graph
    uint64_t _nextSubscriptionEntityId = 1000;
    uint8_t _subscriptionCount = 0;
    StringMessageHandler _defaultHandler;
    uint32_t _samplesDropped = 0;       ///< Samples on a key no subscription claims

    /// @brief Key expressions the router has given numeric ids, so a sample
    /// that names an id instead of a key can still be matched to a topic
    struct RemoteKeyId
    {
        uint32_t id = 0;
        char key[RaftRuntime::Zenoh::ZENOH_AUTOPUB_KEY_MAX] = {};
    };
    static const uint8_t MAX_REMOTE_KEY_IDS = 8;
    RemoteKeyId _remoteKeyIds[MAX_REMOTE_KEY_IDS];
    uint8_t _remoteKeyIdCount = 0;

    /// @brief One router interest still being answered.  A reply is a run of
    /// token declarations followed by a final, all tagged with the interest id,
    /// and only one message goes out per loop pass, so the progress cursor has
    /// to be kept: -1 is the node token, 0..N-1 are backend slots, and the
    /// value past the last slot means "send the final".
    struct PendingInterest
    {
        uint32_t id = 0;
        bool sendNodeToken = false;
        int16_t cursor = -1;
    };
    static const uint8_t MAX_PENDING_INTERESTS = 4;
    PendingInterest _pendingInterests[MAX_PENDING_INTERESTS];
    uint8_t _pendingInterestCount = 0;
    uint32_t _interestsRefused = 0;     ///< Interests dropped: queue full or key expression unsupported

    // Buffers - one receive, one outbound message, both loop-task only
    uint8_t _rxBuf[1024] = {};
    uint8_t _msgBuf[1024] = {};

    // Diagnostics
    uint32_t _sessionCount = 0;         ///< Sessions established since boot
    uint32_t _lastEstablishedMs = 0;

    // Connection handling
    uint32_t getLocalIP();
    void startConnect();
    void stepConnect();
    void serviceSession(uint32_t nowMs);
    bool receiveFromRouter(uint32_t nowMs);
    bool flushToRouter();
    void closeConnection(const char* reason);

    // Declarations and interests
    bool buildNodeIdentity();

    /// @brief This node's identity, as the codec wants it.  The strings are
    /// members that are not reassigned after setup.
    RaftRuntime::Zenoh::ZenohROSCodec::NodeIdentity nodeIdentity() const
    {
        return {_domainId, _sessionIdStr, 1, "/", _nodeNamespace.c_str(), _nodeName.c_str()};
    }
    bool stepDeclarations(uint32_t nowMs);
    bool stepInterestReplies(uint32_t nowMs);
    bool onDiscovery(const RaftRuntime::Zenoh::ZenohNetworkMessage::DiscoveryMessage& message);
    bool onSample(const RaftRuntime::Zenoh::ZenohNetworkMessage::SampleMessage& sample);
    static bool onSampleThunk(void* context,
                              const RaftRuntime::Zenoh::ZenohNetworkMessage::SampleMessage& sample)
    {
        return static_cast<RaftROS*>(context)->onSample(sample);
    }

    /// @brief Send one staged subscription declaration, if any
    bool stepSubscriptionDeclarations(uint32_t nowMs);
    void rememberRemoteKeyId(uint32_t id, std::string_view key);
    const Subscription* subscriptionForSample(
            const RaftRuntime::Zenoh::ZenohNetworkMessage::SampleMessage& sample) const;
    static bool onDiscoveryThunk(void* context,
                                 const RaftRuntime::Zenoh::ZenohNetworkMessage::DiscoveryMessage& message)
    {
        return static_cast<RaftROS*>(context)->onDiscovery(message);
    }

    // API
    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);
};
