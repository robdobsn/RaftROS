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
#include "AutoPub/AutoPubServiceRegistry.h"
#include "AutoPub/AutoPubParameterStore.h"
#include "esp_timer.h"
#include "ZenohAutoPubBackend.h"
#include "ZenohInterestMatch.h"
#include "ZenohROSCodec.h"
#include "ZenohTCPSession.h"
#include <array>
#include <atomic>
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

    // ---- Services (server side) ----
    // A handler runs on the loop task and must not block: it answers from
    // state it already holds (Replied), hands the request back to be completed
    // later (Deferred), or refuses it (Refused).  See AutoPubServiceRegistry.
    using ServiceHandler = RaftRuntime::AutoPub::AutoPubServiceHandler;
    using ServiceRequest = RaftRuntime::AutoPub::AutoPubServiceRequest;
    using ServiceReply = RaftRuntime::AutoPub::AutoPubServiceReply;
    using ServiceOutcome = RaftRuntime::AutoPub::AutoPubServiceOutcome;

    /// @brief Serve a ROS 2 service.
    /// @param name ROS service name, e.g. "/raft_esp32/devices"
    /// @param type wire type name, e.g. "std_srvs::srv::dds_::Trigger_"
    ///        (only the std_srvs types are known: Trigger, SetBool, Empty)
    /// @return slot, or -1 if the table is full or the type is unknown
    int addService(const char* name, const char* type, ServiceHandler handler);

    /// @brief Complete a request a handler deferred (loop task only)
    bool completeService(uint32_t token, const ServiceReply& reply)
    {
        return _services.complete(token, reply);
    }

    // ---- Parameters ----
    // The node serves the six ROS 2 parameter services.  A parameter is a
    // scalar (bool, int64, double, string) held here; its owner's `onSet` is
    // called when a set passes the checks and decides what the change means.
    using ParamValue = RaftRuntime::AutoPub::AutoPubParamValue;
    using ParamType = RaftRuntime::AutoPub::AutoPubParamType;
    using ParamSetCallback = RaftRuntime::AutoPub::AutoPubParamSetCallback;

    /// @brief Declare a parameter.  @return false if the table is full, the
    /// name is taken or does not fit, or the value is not a scalar.
    bool declareParameter(const char* name, const ParamValue& value, const char* description = "",
                          bool readOnly = false, ParamSetCallback onSet = {})
    {
        return _params.declare(name, value, description, readOnly, std::move(onSet)) != decltype(_params)::INVALID_SLOT;
    }
    bool declareParameter(const char* name, bool value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::Bool; v.boolValue = value;
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    bool declareParameter(const char* name, int64_t value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::Integer; v.integerValue = value;
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    bool declareParameter(const char* name, double value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::Double; v.doubleValue = value;
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    bool declareParameter(const char* name, const char* value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::String;
        if (!value || strlen(value) >= sizeof(v.stringValue))
            return false;
        strcpy(v.stringValue, value);
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    /// @brief A parameter's current value, or nullptr
    const ParamValue* parameter(const char* name) const { return _params.value(name); }

    /// @brief Turn the 1 Hz /chatter publisher on or off at run time (the
    /// publisher stays declared; only the sends stop)
    void setChatterEnabled(bool enabled)
    {
        _chatterEnabled = enabled;
        ParamValue v; v.type = ParamType::Bool; v.boolValue = enabled;
        _params.setLocal("chatterEnable", v);      // keep `ros2 param get` truthful
    }
    bool isChatterEnabled() const { return _chatterEnabled; }
    uint8_t attachedDeviceCount() const { return _autoPubSource.attachedCount(); }

    /// @brief SysMod factory, registered by the application as "RaftROS"
    static RaftSysMod* create(const char* pModuleName, RaftJsonIF& sysConfig)
    {
        return new RaftROS(pModuleName, sysConfig);
    }

private:
    // Configuration
    /// @brief Whether the node runs.  `active` in SysTypes or posted settings
    /// decides it at boot (falling back to the block's `enable` for older
    /// configurations); /api/ros/set changes it live.  The SysMod itself is
    /// always present once SysManager has created it, so its REST API is there
    /// to activate it - SysManager's own `enable` key is what decides whether
    /// the SysMod exists at all.
    bool _active = false;
    bool _nodeBuilt = false;        ///< Identity, backend, publishers and parameters set up (once, on first activation)
    bool _routerAddrValid = false;  ///< _routerHost parses as an IPv4 address
    bool _noRouterLogged = false;   ///< The "active but no router configured" line has been printed
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

    /// @brief Is the router there?  A device that cannot reach it publishes
    /// nothing to ROS 2 and would otherwise just retry quietly with a growing
    /// backoff, so after a few failures it says so - naming the address, where
    /// the address came from, and how to change it without a rebuild.
    bool _routerFromConfig = false;         ///< routerHost set in SysTypes/settings, not the Kconfig default
    uint32_t _connectFailures = 0;          ///< Consecutive attempts that did not reach a session
    uint32_t _firstFailureMs = 0;           ///< When the current run of failures began (0 = none)
    uint32_t _lastSessionMs = 0;            ///< Last time a session was established (0 = never)
    uint32_t _lastUnreachableLogMs = 0;
    const char* _lastFailureReason = "";
    uint32_t _localIpForLog = 0;            ///< Device address, for the curl line in the warning
    static const uint32_t UNREACHABLE_WARN_AFTER = 3;
    static const uint32_t UNREACHABLE_LOG_INTERVAL_MS = 30000;
    void noteConnectFailure(const char* reason);
    void warnRouterUnreachable();
    int _closeDetail = 0;               ///< errno / session error for the next "disconnected" line
    uint8_t _routerWarnLine = 0;        ///< Next line of the unreachable warning to print (1..3), 0/4 = none
    uint32_t _lastConnectAttemptMs = 0;
    uint32_t _connectStartedMs = 0;
    uint32_t _reconnectDelayMs = RECONNECT_DELAY_MIN_MS;
    static const uint32_t RECONNECT_DELAY_MIN_MS = 1000;
    static const uint32_t RECONNECT_DELAY_MAX_MS = 30000;
    /// @brief Ceiling while the router's host refuses the connection: the host
    /// is up and the router is not listening yet (a restart, typically), and a
    /// refused non-blocking connect costs almost nothing, so keep trying often
    static const uint32_t RECONNECT_DELAY_REFUSED_MAX_MS = 4000;
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
    uint32_t _chatterPeriodMs = 1000;           ///< The chatterPeriodMs parameter
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
        const char* typeHash = nullptr;         ///< Static literal from the class map
        RaftRuntime::AutoPub::AutoPubQoSProfileId qosProfileId =
            RaftRuntime::AutoPub::AutoPubQoSProfileId::FallbackString;   ///< Resolved when declared
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

    /// @brief One served service, as the transport sees it: a key expression
    /// we give an id (the router addresses requests by that id), a queryable
    /// on it, and an SS liveliness token so the graph shows the server.  All
    /// three are staged and sent one per pass like every other declaration.
    struct ServiceSlot
    {
        enum class State : uint8_t { FREE, PENDING_KEYEXPR, PENDING_QUERYABLE, PENDING_TOKEN, DECLARED };
        State state = State::FREE;
        char key[RaftRuntime::Zenoh::ZENOH_AUTOPUB_KEY_MAX] = {};
        char token[RaftRuntime::Zenoh::ZENOH_AUTOPUB_TOKEN_MAX] = {};
        const char* typeHash = nullptr;
        char wireType[64] = {};         ///< e.g. rcl_interfaces::srv::dds_::GetParameters_
        uint64_t entityId = 0;
    };
    static const uint8_t MAX_SERVICES = 12;     ///< Six parameter services and up to six of the application's
    /// @brief Id spaces kept clear of the publisher (2..), subscriber (100..)
    /// and subscription-token (200..) ids
    static const uint32_t SERVICE_KEYEXPR_ID_BASE = 300;
    static const uint32_t SERVICE_QUERYABLE_ID_BASE = 400;
    static const uint32_t SERVICE_TOKEN_ID_BASE = 500;
    static const uint8_t SERVICE_DISPATCH_BUDGET = 2;    ///< Handlers run per loop pass
    ServiceSlot _serviceSlots[MAX_SERVICES];
    RaftRuntime::AutoPub::AutoPubServiceRegistry<MAX_SERVICES, 4, 1024, 512> _services;
    uint64_t _nextServiceEntityId = 2000;
    uint32_t _requestsUnknownKey = 0;    ///< Requests for a key no service holds

    bool stepServiceDeclarations(uint32_t nowMs);
    bool stepServiceReplies(uint32_t nowMs);
    bool onRequest(const RaftRuntime::Zenoh::ZenohNetworkMessage::RequestMessage& request);
    static bool onRequestThunk(void* context,
                               const RaftRuntime::Zenoh::ZenohNetworkMessage::RequestMessage& request)
    {
        return static_cast<RaftROS*>(context)->onRequest(request);
    }

    // ---- Parameters ----
    static const uint8_t MAX_PARAMETERS = 16;
    RaftRuntime::AutoPub::AutoPubParameterStore<MAX_PARAMETERS> _params;
    uint8_t _paramReplyBuf[1024] = {};          ///< One response at a time; copied by the registry
    RaftJsonIF& _sysConfig;                     ///< The settings overlay posted settings persist into
    char _routerHostPending[16] = {};           ///< A new router address, applied once its reply is out
    void setupParameters();
    bool persistSettings(const char* changesJson);
    bool clearPersistedSettings();
    void applyPendingRouterHost();

    // ---- Configuration API ----
    // /api/ros, /api/ros/set and /api/ros/clear.  A REST handler runs on the
    // web server's task, so it only validates and parks the change here; the
    // loop task applies it (and persists it), keeping every piece of node
    // state on one task.
    struct PendingConfig
    {
        bool hasActive = false;
        bool active = false;
        bool hasRouterHost = false;
        char routerHost[16] = {};
        bool hasRouterPort = false;
        uint32_t routerPort = 0;
        bool persist = false;       ///< Also write the change to the settings overlay (NVS)
        bool clear = false;         ///< Remove the persisted RaftROS settings instead
    };
    PendingConfig _pendingConfig;
    std::atomic<bool> _pendingConfigReady{false};
    void applyPendingConfig();
    void setRouterHost(const char* host);
    bool startNode();
    void stopNode(const char* reason);

    // Loop-budget diagnostics, as on the RTPS build: the worst pass since boot
    // and the worst of each phase within a pass, so a slow pass can be placed
    uint32_t _loopPassMaxUs = 0;
    uint32_t _loopDrainMaxUs = 0;       ///< device sample drain
    uint32_t _loopConnMaxUs = 0;        ///< socket create / connect poll
    uint32_t _loopRxMaxUs = 0;          ///< recv and message parsing
    uint32_t _loopTxMaxUs = 0;          ///< declarations, replies, samples, flush
    uint32_t _loopTxMaxSessionUs = 0, _loopTxMaxStepUs = 0, _loopTxMaxFlushUs = 0;   ///< its parts
    uint32_t _loopRxPartsMaxUs = 0, _loopRxMaxRecvUs = 0, _loopRxMaxParseUs = 0, _loopRxMaxBytes = 0;
    static void noteMax(uint32_t& maxUs, int64_t startUs)
    {
        const uint32_t elapsed = (uint32_t)(esp_timer_get_time() - startUs);
        if (elapsed > maxUs)
            maxUs = elapsed;
    }
    /// @brief A socket call is expected to return at once; one that does not
    /// is worth a line, because it is charged to the loop budget
    static const uint32_t SLOW_CALL_WARN_US = 5000;
    static void warnIfSlow(const char* what, int64_t startUs);

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
    uint8_t _msgBuf[1400] = {};     ///< A 1 kB service reply with its key, attachment and final

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
    RaftRetCode apiRos(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);
};
