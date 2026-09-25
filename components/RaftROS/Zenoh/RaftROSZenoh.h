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
// Not yet here: DeviceManager auto-publish (the device plumbing is still inside
// the RTPS SysMod and is being extracted behind the backend contract), and
// subscriptions.
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
#include "ZenohAutoPubBackend.h"
#include "ZenohInterestMatch.h"
#include "ZenohROSCodec.h"
#include "ZenohTCPSession.h"
#include <array>

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

    // Auto-publish backend (endpoint declarations and samples)
    RaftRuntime::Zenoh::ZenohAutoPubBackend _autoPubBackend;

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
    bool stepDeclarations(uint32_t nowMs);
    bool stepInterestReplies(uint32_t nowMs);
    bool onDiscovery(const RaftRuntime::Zenoh::ZenohNetworkMessage::DiscoveryMessage& message);
    static bool onDiscoveryThunk(void* context,
                                 const RaftRuntime::Zenoh::ZenohNetworkMessage::DiscoveryMessage& message)
    {
        return static_cast<RaftROS*>(context)->onDiscovery(message);
    }

    // API
    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);
};
