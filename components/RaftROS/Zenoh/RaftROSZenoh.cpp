/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS (Zenoh build) - ROS 2 node reaching the graph through a Zenoh router
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftROSZenoh.h"
#include "RaftJson.h"
#include "RestAPIEndpointManager.h"
#include "RaftUtils.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

static const char* MODULE_PREFIX = "RaftROS";

using namespace RaftRuntime::Zenoh;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftROS::RaftROS(const char* pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
}

RaftROS::~RaftROS()
{
    closeConnection("shutdown");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::setup()
{
    // Configure from JSON config (with defaults)
    _isEnabled = configGetBool("enable", false);
    _domainId = configGetLong("domainId", 0);
    _nodeName = configGetString("nodeName", "raft_esp32");
    _nodeNamespace = configGetString("nodeNamespace", "/");
#ifdef CONFIG_RAFTROS_ZENOH_ROUTER_HOST
    _routerHost = configGetString("routerHost", CONFIG_RAFTROS_ZENOH_ROUTER_HOST);
    _routerPort = configGetLong("routerPort", CONFIG_RAFTROS_ZENOH_ROUTER_PORT);
#else
    _routerHost = configGetString("routerHost", "");
    _routerPort = configGetLong("routerPort", 7447);
#endif

    if (!_isEnabled)
    {
        LOG_I(MODULE_PREFIX, "setup DISABLED");
        return;
    }

    // A hostname would need a DNS lookup, and the resolver blocks for as long
    // as the query takes - well past the main-loop budget.  Require an address.
    struct in_addr parsed;
    if (inet_aton(_routerHost.c_str(), &parsed) == 0)
    {
        LOG_E(MODULE_PREFIX, "setup routerHost '%s' is not an IPv4 address - DNS would block the main loop, so it is not used. Disabled.",
              _routerHost.c_str());
        _isEnabled = false;
        return;
    }

    if (!buildNodeIdentity())
    {
        LOG_E(MODULE_PREFIX, "setup could not build node identity for node '%s' namespace '%s' - disabled",
              _nodeName.c_str(), _nodeNamespace.c_str());
        _isEnabled = false;
        return;
    }

    // Bind the auto-publish backend to this session and node identity.  The
    // string members it borrows are owned by this SysMod and never reassigned
    // after setup.
    ZenohAutoPubBackendDeps deps;
    deps.session = &_session;
    deps.sendBuf = _msgBuf;
    deps.sendBufLen = sizeof(_msgBuf);
    deps.domainId = _domainId;
    deps.sessionId = _sessionIdStr;
    deps.nodeId = 1;
    deps.enclave = "/";
    deps.nodeNamespace = _nodeNamespace.c_str();
    deps.nodeName = _nodeName.c_str();
    _autoPubBackend.setup(deps);

    LOG_I(MODULE_PREFIX, "setup backend=zenoh router=%s:%u domain=%u node=%s%s session=%s",
          _routerHost.c_str(), (unsigned)_routerPort, (unsigned)_domainId,
          _nodeNamespace.c_str(), _nodeName.c_str(), _sessionIdStr);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Node identity
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Derive this node's Zenoh session identity and its liveliness token.
/// The identity is the WiFi MAC plus boot randomness, so two boots of the same
/// board are distinct sessions to the router (as they are distinct ROS nodes).
bool RaftROS::buildNodeIdentity()
{
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    for (size_t index = 0; index < sizeof(mac); ++index)
        _identity[index] = mac[index];
    for (size_t index = sizeof(mac); index < _identity.size(); ++index)
        _identity[index] = static_cast<uint8_t>(esp_random());
    // Zenoh prints a session id most-significant byte first with leading zeros
    // stripped, and rmw_zenoh keys are built from that exact text
    for (size_t index = 0; index < _identity.size(); ++index)
        snprintf(_sessionIdStr + index * 2, 3, "%02x", _identity[_identity.size() - 1 - index]);
    size_t firstSignificant = 0;
    while (_sessionIdStr[firstSignificant] == '0' && _sessionIdStr[firstSignificant + 1] != '\0')
        ++firstSignificant;
    if (firstSignificant)
        memmove(_sessionIdStr, _sessionIdStr + firstSignificant, strlen(_sessionIdStr + firstSignificant) + 1);

    return ZenohROSCodec::formatNodeToken(_nodeToken, sizeof(_nodeToken), _domainId, _sessionIdStr,
                                          1, "/", _nodeNamespace.c_str(), _nodeName.c_str());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::loop()
{
    if (!_isEnabled)
        return;
    const uint32_t nowMs = millis();

    switch (_connState)
    {
        case ConnState::DISCONNECTED:
            if (Raft::isTimeout(nowMs, _lastConnectAttemptMs, _reconnectDelayMs) && getLocalIP() != 0)
                startConnect();
            break;
        case ConnState::CONNECTING:
            stepConnect();
            break;
        case ConnState::HANDSHAKE:
        case ConnState::READY:
            serviceSession(nowMs);
            break;
    }
}

uint32_t RaftROS::getLocalIP()
{
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif)
        return 0;
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(netif, &ipInfo) != ESP_OK)
        return 0;
    return ipInfo.ip.addr;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Connection
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Start a non-blocking TCP connect to the router.  Nothing here waits:
/// the connect completes (or does not) in later loop passes.
void RaftROS::startConnect()
{
    _lastConnectAttemptMs = millis();
    _sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_sock < 0)
    {
        LOG_W(MODULE_PREFIX, "connect socket failed errno=%d", errno);
        return;
    }
    const int flags = fcntl(_sock, F_GETFL, 0);
    if (flags < 0 || fcntl(_sock, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        LOG_W(MODULE_PREFIX, "connect could not set non-blocking errno=%d", errno);
        closeConnection("nonblock failed");
        return;
    }
    const int noDelay = 1;
    setsockopt(_sock, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)_routerPort);
    inet_aton(_routerHost.c_str(), &dest.sin_addr);
    const int rslt = connect(_sock, (struct sockaddr*)&dest, sizeof(dest));
    if (rslt < 0 && errno != EINPROGRESS)
    {
        LOG_W(MODULE_PREFIX, "connect to %s:%u failed errno=%d", _routerHost.c_str(),
              (unsigned)_routerPort, errno);
        closeConnection("connect failed");
        return;
    }
    _connectStartedMs = _lastConnectAttemptMs;
    _connState = ConnState::CONNECTING;
}

/// @brief Poll the in-flight connect with a zero timeout - never a wait
void RaftROS::stepConnect()
{
    fd_set writeSet;
    FD_ZERO(&writeSet);
    FD_SET(_sock, &writeSet);
    struct timeval noWait = {0, 0};
    const int ready = select(_sock + 1, nullptr, &writeSet, nullptr, &noWait);
    if (ready < 0)
    {
        closeConnection("select failed");
        return;
    }
    if (ready == 0)
    {
        if (Raft::isTimeout(millis(), _connectStartedMs, CONNECT_TIMEOUT_MS))
            closeConnection("connect timeout");
        return;
    }
    int soError = 0;
    socklen_t errorLen = sizeof(soError);
    if (getsockopt(_sock, SOL_SOCKET, SO_ERROR, &soError, &errorLen) < 0 || soError != 0)
    {
        LOG_W(MODULE_PREFIX, "connect to %s:%u refused errno=%d", _routerHost.c_str(),
              (unsigned)_routerPort, soError);
        closeConnection("connect refused");
        return;
    }

    // TCP is up: open the Zenoh session (this queues our INIT)
    const uint32_t nowMs = millis();
    if (!_session.start(_identity, nowMs))
    {
        closeConnection("session start refused");
        return;
    }
    _nodeTokenState = NodeTokenState::PENDING;
    _pendingInterestCount = 0;
    _connState = ConnState::HANDSHAKE;
    LOG_I(MODULE_PREFIX, "connected to router %s:%u, opening session",
          _routerHost.c_str(), (unsigned)_routerPort);
}

/// @brief One bounded pass over an open session: receive what has arrived,
/// let the session mind its lease, send at most one message, flush.
void RaftROS::serviceSession(uint32_t nowMs)
{
    if (!receiveFromRouter(nowMs))
        return;

    _session.service(nowMs);
    if (_session.state() != ZenohTCPSession::State::Established)
    {
        if (_session.state() == ZenohTCPSession::State::Failed ||
            _session.state() == ZenohTCPSession::State::Closed)
        {
            LOG_W(MODULE_PREFIX, "session ended error=%d", (int)_session.error());
            closeConnection("session ended");
            return;
        }
    }
    else if (_connState == ConnState::HANDSHAKE)
    {
        _connState = ConnState::READY;
        _reconnectDelayMs = RECONNECT_DELAY_MIN_MS;
        _lastEstablishedMs = nowMs;
        ++_sessionCount;
        LOG_I(MODULE_PREFIX, "session established batch=%u lease=%ums",
              (unsigned)_session.negotiatedBatch(), (unsigned)_session.remoteLeaseMs());
    }

    // One outbound message per pass, in priority order: the node token (until a
    // router has it, our endpoints belong to no node), then answers the router
    // is waiting on, then endpoint declarations and samples.
    if (_connState == ConnState::READY && _session.outputSize() == 0)
    {
        if (!stepDeclarations(nowMs) && !stepInterestReplies(nowMs))
            _autoPubBackend.service(nowMs);
    }

    flushToRouter();
}

/// @brief Read whatever has arrived, once.  @return false if the connection died
bool RaftROS::receiveFromRouter(uint32_t nowMs)
{
    const int count = recv(_sock, _rxBuf, sizeof(_rxBuf), 0);
    if (count > 0)
    {
        if (!_session.receive(_rxBuf, (size_t)count, nowMs, onDiscoveryThunk, this))
        {
            LOG_W(MODULE_PREFIX, "session rejected router data error=%d", (int)_session.error());
            closeConnection("bad router data");
            return false;
        }
        return true;
    }
    if (count == 0)
    {
        closeConnection("router closed connection");
        return false;
    }
    if (errno != EWOULDBLOCK && errno != EAGAIN)
    {
        closeConnection("receive failed");
        return false;
    }
    return true;
}

/// @brief Push whatever the session has queued, without waiting.  A partial
/// write is normal and the remainder goes out on a later pass.
bool RaftROS::flushToRouter()
{
    const size_t pending = _session.outputSize();
    if (pending == 0)
        return true;
    const int sent = send(_sock, _session.outputData(), pending, 0);
    if (sent > 0)
    {
        _session.consumeOutput((size_t)sent, millis());
        return true;
    }
    if (sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
    {
        closeConnection("send failed");
        return false;
    }
    return true;
}

/// @brief Drop the connection and back off before retrying, so a router that
/// is down does not turn into a reconnect storm
void RaftROS::closeConnection(const char* reason)
{
    if (_sock >= 0)
    {
        close(_sock);
        _sock = -1;
    }
    if (_connState != ConnState::DISCONNECTED)
    {
        LOG_I(MODULE_PREFIX, "disconnected (%s), retrying in %ums", reason, (unsigned)_reconnectDelayMs);
        _connState = ConnState::DISCONNECTED;
    }
    _session.linkLost();
    // Let the backend see the dead session so it re-stages its declarations
    _autoPubBackend.service(millis());
    _nodeTokenState = NodeTokenState::PENDING;
    _pendingInterestCount = 0;
    _lastConnectAttemptMs = millis();
    _reconnectDelayMs = _reconnectDelayMs >= RECONNECT_DELAY_MAX_MS ?
        RECONNECT_DELAY_MAX_MS : _reconnectDelayMs * 2;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declarations and interests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Declare the node's own liveliness token, once per session
/// @return true if a message was sent
bool RaftROS::stepDeclarations(uint32_t nowMs)
{
    if (_nodeTokenState != NodeTokenState::PENDING)
        return false;
    const size_t msgLen = ZenohNetworkMessage::declareToken(_msgBuf, sizeof(_msgBuf), NODE_TOKEN_ID, _nodeToken);
    if (msgLen == 0)
    {
        LOG_E(MODULE_PREFIX, "node token will not encode - node will not appear in the ROS graph");
        _nodeTokenState = NodeTokenState::DECLARED;   // Do not retry a message that cannot be built
        return false;
    }
    if (!_session.sendNetworkMessage(_msgBuf, msgLen, nowMs))
        return false;
    _nodeTokenState = NodeTokenState::DECLARED;
    LOG_I(MODULE_PREFIX, "declared node token %s", _nodeToken);
    return true;
}

/// @brief Send the next piece of an interest reply: one token declaration per
/// pass, then the final that tells the router the reply is complete.
/// @return true if a message was sent
bool RaftROS::stepInterestReplies(uint32_t nowMs)
{
    if (_pendingInterestCount == 0)
        return false;
    PendingInterest& interest = _pendingInterests[0];

    // -1 is the node token; 0..capacity-1 are backend slots; past that, finish
    size_t msgLen = 0;
    uint32_t tokenId = 0;
    const char* key = nullptr;
    if (interest.cursor < 0)
    {
        if (interest.sendNodeToken && _nodeTokenState == NodeTokenState::DECLARED)
        {
            tokenId = NODE_TOKEN_ID;
            key = _nodeToken;
        }
    }
    else if (interest.cursor < (int16_t)_autoPubBackend.capacity())
    {
        key = _autoPubBackend.tokenKeyForSlot((uint8_t)interest.cursor);
        tokenId = ZenohAutoPubBackend::tokenId((uint8_t)interest.cursor);
    }

    if (key)
        msgLen = ZenohNetworkMessage::declareToken(_msgBuf, sizeof(_msgBuf), tokenId, key, &interest.id);
    else if (interest.cursor >= (int16_t)_autoPubBackend.capacity())
        msgLen = ZenohNetworkMessage::declareFinal(_msgBuf, sizeof(_msgBuf), &interest.id);

    if (msgLen == 0)
    {
        // Nothing to send for this cursor position - advance and try next pass
        ++interest.cursor;
        return false;
    }
    if (!_session.sendNetworkMessage(_msgBuf, msgLen, nowMs))
        return false;

    const bool wasFinal = key == nullptr;
    ++interest.cursor;
    if (wasFinal)
    {
        for (uint8_t index = 1; index < _pendingInterestCount; ++index)
            _pendingInterests[index - 1] = _pendingInterests[index];
        --_pendingInterestCount;
    }
    return true;
}

/// @brief A router told us about an interest.  Queue the ones that ask what
/// this node holds; a key expression we do not fully implement is refused
/// rather than answered incompletely, which would misreport the graph.
bool RaftROS::onDiscovery(const ZenohNetworkMessage::DiscoveryMessage& message)
{
    if (!message.isInterest)
        return true;
    if (message.mode == 0)
    {
        // Interest withdrawn - drop any reply still in progress for it
        for (uint8_t index = 0; index < _pendingInterestCount;)
        {
            if (_pendingInterests[index].id == message.id)
                _pendingInterests[index] = _pendingInterests[--_pendingInterestCount];
            else
                ++index;
        }
        return true;
    }
    if (message.mode == 2)
        return true;
    if (message.scope != 0)
        return false;
    // Only interests asking for token declarations concern us
    if ((message.options & 8) == 0)
        return true;
    if (_pendingInterestCount == MAX_PENDING_INTERESTS)
    {
        ++_interestsRefused;
        return false;
    }

    const auto nodeMatch = ZenohInterestMatch(message.key, _nodeToken);
    if (nodeMatch == ZenohInterestMatchResult::Unsupported)
    {
        ++_interestsRefused;
        return false;
    }
    PendingInterest& interest = _pendingInterests[_pendingInterestCount++];
    interest.id = message.id;
    interest.sendNodeToken = nodeMatch == ZenohInterestMatchResult::Match;
    interest.cursor = -1;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// API / status
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::addRestAPIEndpoints(RestAPIEndpointManager& endpointManager)
{
    endpointManager.addEndpoint("rosstat", RestAPIEndpoint::EndpointType::ENDPOINT_CALLBACK,
                                RestAPIEndpoint::EndpointMethod::ENDPOINT_GET,
                                std::bind(&RaftROS::apiStatus, this,
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3),
                                "Get RaftROS status");
}

String RaftROS::getStatusJSON() const
{
    const char* stStr = "off";
    switch (_connState)
    {
        case ConnState::DISCONNECTED: stStr = "disconnected"; break;
        case ConnState::CONNECTING:   stStr = "connecting"; break;
        case ConnState::HANDSHAKE:    stStr = "handshake"; break;
        case ConnState::READY:        stStr = "ready"; break;
    }
    const auto stats = _autoPubBackend.stats();
    char buf[320];
    snprintf(buf, sizeof(buf),
             R"({"rslt":"ok","backend":"zenoh","en":%s,"domId":%d,"node":"%s","ns":"%s","router":"%s:%u",)"
             R"("conn":"%s","sessions":%u,"pubs":%u,"pending":%u,"samples":%u,"redecl":%u,"intRefused":%u})",
             _isEnabled ? "true" : "false",
             (int)_domainId,
             _nodeName.c_str(),
             _nodeNamespace.c_str(),
             _routerHost.c_str(), (unsigned)_routerPort,
             stStr,
             (unsigned)_sessionCount,
             (unsigned)_autoPubBackend.inUseCount(),
             (unsigned)_autoPubBackend.pendingCount(),
             (unsigned)stats.published,
             (unsigned)stats.redeclares,
             (unsigned)_interestsRefused);
    return buf;
}

RaftRetCode RaftROS::apiStatus(const String& /*reqStr*/, String& respStr, const APISourceInfo& /*sourceInfo*/)
{
    respStr = getStatusJSON();
    return RaftRetCode::RAFT_OK;
}
