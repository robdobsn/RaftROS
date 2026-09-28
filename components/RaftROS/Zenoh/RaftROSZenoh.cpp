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
#include "CDREncoder.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

static const char* MODULE_PREFIX = "RaftROS";

// Verbose logging for bring-up.  Console writes block the loop task, so the
// default build says what happened (connected, declared node, subscribed to a
// topic) and keeps the key expressions and tokens for when they are wanted.
// #define RAFTROS_VERBOSE_LOGGING
#ifdef RAFTROS_VERBOSE_LOGGING
    #define DEBUG_ZENOH_KEYS            // full key expressions and liveliness tokens
#endif

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
    // The router address is layered: the Kconfig default, overridden by
    // "routerHost" in SysTypes, overridden by a value posted to
    // /api/postsettings (persisted in NVS).  Remember which, so a device that
    // cannot reach its router can say whether it is running on a default that
    // was never set for this network.
    _routerFromConfig = configGetString("routerHost", "").length() > 0;
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

    // A standing /chatter publisher, created through the backend exactly as a
    // device endpoint is
    _chatterEnabled = configGetBool("chatterEnable", true);
    if (_chatterEnabled)
    {
        RaftRuntime::AutoPub::AutoPubEndpointDesc chatter;
        chatter.deviceId = {0, 0, 0};
        chatter.msgKind = RaftRuntime::AutoPub::AutoPubMsgKind::String;
        chatter.qosProfileId = RaftRuntime::AutoPub::AutoPubQoSProfileId::FallbackString;
        if (!chatter.setNames("/chatter", RaftRuntime::AutoPub::AutoPubClassMap_typeName(
                                              RaftRuntime::AutoPub::AutoPubMsgKind::String)))
            _chatterEnabled = false;
        else
            _chatterSlot = _autoPubBackend.createPublisher(chatter);
        if (_chatterSlot == ZenohAutoPubBackend::INVALID_SLOT)
        {
            LOG_W(MODULE_PREFIX, "setup could not create the /chatter publisher");
            _chatterEnabled = false;
        }
    }

    // Auto-publish every bus device DeviceManager reports, through the same
    // pipeline the RTPS build uses.  Endpoints created while the session is
    // down are staged by the backend and declared once it is up.
    if (_autoPubSource.setup(_autoPubBackend, getSysManager(),
                             configGetString("qosProfiles", "{}").c_str()))
    {
        LOG_I(MODULE_PREFIX, "setup auto-publish listener registered with DeviceManager");
    }
    else
    {
        LOG_W(MODULE_PREFIX, "setup no DeviceManager - auto-publishing disabled");
    }

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
    const int64_t passStartUs = esp_timer_get_time();

    // Emit the latest sample of every device whose bus callback has stored one
    // since the previous pass.  Runs in every state, as the RTPS build does:
    // with no session nothing is sent, but nothing backs up either.
    _autoPubSource.drainSamples();
    noteMax(_loopDrainMaxUs, passStartUs);

    switch (_connState)
    {
        case ConnState::DISCONNECTED:
            warnRouterUnreachable();        // the remaining lines of a warning in progress, one per pass
            if (Raft::isTimeout(nowMs, _lastConnectAttemptMs, _reconnectDelayMs) && getLocalIP() != 0)
            {
                const int64_t startUs = esp_timer_get_time();
                startConnect();
                noteMax(_loopConnMaxUs, startUs);
            }
            break;
        case ConnState::CONNECTING:
        {
            const int64_t startUs = esp_timer_get_time();
            stepConnect();
            noteMax(_loopConnMaxUs, startUs);
            break;
        }
        case ConnState::HANDSHAKE:
        case ConnState::READY:
            serviceSession(nowMs);
            publishChatter(nowMs);
            break;
    }

    const uint32_t passUs = (uint32_t)(esp_timer_get_time() - passStartUs);
    if (passUs > _loopPassMaxUs)
        _loopPassMaxUs = passUs;
}

void RaftROS::warnIfSlow(const char* what, int64_t startUs)
{
    const uint32_t elapsedUs = (uint32_t)(esp_timer_get_time() - startUs);
    if (elapsedUs > SLOW_CALL_WARN_US)
        LOG_W(MODULE_PREFIX, "%s took %u us on the main loop", what, (unsigned)elapsedUs);
}

uint32_t RaftROS::getLocalIP()
{
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif)
        return 0;
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(netif, &ipInfo) != ESP_OK)
        return 0;
    _localIpForLog = ipInfo.ip.addr;
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
    const int64_t startUs = esp_timer_get_time();
    _sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    warnIfSlow("socket()", startUs);
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
    const int64_t connectUs = esp_timer_get_time();
    const int rslt = connect(_sock, (struct sockaddr*)&dest, sizeof(dest));
    warnIfSlow("connect()", connectUs);
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
    const int64_t selectUs = esp_timer_get_time();
    const int ready = select(_sock + 1, nullptr, &writeSet, nullptr, &noWait);
    warnIfSlow("select()", selectUs);
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
        _closeDetail = soError;
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
    // The backend publishes on the device pipeline's schedule, not ours, so it
    // needs the time every pass - not only on the passes where it is serviced
    _autoPubBackend.setNow(nowMs);
    const int64_t rxStartUs = esp_timer_get_time();
    const bool rxOk = receiveFromRouter(nowMs);
    noteMax(_loopRxMaxUs, rxStartUs);
    if (!rxOk)
        return;
    const int64_t txStartUs = esp_timer_get_time();

    _session.service(nowMs);
    const int64_t stepStartUs = esp_timer_get_time();
    if (_session.state() != ZenohTCPSession::State::Established)
    {
        if (_session.state() == ZenohTCPSession::State::Failed ||
            _session.state() == ZenohTCPSession::State::Closed)
        {
            _closeDetail = (int)_session.error();
            closeConnection("session ended");
            return;
        }
    }
    else if (_connState == ConnState::HANDSHAKE)
    {
        _connState = ConnState::READY;
        _reconnectDelayMs = RECONNECT_DELAY_MIN_MS;
        _lastEstablishedMs = nowMs;
        _lastSessionMs = nowMs;
        ++_sessionCount;
        LOG_I(MODULE_PREFIX, "session %u established batch=%u lease=%ums%s%u%s",
              (unsigned)_sessionCount, (unsigned)_session.negotiatedBatch(), (unsigned)_session.remoteLeaseMs(),
              _connectFailures ? " - router reachable again after " : "", (unsigned)_connectFailures,
              _connectFailures ? " failed attempts" : "");
        _connectFailures = 0;
        _firstFailureMs = 0;
        // The node token goes next pass: one console line per pass, since a
        // write to an unattended USB-Serial-JTAG console stalls for ~10 ms
        return;
    }

    // One outbound message per pass, in priority order: the node token (until a
    // router has it, our endpoints belong to no node), then answers the router
    // is waiting on, then endpoint declarations and samples.
    if (_connState == ConnState::READY && _session.outputSize() == 0)
    {
        if (!stepDeclarations(nowMs) && !stepSubscriptionDeclarations(nowMs) &&
            !stepServiceDeclarations(nowMs) && !stepServiceReplies(nowMs) &&
            !stepInterestReplies(nowMs))
            _autoPubBackend.service(nowMs);
    }
    // Run at most SERVICE_DISPATCH_BUDGET handlers per pass and expire deferred
    // requests; the replies they stage go out above, one per pass
    if (_connState == ConnState::READY)
        _services.service(nowMs, SERVICE_DISPATCH_BUDGET);
    const int64_t flushStartUs = esp_timer_get_time();

    flushToRouter();

    // Place a new worst send-side pass: session bookkeeping / declarations
    // and replies / socket send
    const int64_t endUs = esp_timer_get_time();
    if ((uint32_t)(endUs - txStartUs) > _loopTxMaxUs)
    {
        _loopTxMaxUs = (uint32_t)(endUs - txStartUs);
        _loopTxMaxSessionUs = (uint32_t)(stepStartUs - txStartUs);
        _loopTxMaxStepUs = (uint32_t)(flushStartUs - stepStartUs);
        _loopTxMaxFlushUs = (uint32_t)(endUs - flushStartUs);
    }
}

/// @brief Read whatever has arrived, once.  @return false if the connection died
bool RaftROS::receiveFromRouter(uint32_t nowMs)
{
    const int64_t recvStartUs = esp_timer_get_time();
    const int count = recv(_sock, _rxBuf, sizeof(_rxBuf), 0);
    const int64_t parseStartUs = esp_timer_get_time();
    if (count > 0)
    {
        const bool ok = _session.receive(_rxBuf, (size_t)count, nowMs, onDiscoveryThunk, this, onSampleThunk, onRequestThunk);
        // Place a new worst receive pass: the socket read / the parse and
        // callbacks, and how many bytes it was
        const uint32_t totalUs = (uint32_t)(esp_timer_get_time() - recvStartUs);
        if (totalUs > _loopRxPartsMaxUs)
        {
            _loopRxPartsMaxUs = totalUs;
            _loopRxMaxRecvUs = (uint32_t)(parseStartUs - recvStartUs);
            _loopRxMaxParseUs = (uint32_t)(esp_timer_get_time() - parseStartUs);
            _loopRxMaxBytes = (uint32_t)count;
        }
        if (!ok)
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
    const int64_t sendUs = esp_timer_get_time();
    const int sent = send(_sock, _session.outputData(), pending, 0);
    warnIfSlow("send()", sendUs);
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
    // A close before the session was up is a failed attempt to reach the
    // router; a close of a live session is not (the router was there)
    if (_connState == ConnState::CONNECTING || _connState == ConnState::HANDSHAKE)
        noteConnectFailure(reason);
    if (_sock >= 0)
    {
        const int64_t closeUs = esp_timer_get_time();
        close(_sock);
        warnIfSlow("close()", closeUs);
        _sock = -1;
    }
    if (_connState != ConnState::DISCONNECTED)
    {
        // One line for the whole event: each console line costs ~10 ms of loop
        // time when no host is reading the USB-Serial-JTAG port
        LOG_W(MODULE_PREFIX, "disconnected from %s:%u (%s%s%d), retrying in %ums", _routerHost.c_str(),
              (unsigned)_routerPort, reason, _closeDetail ? " code=" : "", _closeDetail, (unsigned)_reconnectDelayMs);
        _connState = ConnState::DISCONNECTED;
    }
    _closeDetail = 0;
    _session.linkLost();
    // Let the backend see the dead session so it re-stages its declarations
    _autoPubBackend.service(millis());
    _nodeTokenState = NodeTokenState::PENDING;
    _pendingInterestCount = 0;
    _remoteKeyIdCount = 0;
    // The router dropped our subscriptions with the session; stage them again
    for (uint8_t index = 0; index < _subscriptionCount; ++index)
    {
        if (_subscriptions[index].state == Subscription::State::DECLARED)
            _subscriptions[index].state = Subscription::State::PENDING_DECLARE;
        _subscriptions[index].tokenDeclared = false;
    }
    // ... and our services: key expression, queryable and token all go with it
    for (auto& slot : _serviceSlots)
        if (slot.state != ServiceSlot::State::FREE)
            slot.state = ServiceSlot::State::PENDING_KEYEXPR;
    _lastConnectAttemptMs = millis();
    _reconnectDelayMs = _reconnectDelayMs >= RECONNECT_DELAY_MAX_MS ?
        RECONNECT_DELAY_MAX_MS : _reconnectDelayMs * 2;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Router reachability
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::noteConnectFailure(const char* reason)
{
    const uint32_t nowMs = millis();
    ++_connectFailures;
    _lastFailureReason = reason;
    if (_firstFailureMs == 0)
        _firstFailureMs = nowMs;
    if (_connectFailures >= UNREACHABLE_WARN_AFTER &&
        (_lastUnreachableLogMs == 0 ||
         Raft::isTimeout(nowMs, _lastUnreachableLogMs, UNREACHABLE_LOG_INTERVAL_MS)))
    {
        _lastUnreachableLogMs = nowMs;
        _routerWarnLine = 1;            // printed from the next passes, one line each
    }
}

/// @brief Say, once every UNREACHABLE_LOG_INTERVAL_MS, that the router cannot
/// be reached - and what to do about it.  Written for whoever is looking at
/// the log of a device that has gone quiet in ROS 2: which address, why it
/// might be wrong, and each way to change it, the rebuild-free one first.
/// Three lines, one per loop pass: a console write blocks the main loop for
/// the line's duration at the UART rate (~9 ms per 100 characters), and the
/// three together measured 53 ms in one pass - over the 50 ms loop contract.
void RaftROS::warnRouterUnreachable()
{
    switch (_routerWarnLine)
    {
        case 1:
        {
            const bool refused = strcmp(_lastFailureReason, "connect refused") == 0;
            const uint32_t forSecs = (millis() - _firstFailureMs) / 1000;
            LOG_W(MODULE_PREFIX,
                  "ROUTER UNREACHABLE: %s:%u - %u attempts over %us, last: %s. %s Nothing reaches ROS 2 until this is fixed.",
                  _routerHost.c_str(), (unsigned)_routerPort, (unsigned)_connectFailures, (unsigned)forSecs,
                  _lastFailureReason,
                  refused ? "The host answers but no router is listening there - start one with: ros2 run rmw_zenoh_cpp rmw_zenohd"
                          : "The host is not answering - is this the right address for this network?");
            break;
        }
        case 2:
            LOG_W(MODULE_PREFIX,
                  "ROUTER UNREACHABLE: the address is %s.",
                  _routerFromConfig
                      ? "RaftROS.routerHost from SysTypes or posted settings"
                      : "the built-in default (CONFIG_RAFTROS_ZENOH_ROUTER_HOST), which was set for a different network and probably needs changing for this one");
            break;
        case 3:
            LOG_W(MODULE_PREFIX,
                  "ROUTER UNREACHABLE: to change it without a rebuild, from any host on the network: "
                  "curl -X POST http://%s/api/postsettings/reboot -d '{\"RaftROS\":{\"routerHost\":\"<router-ip>\"}}' "
                  "- or set RaftROS.routerHost in SysTypes.json, or CONFIG_RAFTROS_ZENOH_ROUTER_HOST in menuconfig, and rebuild.",
                  inet_ntoa(*(struct in_addr*)&(uint32_t&)_localIpForLog));
            break;
        default:
            return;
    }
    ++_routerWarnLine;
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
    LOG_I(MODULE_PREFIX, "declared node %s%s", _nodeNamespace.c_str(), _nodeName.c_str());
#ifdef DEBUG_ZENOH_KEYS
    LOG_I(MODULE_PREFIX, "node token %s", _nodeToken);
#endif
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

    else if (interest.cursor < (int16_t)(_autoPubBackend.capacity() + MAX_SERVICES))
    {
        const uint8_t svc = (uint8_t)(interest.cursor - _autoPubBackend.capacity());
        if (_serviceSlots[svc].state == ServiceSlot::State::DECLARED)
        {
            key = _serviceSlots[svc].token;
            tokenId = SERVICE_TOKEN_ID_BASE + svc;
        }
    }
    if (key)
        msgLen = ZenohNetworkMessage::declareToken(_msgBuf, sizeof(_msgBuf), tokenId, key, &interest.id);
    else if (interest.cursor >= (int16_t)(_autoPubBackend.capacity() + MAX_SERVICES))
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
    {
        // A key expression the router has given a numeric id: samples may name
        // the id instead of the key, and we have to be able to match them
        if (message.declaration == 0 && !message.key.empty())
            rememberRemoteKeyId(message.id, message.key);
        return true;
    }
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
// Chatter
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief Publish the next /chatter message if one is due.  A message the
/// transport cannot take right now is simply skipped: the next one is a second
/// away, and queueing it behind a stalled session would only publish it late.
void RaftROS::publishChatter(uint32_t nowMs)
{
    if (!_chatterEnabled || _connState != ConnState::READY ||
        !Raft::isTimeout(nowMs, _lastChatterSendMs, CHATTER_PUBLISH_INTERVAL_MS))
        return;

    char message[64];
    snprintf(message, sizeof(message), "Hello from %s [%u]", _nodeName.c_str(),
             (unsigned)_chatterMsgIndex);

    uint8_t payload[128];
    CDREncoder encoder;
    encoder.reset(payload, sizeof(payload));
    if (!encoder.writeEncapsulationHeader() || !encoder.writeString(message))
        return;

    if (_autoPubBackend.publish(_chatterSlot, payload, encoder.getPos()) ==
            RaftRuntime::AutoPub::AutoPubPublishResult::Accepted)
    {
        ++_chatterMsgIndex;
        _lastChatterSendMs = nowMs;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subscriptions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int RaftROS::addStringSubscription(const char* topic, const char* type, StringMessageHandler handler)
{
    using namespace RaftRuntime::AutoPub;
    if (!topic || !*topic || !type || !*type)
        return -1;

    // Accept the DDS form the RTPS build takes as well as a ROS name, so the
    // same application code subscribes on either build
    const char* rosTopic = topic;
    if (strncmp(topic, "rt/", 3) == 0)
        rosTopic = topic + 2;           // "rt/chatter_in" -> "/chatter_in"
    char rosTopicBuf[AUTOPUB_TOPIC_MAX_LEN];
    if (rosTopic[0] != '/')
    {
        if (snprintf(rosTopicBuf, sizeof(rosTopicBuf), "/%s", rosTopic) >= (int)sizeof(rosTopicBuf))
            return -1;
        rosTopic = rosTopicBuf;
    }

    // Reuse an existing slot for the same topic, as the RTPS build does
    for (uint8_t index = 0; index < _subscriptionCount; ++index)
    {
        if (strcmp(_subscriptions[index].rosTopic, rosTopic) == 0)
        {
            if (handler)
                _subscriptions[index].handler = std::move(handler);
            return index;
        }
    }
    if (_subscriptionCount >= MAX_SUBSCRIPTIONS)
    {
        LOG_W(MODULE_PREFIX, "addStringSubscription table full - '%s' not subscribed", rosTopic);
        return -1;
    }

    // A Zenoh key carries the ROS type hash, so a type we have no hash for
    // cannot be subscribed to - say so rather than subscribe to nothing
    const char* typeHash = AutoPubClassMap_typeHash(AutoPubClassMap_kindForTypeName(type));
    if (!typeHash)
    {
        LOG_W(MODULE_PREFIX, "addStringSubscription no ROS type hash for '%s' - '%s' not subscribed",
              type, rosTopic);
        return -1;
    }

    Subscription& subscription = _subscriptions[_subscriptionCount];
    if (snprintf(subscription.rosTopic, sizeof(subscription.rosTopic), "%s", rosTopic) >=
            (int)sizeof(subscription.rosTopic) ||
        snprintf(subscription.type, sizeof(subscription.type), "%s", type) >= (int)sizeof(subscription.type))
    {
        LOG_W(MODULE_PREFIX, "addStringSubscription '%s' does not fit", rosTopic);
        return -1;
    }

    // Entity ids come from the same monotonic space as published endpoints so
    // a subscription and a publisher are never confused for one another
    subscription.entityId = _nextSubscriptionEntityId++;
    subscription.typeHash = typeHash;
    // The key does not depend on QoS.  The liveliness token does, and the
    // QoS comes from the qosProfiles configuration, which another SysMod may
    // not have finished setting up yet - so the token is built when it is
    // declared, not here.
    if (!ZenohROSCodec::formatTopicKey(subscription.key, sizeof(subscription.key), _domainId,
                                       subscription.rosTopic, subscription.type, typeHash))
    {
        LOG_W(MODULE_PREFIX, "addStringSubscription cannot express '%s' as a Zenoh key", rosTopic);
        subscription.key[0] = '\0';
        return -1;
    }

    subscription.handler = std::move(handler);
    subscription.state = Subscription::State::PENDING_DECLARE;
    subscription.tokenDeclared = false;
    const int slot = _subscriptionCount++;
#ifdef DEBUG_ZENOH_KEYS
    LOG_I(MODULE_PREFIX, "addStringSubscription slot=%d topic=%s type=%s", slot, rosTopic, type);
#endif
    return slot;
}

/// @brief Declare one staged subscription: the subscriber first, so samples
/// start flowing, then the liveliness token that puts it in the ROS graph
bool RaftROS::stepSubscriptionDeclarations(uint32_t nowMs)
{
    for (uint8_t index = 0; index < _subscriptionCount; ++index)
    {
        Subscription& subscription = _subscriptions[index];
        if (subscription.state == Subscription::State::PENDING_DECLARE)
        {
            const size_t msgLen = ZenohNetworkMessage::declareSubscriber(
                _msgBuf, sizeof(_msgBuf), SUBSCRIBER_ID_BASE + index, subscription.key);
            if (msgLen == 0)
            {
                LOG_E(MODULE_PREFIX, "subscription '%s' will not encode - dropping it",
                      subscription.rosTopic);
                subscription.state = Subscription::State::DECLARED;   // do not retry
                subscription.tokenDeclared = true;
                continue;
            }
            if (!_session.sendNetworkMessage(_msgBuf, msgLen, nowMs))
                return false;
            subscription.state = Subscription::State::DECLARED;
#ifdef DEBUG_ZENOH_KEYS
            LOG_I(MODULE_PREFIX, "subscriber declared on %s", subscription.key);
#endif
            return true;
        }
        if (subscription.state == Subscription::State::DECLARED && !subscription.tokenDeclared)
        {
            // Resolve the QoS now and build the token that announces it.  The
            // same resolution a publisher gets: a qosProfiles alias override
            // for the topic, otherwise FallbackString.
            subscription.qosProfileId = _autoPubSource.resolveSubscriptionQoS(subscription.rosTopic);
            const ZenohROSCodec::Endpoint endpoint{subscription.entityId,
                ZenohROSCodec::EndpointKind::Subscription, subscription.rosTopic, subscription.type,
                subscription.typeHash, ZenohAutoPubBackend::qosForProfile(subscription.qosProfileId)};
            size_t msgLen = 0;
            if (ZenohROSCodec::formatEndpointToken(subscription.token, sizeof(subscription.token),
                                                   nodeIdentity(), endpoint))
                msgLen = ZenohNetworkMessage::declareToken(
                    _msgBuf, sizeof(_msgBuf), SUBSCRIBER_TOKEN_ID_BASE + index, subscription.token);
            if (msgLen == 0)
            {
                LOG_W(MODULE_PREFIX, "subscription '%s' token will not build - it will receive but not appear in the graph",
                      subscription.rosTopic);
                subscription.tokenDeclared = true;      // do not retry a message that cannot be built
                continue;
            }
            if (!_session.sendNetworkMessage(_msgBuf, msgLen, nowMs))
                return false;
            subscription.tokenDeclared = true;
            LOG_I(MODULE_PREFIX, "subscribed to %s (qos=%s)", subscription.rosTopic,
                  RaftRuntime::AutoPub::AutoPubQoSProfile_name(subscription.qosProfileId));
            return true;
        }
    }
    return false;
}

void RaftROS::rememberRemoteKeyId(uint32_t id, std::string_view key)
{
    if (key.size() >= sizeof(_remoteKeyIds[0].key))
        return;
    for (uint8_t index = 0; index < _remoteKeyIdCount; ++index)
    {
        if (_remoteKeyIds[index].id == id)
        {
            memcpy(_remoteKeyIds[index].key, key.data(), key.size());
            _remoteKeyIds[index].key[key.size()] = '\0';
            return;
        }
    }
    if (_remoteKeyIdCount >= MAX_REMOTE_KEY_IDS)
        return;
    RemoteKeyId& entry = _remoteKeyIds[_remoteKeyIdCount++];
    entry.id = id;
    memcpy(entry.key, key.data(), key.size());
    entry.key[key.size()] = '\0';
}

/// @brief Which subscription, if any, a sample belongs to.  The key may arrive
/// in full or as an id the router declared earlier plus a suffix.
const RaftROS::Subscription* RaftROS::subscriptionForSample(
        const ZenohNetworkMessage::SampleMessage& sample) const
{
    char resolved[RaftRuntime::Zenoh::ZENOH_AUTOPUB_KEY_MAX];
    std::string_view key = sample.key;
    if (sample.keyId != 0)
    {
        const char* prefix = nullptr;
        for (uint8_t index = 0; index < _remoteKeyIdCount; ++index)
        {
            if (_remoteKeyIds[index].id == sample.keyId)
            {
                prefix = _remoteKeyIds[index].key;
                break;
            }
        }
        if (!prefix)
            return nullptr;
        if (snprintf(resolved, sizeof(resolved), "%s%.*s", prefix,
                     (int)sample.key.size(), sample.key.data()) >= (int)sizeof(resolved))
            return nullptr;
        key = resolved;
    }
    for (uint8_t index = 0; index < _subscriptionCount; ++index)
    {
        if (_subscriptions[index].state == Subscription::State::DECLARED &&
            key == _subscriptions[index].key)
            return &_subscriptions[index];
    }
    return nullptr;
}

/// @brief A sample arrived.  Decodes it as std_msgs/String and hands it to the
/// topic's handler, or the default one.  Runs on the loop task.
bool RaftROS::onSample(const ZenohNetworkMessage::SampleMessage& sample)
{
    const Subscription* pSubscription = subscriptionForSample(sample);
    if (!pSubscription)
    {
        // Not ours: a router may forward more than we asked for.  Dropping it
        // is not an error - failing the session over it would be.
        ++_samplesDropped;
        return true;
    }
    if (sample.oversized)
    {
        // Well formed but more than this device holds: dropped, counted, and
        // the session goes on
        ++_samplesDropped;
        return true;
    }
    Subscription& subscription = const_cast<Subscription&>(*pSubscription);
    ++subscription.received;

    char text[256];
    const auto decoded = RaftRuntime::AutoPub::AutoPubStringMessage_decode(
        reinterpret_cast<const uint8_t*>(sample.payload.data()), (uint32_t)sample.payload.size(),
        text, sizeof(text));
    if (!decoded.success)
    {
        LOG_W(MODULE_PREFIX, "sample on %s is not a decodable std_msgs/String (%u bytes)",
              subscription.rosTopic, (unsigned)sample.payload.size());
        return true;
    }

    // The publisher's 16-byte GID sits in the attachment, split as a DDS GUID
    // is so the handler signature matches the RTPS build
    ZenohROSCodec::Attachment attachment;
    const bool haveGid = !sample.attachment.empty() &&
        ZenohROSCodec::decodeAttachment(reinterpret_cast<const uint8_t*>(sample.attachment.data()),
                                        sample.attachment.size(), attachment);
    static const uint8_t UNKNOWN_GID[ZenohROSCodec::GID_SIZE] = {};
    const uint8_t* gid = haveGid ? attachment.publisherGid.data() : UNKNOWN_GID;

    const StringMessageHandler& handler = subscription.handler ? subscription.handler : _defaultHandler;
    if (handler)
        handler(gid + 12, gid, text, decoded.textLen);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Services
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int RaftROS::addService(const char* name, const char* type, ServiceHandler handler)
{
    using namespace RaftRuntime::AutoPub;
    if (!name || !*name || !type)
        return -1;
    const AutoPubServiceKind kind = AutoPubServiceCodec_kindForWireType(type);
    const char* typeHash = AutoPubClassMap_serviceTypeHash(type);
    if (kind == AutoPubServiceKind::Unknown || !typeHash)
    {
        LOG_W(MODULE_PREFIX, "addService '%s': type '%s' is not one this build can serve", name, type);
        return -1;
    }
    // ROS service names are absolute; accept a missing leading slash
    char rosName[AUTOPUB_SERVICE_NAME_MAX];
    if (snprintf(rosName, sizeof(rosName), "%s%s", name[0] == '/' ? "" : "/", name) >= (int)sizeof(rosName))
        return -1;

    const uint8_t slot = _services.add(rosName, kind, std::move(handler));
    if (slot == decltype(_services)::INVALID_SLOT)
    {
        LOG_W(MODULE_PREFIX, "addService '%s': table full or name already served", rosName);
        return -1;
    }
    ServiceSlot& svc = _serviceSlots[slot];
    svc.typeHash = typeHash;
    svc.entityId = _nextServiceEntityId++;
    if (!ZenohROSCodec::formatTopicKey(svc.key, sizeof(svc.key), _domainId, rosName, type, typeHash))
    {
        LOG_W(MODULE_PREFIX, "addService '%s' cannot be expressed as a Zenoh key", rosName);
        _services.remove(slot);
        return -1;
    }
    svc.state = ServiceSlot::State::PENDING_KEYEXPR;
    LOG_I(MODULE_PREFIX, "serving %s (%s)", rosName, type);
    return slot;
}

/// @brief Send the next piece of a service's declaration: its key-expression
/// id, then the queryable on that id, then the SS token.  One message per pass.
bool RaftROS::stepServiceDeclarations(uint32_t nowMs)
{
    for (uint8_t index = 0; index < MAX_SERVICES; ++index)
    {
        ServiceSlot& svc = _serviceSlots[index];
        size_t msgLen = 0;
        ServiceSlot::State next = svc.state;
        switch (svc.state)
        {
            case ServiceSlot::State::PENDING_KEYEXPR:
                msgLen = ZenohNetworkMessage::declareKeyExpr(_msgBuf, sizeof(_msgBuf),
                                                             SERVICE_KEYEXPR_ID_BASE + index, svc.key);
                next = ServiceSlot::State::PENDING_QUERYABLE;
                break;
            case ServiceSlot::State::PENDING_QUERYABLE:
                msgLen = ZenohNetworkMessage::declareQueryable(_msgBuf, sizeof(_msgBuf),
                                                               SERVICE_QUERYABLE_ID_BASE + index,
                                                               SERVICE_KEYEXPR_ID_BASE + index);
                next = ServiceSlot::State::PENDING_TOKEN;
                break;
            case ServiceSlot::State::PENDING_TOKEN:
            {
                const ZenohROSCodec::Endpoint endpoint{svc.entityId, ZenohROSCodec::EndpointKind::Service,
                    _services.rosName(index), RaftRuntime::AutoPub::AutoPubServiceCodec_wireType(_services.kind(index)),
                    svc.typeHash, {ZenohROSCodec::Reliability::Reliable, ZenohROSCodec::Durability::Volatile, 10}};
                if (ZenohROSCodec::formatEndpointToken(svc.token, sizeof(svc.token), nodeIdentity(), endpoint))
                    msgLen = ZenohNetworkMessage::declareToken(_msgBuf, sizeof(_msgBuf),
                                                               SERVICE_TOKEN_ID_BASE + index, svc.token);
                next = ServiceSlot::State::DECLARED;
                break;
            }
            default:
                continue;
        }
        if (msgLen == 0)
        {
            LOG_W(MODULE_PREFIX, "service %s: declaration will not build - skipping that step",
                  _services.rosName(index));
            svc.state = next;               // do not retry a message that cannot be built
            continue;
        }
        if (!_session.sendNetworkMessage(_msgBuf, msgLen, nowMs))
            return false;
        svc.state = next;
        if (next == ServiceSlot::State::DECLARED)
            LOG_I(MODULE_PREFIX, "service %s announced", _services.rosName(index));
        return true;
    }
    return false;
}

/// @brief A request arrived (session receive path).  Match its key id to a
/// service and hand it to the registry; the handler runs later in service().
bool RaftROS::onRequest(const ZenohNetworkMessage::RequestMessage& request)
{
    if (request.keyId >= SERVICE_KEYEXPR_ID_BASE && request.keyId < SERVICE_KEYEXPR_ID_BASE + MAX_SERVICES)
    {
        const uint8_t slot = (uint8_t)(request.keyId - SERVICE_KEYEXPR_ID_BASE);
        if (_serviceSlots[slot].state == ServiceSlot::State::DECLARED ||
            _serviceSlots[slot].state == ServiceSlot::State::PENDING_TOKEN)
        {
            _services.accept(slot, request.requestId,
                             reinterpret_cast<const uint8_t*>(request.payload.data()), (uint32_t)request.payload.size(),
                             reinterpret_cast<const uint8_t*>(request.attachment.data()), (uint32_t)request.attachment.size(),
                             request.timeoutMs, millis());
            return true;
        }
    }
    // Not ours, or not yet declared: the client times out; the session is fine
    ++_requestsUnknownKey;
    return true;
}

/// @brief Send what the registry has staged: a response (or error) on one
/// pass, its final on the next
bool RaftROS::stepServiceReplies(uint32_t nowMs)
{
    using RaftRuntime::AutoPub::AutoPubServiceSend;
    const AutoPubServiceSend send = _services.next();
    if (send.kind == AutoPubServiceSend::Kind::None)
        return false;
    const char* key = _serviceSlots[send.slot].key;
    size_t msgLen = 0;
    switch (send.kind)
    {
        case AutoPubServiceSend::Kind::Response:
            msgLen = ZenohNetworkMessage::writeReply(_msgBuf, sizeof(_msgBuf), send.requestId, key,
                                                     send.payload, send.payloadLen,
                                                     send.attachment, RaftRuntime::AutoPub::AUTOPUB_SERVICE_ATTACHMENT_SIZE);
            break;
        case AutoPubServiceSend::Kind::Error:
            msgLen = ZenohNetworkMessage::writeReplyError(_msgBuf, sizeof(_msgBuf), send.requestId, key, send.reason);
            break;
        case AutoPubServiceSend::Kind::Final:
            msgLen = ZenohNetworkMessage::writeResponseFinal(_msgBuf, sizeof(_msgBuf), send.requestId);
            break;
        default:
            return false;
    }
    if (msgLen == 0)
    {
        _services.sent();                   // cannot build it; move on rather than wedge
        return false;
    }
    if (!_session.sendNetworkMessage(_msgBuf, msgLen, nowMs))
        return false;
    _services.sent();
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
    const auto svcStats = _services.stats();
    char buf[700];
    snprintf(buf, sizeof(buf),
             R"({"rslt":"ok","backend":"zenoh","en":%s,"domId":%d,"node":"%s","ns":"%s","router":"%s:%u",)"
             R"("conn":"%s","sessions":%u,"devices":%u,"pubs":%u,"pending":%u,"samples":%u,"redecl":%u,)"
             R"("subs":%u,"rxDropped":%u,"intRefused":%u,"stackFreeB":%u,)"
             R"("routerSource":"%s","routerReachable":%s,"connectFails":%u,"lastSessionAgoS":%d,)"
             R"("heapFreeB":%u,"heapMinB":%u,"loopMaxUs":%u,"loopMaxDrainUs":%u,"loopMaxConnUs":%u,"loopMaxRxUs":%u,"loopMaxTxUs":%u,"loopMaxTxParts":"%u/%u/%u","loopMaxRxParts":"%u/%u/%uB",)"
             R"("services":%u,"svcAccepted":%u,"svcCompleted":%u,"svcDeferred":%u,"svcTimedOut":%u,)"
             R"("svcRefused":%u,"svcUnknownKey":%u})",
             _isEnabled ? "true" : "false",
             (int)_domainId,
             _nodeName.c_str(),
             _nodeNamespace.c_str(),
             _routerHost.c_str(), (unsigned)_routerPort,
             stStr,
             (unsigned)_sessionCount,
             (unsigned)_autoPubSource.attachedCount(),
             (unsigned)_autoPubBackend.inUseCount(),
             (unsigned)_autoPubBackend.pendingCount(),
             (unsigned)stats.published,
             (unsigned)stats.redeclares,
             (unsigned)_subscriptionCount,
             (unsigned)_samplesDropped,
             (unsigned)_interestsRefused,
             // Headroom left on the task this SysMod runs on (shared with every
             // other SysMod on the loop), which is what decides whether an
             // image fits
             (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)),
             _routerFromConfig ? "config" : "default",
             (_connState == ConnState::READY || _connState == ConnState::HANDSHAKE) ? "true" : "false",
             (unsigned)_connectFailures,
             _lastSessionMs ? (int)((millis() - _lastSessionMs) / 1000) : -1,
             // System heap, not this module's: the figures a long soak reads
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)_loopPassMaxUs, (unsigned)_loopDrainMaxUs, (unsigned)_loopConnMaxUs,
             (unsigned)_loopRxMaxUs, (unsigned)_loopTxMaxUs,
             (unsigned)_loopTxMaxSessionUs, (unsigned)_loopTxMaxStepUs, (unsigned)_loopTxMaxFlushUs,
             (unsigned)_loopRxMaxRecvUs, (unsigned)_loopRxMaxParseUs, (unsigned)_loopRxMaxBytes,
             (unsigned)_services.inUseCount(),
             (unsigned)svcStats.accepted, (unsigned)svcStats.completed,
             (unsigned)svcStats.deferred, (unsigned)svcStats.timedOut,
             (unsigned)(svcStats.refusedBusy + svcStats.refusedBad + svcStats.refusedByHandler),
             (unsigned)_requestsUnknownKey);
    return buf;
}

RaftRetCode RaftROS::apiStatus(const String& /*reqStr*/, String& respStr, const APISourceInfo& /*sourceInfo*/)
{
    respStr = getStatusJSON();
    return RaftRetCode::RAFT_OK;
}
