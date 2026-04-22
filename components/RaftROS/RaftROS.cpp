/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework (Phase 1: Discoverable Node)
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftROS.h"
#include "runtime/dispatch/RTPSUserDispatch.h"
#include "RestAPIEndpointManager.h"
#include "RTPSTypes.h"
#include "runtime/wire/RTPSMessage.h"
#include "runtime/reliability/RTPSAckNackRunner.h"
#include "runtime/schedule/RTPSRuntimeSchedule.h"
#include "runtime/announce/RTPSInitialAnnouncePlan.h"
#include "runtime/announce/RTPSInitialAnnounceRunner.h"
#include "runtime/announce/RTPSWriterHeartbeatRunner.h"
#include "runtime/receive/RTPSRxSubmessageRunner.h"
#include "runtime/receive/RTPSRunnerAdapterHelpers.h"
#include "runtime/discovery/RTPSDiscoveryRuntime.h"
#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

// Socket / network headers (ESP-IDF / lwIP)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_mac.h"
#include "esp_netif.h"

#define WARN_SDSP_PARSE_FAILURE
#define WARN_SDSP_SEND_FAILURE
#define WARN_INVALID_RTPS_HEADER

// Verbose per-packet debug logging.  Each of these emits multiple LOG_I lines per event
// which accumulates to tens of ms of stalled loop() time when the ESP32-S3 is handling
// multiple DDS peers.  Comment out RAFTROS_VERBOSE_LOGGING for production / when
// profiling loop() latency.
// #define RAFTROS_VERBOSE_LOGGING
#ifdef RAFTROS_VERBOSE_LOGGING
    #define DEBUG_SDSP_SEND
    #define DEBUG_SDSP_RECEIVE
    #define DEBUG_SOCKET_CREATION
    #define DEBUG_HEALTH_COUNTS
    #define DEBUG_DISCOVERY
    #define DEBUG_RECEIVE_METATRAFFIC
    #define DEBUG_RECEIVED_HEARTBEAT
    #define DEBUG_SDSP_METATRAFFIC
    #define DEBUG_SDSP_DATA
    #define DEBUG_SDSP_SUBMESSAGE
    #define DEBUG_RECEIVE_USER_DATA
    #define DEBUG_ON_USER_DATA_RECEIVE
    #define DEBUG_ON_USER_DATA_HEARTBEAT
    #define DEBUG_ON_ACKNACK
    #define DEBUG_PURGE_STALE
    #define RAFTROS_SEND_WRITER_HEARTBEATS
    #define DEBUG_PARTICIPANT_PROCESSING
    #define DEBUG_PUBLISH_CHATTER
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftROS::RaftROS(const char* pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
    // Pre-populate subscription registry slot 0 with the default /chatter_in entry so the
    // N-ary announce/HB/ros_discovery_info iteration paths always see a consistent view.
    // Storage for the topic / type strings lives in _subscriptionTopic / _subscriptionType,
    // which are String members; the registry stores non-owning char* pointers into them.
    _subscriptionRegistry.add(_subscriptionTopic.c_str(), _subscriptionType.c_str());
}

RaftROS::~RaftROS()
{
    closeSockets();
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
    _leaseDurationSec = configGetLong("leaseDurationSec", 120);
    _spdpIntervalMs = configGetLong("spdpIntervalMs", 30000);

    // Check enabled
    if (!_isEnabled)
    {
        LOG_I(MODULE_PREFIX, "setup DISABLED");
        return;
    }

    // Initialize RTPS participant (GUID will be set when MAC is available)
    _participant.init(_domainId, _nodeName.c_str(), 0, nullptr);

    // Debug
    LOG_I(MODULE_PREFIX, "setup domainId %d nodeName %s ns %s lease %ds spdp %dms",
          (int)_domainId, _nodeName.c_str(), _nodeNamespace.c_str(),
          (int)_leaseDurationSec, (int)_spdpIntervalMs);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop - connection state machine
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::loop()
{
    // Validate
    if (!_isEnabled)
        return;

    // State machine on connection state
    switch (_connState)
    {
    case ConnState::DISCONNECTED:
    {
        // Wait until we have a valid IP address
        uint32_t ip = getLocalIP();
        if (ip != 0)
        {
            _myIpAddr = ip;

            // Set GUID from WiFi MAC
            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            _participant.setGuidPrefixFromMAC(mac);

            if (createSockets())
            {
#ifdef DEBUG_SOCKET_CREATION
                LOG_I(MODULE_PREFIX, "loop sockets created, IP %s, SPDP port %d",
                      inet_ntoa(*(struct in_addr*)&_myIpAddr),
                      (int)_participant.getSPDPMulticastPort());
#endif
                _connState = ConnState::ANNOUNCING;
                _lastSpdpSendMs = 0;  // trigger immediate first send
            }
        }
        break;
    }
    case ConnState::ANNOUNCING:
    case ConnState::ACTIVE:
    {
        // Periodic SPDP announcement
        uint32_t now = millis();
        if (RTPSRuntimeSchedule_isPeriodicDue(now, _lastSpdpSendMs, _spdpIntervalMs, true))
        {
            sendSPDP();
            _lastSpdpSendMs = now;
        }

        // Periodic writer heartbeats + ros_discovery_info resend.  Rather than bursting
        // ~14 sendto()s (N peers x ~7 actions) in a single tick, kick off a pass here
        // once per interval and drain one step per loop() iteration below.
        if (_connState == ConnState::ACTIVE &&
            !_hbPass.active &&
            RTPSRuntimeSchedule_isPeriodicDue(now, _lastWriterHbMs, WRITER_HB_INTERVAL_MS, true))
        {
            startWriterHeartbeatPass();
            _lastWriterHbMs = now;
        }
        if (_hbPass.active)
        {
            stepWriterHeartbeatPass();
        }

        // Periodic chatter message publishing
        if (_connState == ConnState::ACTIVE &&
            RTPSRuntimeSchedule_isPeriodicDue(now, _lastChatterSendMs, CHATTER_PUBLISH_INTERVAL_MS, true))
        {
            publishChatter();
            _lastChatterSendMs = now;
        }

        // Purge stale discovered participants whose lease has expired
        purgeStaleParticipants();

        // Check for incoming SPDP (non-blocking)
        recvSPDP();

        // Check for incoming metatraffic (SEDP, ACKNACK, etc.)
        recvMetatraffic();

        // Check for incoming user data (ros_discovery_info from remote, etc.)
        recvUserData();

        // Drain one step of any pending initial-announce burst (spreads SEDP / liveliness
        // / ros_discovery_info sends across many loop() ticks instead of bursting them all
        // in a single iteration when a new participant is discovered).
        drainPendingAnnounces();

        // Diagnostic: periodic health line + detect size transitions
        if (_discovered.size() != _lastLoggedDiscoveredCount)
        {
#ifdef DEBUG_HEALTH_COUNTS
            LOG_I(MODULE_PREFIX, "health discoveredCount %u->%u state=%d",
                  (unsigned)_lastLoggedDiscoveredCount,
                  (unsigned)_discovered.size(),
                  (int)_connState);
#endif
            _lastLoggedDiscoveredCount = _discovered.size();
        }
        if (RTPSRuntimeSchedule_isPeriodicDue(now, _lastDiscoveredHealthLogMs,
                                              DISCOVERED_HEALTH_LOG_INTERVAL_MS, true))
        {
#ifdef DEBUG_HEALTH_COUNTS
            LOG_I(MODULE_PREFIX, "health periodic discoveredCount=%u state=%d",
                  (unsigned)_discovered.size(), (int)_connState);
#endif
            _lastDiscoveredHealthLogMs = now;
        }
        break;
    }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get local WiFi station IP (network byte order), 0 if not connected
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: the NetworkSystem module should provide this as a service instead of RaftROS directly calling ESP-IDF APIs and this only works for WiFi station interface.
// ideally there would be an event driven approach here perhaps?

uint32_t RaftROS::getLocalIP()
{
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif)
        return 0;
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(netif, &ipInfo) != ESP_OK)
        return 0;
    return ipInfo.ip.addr;  // already in network byte order
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create the three UDP sockets
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: handle IP changes on the fly (currently we only read the IP at startup and set the GUID prefix from the MAC at startup, but we could potentially support dynamic IP/GUID updates if needed)
// also consider if any additional error handling is required

bool RaftROS::createSockets()
{
    // ---- SPDP multicast socket (send & receive) ----
    _spdpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_spdpSock < 0)
    {
        LOG_E(MODULE_PREFIX, "createSockets SPDP socket failed");
        return false;
    }

    int reuse = 1;
    setsockopt(_spdpSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(_participant.getSPDPMulticastPort());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(_spdpSock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        LOG_E(MODULE_PREFIX, "createSockets SPDP bind failed");
        closeSockets();
        return false;
    }

    // Join SPDP multicast group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(RTPS_DEFAULT_MULTICAST_ADDR);
    mreq.imr_interface.s_addr = _myIpAddr;
    if (setsockopt(_spdpSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        LOG_E(MODULE_PREFIX, "createSockets IGMP join failed");
        closeSockets();
        return false;
    }

    // Set multicast outgoing interface
    struct in_addr ifAddr;
    ifAddr.s_addr = _myIpAddr;
    setsockopt(_spdpSock, IPPROTO_IP, IP_MULTICAST_IF, &ifAddr, sizeof(ifAddr));

    // Non-blocking
    int flags = fcntl(_spdpSock, F_GETFL, 0);
    fcntl(_spdpSock, F_SETFL, flags | O_NONBLOCK);

    // ---- Metatraffic unicast socket (SEDP) ----
    _metatrafficSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_metatrafficSock < 0) { closeSockets(); return false; }

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(_participant.getMetatrafficUnicastPort());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(_metatrafficSock, (struct sockaddr*)&addr, sizeof(addr));
    flags = fcntl(_metatrafficSock, F_GETFL, 0);
    fcntl(_metatrafficSock, F_SETFL, flags | O_NONBLOCK);

    // ---- User data unicast socket (ros_discovery_info etc.) ----
    _userDataSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_userDataSock < 0) { closeSockets(); return false; }

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(_participant.getUserUnicastPort());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(_userDataSock, (struct sockaddr*)&addr, sizeof(addr));
    flags = fcntl(_userDataSock, F_GETFL, 0);
    fcntl(_userDataSock, F_SETFL, flags | O_NONBLOCK);

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Close all sockets
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: see note above about handling IP changes on the fly - we may need to recreate sockets and rejoin multicast group if IP changes

void RaftROS::closeSockets()
{
    if (_spdpSock >= 0)        { close(_spdpSock);        _spdpSock = -1; }
    if (_metatrafficSock >= 0) { close(_metatrafficSock); _metatrafficSock = -1; }
    if (_userDataSock >= 0)    { close(_userDataSock);    _userDataSock = -1; }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send SPDP multicast announcement
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::sendSPDP()
{
    // Build SPDP announcement message with updated sequence number
    _spdpSeqNum++;
    uint32_t msgLen = _spdpHandler.buildAnnouncementMessage(
        _sendBuf, sizeof(_sendBuf),
        _participant, _myIpAddr,
        _leaseDurationSec, _spdpSeqNum);
    if (msgLen == 0)
        return;

    // Send SPDP announcement via multicast
    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(_participant.getSPDPMulticastPort());
    dest.sin_addr.s_addr = inet_addr(RTPS_DEFAULT_MULTICAST_ADDR);

    // Send with sendto (since multicast) - note that we don't need to specify the outgoing interface here since we set it on the socket with IP_MULTICAST_IF
    int sent = sendto(_spdpSock, _sendBuf, msgLen, 0,
                      (struct sockaddr*)&dest, sizeof(dest));
    if (sent < 0)
    {
#ifdef WARN_SDSP_SEND_FAILURE
        LOG_W(MODULE_PREFIX, "sendSPDP sendto failed errno %d", errno);
#endif
    }
    else
    {
#ifdef DEBUG_SDSP_SEND
        LOG_I(MODULE_PREFIX, "sendSPDP sent %d bytes seq %llu to %s:%d",
              sent, (unsigned long long)_spdpSeqNum,
              inet_ntoa(dest.sin_addr), (int)ntohs(dest.sin_port));
#endif
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive and process incoming SPDP (non-blocking)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::recvSPDP()
{
    // Receive SPDP announcement via multicast
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(_spdpSock, _recvBuf, sizeof(_recvBuf), 0,
                     (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0)
        return;

#ifdef DEBUG_SDSP_RECEIVE
    // Debug log raw message and source (note that the message may be binary and not null-terminated, so we print the first few bytes as chars if available)
    LOG_I(MODULE_PREFIX, "recvSPDP raw %d bytes from %s:%d magic=%c%c%c%c",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port),
          n > 0 ? _recvBuf[0] : '?', n > 1 ? _recvBuf[1] : '?',
          n > 2 ? _recvBuf[2] : '?', n > 3 ? _recvBuf[3] : '?');
#endif

    DiscoveredParticipant remote;
    if (!_spdpHandler.parseAnnouncementMessage(_recvBuf, (uint32_t)n, remote))
    {
#ifdef WARN_SDSP_PARSE_FAILURE
        LOG_W(MODULE_PREFIX, "recvSPDP parse FAILED (%d bytes)", n);
#endif
        return;
    }

    // Skip our own announcements
    if (memcmp(remote.guidPrefix, _participant.getGuidPrefix(), 12) == 0)
        return;

    processDiscoveredParticipant(remote, fromAddr);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Process a discovered participant (from SPDP multicast or metatraffic unicast)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr)
{
    uint32_t nowMs = millis();

#ifdef DEBUG_DISCOVERY
    // Format IP strings before logging (inet_ntoa uses static buffer)
    char srcIpStr[16], locIpStr[16];
    strncpy(srcIpStr, inet_ntoa(fromAddr.sin_addr), sizeof(srcIpStr));
    srcIpStr[sizeof(srcIpStr)-1] = '\0';
    struct in_addr locAddr;
    locAddr.s_addr = remote.ipAddr;
    strncpy(locIpStr, inet_ntoa(locAddr), sizeof(locIpStr));
    locIpStr[sizeof(locIpStr)-1] = '\0';
    LOG_I(MODULE_PREFIX, "discovered from %s:%d locator=%s metaPort=%d userPort=%d lease=%d",
          srcIpStr, (int)ntohs(fromAddr.sin_port),
          locIpStr,
          (int)remote.metatrafficPort, (int)remote.userDataPort,
          (int)remote.leaseDurationSec);
#endif

    // Merge participant into discovered set and apply side effects on new additions.
#ifdef DEBUG_DISCOVERY
    std::size_t prevCount = _discovered.size();
#endif
    RaftRuntime::RTPS::Runtime::DiscoveryRuntime::MergeResult mergeResult =
        RaftRuntime::RTPS::Runtime::DiscoveryRuntime::mergeParticipant(
            _discovered, remote, nowMs, MAX_DISCOVERED);


#ifdef DEBUG_DISCOVERY
    const char* mergeStr = (mergeResult == RaftRuntime::RTPS::Runtime::DiscoveryRuntime::MergeResult::AddedNew) ? "NEW"
                        : (mergeResult == RaftRuntime::RTPS::Runtime::DiscoveryRuntime::MergeResult::RefreshedExisting) ? "REFRESH"
                        : "FULL";
    LOG_I(MODULE_PREFIX, "processDiscoveredParticipant %s guidPfx=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x count %u->%u",
          mergeStr,
          remote.guidPrefix[0], remote.guidPrefix[1], remote.guidPrefix[2], remote.guidPrefix[3],
          remote.guidPrefix[4], remote.guidPrefix[5], remote.guidPrefix[6], remote.guidPrefix[7],
          remote.guidPrefix[8], remote.guidPrefix[9], remote.guidPrefix[10], remote.guidPrefix[11],
          (unsigned)prevCount, (unsigned)_discovered.size());
#endif

    // If we added a new participant, handle side effects and check if we should transition to ACTIVE state
    if (mergeResult == RaftRuntime::RTPS::Runtime::DiscoveryRuntime::MergeResult::AddedNew)
    {
        handleNewParticipant(remote, fromAddr);

        bool wasActive = (_connState == ConnState::ACTIVE);
        RTPSParticipantSetPolicyResult setPolicy =
            RaftRuntime::RTPS::Runtime::DiscoveryRuntime::applyParticipantSetPolicy(
                wasActive,
                (uint32_t)_discovered.size(),
                true,
                true);

        if (setPolicy.shouldBeActive && !wasActive)
        {
#ifdef DEBUG_DISCOVERY
            LOG_I(MODULE_PREFIX, "stateTransition ANNOUNCING->ACTIVE discoveredCount=%u",
                  (unsigned)_discovered.size());
#endif

            // Transition to ACTIVE state
            _connState = ConnState::ACTIVE;
        }

        // If the policy indicates, trigger an immediate writer heartbeat (to speed up discovery for new participants).
        if (setPolicy.triggerImmediateWriterHeartbeat)
            RaftRuntime::RTPS::Runtime::DiscoveryRuntime::onActivated(_lastWriterHbMs);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive and log incoming metatraffic (SEDP, ACKNACK, etc.)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::recvMetatraffic()
{
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(_metatrafficSock, _recvBuf, sizeof(_recvBuf), 0,
                     (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0)
        return;

#ifdef DEBUG_RECEIVE_METATRAFFIC
    LOG_I(MODULE_PREFIX, "recvMetatraffic %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));
#endif

    struct RxCtx
    {
        RTPSRxRunnerAdapterBaseCtx base;
        RaftROS* self = nullptr;
    } rxCtx;

    rxCtx.self = this;

    rxCtx.base.localGuidPrefix = _participant.getGuidPrefix();
    rxCtx.base.discovered = &_discovered;
    rxCtx.base.ackSendSock = _metatrafficSock;
    rxCtx.base.readerPolicy = RTPSRxAdapterReaderPolicy::BuiltinEndpointMap;
    rxCtx.base.ackDestPolicy = RTPSRxAdapterAckDestPolicy::ReplyToSender;

    RTPSRxSubmessageRunnerCallbacks callbacks;
    RTPSRunnerAdapter_applyRxBaseCallbacks(callbacks);
    callbacks.resolveReaderWriterState = [](void* userCtx, RTPSRxChannel,
                                            const uint8_t* srcGuidPrefix,
                                            const uint8_t* writerEID)
        -> RaftRuntime::RTPS::Runtime::Reader::RTPSReaderWriterState*
    {
        return static_cast<RxCtx*>(userCtx)->self->_readerStateMap.getOrCreate(srcGuidPrefix, writerEID);
    };
    callbacks.onInvalidHeader = [](void*, RTPSRxChannel)
    {
#ifdef WARN_INVALID_RTPS_HEADER
        LOG_W(MODULE_PREFIX, "recvMetatraffic invalid RTPS header");
#endif
    };
    callbacks.onHeartbeat = [](void*, RTPSRxChannel, const uint8_t* writerEID, uint32_t lastSNLow, bool responded, int sentBytes)
    {
#ifdef DEBUG_RECEIVED_HEARTBEAT
        if (responded)
        {
            LOG_I(MODULE_PREFIX, "  HEARTBEAT writerEID=%02X%02X%02X%02X lastSN=%u -> ACKNACK %d bytes",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow, sentBytes);
        }
        else
        {
            LOG_I(MODULE_PREFIX, "  HEARTBEAT(final) writerEID=%02X%02X%02X%02X lastSN=%u",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow);
        }
#endif
    };
    callbacks.onData = [](void* userCtx,
                          RTPSRxChannel,
                          const uint8_t* packet,
                          uint32_t packetLen,
                          const uint8_t*,
                          const struct sockaddr_in& from,
                          const uint8_t* pContent,
                          uint32_t contentLen)
    {
        RaftROS* self = static_cast<RxCtx*>(userCtx)->self;
        const uint8_t* writerEID = pContent + 8;
        if (memcmp(writerEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
        {
#ifdef DEBUG_SDSP_METATRAFFIC
            LOG_I(MODULE_PREFIX, "  SPDP DATA on metatraffic, parsing as participant announcement");
#endif
            DiscoveredParticipant remote;
            if (self->_spdpHandler.parseAnnouncementMessage(packet, packetLen, remote))
            {
                if (memcmp(remote.guidPrefix, self->_participant.getGuidPrefix(), 12) != 0)
                    self->processDiscoveredParticipant(remote, from);
            }
        }
        else
        {
#ifdef DEBUG_SDSP_DATA
            LOG_I(MODULE_PREFIX, "  submsg: DATA writerEID=%02X%02X%02X%02X len=%d",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], (int)contentLen);
#endif

        }
    };
    callbacks.onAckNack = [](void* userCtx,
                             RTPSRxChannel,
                             const uint8_t* srcGuidPrefix,
                             const uint8_t* pContent,
                             uint32_t contentLen,
                             const struct sockaddr_in& from)
    {
        static_cast<RxCtx*>(userCtx)->self->handleAcknack(srcGuidPrefix, pContent, contentLen, from);
    };
    callbacks.onOther = [](void*, RTPSRxChannel, RTPSSubmessageId submsgId, uint8_t flags, uint32_t contentLen)
    {
#ifdef DEBUG_SDSP_SUBMESSAGE
        const char* name = "?";
        switch (submsgId) {
            case SUBMSG_DATA:       name = "DATA"; break;
            case SUBMSG_HEARTBEAT:  name = "HEARTBEAT"; break;
            case SUBMSG_ACKNACK:    name = "ACKNACK"; break;
            case SUBMSG_INFO_DST:   name = "INFO_DST"; break;
            case SUBMSG_INFO_TS:    name = "INFO_TS"; break;
            default: break;
        }
        LOG_I(MODULE_PREFIX, "  submsg: %s (0x%02X) flags=0x%02X len=%d",
              name, (int)submsgId, (int)flags, (int)contentLen);
#else
        (void)submsgId; (void)flags; (void)contentLen;
#endif
    };

    RTPSRxSubmessageRunner_run(
        _recvBuf, (uint32_t)n, fromAddr,
        RTPSRxChannel::Metatraffic,
        _acknackCount,
        callbacks,
        &rxCtx);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive and process incoming user data (ros_discovery_info from remote, etc.)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::recvUserData()
{
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(_userDataSock, _recvBuf, sizeof(_recvBuf), 0,
                     (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0)
        return;

#ifdef DEBUG_RECEIVE_USER_DATA
    LOG_I(MODULE_PREFIX, "recvUserData %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));
#endif

    struct RxCtx
    {
        RTPSRxRunnerAdapterBaseCtx base;
        RaftROS* self = nullptr;
    } rxCtx;

    rxCtx.self = this;

    rxCtx.base.localGuidPrefix = _participant.getGuidPrefix();
    rxCtx.base.discovered = &_discovered;
    rxCtx.base.ackSendSock = _metatrafficSock;
    rxCtx.base.readerPolicy = RTPSRxAdapterReaderPolicy::UseHeartbeatReader;
    rxCtx.base.ackDestPolicy = RTPSRxAdapterAckDestPolicy::RouteToDiscoveredMetatraffic;

    RTPSRxSubmessageRunnerCallbacks callbacks;
    RTPSRunnerAdapter_applyRxBaseCallbacks(callbacks);
    callbacks.resolveReaderWriterState = [](void* userCtx, RTPSRxChannel,
                                            const uint8_t* srcGuidPrefix,
                                            const uint8_t* writerEID)
        -> RaftRuntime::RTPS::Runtime::Reader::RTPSReaderWriterState*
    {
        return static_cast<RxCtx*>(userCtx)->self->_readerStateMap.getOrCreate(srcGuidPrefix, writerEID);
    };
    callbacks.onInvalidHeader = nullptr;
    callbacks.onHeartbeat = [](void*, RTPSRxChannel, const uint8_t* writerEID, uint32_t lastSNLow, bool responded, int sentBytes)
    {
#ifdef DEBUG_ON_USER_DATA_HEARTBEAT
        if (responded)
        {
            LOG_I(MODULE_PREFIX, "  UD HEARTBEAT writerEID=%02X%02X%02X%02X lastSN=%u -> ACKNACK %d bytes",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                  lastSNLow, sentBytes);
        }
#endif
    };
    callbacks.onData = [](void* userCtx,
                          RTPSRxChannel,
                          const uint8_t*,
                          uint32_t,
                          const uint8_t* srcGuidPrefix,
                          const struct sockaddr_in&,
                          const uint8_t* pContent,
                          uint32_t contentLen)
    {
        const uint8_t* writerEID = pContent + 8;
        if (memcmp(writerEID, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0)
            return;
        if (contentLen < 24)
            return;
        const uint8_t* payload = pContent + 20;
        uint32_t payloadLen = contentLen - 20;
        char text[128] = {0};
        auto dec = RaftRuntime::RTPS::Runtime::UserDispatch::decodeStdMsgsString(
            payload, payloadLen, text, sizeof(text));
        if (!dec.success)
            return;
        RaftROS* self = static_cast<RxCtx*>(userCtx)->self;
#ifdef DEBUG_ON_USER_DATA_RECEIVE
        LOG_I(MODULE_PREFIX,
              "  UD user-topic writerEID=%02X%02X%02X%02X src=%02X%02X%02X%02X... \"%s\" (%u chars)",
              writerEID[0], writerEID[1], writerEID[2], writerEID[3],
              srcGuidPrefix[0], srcGuidPrefix[1], srcGuidPrefix[2], srcGuidPrefix[3],
              text, (unsigned)dec.textLen);
#endif
        if (self->_stringMessageHandler)
            self->_stringMessageHandler(writerEID, srcGuidPrefix, text, dec.textLen);
    };
    callbacks.onAckNack = [](void* userCtx,
                             RTPSRxChannel,
                             const uint8_t* srcGuidPrefix,
                             const uint8_t* pContent,
                             uint32_t contentLen,
                             const struct sockaddr_in& from)
    {
        static_cast<RxCtx*>(userCtx)->self->handleAcknack(srcGuidPrefix, pContent, contentLen, from);
    };
    callbacks.onOther = [](void*, RTPSRxChannel, RTPSSubmessageId submsgId, uint8_t flags, uint32_t contentLen)
    {
#ifdef DEBUG_ON_USER_DATA_RECEIVE
        const char* name = "?";
        switch (submsgId) {
            case SUBMSG_DATA:       name = "DATA"; break;
            case SUBMSG_HEARTBEAT:  name = "HEARTBEAT"; break;
            case SUBMSG_ACKNACK:    name = "ACKNACK"; break;
            case SUBMSG_INFO_DST:   name = "INFO_DST"; break;
            case SUBMSG_INFO_TS:    name = "INFO_TS"; break;
            default: break;
        }
        LOG_I(MODULE_PREFIX, "  UD submsg: %s (0x%02X) flags=0x%02X len=%d",
              name, (int)submsgId, (int)flags, (int)contentLen);
#else
        (void)submsgId; (void)flags; (void)contentLen;
#endif
    };

    RTPSRxSubmessageRunner_run(
        _recvBuf, (uint32_t)n, fromAddr,
        RTPSRxChannel::UserData,
        _acknackCount,
        callbacks,
        &rxCtx);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle incoming ACKNACK — retransmit requested data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen,
                            const struct sockaddr_in& fromAddr)
{
    (void)fromAddr;

    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

    RTPSAckActionStandardCtx ctx;
    ctx.srcGuidPrefix = srcGuidPrefix;

    auto buildRosDiscInfoThunk = [](void* payloadCtx, uint8_t* buf, uint32_t bufLen) -> uint32_t
    {
        return static_cast<RaftROS*>(payloadCtx)->buildRosDiscInfoWithGids(buf, bufLen);
    };
    auto buildChatterThunk = [](void* payloadCtx, uint8_t* buf, uint32_t bufLen) -> uint32_t
    {
        RaftROS* self = static_cast<RaftROS*>(payloadCtx);
        char msgStr[64];
        snprintf(msgStr, sizeof(msgStr), "Hello from %s [%lu]",
                 self->_nodeName.c_str(),
                 (unsigned long)(self->_chatterMsgIndex > 0 ? self->_chatterMsgIndex - 1 : 0));
        return self->buildChatterPayload(buf, bufLen, msgStr);
    };

    RTPSAckActionStandardInitConfig initConfig;
    initConfig.discovered = &_discovered;
    initConfig.metatrafficSock = _metatrafficSock;
    initConfig.userDataSock = _userDataSock;
    initConfig.sendBuf = _sendBuf;
    initConfig.sendBufLen = sizeof(_sendBuf);
    initConfig.heartbeatCount = &_heartbeatCount;
    initConfig.dumpRosDiscoveryPayloadHex = false;
    initConfig.runtimeFlavor = RTPSAckNackRuntimeFlavor::EspStyle;
    initConfig.rosDiscoveryPublicationSeqNum = _sedpSeqNum;
    initConfig.rosDiscoverySubscriptionSeqNum = _sedpSubSeqNum;
    initConfig.chatterPublicationSeqNum = _chatterSedpSeqNum;
    initConfig.rosDiscoveryInfoSeqNum = _rosDiscSeqNum;
    initConfig.chatterDataSeqNum = _chatterSeqNum;
    initConfig.sedpHandler = &_sedpHandler;
    initConfig.participant = &_participant;
    initConfig.myIpAddr = _myIpAddr;
    initConfig.sedpBuildHeartbeatPolicy =
        RTPSAckActionSedpBuildHeartbeatPolicy::PassZero;
    initConfig.buildRosDiscInfoPayload = buildRosDiscInfoThunk;
    initConfig.buildChatterPayload = buildChatterThunk;
    initConfig.payloadCtx = this;

    RTPSRunnerAdapter_initStandardAckActionCtx(ctx, initConfig);

    RTPSAckNackRunnerCallbackInitConfig callbackInit;
    callbackInit.logParsed = [](void*, const RTPSAckNackFields& fields, RTPSAckNackWriterKind writerKind)
    {
#ifdef DEBUG_ON_ACKNACK
        LOG_I(MODULE_PREFIX, "  ACKNACK readerEID=%02X%02X%02X%02X writerEID=%02X%02X%02X%02X (%s) base=%u numBits=%u",
              fields.readerEID[0], fields.readerEID[1], fields.readerEID[2], fields.readerEID[3],
              fields.writerEID[0], fields.writerEID[1], fields.writerEID[2], fields.writerEID[3],
              writerKindToStr(writerKind),
              fields.bitmapBaseLow, fields.numBits);
#endif
    };
    callbackInit.unknownRemote = [](void*)
    {
        LOG_W(MODULE_PREFIX, "  ACKNACK from unknown participant");
    };
    callbackInit.getChatterSeq = &RTPSRunnerAdapter_standardGetChatterSeq;
    callbackInit.executeAction = &RTPSRunnerAdapter_standardExecuteAction;

    RTPSAckNackRunnerCallbacks callbacks;
    RTPSRunnerAdapter_initAckCallbacks(callbacks, callbackInit);

    const auto decisionOptions =
        makeAckNackDecisionOptionsForFlavor(RTPSAckNackRuntimeFlavor::EspStyle);
    RTPSAckNackRunner_run(srcGuidPrefix, pContent, contentLen, decisionOptions, callbacks, &ctx);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Purge stale discovered participants whose lease has expired
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::purgeStaleParticipants()
{
    uint32_t now = millis();
    std::size_t prevCount = _discovered.size();
    for (auto it = _discovered.begin(); it != _discovered.end(); )
    {
        if (RaftRuntime::RTPS::Runtime::DiscoveryRuntime::isLeaseExpired(
            now, it->discoveredTimeMs, it->leaseDurationSec))
        {
#ifdef DEBUG_PURGE_STALE
            LOG_I(MODULE_PREFIX, "purgeStale removing participant guidPfx=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x lease expired (%d sec ago, leaseDur=%us)",
                  it->guidPrefix[0], it->guidPrefix[1], it->guidPrefix[2], it->guidPrefix[3],
                  it->guidPrefix[4], it->guidPrefix[5], it->guidPrefix[6], it->guidPrefix[7],
                  it->guidPrefix[8], it->guidPrefix[9], it->guidPrefix[10], it->guidPrefix[11],
                  (int)((now - it->discoveredTimeMs) / 1000),
                  (unsigned)it->leaseDurationSec);
#endif
            it = _discovered.erase(it);
        }
        else
        {
            ++it;
        }
    }
    if (_discovered.size() != prevCount)
    {
#ifdef DEBUG_PURGE_STALE
        LOG_I(MODULE_PREFIX, "purgeStale count %u->%u",
              (unsigned)prevCount, (unsigned)_discovered.size());
        if (_discovered.empty() && prevCount > 0)
        {
            LOG_W(MODULE_PREFIX, "purgeStale discovered list now EMPTY - chatter/HB will stop (state=%d)",
                  (int)_connState);
        }
#endif
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Periodic writer-heartbeat pass.  Kicked off once per WRITER_HB_INTERVAL_MS by loop(),
// then drained one (peer, step) per loop() iteration so that the ~14 sendto() operations
// that would otherwise fire in a single tick (N peers x ~7 actions + extra-sub slots)
// are spread across that many loop() iterations.  Each step costs <1 ms on ESP32-S3.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::startWriterHeartbeatPass()
{
    // Snapshot counters that don't mutate during the pass (sedpSeqNum, sedpSubSeqNum,
    // chatterSedpSeqNum, rosDiscSeqNum).  heartbeatCount + livelinessSeqNum mutate
    // inside the runner and are written back to the member scalars at pass end.
    _hbPass.counters = RTPSWriterHeartbeatCounterState{
        _sedpSeqNum,
        _sedpSubSeqNum,
        _chatterSedpSeqNum,
        _sedpSubSeqNum + 1,
        _livelinessSeqNum,
        _rosDiscSeqNum,
        _heartbeatCount,
        false,
    };
    _hbPass.sequence = RTPSWriterHeartbeatRunner_buildSequence(
        RTPSWriterHeartbeatRuntimeFlavor::EspStyle);
    _hbPass.peerIdx = 0;
    _hbPass.stepIdx = 0;
    _hbPass.extraSubSlot = 1;
    _hbPass.mainPhaseDoneForPeer = false;
    _hbPass.active = !_discovered.empty();
}

void RaftROS::stepWriterHeartbeatPass()
{
    if (!_hbPass.active)
        return;

    // Advance peerIdx past any peers that have fully completed (main + extra-sub phases).
    // If we've exhausted all peers, finalise the pass here.
    if (_hbPass.peerIdx >= _discovered.size())
    {
        _heartbeatCount = _hbPass.counters.heartbeatCount;
        _livelinessSeqNum = _hbPass.counters.livelinessSeqNum;
        _hbPass.active = false;
        return;
    }

    const DiscoveredParticipant& remote = _discovered[_hbPass.peerIdx];

    // Phase 1: main sequence (one step per tick).
    if (!_hbPass.mainPhaseDoneForPeer)
    {
        struct ExecCtx
        {
            RaftROS* self = nullptr;
            const DiscoveredParticipant* remote = nullptr;
        } execCtx = { this, &remote };

        RTPSWriterHeartbeatRunnerCallbacks callbacks;
        callbacks.buildPayload = [](void* userCtx,
                                    RTPSWriterHeartbeatAction action,
                                    uint64_t sequenceNumber,
                                    uint32_t heartbeatCount) -> uint32_t
        {
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            RaftROS* self = ctx->self;
            const DiscoveredParticipant& remoteRef = *ctx->remote;

            switch (action)
            {
                case RTPSWriterHeartbeatAction::SedpRosDiscoveryPublication:
                    return self->_sedpHandler.buildPublicationMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_WRITER,
                        "ros_discovery_info",
                        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                        RELIABILITY_RELIABLE,
                        DURABILITY_TRANSIENT_LOCAL,
                        sequenceNumber, self->_myIpAddr);
                case RTPSWriterHeartbeatAction::SedpRosDiscoverySubscription:
                    return self->_sedpHandler.buildSubscriptionMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_READER,
                        "ros_discovery_info",
                        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                        RELIABILITY_RELIABLE,
                        DURABILITY_TRANSIENT_LOCAL,
                        sequenceNumber, self->_myIpAddr);
                case RTPSWriterHeartbeatAction::SedpChatterPublication:
                    return self->_sedpHandler.buildPublicationMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_CHATTER_WRITER,
                        CHATTER_DDS_TOPIC,
                        CHATTER_DDS_TYPE,
                        RELIABILITY_RELIABLE,
                        DURABILITY_VOLATILE,
                        sequenceNumber, self->_myIpAddr);
                case RTPSWriterHeartbeatAction::SedpChatterSubscription:
                    return self->_sedpHandler.buildSubscriptionMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_CHATTER_READER,
                        self->_subscriptionTopic.c_str(),
                        self->_subscriptionType.c_str(),
                        RELIABILITY_RELIABLE,
                        DURABILITY_VOLATILE,
                        sequenceNumber, self->_myIpAddr);
                case RTPSWriterHeartbeatAction::ParticipantMessageData:
                    return self->_sedpHandler.buildParticipantMessageData(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        sequenceNumber, heartbeatCount);
                case RTPSWriterHeartbeatAction::RosDiscoveryInfoData:
                {
                    uint8_t rosDiscPayload[256];
                    uint32_t rosDiscLen = self->buildRosDiscInfoWithGids(
                        rosDiscPayload, sizeof(rosDiscPayload));
                    if (rosDiscLen == 0)
                        return 0;
                    return self->_sedpHandler.buildUserDataMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_WRITER,
                        rosDiscPayload, rosDiscLen,
                        sequenceNumber, heartbeatCount);
                }
                default:
                    return 0;
            }
        };

        callbacks.sendPayload = [](void* userCtx,
                                   RTPSWriterHeartbeatSendTarget sendTarget,
                                   uint32_t payloadLen) -> int
        {
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            RaftROS* self = ctx->self;
            const DiscoveredParticipant& remoteRef = *ctx->remote;

            int sock = -1;
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_addr.s_addr = remoteRef.ipAddr;
            if (sendTarget == RTPSWriterHeartbeatSendTarget::Metatraffic)
            {
                sock = self->_metatrafficSock;
                dest.sin_port = htons(remoteRef.metatrafficPort);
            }
            else
            {
                sock = self->_userDataSock;
                dest.sin_port = htons(remoteRef.userDataPort);
            }
            return sendto(sock, self->_sendBuf, payloadLen, 0,
                          (struct sockaddr*)&dest, sizeof(dest));
        };

        callbacks.logSend = [](void* userCtx,
                               RTPSWriterHeartbeatAction action,
                               int sentBytes,
                               uint32_t payloadLen,
                               RTPSWriterHeartbeatSendTarget)
        {
            if (action != RTPSWriterHeartbeatAction::RosDiscoveryInfoData)
                return;
#ifdef RAFTROS_VERBOSE_LOGGING
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            const DiscoveredParticipant& remoteRef = *ctx->remote;
            LOG_I(MODULE_PREFIX, "sendWriterHB rosDisc %d/%d to port %d",
                  sentBytes, (int)payloadLen, (int)remoteRef.userDataPort);
#else
            (void)userCtx; (void)sentBytes; (void)payloadLen;
#endif
        };

        RTPSWriterHeartbeatRunner_runStep(
            RTPSWriterHeartbeatRuntimeFlavor::EspStyle,
            _hbPass.sequence,
            _hbPass.stepIdx,
            _hbPass.counters,
            callbacks,
            &execCtx);

        _hbPass.stepIdx++;
        if (_hbPass.stepIdx >= _hbPass.sequence.numSteps)
        {
            _hbPass.mainPhaseDoneForPeer = true;
            _hbPass.extraSubSlot = 1;
        }
        return;
    }

    // Phase 2: one extra-sub slot (1..N) per tick for the current peer.
    if (_hbPass.extraSubSlot < _subscriptionRegistry.count)
    {
        const uint8_t slot = _hbPass.extraSubSlot;
        const auto& entry = _subscriptionRegistry.entries[slot];
        const uint32_t payloadLen = _sedpHandler.buildSubscriptionMessage(
            _sendBuf, sizeof(_sendBuf),
            _participant, remote.guidPrefix,
            entry.entityId, entry.topic, entry.type,
            RELIABILITY_RELIABLE, DURABILITY_VOLATILE,
            _extraSubscriptionSeqNums[slot], _myIpAddr);
        if (payloadLen != 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(remote.metatrafficPort);
            dest.sin_addr.s_addr = remote.ipAddr;
            (void)sendto(_metatrafficSock, _sendBuf, payloadLen, 0,
                         (struct sockaddr*)&dest, sizeof(dest));
        }
        _hbPass.extraSubSlot++;
        return;
    }

    // Current peer complete - advance to next peer (or finalise on next tick).
    _hbPass.peerIdx++;
    _hbPass.stepIdx = 0;
    _hbPass.extraSubSlot = 1;
    _hbPass.mainPhaseDoneForPeer = false;
    if (_hbPass.peerIdx >= _discovered.size())
    {
        _heartbeatCount = _hbPass.counters.heartbeatCount;
        _livelinessSeqNum = _hbPass.counters.livelinessSeqNum;
        _hbPass.active = false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle a newly discovered participant: enqueue SEDP + liveliness burst to drain one step per loop tick.
// The actual send work is performed by drainPendingAnnounces(), which is called every loop iteration.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr)
{
    if (_pendingAnnounces.size() >= MAX_PENDING_ANNOUNCES)
    {
        LOG_W(MODULE_PREFIX, "handleNewParticipant pending-announce queue FULL (%u) - dropping burst for %02X%02X%02X%02X...",
              (unsigned)_pendingAnnounces.size(),
              remote.guidPrefix[0], remote.guidPrefix[1], remote.guidPrefix[2], remote.guidPrefix[3]);
        return;
    }

    PendingAnnounce entry;
    entry.remote = remote;
    entry.senderAddr = senderAddr;

    RTPSInitialAnnouncePlan announcePlan = RTPSInitialAnnouncePlan_default();
    announcePlan.sendSedpChatterReader = true;
    entry.sequence = RTPSInitialAnnouncePlan_buildSequence(
        announcePlan, RTPSInitialAnnounceRuntimeFlavor::EspStyle);

    entry.runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::EspStyle;
    entry.runCtx.seqCounters = {
        _spdpSeqNum,
        _sedpSeqNum,
        _sedpSubSeqNum,
        _chatterSedpSeqNum,
        _sedpSubSeqNum + 1,
        _livelinessSeqNum,
        _rosDiscSeqNum,
    };
    entry.runCtx.heartbeatCount = _heartbeatCount;
    entry.runCtx.previousPayloadLen = 0;
    entry.stepIdx = 0;
    entry.extraSubSlot = 1;
    entry.mainPhaseDone = false;

    _pendingAnnounces.push_back(entry);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Drain one step of the front pending-announce entry.  Called every loop() tick so the
// initial-announce burst (6+ sends) is spread across that many ticks instead of all firing
// in a single iteration.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::drainPendingAnnounces()
{
    if (_pendingAnnounces.empty())
        return;

    PendingAnnounce& entry = _pendingAnnounces.front();

    struct ExecCtx
    {
        RaftROS* self = nullptr;
        const DiscoveredParticipant* remote = nullptr;
        const struct sockaddr_in* senderAddr = nullptr;
    } execCtx = { this, &entry.remote, &entry.senderAddr };

    if (!entry.mainPhaseDone)
    {
        RTPSInitialAnnounceRunnerCallbacks callbacks;

        callbacks.buildPayload = [](void* userCtx,
                                    const RTPSInitialAnnounceStep& step,
                                    uint64_t sequenceNumber,
                                    uint32_t heartbeatCount,
                                    uint32_t previousPayloadLen) -> uint32_t
        {
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            RaftROS* self = ctx->self;
            const DiscoveredParticipant& remoteRef = *ctx->remote;

            const RTPSInitialAnnounceBuildSpec buildSpec = RTPSInitialAnnouncePlan_getBuildSpec(step.action);
            const uint32_t sedpHeartbeatForBuild = 0;

            switch (buildSpec.buildKind)
            {
                case RTPSInitialAnnounceBuildKind::SpdpAnnouncement:
                    return self->_spdpHandler.buildAnnouncementMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, self->_myIpAddr,
                        self->_leaseDurationSec, sequenceNumber);
                case RTPSInitialAnnounceBuildKind::ReusePrevious:
                    return previousPayloadLen;
                case RTPSInitialAnnounceBuildKind::SedpPublication:
                case RTPSInitialAnnounceBuildKind::SedpSubscription:
                {
                    const RTPSInitialAnnounceSedpEndpointSpec sedpSpec =
                        RTPSInitialAnnouncePlan_getSedpEndpointSpec(buildSpec.sedpEndpointProfile);
                    if (!sedpSpec.entityId)
                        return 0;
                    if (buildSpec.buildKind == RTPSInitialAnnounceBuildKind::SedpPublication)
                    {
                        return self->_sedpHandler.buildPublicationMessage(
                            self->_sendBuf, sizeof(self->_sendBuf),
                            self->_participant, remoteRef.guidPrefix,
                            sedpSpec.entityId, sedpSpec.topicName, sedpSpec.typeName,
                            sedpSpec.reliabilityKind, sedpSpec.durabilityKind,
                            sequenceNumber, self->_myIpAddr, sedpHeartbeatForBuild);
                    }
                    return self->_sedpHandler.buildSubscriptionMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        sedpSpec.entityId,
                        (buildSpec.sedpEndpointProfile == RTPSInitialAnnounceSedpEndpointProfile::ChatterReader)
                            ? self->_subscriptionTopic.c_str() : sedpSpec.topicName,
                        (buildSpec.sedpEndpointProfile == RTPSInitialAnnounceSedpEndpointProfile::ChatterReader)
                            ? self->_subscriptionType.c_str() : sedpSpec.typeName,
                        sedpSpec.reliabilityKind, sedpSpec.durabilityKind,
                        sequenceNumber, self->_myIpAddr, sedpHeartbeatForBuild);
                }
                case RTPSInitialAnnounceBuildKind::ParticipantMessageData:
                    return self->_sedpHandler.buildParticipantMessageData(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        sequenceNumber, heartbeatCount);
                case RTPSInitialAnnounceBuildKind::RosDiscoveryUserData:
                case RTPSInitialAnnounceBuildKind::None:
                default:
                    return 0;
            }
        };

        callbacks.sendPayload = [](void* userCtx,
                                   RTPSInitialAnnounceAction action,
                                   uint32_t payloadLen) -> int
        {
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            RaftROS* self = ctx->self;
            const DiscoveredParticipant& remoteRef = *ctx->remote;
            const struct sockaddr_in& senderAddrRef = *ctx->senderAddr;

            if (payloadLen == 0)
                return -1;
            const RTPSInitialAnnounceSendTarget target = RTPSInitialAnnouncePlan_getSendTarget(action);

            int sock = -1;
            switch (target.socket)
            {
                case RTPSInitialAnnounceSocket::Spdp:
                    sock = self->_spdpSock;
                    break;
                case RTPSInitialAnnounceSocket::Metatraffic:
                    sock = self->_metatrafficSock;
                    break;
                case RTPSInitialAnnounceSocket::UserData:
                    sock = self->_userDataSock;
                    break;
                default:
                    break;
            }
            if (sock < 0)
                return -1;

            struct sockaddr_in dest = {};
            switch (target.addressing)
            {
                case RTPSInitialAnnounceAddressing::SenderAddr:
                    dest = senderAddrRef;
                    break;
                case RTPSInitialAnnounceAddressing::SenderAddrWithSpdpPort:
                    dest = senderAddrRef;
                    dest.sin_port = htons(self->_participant.getSPDPMulticastPort());
                    break;
                case RTPSInitialAnnounceAddressing::RemoteMetatrafficUnicast:
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(remoteRef.metatrafficPort);
                    dest.sin_addr.s_addr = remoteRef.ipAddr;
                    break;
                case RTPSInitialAnnounceAddressing::RemoteUserDataUnicast:
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(remoteRef.userDataPort);
                    dest.sin_addr.s_addr = remoteRef.ipAddr;
                    break;
                default:
                    return -1;
            }

            return sendto(sock, self->_sendBuf, payloadLen, 0,
                          (struct sockaddr*)&dest, sizeof(dest));
        };

        callbacks.logResult = [](void* userCtx,
                                 const RTPSInitialAnnounceStep&,
                                 const RTPSInitialAnnounceLogSpec& logSpec,
                                 const RTPSInitialAnnounceSendResultSpec& sendResult,
                                 int sentBytes,
                                 uint32_t payloadLen)
        {
#ifdef DEBUG_PARTICIPANT_PROCESSING
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            const DiscoveredParticipant& remoteRef = *ctx->remote;
            const struct sockaddr_in& senderAddrRef = *ctx->senderAddr;

            const char* statusPrefix = sendResult.isFailure ? "FAILED " : "";
            if (logSpec.logToSenderAddress)
            {
                LOG_I(MODULE_PREFIX, "handleNewParticipant %s%s sent %d/%d bytes to %s:%d",
                      statusPrefix,
                      logSpec.actionLabel,
                      sentBytes, (int)payloadLen,
                      inet_ntoa(((struct sockaddr_in&)senderAddrRef).sin_addr),
                      (int)ntohs(senderAddrRef.sin_port));
            }
            else
            {
                LOG_I(MODULE_PREFIX, "handleNewParticipant %s%s sent %d/%d bytes to port %d",
                      statusPrefix,
                      logSpec.actionLabel,
                      sentBytes, (int)payloadLen, (int)remoteRef.metatrafficPort);
            }
#else
            (void)userCtx; (void)logSpec; (void)sendResult; (void)sentBytes; (void)payloadLen;
#endif
        };

        RTPSInitialAnnounceRunner_runStep(
            entry.sequence, entry.stepIdx, entry.runCtx, callbacks, &execCtx);
        entry.stepIdx++;

        if (entry.stepIdx >= entry.sequence.numSteps)
        {
            // Main sequence complete - persist the mutated counters back to the wrapper.
            _spdpSeqNum = entry.runCtx.seqCounters.spdpSeqNum;
            _heartbeatCount = entry.runCtx.heartbeatCount;
            entry.mainPhaseDone = true;
        }
        return;
    }

    // Extra-subscription-slot phase: emit one SEDP sub announce per tick for slots 1..N.
    if (entry.extraSubSlot < _subscriptionRegistry.count)
    {
        const uint8_t slot = entry.extraSubSlot;
        const auto& regEntry = _subscriptionRegistry.entries[slot];
        const uint32_t payloadLen = _sedpHandler.buildSubscriptionMessage(
            _sendBuf, sizeof(_sendBuf),
            _participant, entry.remote.guidPrefix,
            regEntry.entityId, regEntry.topic, regEntry.type,
            RELIABILITY_RELIABLE, DURABILITY_VOLATILE,
            _extraSubscriptionSeqNums[slot], _myIpAddr);
        if (payloadLen > 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(entry.remote.metatrafficPort);
            dest.sin_addr.s_addr = entry.remote.ipAddr;
#ifdef DEBUG_PARTICIPANT_PROCESSING
            const int sent = sendto(_metatrafficSock, _sendBuf, payloadLen, 0,
                                    (struct sockaddr*)&dest, sizeof(dest));
            LOG_I(MODULE_PREFIX,
                  "handleNewParticipant extra SEDP sub slot=%u topic=%s sent %d/%d bytes to port %d",
                  (unsigned)slot, regEntry.topic, sent, (int)payloadLen,
                  (int)entry.remote.metatrafficPort);
#else
            (void)sendto(_metatrafficSock, _sendBuf, payloadLen, 0,
                         (struct sockaddr*)&dest, sizeof(dest));
#endif
        }
        entry.extraSubSlot++;
        return;
    }

    // All phases complete - remove this entry.
    _pendingAnnounces.erase(_pendingAnnounces.begin());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: build ros_discovery_info payload with our writer GIDs included
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RaftROS::buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen)
{
    const uint8_t* writerIds[] = { ENTITYID_CHATTER_WRITER };

    // Collect reader GIDs from the registry: slot 0 (default CHATTER_READER) plus any
    // additional subscriptions appended via addStringSubscription().
    using RaftRuntime::RTPS::Runtime::Dispatch::RTPS_SUBSCRIPTION_REGISTRY_CAPACITY;
    const uint8_t* readerIds[RTPS_SUBSCRIPTION_REGISTRY_CAPACITY] = {0};
    const uint32_t numReaderIds = _subscriptionRegistry.readerEntityIds(
        readerIds, RTPS_SUBSCRIPTION_REGISTRY_CAPACITY);

    return SPDPHandler::buildRosDiscoveryInfoPayload(
        pBuf, bufLen,
        _participant.getParticipantGuid(),
        _nodeName.c_str(),
        _nodeNamespace.c_str(),
        writerIds, 1,
        readerIds, numReaderIds);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build a CDR-encoded std_msgs/String payload
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RaftROS::buildChatterPayload(uint8_t* pBuf, uint32_t bufLen, const char* message)
{
    uint32_t msgLen = (uint32_t)strlen(message) + 1;  // include null terminator
    uint32_t totalLen = 4 + 4 + ((msgLen + 3) & ~3u);  // CDR encap + string length + data + pad
    if (bufLen < totalLen)
        return 0;

    uint32_t pos = 0;

    // CDR Encapsulation Header (CDR_LE)
    pBuf[pos++] = 0x00;
    pBuf[pos++] = 0x01;
    pBuf[pos++] = 0x00;
    pBuf[pos++] = 0x00;

    // CDR string: uint32_t length (including null), then chars + null + pad
    pBuf[pos++] = msgLen & 0xFF;
    pBuf[pos++] = (msgLen >> 8) & 0xFF;
    pBuf[pos++] = (msgLen >> 16) & 0xFF;
    pBuf[pos++] = (msgLen >> 24) & 0xFF;
    memcpy(pBuf + pos, message, msgLen);
    pos += msgLen;
    while (pos % 4 != 0) pBuf[pos++] = 0;

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Publish a chatter message to all discovered participants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::publishChatter()
{
    // Build message string
    char msgStr[64];
    snprintf(msgStr, sizeof(msgStr), "Hello from %s [%lu]", _nodeName.c_str(), (unsigned long)_chatterMsgIndex++);

    // CDR-encode the std_msgs/String
    uint8_t chatterPayload[256];
    uint32_t payloadLen = buildChatterPayload(chatterPayload, sizeof(chatterPayload), msgStr);
    if (payloadLen == 0)
        return;

    _chatterSeqNum++;

    for (const auto& remote : _discovered)
    {
        _heartbeatCount++;
        uint32_t msgLen = _sedpHandler.buildUserDataMessage(
            _sendBuf, sizeof(_sendBuf),
            _participant, remote.guidPrefix,
            ENTITYID_CHATTER_WRITER,
            chatterPayload, payloadLen,
            _chatterSeqNum, _heartbeatCount,
            _chatterSeqNum /* firstSN = current for VOLATILE QoS */);

        if (msgLen > 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(remote.userDataPort);
            dest.sin_addr.s_addr = remote.ipAddr;
            int sent = sendto(_userDataSock, _sendBuf, msgLen, 0,
                              (struct sockaddr*)&dest, sizeof(dest));
#ifdef DEBUG_PUBLISH_CHATTER
            char destIpStr[16];
            strncpy(destIpStr, inet_ntoa(*(struct in_addr*)&remote.ipAddr), sizeof(destIpStr));
            destIpStr[sizeof(destIpStr)-1] = '\0';
            LOG_I(MODULE_PREFIX, "publishChatter seq=%llu \"%s\" sent %d/%d to %s:%d (discCount=%u)",
                  (unsigned long long)_chatterSeqNum, msgStr, sent, (int)msgLen,
                  destIpStr, (int)remote.userDataPort,
                  (unsigned)_discovered.size());
#else
            (void)sent;
#endif
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// REST API
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
        case ConnState::ANNOUNCING:   stStr = "announcing"; break;
        case ConnState::ACTIVE:       stStr = "active"; break;
    }
    char buf[256];
    snprintf(buf, sizeof(buf),
             R"({"rslt":"ok","en":%s,"domId":%d,"node":"%s","ns":"%s","conn":"%s","disc":%d,"spdpSeq":%llu})",
             _isEnabled ? "true" : "false",
             (int)_domainId,
             _nodeName.c_str(),
             _nodeNamespace.c_str(),
             stStr,
             (int)_discovered.size(),
             (unsigned long long)_spdpSeqNum);
    return buf;
}

RaftRetCode RaftROS::apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo)
{
    respStr = getStatusJSON();
    return RaftRetCode::RAFT_OK;
}
