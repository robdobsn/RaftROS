/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework (Phase 1: Discoverable Node)
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftROSRTPS.h"
#include "runtime/dispatch/RTPSUserDispatch.h"
#include "runtime/autopub/RTPSAutoPubTopicNaming.h"
#include "runtime/autopub/RTPSAutoPubClassMap.h"
#include "runtime/autopub/RTPSAutoPubCDRSerializer.h"
#include "runtime/autopub/RTPSAutoPubQoSProfile.h"
#include "runtime/autopub/RTPSAutoPubSampleEmitter.h"
#include "RaftJson.h"
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
#include "SysManagerIF.h"
#include "DeviceManager.h"
#include "DeviceTypeRecords.h"
#include "DevicePollingInfo.h"
#include "BusAddrStatus.h"
#include "RaftBus.h"
#include "RaftDevice.h"
#include "esp_timer.h"

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

namespace
{

void patchRosDiscoveryInfoReaderId(uint8_t* packet, uint32_t packetLen, const uint8_t* readerEntityId)
{
    if (!packet || !readerEntityId)
        return;

    uint8_t srcGuidPrefix[12];
    uint32_t offset = RTPSMessage::parseHeader(packet, packetLen, srcGuidPrefix);
    if (offset == 0)
        return;

    while (offset < packetLen)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContentConst = nullptr;
        uint32_t contentLen = 0;
        const uint32_t submsgSize = RTPSMessage::parseSubmessage(
            packet + offset, packetLen - offset, submsgId, flags, pContentConst, contentLen);
        if (submsgSize == 0)
            break;

        uint8_t* pContent = const_cast<uint8_t*>(pContentConst);
        if (submsgId == SUBMSG_DATA && contentLen >= 12 &&
            memcmp(pContent + 8, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0)
        {
            memcpy(pContent + 4, readerEntityId, 4);
        }
        else if (submsgId == SUBMSG_HEARTBEAT && contentLen >= 8 &&
                 memcmp(pContent + 4, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0)
        {
            memcpy(pContent, readerEntityId, 4);
        }

        offset += submsgSize;
    }
}

bool addRdiReaderEntityIdCandidate(DiscoveredParticipant& dp, const uint8_t* readerEntityId)
{
    if (!readerEntityId)
        return false;
    for (uint8_t i = 0; i < dp.rdiReaderEntityIdCount; i++)
    {
        if (memcmp(dp.rdiReaderEntityIds[i], readerEntityId, 4) == 0)
            return false;
    }
    if (dp.rdiReaderEntityIdCount >= DiscoveredParticipant::RDI_READER_ENTITY_ID_CANDIDATE_CAPACITY)
        return false;
    memcpy(dp.rdiReaderEntityIds[dp.rdiReaderEntityIdCount], readerEntityId, 4);
    dp.rdiReaderEntityIdCount++;
    return true;
}

} // namespace

// Slice 4.11 — ensure the QoS mirrors in the header-only autopub profile match
// the canonical RTPS enum values so the stored profile id can be translated
// without hidden drift between the two headers.
static_assert(
    RaftRuntime::RTPS::Runtime::AutoPub::RTPS_AUTOPUB_RELIABILITY_BEST_EFFORT ==
        RELIABILITY_BEST_EFFORT,
    "RTPS_AUTOPUB_RELIABILITY_BEST_EFFORT mirror drift");
static_assert(
    RaftRuntime::RTPS::Runtime::AutoPub::RTPS_AUTOPUB_RELIABILITY_RELIABLE ==
        RELIABILITY_RELIABLE,
    "RTPS_AUTOPUB_RELIABILITY_RELIABLE mirror drift");
static_assert(
    RaftRuntime::RTPS::Runtime::AutoPub::RTPS_AUTOPUB_DURABILITY_VOLATILE ==
        DURABILITY_VOLATILE,
    "RTPS_AUTOPUB_DURABILITY_VOLATILE mirror drift");
static_assert(
    RaftRuntime::RTPS::Runtime::AutoPub::RTPS_AUTOPUB_DURABILITY_TRANSIENT_LOCAL ==
        DURABILITY_TRANSIENT_LOCAL,
    "RTPS_AUTOPUB_DURABILITY_TRANSIENT_LOCAL mirror drift");

// Verbose per-packet debug logging.  Each of these emits multiple LOG_I lines per event
// which accumulates to tens of ms of stalled loop() time when the ESP32-S3 is handling
// multiple DDS peers.  Comment out RAFTROS_VERBOSE_LOGGING for production / when
// profiling loop() latency.
//
// Logging is not free even when nothing is reading the console: with the USB serial
// console unattached, writes block the calling task.  Measured on an ESP32-S3 with one
// peer, the discovery/ACKNACK logs below emitted ~4.5 lines/s which stalled loop() past
// the 200ms sensor sample interval often enough to lose ~5% of auto-published samples.
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
    #define DEBUG_SEDP_SUB_ACK
    #define DEBUG_AUTOPUB_SEDP_ANNOUNCE
    #define DEBUG_ROS_DISCOVERY_INFO
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

    // Slice 4.11 — parse any per-device / per-class QoS overrides up-front.
    autoPubParseQoSOverrides();

    // Check enabled
    if (!_isEnabled)
    {
        LOG_I(MODULE_PREFIX, "setup DISABLED");
        return;
    }

    // Initialize RTPS participant (GUID will be set when MAC is available)
    _participant.init(_domainId, _nodeName.c_str(), 0, nullptr);

    // Wire the auto-publish backend to the RTPS objects it sends with.  The
    // socket is passed by pointer because it is recreated on reconnect.
    {
        RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubBackendDeps deps;
        deps.participant = &_participant;
        deps.sedpHandler = &_sedpHandler;
        deps.discovered = &_discovered;
        deps.userDataSock = &_userDataSock;
        deps.sendBuf = _autoPubSendBuf;
        deps.sendBufLen = sizeof(_autoPubSendBuf);
        deps.metatrafficSock = &_metatrafficSock;
        deps.metaSendBuf = _sendBuf;
        deps.metaSendBufLen = sizeof(_sendBuf);
        deps.myIpAddr = &_myIpAddr;
        _autoPubBackend.setup(deps);
    }

    // Phase 4 / Slice 4.3 — register with DeviceManager so we get notified every
    // time a bus device comes online or goes offline.  Writers and SEDP announce
    // are still deferred (later slices); for now the hook is used to decode poll
    // data and log it so we can validate the plumbing on hardware.
    SysManagerIF* pSysMan = getSysManager();
    DeviceManager* pDevMan = pSysMan ? pSysMan->getDeviceManager() : nullptr;
    if (pDevMan)
    {
        pDevMan->registerForDeviceStatusChange(
            [this](RaftDevice& device, const BusAddrStatus& addrStatus) {
                this->autoPubOnDeviceStatusChange(device, addrStatus);
            });
        _autoPubStatusCBRegistered = true;
        LOG_I(MODULE_PREFIX, "setup auto-publish listener registered with DeviceManager");
    }
    else
    {
        LOG_W(MODULE_PREFIX, "setup no DeviceManager — auto-publishing disabled");
    }

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

    // Emit the latest auto-published sample of every device whose bus
    // callback has stored one since the previous pass.  Runs in every state,
    // as the bus-task path did: with no discovered peers nothing is sent but
    // the writer sequence still advances once per sample.
    autoPubDrainSamples();

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
            // Auto-publish diagnostic: always emit (not gated on DEBUG) so we
            // can see registry occupancy + ros_discovery_info seq across time
            // even on release builds while investigating the graph-visibility
            // regression.
            const auto poolCounters = _autoPubPool.counters();
            LOG_I(MODULE_PREFIX,
                  "autoPubStatus slots=%u/%u devices=%u discovered=%u rosDiscSeq=%u state=%d stale=%u busy=%u empty=%u drainSkips=%u drainGapMax=%ums drainGaps>150ms=%u",
                  (unsigned)_autoPubBackend.lifecycle().inUseCount(),
                  (unsigned)RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY,
                  (unsigned)_autoPubPool.inUseCount(),
                  (unsigned)_discovered.size(),
                  (unsigned)_rosDiscSeqNum, (int)_connState,
                  (unsigned)poolCounters.staleDrops, (unsigned)poolCounters.busyDrops,
                  (unsigned)poolCounters.emptyFills, (unsigned)poolCounters.drainBusySkips,
                  (unsigned)(_autoPubDrainGapMaxUs / 1000), (unsigned)_autoPubDrainGapsOver150ms);
            _autoPubDrainGapMaxUs = 0;
            _autoPubDrainGapsOver150ms = 0;
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
    // Receive a bounded batch per loop tick. FastDDS discovery often arrives
    // in bursts, and this socket also carries metatraffic multicast DATA.
    for (uint8_t rxBudget = 0; rxBudget < 8; rxBudget++)
    {
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
            // FastDDS may send SEDP builtin endpoint DATA on the metatraffic
            // multicast port (7400), which shares this socket with SPDP. If this
            // is not a participant announcement, feed it through the ordinary
            // metatraffic parser instead of dropping it.
            processMetatrafficPacket(_recvBuf, (uint32_t)n, fromAddr);
            continue;
        }

        // Skip our own announcements
        if (memcmp(remote.guidPrefix, _participant.getGuidPrefix(), 12) == 0)
            continue;

        processDiscoveredParticipant(remote, fromAddr);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Process a discovered participant (from SPDP multicast or metatraffic unicast)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr)
{
    uint32_t nowMs = millis();

    // Override parsed locator when it isn't reachable from us. Hosts that sit
    // on multiple interfaces (very common with WSL2 / Docker / Windows with
    // Hyper-V) advertise every local IP in their SPDP ParticipantProxyData —
    // including 169.254.x.x APIPA addresses from disabled virtual adapters.
    // If we pick one of those, sendto() fails with -1 (no route to host) and
    // SEDP never reaches the peer. The canonical RTPS fallback is the UDP
    // source address of the packet itself, which by definition is reachable
    // because we just received from it.
    {
        const uint32_t parsed = remote.ipAddr;           // network order
        const uint32_t parsedHostOrder = ntohl(parsed);
        const bool parsedIsLinkLocal = (parsedHostOrder & 0xFFFF0000u) == 0xA9FE0000u; // 169.254/16
        const bool parsedIsLoopback  = (parsedHostOrder & 0xFF000000u) == 0x7F000000u; // 127/8
        const bool parsedIsZero      = (parsed == 0);
        if ((parsedIsLinkLocal || parsedIsLoopback || parsedIsZero) &&
            fromAddr.sin_addr.s_addr != 0)
        {
            // Throttle: only log when the (parsed, sender) pair changes, so
            // ordinary steady-state SPDP re-announces don't spam the log.
            static uint32_t sLastParsed = 0;
            static uint32_t sLastSender = 0;
            if (parsed != sLastParsed || fromAddr.sin_addr.s_addr != sLastSender)
            {
                LOG_I(MODULE_PREFIX,
                      "overriding unreachable SPDP locator %08x with senderIp %s",
                      (unsigned)parsed, inet_ntoa(fromAddr.sin_addr));
                sLastParsed = parsed;
                sLastSender = fromAddr.sin_addr.s_addr;
            }
            remote.ipAddr = fromAddr.sin_addr.s_addr;
        }
    }

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
    // FastDDS sends SPDP plus several SEDP DATA(r/w) samples in a tight burst
    // when a new participant appears. Drain enough packets per loop to avoid
    // losing the full DATA(r) locator sample and later seeing only key-only
    // retransmits.
    for (uint8_t rxBudget = 0; rxBudget < 32; rxBudget++)
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

        processMetatrafficPacket(_recvBuf, (uint32_t)n, fromAddr);
    }
}

void RaftROS::processMetatrafficPacket(const uint8_t* packet, uint32_t packetLen, const struct sockaddr_in& fromAddr)
{
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
    // FastDDS may source builtin HEARTBEATs from a process-local UDP socket
    // rather than the participant's metatraffic unicast port. Reply to the
    // source tuple so SEDP retransmit ACKNACKs reach the writer that sent the HB.
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
#ifdef DEBUG_SEDP_SUB_ACK
        if (responded && memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
        {
            LOG_I(MODULE_PREFIX,
                  "SEDP sub HEARTBEAT lastSN=%u -> ACKNACK %d bytes",
                  (unsigned)lastSNLow, sentBytes);
        }
#endif
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
    callbacks.onHeartbeatDecision = [](void*, RTPSRxChannel,
                                       const uint8_t* writerEID,
                                       const RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatFields& fields,
                                       const RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatDecision& decision,
                                       bool responded,
                                       int sentBytes)
    {
#ifdef DEBUG_SEDP_SUB_ACK
        if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) != 0)
            return;
        LOG_I(MODULE_PREFIX,
              "SEDP sub ACKDEC first=%u last=%u hbCount=%u final=%d ackBase=%u bits=%u bitmap=0x%08x%08x responded=%d sent=%d",
              (unsigned)(fields.firstSN & 0xFFFFFFFFu),
              (unsigned)(fields.lastSN & 0xFFFFFFFFu),
              (unsigned)fields.count,
              (int)fields.finalFlag,
              (unsigned)(decision.ackNackBase & 0xFFFFFFFFu),
              (unsigned)decision.ackNackNumBits,
              (unsigned)((decision.ackNackBitmap >> 32) & 0xFFFFFFFFu),
              (unsigned)(decision.ackNackBitmap & 0xFFFFFFFFu),
              (int)responded,
              sentBytes);
#endif
    };
    callbacks.onAckNackBuilt = [](void*, RTPSRxChannel,
                                  const uint8_t* readerEID,
                                  const uint8_t* writerEID,
                                  const RaftRuntime::RTPS::Runtime::Reader::RTPSReaderHeartbeatDecision& decision,
                                  const uint8_t* ackBuf,
                                  uint32_t ackLen,
                                  const struct sockaddr_in& fromAddr,
                                  const struct sockaddr_in& destAddr)
    {
#ifdef DEBUG_SEDP_SUB_ACK
        static uint16_t sedpSubAckLogCount = 0;
        if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) != 0)
            return;
        sedpSubAckLogCount++;
        if ((sedpSubAckLogCount > 12) && ((sedpSubAckLogCount % 32) != 0))
            return;
        const uint32_t subOff = 36; // RTPS header (20) + INFO_DST (16).
        if (ackLen < subOff + 32)
            return;
        char srcIp[16] = {0};
        char dstIp[16] = {0};
        snprintf(srcIp, sizeof(srcIp), "%s", inet_ntoa(fromAddr.sin_addr));
        snprintf(dstIp, sizeof(dstIp), "%s", inet_ntoa(destAddr.sin_addr));
        LOG_I(MODULE_PREFIX,
              "SEDP sub ACKWIRE src=%s:%u dst=%s:%u reader=%02x%02x%02x%02x writer=%02x%02x%02x%02x base=%u bits=%u sub=%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
              srcIp,
              (unsigned)ntohs(fromAddr.sin_port),
              dstIp,
              (unsigned)ntohs(destAddr.sin_port),
              readerEID[0], readerEID[1], readerEID[2], readerEID[3],
              writerEID[0], writerEID[1], writerEID[2], writerEID[3],
              (unsigned)(decision.ackNackBase & 0xFFFFFFFFu),
              (unsigned)decision.ackNackNumBits,
              ackBuf[subOff + 0], ackBuf[subOff + 1], ackBuf[subOff + 2], ackBuf[subOff + 3],
              ackBuf[subOff + 4], ackBuf[subOff + 5], ackBuf[subOff + 6], ackBuf[subOff + 7],
              ackBuf[subOff + 8], ackBuf[subOff + 9], ackBuf[subOff + 10], ackBuf[subOff + 11],
              ackBuf[subOff + 12], ackBuf[subOff + 13], ackBuf[subOff + 14], ackBuf[subOff + 15],
              ackBuf[subOff + 16], ackBuf[subOff + 17], ackBuf[subOff + 18], ackBuf[subOff + 19],
              ackBuf[subOff + 20], ackBuf[subOff + 21], ackBuf[subOff + 22], ackBuf[subOff + 23],
              ackBuf[subOff + 24], ackBuf[subOff + 25], ackBuf[subOff + 26], ackBuf[subOff + 27],
              ackBuf[subOff + 28], ackBuf[subOff + 29], ackBuf[subOff + 30], ackBuf[subOff + 31]);
#endif
    };
    callbacks.onData = [](void* userCtx,
                          RTPSRxChannel,
                          const uint8_t* packet,
                          uint32_t packetLen,
                          const uint8_t*,
                          const struct sockaddr_in& from,
                          const uint8_t* pContent,
                          uint32_t contentLen,
                          uint8_t dataFlags)
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
        else if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
        {
            // SEDP publication announce from remote.  Parse the advertised topic/writerGuid
            // and, if the topic matches one of our registered subscriptions, record the
            // (remote writerGuid -> local slot) mapping so recvUserData() can route
            // per-topic to the correct handler.
            if (contentLen < 24)
                return;
            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(pContent, contentLen, dataFlags, pPayload, payloadLen);
            if (!pPayload || payloadLen < 4)
            {
                // Either malformed, or an inline-QoS-only DATA submessage
                // (e.g. publication disposal/unregister: D=0, Q=1, K=1) with
                // no serialized payload. Nothing to parse.
                return;
            }
            RaftRuntime::RTPS::Runtime::Dispatch::RTPSParsedPublicationAnnounce parsed;
            if (!RaftRuntime::RTPS::Runtime::Dispatch::RTPSSEDPPublicationParser_parse(
                    pPayload, payloadLen, parsed))
                return;
            if (!parsed.topic || parsed.topicLen == 0)
                return;
            for (uint8_t slot = 0; slot < self->_subscriptionRegistry.count; slot++)
            {
                const char* regTopic = self->_subscriptionRegistry.entries[slot].topic;
                if (!regTopic) continue;
                const size_t regLen = strlen(regTopic);
                if (regLen == parsed.topicLen &&
                    memcmp(regTopic, parsed.topic, regLen) == 0)
                {
                    const bool wasNew = self->_remotePubMap.upsert(parsed.writerGuid, (int8_t)slot);
#ifdef DEBUG_SDSP_DATA
                    LOG_I(MODULE_PREFIX,
                          "  SEDP pub matched topic='%.*s' -> slot=%u (%s)",
                          (int)parsed.topicLen, parsed.topic, (unsigned)slot,
                          wasNew ? "new" : "updated");
#else
                    (void)wasNew;
#endif
                    break;
                }
            }
        }
        else if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
        {
            // SEDP subscription announce from remote.  We extract the per-reader
            // PID_UNICAST_LOCATOR for the remote `ros_discovery_info` reader and
            // record its port on the matching DiscoveredParticipant.
            //
            // Why: rmw_fastrtps creates the rdi subscription with
            // `require_unique_network_flow_endpoints = OPTIONALLY_REQUIRED`.
            // FastDDS honors that by binding the rdi reader to its OWN UDP socket,
            // distinct from the participant default user-data port. If we send
            // rdi DATA to the default user-data port (RTPS port formula), the
            // payload is silently discarded at the UDP demuxer because no reader
            // is bound there. Result: rdi writer is reported by the host's
            // graph cache as `_NODE_NAME_UNKNOWN_` even though SEDP matched.
            if (contentLen < 24)
                return;
            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(pContent, contentLen, dataFlags, pPayload, payloadLen);
            if (!pPayload || payloadLen < 4)
            {
                const uint8_t* keyHash = nullptr;
                const uint8_t* inlineQos = nullptr;
                uint32_t inlineQosLen = 0;
                if ((dataFlags & 0x02) && contentLen >= 20)
                {
                    const uint32_t inlineQosOff =
                        4u + (uint32_t)(pContent[2] | (pContent[3] << 8));
                    if (inlineQosOff < contentLen)
                    {
                        inlineQos = pContent + inlineQosOff;
                        inlineQosLen = contentLen - inlineQosOff;
                    }

                    uint32_t off = inlineQosOff;
                    while (off + 4 <= contentLen)
                    {
                        const uint16_t pid = (uint16_t)(pContent[off] | (pContent[off + 1] << 8));
                        const uint16_t plen = (uint16_t)(pContent[off + 2] | (pContent[off + 3] << 8));
                        off += 4;
                        if (pid == PID_SENTINEL)
                            break;
                        if ((uint32_t)off + plen > contentLen)
                            break;
                        if (pid == PID_KEY_HASH && plen >= 16)
                        {
                            keyHash = pContent + off;
                            break;
                        }
                        off += plen;
                    }
                }
                RaftRuntime::RTPS::Runtime::Dispatch::RTPSParsedSubscriptionAnnounce inlineSub;
                const bool inlineParseOk =
                    RaftRuntime::RTPS::Runtime::Dispatch::RTPSSEDPSubscriptionParser_parseParameterList(
                        inlineQos, inlineQosLen, inlineSub);
                static constexpr const char* RDI_TOPIC = "ros_discovery_info";
                static constexpr uint32_t RDI_TOPIC_LEN = 18;
                if (inlineParseOk &&
                    inlineSub.topic &&
                    inlineSub.topicLen == RDI_TOPIC_LEN &&
                    memcmp(inlineSub.topic, RDI_TOPIC, RDI_TOPIC_LEN) == 0)
                {
                    const uint8_t* senderGuidPfx =
                        inlineSub.hasReaderGuid ? inlineSub.readerGuid : (packet + 8);
                    bool foundInDiscovered = false;
                    for (auto& dp : self->_discovered)
                    {
                        if (memcmp(dp.guidPrefix, senderGuidPfx, 12) == 0)
                        {
                            foundInDiscovered = true;
                            if (inlineSub.unicastLocatorPort != 0 &&
                                dp.rdiReaderUnicastPort != inlineSub.unicastLocatorPort)
                            {
                                LOG_I(MODULE_PREFIX,
                                      "SEDP inline rdi reader port for participant %02x%02x%02x%02x: %u (was %u)",
                                      dp.guidPrefix[0], dp.guidPrefix[1],
                                      dp.guidPrefix[2], dp.guidPrefix[3],
                                      (unsigned)inlineSub.unicastLocatorPort,
                                      (unsigned)dp.rdiReaderUnicastPort);
                                dp.rdiReaderUnicastPort = inlineSub.unicastLocatorPort;
                            }
                            if (inlineSub.hasReaderGuid)
                                (void)addRdiReaderEntityIdCandidate(dp, inlineSub.readerGuid + 12);
                            break;
                        }
                    }
                    if (!foundInDiscovered)
                    {
                        DiscoveredParticipant partial = {};
                        memcpy(partial.guidPrefix, senderGuidPfx, 12);
                        partial.ipAddr = from.sin_addr.s_addr;
                        partial.rdiReaderUnicastPort = inlineSub.unicastLocatorPort;
                        if (inlineSub.hasReaderGuid)
                            (void)addRdiReaderEntityIdCandidate(partial, inlineSub.readerGuid + 12);
                        LOG_I(MODULE_PREFIX,
                              "SEDP inline rdi partial-disc guidPfx=%02x%02x%02x%02x rdiPort=%u",
                              partial.guidPrefix[0], partial.guidPrefix[1],
                              partial.guidPrefix[2], partial.guidPrefix[3],
                              (unsigned)inlineSub.unicastLocatorPort);
                        self->processDiscoveredParticipant(partial, from);
                    }
                    return;
                }
                if (keyHash &&
                    memcmp(keyHash + 12, ENTITYID_ROS_DISC_INFO_READER, 4) == 0)
                {
                    for (auto& dp : self->_discovered)
                    {
                        if (memcmp(dp.guidPrefix, keyHash, 12) == 0 &&
                            addRdiReaderEntityIdCandidate(dp, keyHash + 12))
                        {
                            break;
                        }
                    }
                }
                return;
            }
            RaftRuntime::RTPS::Runtime::Dispatch::RTPSParsedSubscriptionAnnounce parsedSub;
            const bool subParseOk =
                RaftRuntime::RTPS::Runtime::Dispatch::RTPSSEDPSubscriptionParser_parse(
                    pPayload, payloadLen, parsedSub);
            if (!subParseOk)
                return;
            // Filter on topic name == "ros_discovery_info" (the rdi reader is
            // the only one for which a per-reader port matters; other readers
            // share the participant default user-data port).
            static constexpr const char* RDI_TOPIC = "ros_discovery_info";
            static constexpr uint32_t RDI_TOPIC_LEN = 18;
            if (!parsedSub.topic ||
                parsedSub.topicLen != RDI_TOPIC_LEN ||
                memcmp(parsedSub.topic, RDI_TOPIC, RDI_TOPIC_LEN) != 0)
                return;
            if (parsedSub.hasReaderGuid)
            {
                LOG_I(MODULE_PREFIX,
                      "SEDP rdi reader guidPfx=%02x%02x%02x%02x readerEID=%02x%02x%02x%02x port=%u",
                      parsedSub.readerGuid[0], parsedSub.readerGuid[1],
                      parsedSub.readerGuid[2], parsedSub.readerGuid[3],
                      parsedSub.readerGuid[12], parsedSub.readerGuid[13],
                      parsedSub.readerGuid[14], parsedSub.readerGuid[15],
                      (unsigned)parsedSub.unicastLocatorPort);
            }
            // Locate the matching DiscoveredParticipant by readerGuid prefix
            // (first 12 bytes of the reader GUID == participant guidPrefix).
            // Some FastDDS graph readers use the participant default user-data
            // port and therefore do not advertise a per-reader locator here.
            // Still record their entityId so the RDI DATA fanout can address
            // the reader explicitly instead of relying on ENTITYID_UNKNOWN.
            //
            // Fix #9: WiFi/IGMP snooping can cause SPDP multicast from new
            // participants to never reach the ESP (group membership expiry on the
            // AP). In that case the participant is absent from _discovered even
            // though it has already started sending SEDP directly to ESP:7410
            // (because it DID receive the ESP's own SPDP multicast and learned
            // our metatraffic unicast locator). We use the SEDP subscription
            // announcement itself as a discovery signal: extract guidPrefix from
            // the RTPS packet header and ipAddr from the UDP source, and insert a
            // *partial* DiscoveredParticipant (metatrafficPort = 0, userDataPort =
            // 0) so that the HB pass can route rdi DATA to rdiReaderUnicastPort.
            // SEDP sends to the partial entry fail silently (port 0); they are
            // harmless and will be replaced once a full SPDP exchange completes.
            const uint8_t* senderGuidPfx =
                (parsedSub.hasReaderGuid) ? parsedSub.readerGuid : (packet + 8);
            bool foundInDiscovered = false;
            for (auto& dp : self->_discovered)
            {
                if (memcmp(dp.guidPrefix, senderGuidPfx, 12) == 0)
                {
                    foundInDiscovered = true;
                    if (parsedSub.unicastLocatorPort != 0 &&
                        dp.rdiReaderUnicastPort != parsedSub.unicastLocatorPort)
                    {
                        LOG_I(MODULE_PREFIX,
                              "SEDP rdi reader port for participant %02x%02x%02x%02x: %u (was %u)",
                              dp.guidPrefix[0], dp.guidPrefix[1], dp.guidPrefix[2], dp.guidPrefix[3],
                              (unsigned)parsedSub.unicastLocatorPort,
                              (unsigned)dp.rdiReaderUnicastPort);
                        dp.rdiReaderUnicastPort = parsedSub.unicastLocatorPort;
                    }
                    if (parsedSub.hasReaderGuid)
                        (void)addRdiReaderEntityIdCandidate(dp, parsedSub.readerGuid + 12);
                    break;
                }
            }
            if (!foundInDiscovered)
            {
                // Participant not yet in _discovered (missed SPDP multicast).
                // Create a partial entry so rdi DATA reaches this rdi reader.
                DiscoveredParticipant partial = {};
                memcpy(partial.guidPrefix, senderGuidPfx, 12);
                partial.ipAddr               = from.sin_addr.s_addr;
                partial.rdiReaderUnicastPort = parsedSub.unicastLocatorPort;
                if (parsedSub.hasReaderGuid)
                    (void)addRdiReaderEntityIdCandidate(partial, parsedSub.readerGuid + 12);
                // leaseDurationSec = 0 → uses DEFAULT_LEASE_TIMEOUT_MS (240 s)
                LOG_I(MODULE_PREFIX,
                      "SEDP rdi partial-disc guidPfx=%02x%02x%02x%02x rdiPort=%u",
                      partial.guidPrefix[0], partial.guidPrefix[1],
                      partial.guidPrefix[2], partial.guidPrefix[3],
                      (unsigned)parsedSub.unicastLocatorPort);
                self->processDiscoveredParticipant(partial, from);
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
        packet, packetLen, fromAddr,
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
                          uint32_t contentLen,
                          uint8_t dataFlags)
    {
        const uint8_t* writerEID = pContent + 8;
        if (memcmp(writerEID, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0)
            return;
        if (contentLen < 24)
            return;
        const uint8_t* payload = nullptr;
        uint32_t payloadLen = 0;
        RTPSData_getSerializedPayload(pContent, contentLen, dataFlags, payload, payloadLen);
        if (!payload || payloadLen < 4)
            return;
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
        // Per-topic dispatch: look up (remote guidPrefix + writerEID) -> slot.
        // Route via _remotePubMap: if the remote writer has been correlated to a
        // local subscription slot (via SEDP pub parsing), dispatch to the per-slot
        // handler stored by addStringSubscription().  For any slot where no
        // per-slot handler was registered, fall back to the legacy single
        // _stringMessageHandler.  Also falls back when no mapping exists yet
        // (slot == -1), so applications that never call addStringSubscription()
        // keep working.
        const int8_t slot = self->_remotePubMap.findSlot(srcGuidPrefix, writerEID);
        static constexpr int8_t kMaxSlot =
            (int8_t)(sizeof(self->_extraSubscriptionHandlers) /
                     sizeof(self->_extraSubscriptionHandlers[0]));
        if (slot >= 0 && slot < kMaxSlot && self->_extraSubscriptionHandlers[slot])
        {
            self->_extraSubscriptionHandlers[slot](writerEID, srcGuidPrefix,
                                                   text, dec.textLen);
            return;
        }
        // Fallback: SEDP pub not yet parsed (slot == -1) or no per-slot handler.
        // Try the legacy _stringMessageHandler first, then slot-0 handler so that
        // applications using addStringSubscription() still receive messages that
        // arrive before the SEDP publication correlation is complete.
        if (self->_stringMessageHandler)
        {
            self->_stringMessageHandler(writerEID, srcGuidPrefix, text, dec.textLen);
            return;
        }
        if (self->_extraSubscriptionHandlers[0])
            self->_extraSubscriptionHandlers[0](writerEID, srcGuidPrefix, text, dec.textLen);
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
                        sequenceNumber, self->_myIpAddr, heartbeatCount,
                        self->computeSedpPubHeartbeatLastSN());
                case RTPSWriterHeartbeatAction::SedpRosDiscoverySubscription:
                    return self->_sedpHandler.buildSubscriptionMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_READER,
                        "ros_discovery_info",
                        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                        RELIABILITY_RELIABLE,
                        DURABILITY_TRANSIENT_LOCAL,
                        sequenceNumber, self->_myIpAddr, heartbeatCount,
                        self->computeSedpSubHeartbeatLastSN());
                case RTPSWriterHeartbeatAction::SedpChatterPublication:
                    return self->_sedpHandler.buildPublicationMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_CHATTER_WRITER,
                        CHATTER_DDS_TOPIC,
                        CHATTER_DDS_TYPE,
                        RELIABILITY_RELIABLE,
                        DURABILITY_VOLATILE,
                        sequenceNumber, self->_myIpAddr, heartbeatCount,
                        self->computeSedpPubHeartbeatLastSN());
                case RTPSWriterHeartbeatAction::SedpChatterSubscription:
                    return self->_sedpHandler.buildSubscriptionMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_CHATTER_READER,
                        self->_subscriptionTopic.c_str(),
                        self->_subscriptionType.c_str(),
                        RELIABILITY_RELIABLE,
                        DURABILITY_VOLATILE,
                        sequenceNumber, self->_myIpAddr, heartbeatCount,
                        self->computeSedpSubHeartbeatLastSN());
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
                    // Jazzy rmw_fastrtps creates the internal graph
                    // subscription with keyed-topic support disabled; keep
                    // ros_discovery_info DATA as a plain unkeyed sample.
                    // The participant GID is carried in the serialized
                    // ParticipantEntitiesInfo payload itself.
                    //
                    // firstSN == lastSN == current: writer holds only the
                    // most recent rdi sample (rebuilt each HB from current
                    // state).  Old SNs no longer exist, so readers don't
                    // ACKNACK for them.  When _rosDiscSeqNum is bumped on
                    // autopub attach/detach the new sample displaces the
                    // old one in the reader's cache (same instance key).
                    return self->_sedpHandler.buildUserDataMessage(
                        self->_sendBuf, sizeof(self->_sendBuf),
                        self->_participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_WRITER,
                        rosDiscPayload, rosDiscLen,
                        sequenceNumber, heartbeatCount,
                        /*firstSN=*/sequenceNumber,
                        /*keyHash16=*/nullptr);
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
                // The only UserData send within the builtin writer-HB pass is
                // RosDiscoveryInfoData. The host's rdi reader is created with
                // OPTIONALLY_REQUIRED unique flow endpoints and therefore
                // listens on its own UDP port (advertised in SEDP DATA(r) via
                // PID_UNICAST_LOCATOR — see recvMetatraffic), distinct from
                // `userDataPort`. If we know that port, use it; otherwise fall
                // back to the participant default (which works for hosts that
                // didn't request a unique flow endpoint).
                const uint16_t rdiPort =
                    remoteRef.rdiReaderUnicastPort != 0
                        ? remoteRef.rdiReaderUnicastPort
                        : remoteRef.userDataPort;
                dest.sin_port = htons(rdiPort);
            }
            const int primarySent = sendto(sock, self->_sendBuf, payloadLen, 0,
                                           (struct sockaddr*)&dest, sizeof(dest));
            if (sendTarget == RTPSWriterHeartbeatSendTarget::UserData)
            {
                patchRosDiscoveryInfoReaderId(
                    self->_sendBuf, payloadLen, ENTITYID_ROS_DISC_INFO_READER);
                (void)sendto(sock, self->_sendBuf, payloadLen, 0,
                              (struct sockaddr*)&dest, sizeof(dest));
                if (remoteRef.rdiReaderUnicastPort == 0 &&
                    remoteRef.metatrafficPort != 0 &&
                    remoteRef.metatrafficPort != ntohs(dest.sin_port))
                {
                    struct sockaddr_in metaDest = dest;
                    metaDest.sin_port = htons(remoteRef.metatrafficPort);
                    (void)sendto(self->_metatrafficSock,
                                  self->_sendBuf, payloadLen, 0,
                                  (struct sockaddr*)&metaDest,
                                  sizeof(metaDest));
                }
            }
            if (sendTarget == RTPSWriterHeartbeatSendTarget::UserData &&
                remoteRef.rdiReaderEntityIdCount > 0)
            {
                for (uint8_t i = 0; i < remoteRef.rdiReaderEntityIdCount; i++)
                {
                    patchRosDiscoveryInfoReaderId(
                        self->_sendBuf, payloadLen, remoteRef.rdiReaderEntityIds[i]);
                    (void)sendto(sock, self->_sendBuf, payloadLen, 0,
                                  (struct sockaddr*)&dest, sizeof(dest));
                }
            }
            return primarySent;
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
            const uint16_t rdiPort =
                remoteRef.rdiReaderUnicastPort != 0
                    ? remoteRef.rdiReaderUnicastPort
                    : remoteRef.userDataPort;
            LOG_I(MODULE_PREFIX,
                  "sendWriterHB rosDisc %d/%d to rdiPort %u userPort %u readerEIDs %u",
                  sentBytes, (int)payloadLen, (unsigned)rdiPort,
                  (unsigned)remoteRef.userDataPort,
                  (unsigned)remoteRef.rdiReaderEntityIdCount);
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
    _hbPass.extraSubPhaseDoneForPeer = true;

    // Phase 3 (Phase 4 / Slice 4.6): one autopub writer slot per tick for the
    // current peer.  Emits SEDP PublicationBuiltinTopic DATA(w) so peers learn
    // about every dynamically-attached Raft device DataWriter.
    using RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY;
    if (_hbPass.extraPubSlot < DYNAMIC_WRITER_REGISTRY_CAPACITY)
    {
        const uint8_t slot = _hbPass.extraPubSlot;
        _hbPass.extraPubSlot++;
        (void)emitAutoPubSedpAnnounce(remote, slot);
        return;
    }

    // Current peer complete - advance to next peer (or finalise on next tick).
    _hbPass.peerIdx++;
    _hbPass.stepIdx = 0;
    _hbPass.extraSubSlot = 1;
    _hbPass.extraPubSlot = 0;
    _hbPass.mainPhaseDoneForPeer = false;
    _hbPass.extraSubPhaseDoneForPeer = false;
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
    // Auto-publish diagnostic: every new peer attempt logged at INFO so we
    // can correlate peer arrival with autopub SEDP announce firings.
    {
        char senderIpStr[16] = {0};
        strncpy(senderIpStr, inet_ntoa(senderAddr.sin_addr), sizeof(senderIpStr) - 1);
        LOG_I(MODULE_PREFIX,
              "handleNewParticipant NEW peer guidPfx=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x metaPort=%u userPort=%u senderIp=%s autoPubSlotsInUse=%u",
              remote.guidPrefix[0], remote.guidPrefix[1], remote.guidPrefix[2], remote.guidPrefix[3],
              remote.guidPrefix[4], remote.guidPrefix[5], remote.guidPrefix[6], remote.guidPrefix[7],
              remote.guidPrefix[8], remote.guidPrefix[9], remote.guidPrefix[10], remote.guidPrefix[11],
              (unsigned)remote.metatrafficPort, (unsigned)remote.userDataPort, senderIpStr,
              (unsigned)_autoPubBackend.lifecycle().inUseCount());
    }

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
    announcePlan.sendInitialRosDiscoveryUserData = true;
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
                            sequenceNumber, self->_myIpAddr, sedpHeartbeatForBuild,
                            self->computeSedpPubHeartbeatLastSN());
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
                        sequenceNumber, self->_myIpAddr, sedpHeartbeatForBuild,
                        self->computeSedpSubHeartbeatLastSN());
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
    entry.extraSubPhaseDone = true;

    // Extra-publication phase (Phase 4 / Slice 4.6) — walk the autopub writer
    // registry and emit one SEDP PublicationBuiltinTopic DATA(w) per in-use
    // slot, one slot per tick.  Slots can be sparse (detach leaves gaps), so
    // we advance past free slots without emitting anything.
    using RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY;
    if (entry.extraPubSlot < DYNAMIC_WRITER_REGISTRY_CAPACITY)
    {
        const uint8_t slot = entry.extraPubSlot;
        entry.extraPubSlot++;
        (void)emitAutoPubSedpAnnounce(entry.remote, slot);
        return;
    }

    // All phases complete - remove this entry.
    _pendingAnnounces.erase(_pendingAnnounces.begin());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Phase 4 / Slice 4.6 — build + send an SEDP PublicationBuiltinTopic DATA(w)
// for a single autopub writer slot.  Returns true if a packet was transmitted.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RaftROS::emitAutoPubSedpAnnounce(const DiscoveredParticipant& remote, uint8_t slot)
{
    // Message building and sending belong to the backend; the SEDP
    // publications high-water mark stays here because it spans every
    // announced endpoint (chatter and ros_discovery_info as well as autopub).
    const bool announced = _autoPubBackend.announceToPeer(slot, remote,
                                                          computeSedpPubHeartbeatLastSN());
#ifdef DEBUG_AUTOPUB_SEDP_ANNOUNCE
    const auto* pEntry = _autoPubBackend.lifecycle().get(slot);
    LOG_I(MODULE_PREFIX,
          "autoPubSEDP slot=%u topic=%s announced=%d peerGuidPfx=%02x%02x%02x%02x peerIP=%08x port=%d",
          (unsigned)slot, (pEntry && pEntry->topic) ? pEntry->topic : "?", (int)announced,
          remote.guidPrefix[0], remote.guidPrefix[1], remote.guidPrefix[2], remote.guidPrefix[3],
          (unsigned)remote.ipAddr, (int)remote.metatrafficPort);
#endif
    return announced;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute SEDP publications/subscriptions writer high-water mark for HEARTBEAT.
//
// The SEDP publications writer is a single RTPS endpoint shared by every
// publication we announce (rdi/chatter/autopub slots).  Each emitted DATA(w)
// uses a fixed per-endpoint seq number, but the writer's HEARTBEAT lastSN MUST
// be the maximum seq ever advertised across all of those endpoints — otherwise
// it regresses every time a lower-numbered endpoint is re-announced and
// FastDDS readers silently un-match the writer (root cause of
// `_NODE_NAME_UNKNOWN_`).  Same for the subscriptions writer.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint64_t RaftROS::computeSedpPubHeartbeatLastSN() const
{
    uint64_t maxSeq = _sedpSeqNum;
    if (_chatterSedpSeqNum > maxSeq) maxSeq = _chatterSedpSeqNum;
    using RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY;
    for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
    {
        const auto* pEntry = _autoPubBackend.lifecycle().get(slot);
        if (pEntry && pEntry->inUse && pEntry->sedpSeqNum > maxSeq)
            maxSeq = pEntry->sedpSeqNum;
    }
    return maxSeq;
}

uint64_t RaftROS::computeSedpSubHeartbeatLastSN() const
{
    uint64_t maxSeq = _sedpSubSeqNum;
    for (uint8_t i = 0; i < sizeof(_extraSubscriptionSeqNums) / sizeof(_extraSubscriptionSeqNums[0]); i++)
    {
        if (_extraSubscriptionSeqNums[i] > maxSeq)
            maxSeq = _extraSubscriptionSeqNums[i];
    }
    return maxSeq;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: build ros_discovery_info payload with our writer GIDs included
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RaftROS::buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen)
{
    // Writer GIDs: static chatter writer + any in-use autopub dynamic writer
    // entityIds.  ROS 2 tooling (`ros2 topic info`, graph API) determines the
    // "Publisher count" for a topic by enumerating writer_gid_seq in
    // ros_discovery_info — a DDS writer not listed here is invisible to the
    // ROS graph even when its SEDP announce is accepted.
    using RaftRuntime::RTPS::Runtime::AutoPub::DYNAMIC_WRITER_REGISTRY_CAPACITY;
    const uint8_t* writerIds[1 + DYNAMIC_WRITER_REGISTRY_CAPACITY] = {0};
    uint32_t numWriterIds = 0;
    writerIds[numWriterIds++] = ENTITYID_CHATTER_WRITER;
    for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
    {
        const auto* pEntry = _autoPubBackend.lifecycle().get(slot);
        if (!pEntry)
            continue;
        writerIds[numWriterIds++] = pEntry->entityId;
    }

    // Collect reader GIDs from the registry: slot 0 (default CHATTER_READER) plus any
    // additional subscriptions appended via addStringSubscription().
    using RaftRuntime::RTPS::Runtime::Dispatch::RTPS_SUBSCRIPTION_REGISTRY_CAPACITY;
    const uint8_t* readerIds[RTPS_SUBSCRIPTION_REGISTRY_CAPACITY] = {0};
    const uint32_t numReaderIds = _subscriptionRegistry.readerEntityIds(
        readerIds, RTPS_SUBSCRIPTION_REGISTRY_CAPACITY);

    const uint32_t payloadLen = SPDPHandler::buildRosDiscoveryInfoPayload(
        pBuf, bufLen,
        _participant.getParticipantGuid(),
        _nodeName.c_str(),
        _nodeNamespace.c_str(),
        writerIds, numWriterIds,
        readerIds, numReaderIds);

    // Auto-publish diagnostic: log the writer-GID list whenever the
    // ros_discovery_info payload is (re)built.  A writer not appearing here
    // is invisible to `ros2 topic info` / graph queries even if its SEDP
    // announce is accepted.
#ifdef DEBUG_ROS_DISCOVERY_INFO
    LOG_I(MODULE_PREFIX,
          "buildRosDiscInfo seq=%u writerGids=%u readerGids=%u payloadLen=%u (chatter+autopub slots)",
          (unsigned)_rosDiscSeqNum, (unsigned)numWriterIds, (unsigned)numReaderIds,
          (unsigned)payloadLen);
#endif
    return payloadLen;
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
            {
                char destIpStr[16];
                strncpy(destIpStr, inet_ntoa(*(struct in_addr*)&remote.ipAddr), sizeof(destIpStr));
                destIpStr[sizeof(destIpStr)-1] = '\0';
#ifdef DEBUG_PUBLISH_CHATTER
                if ((_chatterSeqNum % 5) == 1)
                {
                    LOG_I(MODULE_PREFIX,
                          "publishChatter seq=%llu sent %d/%d to %s:%d peerGuidPfx=%02x%02x%02x%02x (discCount=%u)",
                          (unsigned long long)_chatterSeqNum, sent, (int)msgLen,
                          destIpStr, (int)remote.userDataPort,
                          remote.guidPrefix[0], remote.guidPrefix[1],
                          remote.guidPrefix[2], remote.guidPrefix[3],
                          (unsigned)_discovered.size());
                }
#endif
            }
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Phase 4 / Slice 4.3 — Auto-publishing device attach/detach + data decode
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Slice 4.5 — the CDR serialiser reads the decoded poll struct via a mirror of
// RaftCore's AttrFieldDesc layout to stay out of RaftCore's include graph
// (so the shared-runtime unit tests don't drag in RaftArduino/RaftBus).
// Verify the binary layouts are still identical on this build.
static_assert(sizeof(RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubAttrFieldDesc)
              == sizeof(AttrFieldDesc),
              "RTPSAutoPubAttrFieldDesc must match AttrFieldDesc size");
static_assert(offsetof(RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubAttrFieldDesc, offset)
              == offsetof(AttrFieldDesc, offset),
              "RTPSAutoPubAttrFieldDesc offset mismatch");
static_assert(offsetof(RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubAttrFieldDesc, divisor)
              == offsetof(AttrFieldDesc, divisor),
              "RTPSAutoPubAttrFieldDesc divisor mismatch");
static_assert((uint8_t)RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubAttrType::Float
              == (uint8_t)AttrType::Float,
              "RTPSAutoPubAttrType enum ordering must match AttrType");
static_assert((uint8_t)RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubAttrType::Bool
              == (uint8_t)AttrType::Bool,
              "RTPSAutoPubAttrType enum ordering must match AttrType");

void RaftROS::autoPubOnDeviceStatusChange(RaftDevice& device, const BusAddrStatus& addrStatus)
{
    // Auto-publish diagnostic: log every callback fire so we can tell whether
    // DeviceManager is calling us at all and with what flags.
    LOG_I(MODULE_PREFIX,
          "autoPubStatusCb devID=%s typeIdx=%u online=%d isChange=%d isNewlyId=%d",
          device.getDeviceID().toString().c_str(),
          (unsigned)addrStatus.deviceTypeIndex,
          (int)addrStatus.onlineState,
          (int)addrStatus.isChange,
          (int)addrStatus.isNewlyIdentified);

    // Act on either an online/offline transition (isChange) or on the
    // first-identification event for an already-online device
    // (isNewlyIdentified).  I2C devices are reported ONLINE as soon as
    // the bus scanner sees an ACK at the address; identification (and
    // therefore a valid deviceTypeIndex) happens asynchronously in a
    // follow-up status change with isNewlyIdentified=true but no
    // online-state change.  Auto-publish needs the typeIdx, so we can't
    // commit on isChange alone.
    if (!addrStatus.isChange && !addrStatus.isNewlyIdentified)
        return;

    switch (addrStatus.onlineState)
    {
        case DeviceOnlineState::ONLINE:
            // Skip attach until the bus has finished identification —
            // deviceTypeIndex is required for class-map lookup and
            // SEDP type-name emission.
            if (addrStatus.deviceTypeIndex == DEVICE_TYPE_INDEX_INVALID)
                return;
            autoPubAttachDevice(device, addrStatus);
            break;
        case DeviceOnlineState::OFFLINE:
        case DeviceOnlineState::PENDING_DELETION:
            autoPubDetachDevice(device, addrStatus);
            break;
        case DeviceOnlineState::INITIAL:
        default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Slice 4.11 — QoS profile override plumbing
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::autoPubParseQoSOverrides()
{
    using namespace RaftRuntime::RTPS::Runtime::AutoPub;

    _qosAliasOverrides.clear();
    _qosClassOverrides.clear();

    // Read the whole block as raw JSON so we can walk it with RaftJson.  An
    // empty / absent block just leaves both override tables empty, which
    // causes the resolver to fall back to the per-class built-in default.
    const String rawBlock = configGetString("qosProfiles", "{}");
    RaftJson blockJson(rawBlock);

    // Walk the top-level keys once.  Each top-level key is either
    //   * the literal "classDefaults" → nested object of CLAS→profileName
    //   * a device alias → profile name (string) or {profile:"<name>"} object
    std::vector<String> topKeys;
    blockJson.getKeys("", topKeys);
    for (const String& k : topKeys)
    {
        if (k == "classDefaults")
        {
            const String classRaw = blockJson.getString(k.c_str(), "{}");
            RaftJson classJson(classRaw);
            std::vector<String> classKeys;
            classJson.getKeys("", classKeys);
            for (const String& clasCode : classKeys)
            {
                const String profileName = classJson.getString(clasCode.c_str(), "");
                RTPSAutoPubQoSProfileId id = RTPSAutoPubQoSProfileId::FallbackString;
                if (!RTPSAutoPubQoSProfile_parseName(profileName.c_str(), id))
                {
                    LOG_W(MODULE_PREFIX,
                          "qosProfiles.classDefaults.%s: unknown profile '%s' — ignored",
                          clasCode.c_str(), profileName.c_str());
                    continue;
                }
                _qosClassOverrides.push_back({ clasCode, id });
            }
            continue;
        }

        // Alias mapping.  Accept either a bare string or a {profile:"..."} obj.
        const String rawVal = blockJson.getString(k.c_str(), "");
        String profileName;
        if (rawVal.length() >= 2 && rawVal.charAt(0) == '{')
        {
            RaftJson obj(rawVal);
            profileName = obj.getString("profile", "");
        }
        else
        {
            profileName = rawVal;
        }
        RTPSAutoPubQoSProfileId id = RTPSAutoPubQoSProfileId::FallbackString;
        if (!RTPSAutoPubQoSProfile_parseName(profileName.c_str(), id))
        {
            LOG_W(MODULE_PREFIX,
                  "qosProfiles.%s: unknown profile '%s' — ignored",
                  k.c_str(), profileName.c_str());
            continue;
        }
        _qosAliasOverrides.push_back({ k, id });
    }

    if (!_qosAliasOverrides.empty() || !_qosClassOverrides.empty())
    {
        LOG_I(MODULE_PREFIX,
              "autoPubQoS overrides parsed: aliases=%u classDefaults=%u",
              (unsigned)_qosAliasOverrides.size(),
              (unsigned)_qosClassOverrides.size());
    }
}

RaftRuntime::RTPS::Runtime::AutoPub::RTPSAutoPubQoSProfileId
RaftROS::autoPubResolveQoSProfileId(
        const char* pTopicAlias,
        const char* const* pClasArray, size_t clasCount,
        const char* pDeviceTypeName) const
{
    using namespace RaftRuntime::RTPS::Runtime::AutoPub;

    // 1. Per-device alias (highest priority).
    if (pTopicAlias && *pTopicAlias)
    {
        for (const auto& ov : _qosAliasOverrides)
        {
            if (ov.key == pTopicAlias)
                return ov.id;
        }
    }

    // 2. Per-class override — first class code that matches wins.
    if (pClasArray)
    {
        for (size_t i = 0; i < clasCount; i++)
        {
            const char* c = pClasArray[i];
            if (!c || !*c)
                continue;
            for (const auto& ov : _qosClassOverrides)
            {
                if (ov.key == c)
                    return ov.id;
            }
        }
    }

    // 3. Built-in default from the class list.
    return RTPSAutoPubQoSProfile_defaultForClasses(pClasArray, clasCount, pDeviceTypeName);
}

bool RaftROS::autoPubAttachDevice(RaftDevice& device, const BusAddrStatus& addrStatus)
{
    using namespace RaftRuntime::RTPS::Runtime::AutoPub;

    const RaftDeviceID devID = device.getDeviceID();
    if (!devID.isValid() || addrStatus.deviceTypeIndex == DEVICE_TYPE_INDEX_INVALID)
    {
        LOG_W(MODULE_PREFIX, "autoPubAttach invalid devID=%s typeIdx=%u — skipping",
              devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex);
        return false;
    }

    // Build the key used for lifecycle slot allocation.
    const RTPSDynamicWriterKey key{ (uint8_t)devID.getBusNum(), (uint32_t)devID.getAddress() };

    // Already attached? (Shouldn't normally happen — DeviceManager re-emits
    // online on reconnect after offline, by which point we've detached.)
    if (_autoPubBackend.lifecycle().find(key) >= 0)
    {
        LOG_I(MODULE_PREFIX, "autoPubAttach already attached devID=%s — skipping",
              devID.toString().c_str());
        return false;
    }

    // Slice 4.4 — look up the ROS 2 message mapping from the device's class tags.
    // Falls back to String/"raw" for unknown classes.  Actuators (SRVO/PUMP/PIX)
    // return `excluded=true` and are not published at all.
    DeviceTypeRecord devTypeRec;
    const bool haveTypeRec = deviceTypeRecords.getDeviceInfo(addrStatus.deviceTypeIndex, devTypeRec);
    const char* pDeviceTypeName = haveTypeRec ? devTypeRec.deviceType : nullptr;

    // Parse the "clas" array out of devInfoJson.  This runs only once per
    // attach (on connect), so the RaftJson parse cost is negligible.
    std::vector<String> clasStrs;
    if (haveTypeRec && devTypeRec.devInfoJson)
    {
        RaftJson devInfo(devTypeRec.devInfoJson, false);
        devInfo.getArrayElems("clas", clasStrs);
    }
    std::vector<const char*> clasPtrs;
    clasPtrs.reserve(clasStrs.size());
    for (const auto& s : clasStrs)
        clasPtrs.push_back(s.c_str());

    // Decide what this device publishes: class mapping -> ROS topic/type ->
    // semantic QoS.  Shared with any backend (AutoPub/AutoPubAttachPlan.h);
    // the SysTypes QoS overrides stay here because they are SysMod config.
    using RaftRuntime::AutoPub::AutoPubAttachPlan_build;
    using RaftRuntime::AutoPub::AutoPubQoSProfile_name;
    const RaftRuntime::AutoPub::AutoPubDeviceId planDeviceId{
        (uint8_t)devID.getBusNum(), (uint32_t)devID.getAddress(), 0};
    const auto plan = AutoPubAttachPlan_build(
        planDeviceId,
        clasPtrs.empty() ? nullptr : clasPtrs.data(), clasPtrs.size(), pDeviceTypeName,
        [this](const char* alias, const char* const* clasArray, size_t clasCount,
               const char* deviceTypeName) {
            return autoPubResolveQoSProfileId(alias, clasArray, clasCount, deviceTypeName);
        });

    if (plan.excluded)
    {
        LOG_I(MODULE_PREFIX, "autoPubAttach devID=%s type=%s is an actuator — excluded from auto-publish",
              devID.toString().c_str(), pDeviceTypeName ? pDeviceTypeName : "?");
        return false;
    }
    if (plan.endpointCount == 0)
    {
        LOG_W(MODULE_PREFIX, "autoPubAttach devID=%s type=%s — no publishable endpoint (name too long?)",
              devID.toString().c_str(), pDeviceTypeName ? pDeviceTypeName : "?");
        return false;
    }
    if (plan.endpointCount > 1)
    {
        LOG_I(MODULE_PREFIX, "autoPubAttach devID=%s composite: primary=%s/%s, secondary=%s/%s",
              devID.toString().c_str(),
              plan.endpoints[0].topic, plan.endpoints[0].type,
              plan.endpoints[1].topic, plan.endpoints[1].type);
    }

    // Create the primary publisher on the selected backend
    const uint8_t slot = _autoPubBackend.createPublisher(plan.endpoints[0]);
    if (slot == RTPSAutoPubBackend::INVALID_SLOT)
    {
        LOG_W(MODULE_PREFIX, "autoPubAttach backend full — cannot attach devID=%s typeIdx=%u",
              devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex);
        return false;
    }
    LOG_I(MODULE_PREFIX, "autoPubAttach devID=%s slot=%u topic=%s type=%s qos=%s",
          devID.toString().c_str(), (unsigned)slot,
          plan.endpoints[0].topic, plan.endpoints[0].type,
          AutoPubQoSProfile_name(plan.endpoints[0].qosProfileId));

    // Build per-device context — cache decode metadata + pre-allocate decode buffer.
    DynamicWriterCtx* pCtx = new DynamicWriterCtx();
    pCtx->pOwner = this;
    pCtx->deviceID = devID;
    pCtx->deviceTypeIndex = addrStatus.deviceTypeIndex;
    pCtx->slot = slot;

    if (haveTypeRec && devTypeRec.pollResultDecodeFn && devTypeRec.pollFieldDescs)
    {
        pCtx->decodeFn = devTypeRec.pollResultDecodeFn;
        pCtx->pFieldDescs = devTypeRec.pollFieldDescs;
        pCtx->fieldCount = devTypeRec.pollFieldCount;
        pCtx->structSize = devTypeRec.pollStructSize;
        pCtx->pollDataSizeBytes = devTypeRec.pollDataSizeBytes;

        // Allow up to 8 records per decode for FIFO-backed devices.  Keeps the
        // heap allocation under 1 kB for typical struct sizes (IMU = 24 B).
        static constexpr uint16_t MAX_DECODE_RECORDS = 8;
        pCtx->maxDecodeRecords = MAX_DECODE_RECORDS;
        pCtx->decodeBufSize = (uint32_t)pCtx->structSize * MAX_DECODE_RECORDS;
        if (pCtx->decodeBufSize > 0)
            pCtx->pDecodeBuf = new uint8_t[pCtx->decodeBufSize];
    }
    else
    {
        LOG_W(MODULE_PREFIX, "autoPubAttach decode unavailable devID=%s typeIdx=%u — callbacks will be rawlen-only",
              devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex);
    }

    // Slice 4.5 — cache the ROS 2 message kind so the hot path skips the
    // class-map lookup.  CDR payload buffers are shared (loop task only).
    pCtx->msgKind = plan.endpoints[0].msgKind;

    // Z2 — claim a generation-safe mailbox handle for the bus data callback.
    // Without one the writer is still announced but publishes nothing.
    if (pCtx->pDecodeBuf)
    {
        pCtx->handle = _autoPubPool.acquire(pCtx, pCtx->structSize);
        if (!pCtx->handle.isValid())
        {
            LOG_W(MODULE_PREFIX, "autoPubAttach no mailbox devID=%s structSize=%u (max %u, devices %u/%u) — writer will not publish",
                  devID.toString().c_str(), (unsigned)pCtx->structSize,
                  (unsigned)AUTOPUB_MAILBOX_RECORD_SIZE,
                  (unsigned)_autoPubPool.inUseCount(), (unsigned)_autoPubPool.capacity());
        }
    }

    _autoPubCtxs[slot] = pCtx;

    const uint8_t* pEntityId = _autoPubBackend.entityIdForSlot(slot);
    LOG_I(MODULE_PREFIX, "autoPubAttach devID=%s typeIdx=%u slot=%u topic=%s type=%s eid=%02x%02x%02x%02x fields=%u structSize=%u",
          devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex,
          (unsigned)slot, plan.endpoints[0].topic, plan.endpoints[0].type,
          pEntityId ? pEntityId[0] : 0, pEntityId ? pEntityId[1] : 0,
          pEntityId ? pEntityId[2] : 0, pEntityId ? pEntityId[3] : 0,
          (unsigned)pCtx->fieldCount, (unsigned)pCtx->structSize);

    // Composite secondary endpoint (e.g. AHT20 publishes Temperature on the
    // primary slot and RelativeHumidity here).  It shares the decoded record
    // and field descriptors with the primary but gets its own backend
    // publisher (distinct topic and wire identity).  Failure is non-fatal:
    // the primary keeps publishing.
    if (plan.endpointCount > 1)
    {
        const uint8_t secSlot = _autoPubBackend.createPublisher(plan.endpoints[1]);
        if (secSlot == RTPSAutoPubBackend::INVALID_SLOT)
        {
            LOG_W(MODULE_PREFIX, "autoPubAttach composite backend full — only primary attached devID=%s",
                  devID.toString().c_str());
        }
        else
        {
            pCtx->secondarySlot = secSlot;
            pCtx->secondaryMsgKind = plan.endpoints[1].msgKind;
            LOG_I(MODULE_PREFIX, "autoPubAttach secondary devID=%s slot=%u topic=%s type=%s qos=%s",
                  devID.toString().c_str(), (unsigned)secSlot,
                  plan.endpoints[1].topic, plan.endpoints[1].type,
                  AutoPubQoSProfile_name(plan.endpoints[1].qosProfileId));
        }
    }

    // Install the per-device data callback.  DeviceManager forwards this to the
    // underlying bus; callbacks run on the bus polling task (limited stack).
    // The callback info is the encoded pool handle, never the ctx pointer, so
    // a callback that races with (or outlives) detach cannot reach freed state.
    SysManagerIF* pSysMan = getSysManager();
    DeviceManager* pDevMan = pSysMan ? pSysMan->getDeviceManager() : nullptr;
    if (pDevMan && pCtx->handle.isValid())
    {
        pDevMan->registerForDeviceData(
            devID,
            [this](uint16_t deviceTypeIdx, std::vector<uint8_t> data, const void* pCallbackInfo) {
                this->autoPubOnDeviceData(deviceTypeIdx, std::move(data), pCallbackInfo);
            },
            /*minTimeBetweenReportsMs=*/0,
            /*pCallbackInfo=*/pCtx->handle.toCallbackInfo(),
            /*unregister=*/false);
    }

    // Announce the newly-allocated writer slot(s) to every already-discovered
    // participant.  Without this, a writer that comes up AFTER the initial
    // SEDP announce sweep for a participant has completed would never be seen
    // by that participant — symptom: `ros2 topic info` shows the topic with
    // "Publisher count: 0" and `ros2 topic echo` stays silent.
    for (const auto& remote : _discovered)
    {
        (void)emitAutoPubSedpAnnounce(remote, slot);
        if (pCtx->secondarySlot != 0xFF)
            (void)emitAutoPubSedpAnnounce(remote, pCtx->secondarySlot);
    }

    // Bump _rosDiscSeqNum only when at least one participant could already
    // have seen the previous rdi sample. If auto-publish attaches before any
    // ROS participant is discovered, advancing from seq=1 to seq=2 creates a
    // gap for the first later reader; FastDDS ACKNACKs seq=1 and withholds
    // seq=2 from the graph listener.
    if (!_discovered.empty())
        _rosDiscSeqNum++;
    LOG_I(MODULE_PREFIX,
          "autoPubAttach slot=%u secSlot=%d (rosDiscSeqNum now %u discovered=%u)",
          (unsigned)slot, (int)pCtx->secondarySlot,
          (unsigned)_rosDiscSeqNum, (unsigned)_discovered.size());

    return true;
}

void RaftROS::autoPubDetachDevice(RaftDevice& device, const BusAddrStatus& /*addrStatus*/)
{
    using namespace RaftRuntime::RTPS::Runtime::AutoPub;
    const RaftDeviceID devID = device.getDeviceID();
    const RTPSDynamicWriterKey key{ (uint8_t)devID.getBusNum(), (uint32_t)devID.getAddress() };

    const int slot = _autoPubBackend.lifecycle().find(key);
    if (slot < 0)
        return;

    DynamicWriterCtx* pCtx = _autoPubCtxs[slot];
    _autoPubCtxs[slot] = nullptr;

    // Drop the DeviceManager data registration.  This only removes the pending
    // request: a registration already forwarded to the bus is not withdrawn,
    // and a callback may be in flight on the bus task right now.  Releasing
    // the pool handle is what makes freeing pCtx safe — it waits for any
    // producer holding the lock, and every later callback carrying the old
    // handle is dropped as stale without touching pCtx.
    SysManagerIF* pSysMan = getSysManager();
    DeviceManager* pDevMan = pSysMan ? pSysMan->getDeviceManager() : nullptr;
    if (pDevMan && pCtx && pCtx->handle.isValid())
    {
        pDevMan->registerForDeviceData(devID, nullptr, 0, pCtx->handle.toCallbackInfo(),
                                       /*unregister=*/true);
    }
    if (pCtx)
        _autoPubPool.release(pCtx->handle);

    // Slice 4.8 — emit an SEDP dispose to every currently-discovered peer so
    // they drop the matching writer within a few hundred ms (rather than
    // waiting for the participant lease to expire).  Must run *before* we
    // release the registry slot because we need the live `entityId`.
    // Slice 4.10 — also disposes the composite secondary slot (if any).
    // Tell discovered peers to drop the writer now rather than waiting for
    // the participant lease to expire.  Must run before the slot is released:
    // the backend needs the live endpoint identity.
    auto disposeSlot = [&](uint8_t slotIdx)
    {
        if (slotIdx == 0xFF)
            return;
        // A fixed unique dispose seq per slot, distinct from that slot's
        // announce seq and from other slots'.  See AUTOPUB_SEDP_DISPOSE_BASE_SEQ.
        using RaftRuntime::RTPS::Runtime::AutoPub::AUTOPUB_SEDP_DISPOSE_BASE_SEQ;
        const uint64_t disposeSeq = AUTOPUB_SEDP_DISPOSE_BASE_SEQ + slotIdx;
        int disposedTo = 0;
        for (const auto& remote : _discovered)
        {
            if (_autoPubBackend.disposeAtPeer(slotIdx, remote, disposeSeq))
                disposedTo++;
        }
        if (disposedTo > 0)
        {
            LOG_I(MODULE_PREFIX, "autoPubDispose devID=%s slot=%u disposedTo=%d peers",
                  devID.toString().c_str(), (unsigned)slotIdx, disposedTo);
        }
    };

    disposeSlot(static_cast<uint8_t>(slot));
    if (pCtx && pCtx->secondarySlot != 0xFF)
        disposeSlot(pCtx->secondarySlot);

    _autoPubBackend.destroyPublisher((uint8_t)slot);
    if (pCtx && pCtx->secondarySlot != 0xFF)
        _autoPubBackend.destroyPublisher(pCtx->secondarySlot);

    // See autoPubAttachDevice: only advance the RDI writer sequence when a
    // discovered participant may already have accepted the previous sample.
    if (!_discovered.empty())
        _rosDiscSeqNum++;
    LOG_I(MODULE_PREFIX, "autoPubDetach devID=%s slot=%d secSlot=%d samples=%u (rosDiscSeqNum now %u discovered=%u)",
          devID.toString().c_str(), slot,
          pCtx ? (int)pCtx->secondarySlot : -1,
          pCtx ? (unsigned)pCtx->sampleCount : 0u,
          (unsigned)_rosDiscSeqNum, (unsigned)_discovered.size());
    delete pCtx;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bus data callback — runs on the bus worker task.  Decodes under the pool
// lock and stores only the latest record; no serialisation or network I/O.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::autoPubOnDeviceData(uint16_t /*deviceTypeIdx*/, std::vector<uint8_t> data, const void* pCallbackInfo)
{
    using RaftRuntime::AutoPub::AutoPubPublisherHandle;

    // The callback info is an encoded handle; a stale one (device detached,
    // slot reused) is rejected by the pool before pCtx is ever reached.
    const AutoPubPublisherHandle handle = AutoPubPublisherHandle::fromCallbackInfo(pCallbackInfo);
    _autoPubPool.produce(handle,
        [&](void* pUser, uint8_t* pRecord, uint32_t recordSize) -> uint32_t {
            DynamicWriterCtx* pCtx = static_cast<DynamicWriterCtx*>(pUser);
            pCtx->sampleCount++;

            // Diagnostics: gap since this device's previous data callback
            const int64_t nowUs = esp_timer_get_time();
            if (pCtx->lastCallbackUs != 0)
            {
                const int64_t gapMs = (nowUs - pCtx->lastCallbackUs) / 1000;
                const uint8_t bucket = gapMs < 50 ? 0 : gapMs < 150 ? 1 : gapMs < 250 ? 2 : gapMs < 400 ? 3 : 4;
                pCtx->cbGapHist[bucket].fetch_add(1, std::memory_order_relaxed);
            }
            pCtx->lastCallbackUs = nowUs;
            if (!(pCtx->decodeFn && pCtx->pDecodeBuf && pCtx->structSize > 0 &&
                  pCtx->structSize == recordSize))
                return 0;

            // Pad up to the expected fixed record stride used by the generated decoder.
            const uint32_t expectedRecordSize = pCtx->pollDataSizeBytes
                                              + DevicePollingInfo::POLL_RESULT_TIMESTAMP_SIZE;
            if (data.size() < expectedRecordSize)
                data.resize(expectedRecordSize, 0);

            const uint32_t numRecords = pCtx->decodeFn(data.data(), data.size(),
                                                       pCtx->pDecodeBuf, pCtx->decodeBufSize,
                                                       pCtx->maxDecodeRecords, pCtx->decodeState);
            // Keep only the latest record (FIFO latest-only default, design
            // doc §13); check the decoder's count before forming the pointer.
            if (numRecords == 0 || numRecords > pCtx->decodeBufSize / pCtx->structSize)
                return 0;
            std::memcpy(pRecord, pCtx->pDecodeBuf + (numRecords - 1) * pCtx->structSize,
                        pCtx->structSize);
            return pCtx->structSize;
        },
        AUTOPUB_PRODUCER_LOCK_TIMEOUT_MS);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mailbox drain — runs on the loop task, which also owns attach/detach,
// _discovered, the writer registry and the sockets.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::autoPubDrainSamples()
{
    using namespace RaftRuntime::RTPS::Runtime::AutoPub;
    using namespace RaftRuntime::AutoPub;

    // Diagnostics: gap between drain passes (an overwrite needs no drain for
    // a whole poll interval, or a burst of callbacks on the bus task)
    const int64_t nowUs = esp_timer_get_time();
    if (_autoPubLastDrainUs != 0)
    {
        const uint32_t gapUs = (uint32_t)(nowUs - _autoPubLastDrainUs);
        if (gapUs > _autoPubDrainGapMaxUs)
            _autoPubDrainGapMaxUs = gapUs;
        if (gapUs > 150000)
            _autoPubDrainGapsOver150ms++;
    }
    _autoPubLastDrainUs = nowUs;

    _autoPubPool.drain([&](const AutoPubDrainedSample& sample) {
        // Safe: release() and delete of the ctx also run on this task.
        const DynamicWriterCtx* pCtx = static_cast<const DynamicWriterCtx*>(sample.pUser);

        // Serialise the latest decoded record and emit it with VOLATILE
        // best-effort QoS to every currently-discovered peer (design doc
        // §7.2).  Composite devices (AHT20, BMP280, ...) serialise the same
        // record a second time with `secondaryMsgKind` and emit on the
        // secondary registry slot.
        AutoPubDecodedBatch batch;
        batch.data = sample.record;
        batch.capacity = sample.length;
        batch.recordSize = pCtx->structSize;
        batch.recordCount = 1;
        batch.fields = reinterpret_cast<const AutoPubAttrFieldDesc*>(pCtx->pFieldDescs);
        batch.fieldCount = pCtx->fieldCount;
        const AutoPubSampleOutput outputs[] = {
            {pCtx->msgKind, _autoPubCDRBufs[0], AUTOPUB_CDR_BUF_SIZE},
            {pCtx->secondaryMsgKind,
             pCtx->secondarySlot == 0xFF ? nullptr : _autoPubCDRBufs[1],
             AUTOPUB_CDR_BUF_SIZE}
        };
        AutoPubSampleResult results[2];
        uint64_t emissionSeq[2] = {0, 0};
        uint32_t emissionPeers[2] = {0, 0};
        auto publish = [&](uint8_t outputIndex, const uint8_t* payload, uint32_t length, uint32_t) {
            const uint8_t slot = outputIndex == 0 ? pCtx->slot : pCtx->secondarySlot;
            return _autoPubBackend.publish(slot, payload, length,
                                           &emissionSeq[outputIndex], &emissionPeers[outputIndex]);
        };

        const bool batchOK = AutoPubSampleRunner::run(batch, outputs, 2, results, publish);

        // Rate-limit on the mailbox's stored-record count.
        if (sample.produced > 3 && (sample.produced % 100) != 0)
            return;
        if (!batchOK)
        {
            LOG_W(MODULE_PREFIX, "autoPubData devID=%s invalid mailbox record len=%u stride=%u fields=%u (sample #%u)",
                  pCtx->deviceID.toString().c_str(), (unsigned)sample.length,
                  (unsigned)pCtx->structSize, (unsigned)pCtx->fieldCount,
                  (unsigned)sample.produced);
        }
        else if (pCtx->secondarySlot == 0xFF)
        {
            LOG_I(MODULE_PREFIX,
                  "autoPubData devID=%s typeIdx=%u slot=%u cdrBytes=%u serOK=%d seq=%u peers=%u pub=%u overwritten=%u cbGaps<50/150/250/400/+=%u/%u/%u/%u/%u (sample #%u)",
                  pCtx->deviceID.toString().c_str(), (unsigned)pCtx->deviceTypeIndex,
                  (unsigned)pCtx->slot,
                  (unsigned)results[0].bytesWritten, (int)results[0].serialized,
                  (unsigned)emissionSeq[0], (unsigned)emissionPeers[0],
                  (unsigned)results[0].publishResult,
                  (unsigned)sample.overwritten,
                  (unsigned)pCtx->cbGapHist[0].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[1].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[2].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[3].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[4].load(std::memory_order_relaxed),
                  (unsigned)sample.produced);
        }
        else
        {
            LOG_I(MODULE_PREFIX,
                  "autoPubData devID=%s typeIdx=%u slot=%u/%u priCDR=%u serOK=%d seq=%u peers=%u pub=%u | secCDR=%u secOK=%d secSeq=%u secPeers=%u pub=%u overwritten=%u (sample #%u)",
                  pCtx->deviceID.toString().c_str(), (unsigned)pCtx->deviceTypeIndex,
                  (unsigned)pCtx->slot, (unsigned)pCtx->secondarySlot,
                  (unsigned)results[0].bytesWritten, (int)results[0].serialized,
                  (unsigned)emissionSeq[0], (unsigned)emissionPeers[0],
                  (unsigned)results[0].publishResult,
                  (unsigned)results[1].bytesWritten, (int)results[1].serialized,
                  (unsigned)emissionSeq[1], (unsigned)emissionPeers[1],
                  (unsigned)results[1].publishResult,
                  (unsigned)sample.overwritten, (unsigned)sample.produced);
        }
    });
}
