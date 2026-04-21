/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework (Phase 1: Discoverable Node)
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftROS.h"
#include "RestAPIEndpointManager.h"
#include "RTPSTypes.h"
#include "RTPSMessage.h"
#include "RTPSAckNack.h"
#include "RTPSAckNackRunner.h"
#include "RTPSReliabilityPolicy.h"
#include "RTPSRuntimeSchedule.h"
#include "RTPSBuiltinEndpointMap.h"
#include "RTPSInitialAnnouncePlan.h"
#include "RTPSInitialAnnounceRunner.h"
#include "RTPSWriterHeartbeatRunner.h"
#include "RTPSRxSubmessageRunner.h"
#include "RTPSRunnerAdapterHelpers.h"
#include "runtime/discovery/RTPSDiscoveryRuntime.h"

// Socket / network headers (ESP-IDF / lwIP)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_mac.h"
#include "esp_netif.h"

static const char* MODULE_PREFIX = "RaftROS";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftROS::RaftROS(const char* pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
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
    _isEnabled = configGetBool("enable", false);
    _domainId = configGetLong("domainId", 0);
    _nodeName = configGetString("nodeName", "raft_esp32");
    _nodeNamespace = configGetString("nodeNamespace", "/");
    _leaseDurationSec = configGetLong("leaseDurationSec", 120);
    _spdpIntervalMs = configGetLong("spdpIntervalMs", 30000);

    if (!_isEnabled)
    {
        LOG_I(MODULE_PREFIX, "setup DISABLED");
        return;
    }

    // Initialize RTPS participant (GUID will be set when MAC is available)
    _participant.init(_domainId, _nodeName.c_str(), 0, nullptr);

    LOG_I(MODULE_PREFIX, "setup domainId %d nodeName %s ns %s lease %ds spdp %dms",
          (int)_domainId, _nodeName.c_str(), _nodeNamespace.c_str(),
          (int)_leaseDurationSec, (int)_spdpIntervalMs);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop - connection state machine
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::loop()
{
    if (!_isEnabled)
        return;

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
                LOG_I(MODULE_PREFIX, "loop sockets created, IP %s, SPDP port %d",
                      inet_ntoa(*(struct in_addr*)&_myIpAddr),
                      (int)_participant.getSPDPMulticastPort());
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

        // Periodic writer heartbeats + ros_discovery_info resend
        if (_connState == ConnState::ACTIVE &&
            RTPSRuntimeSchedule_isPeriodicDue(now, _lastWriterHbMs, WRITER_HB_INTERVAL_MS, true))
        {
            sendWriterHeartbeats();
            _lastWriterHbMs = now;
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
        break;
    }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get local WiFi station IP (network byte order), 0 if not connected
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    _spdpSeqNum++;
    uint32_t msgLen = _spdpHandler.buildAnnouncementMessage(
        _sendBuf, sizeof(_sendBuf),
        _participant, _myIpAddr,
        _leaseDurationSec, _spdpSeqNum);

    if (msgLen == 0)
        return;

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(_participant.getSPDPMulticastPort());
    dest.sin_addr.s_addr = inet_addr(RTPS_DEFAULT_MULTICAST_ADDR);

    int sent = sendto(_spdpSock, _sendBuf, msgLen, 0,
                      (struct sockaddr*)&dest, sizeof(dest));
    if (sent < 0)
    {
        LOG_E(MODULE_PREFIX, "sendSPDP sendto failed errno %d", errno);
    }
    else
    {
        LOG_I(MODULE_PREFIX, "sendSPDP sent %d bytes seq %llu to %s:%d",
              sent, (unsigned long long)_spdpSeqNum,
              inet_ntoa(dest.sin_addr), (int)ntohs(dest.sin_port));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive and process incoming SPDP (non-blocking)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::recvSPDP()
{
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(_spdpSock, _recvBuf, sizeof(_recvBuf), 0,
                     (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0)
        return;

    LOG_I(MODULE_PREFIX, "recvSPDP raw %d bytes from %s:%d magic=%c%c%c%c",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port),
          n > 0 ? _recvBuf[0] : '?', n > 1 ? _recvBuf[1] : '?',
          n > 2 ? _recvBuf[2] : '?', n > 3 ? _recvBuf[3] : '?');

    DiscoveredParticipant remote;
    if (!_spdpHandler.parseAnnouncementMessage(_recvBuf, (uint32_t)n, remote))
    {
        LOG_W(MODULE_PREFIX, "recvSPDP parse FAILED (%d bytes)", n);
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

    // Merge participant into discovered set and apply side effects on new additions.
    RaftROS::RTPS::Runtime::DiscoveryRuntime::MergeResult mergeResult =
        RaftROS::RTPS::Runtime::DiscoveryRuntime::mergeParticipant(
            _discovered, remote, nowMs, MAX_DISCOVERED);
    if (mergeResult == RaftROS::RTPS::Runtime::DiscoveryRuntime::MergeResult::AddedNew)
    {
        handleNewParticipant(remote, fromAddr);

        RTPSParticipantSetPolicyResult setPolicy =
            RaftROS::RTPS::Runtime::DiscoveryRuntime::applyParticipantSetPolicy(
                _connState == ConnState::ACTIVE,
                (uint32_t)_discovered.size(),
                true,
                true);

        if (setPolicy.shouldBeActive)
            _connState = ConnState::ACTIVE;
        if (setPolicy.triggerImmediateWriterHeartbeat)
            RaftROS::RTPS::Runtime::DiscoveryRuntime::onActivated(_lastWriterHbMs);
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

    LOG_I(MODULE_PREFIX, "recvMetatraffic %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));

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
    callbacks.onInvalidHeader = [](void*, RTPSRxChannel)
    {
        LOG_W(MODULE_PREFIX, "recvMetatraffic invalid RTPS header");
    };
    callbacks.onHeartbeat = [](void*, RTPSRxChannel, const uint8_t* writerEID, uint32_t lastSNLow, bool responded, int sentBytes)
    {
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
            LOG_I(MODULE_PREFIX, "  SPDP DATA on metatraffic, parsing as participant announcement");
            DiscoveredParticipant remote;
            if (self->_spdpHandler.parseAnnouncementMessage(packet, packetLen, remote))
            {
                if (memcmp(remote.guidPrefix, self->_participant.getGuidPrefix(), 12) != 0)
                    self->processDiscoveredParticipant(remote, from);
            }
        }
        else
        {
            LOG_I(MODULE_PREFIX, "  submsg: DATA writerEID=%02X%02X%02X%02X len=%d",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], (int)contentLen);
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

    LOG_I(MODULE_PREFIX, "recvUserData %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));

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
    callbacks.onInvalidHeader = nullptr;
    callbacks.onHeartbeat = [](void*, RTPSRxChannel, const uint8_t* writerEID, uint32_t lastSNLow, bool responded, int sentBytes)
    {
        if (responded)
        {
            LOG_I(MODULE_PREFIX, "  UD HEARTBEAT writerEID=%02X%02X%02X%02X lastSN=%u -> ACKNACK %d bytes",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                  lastSNLow, sentBytes);
        }
    };
    callbacks.onData = nullptr;
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

    struct AckExecCtx
    {
        RTPSAckNackRunnerExecAdapterCtx exec;
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionSedpSequenceContext
            sedpSequenceContext;
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionUserDataSequenceContext
            userDataSequenceContext;
        RTPSAckNackRunnerActionExecSpec sedpRosDiscoveryPublicationSpec;
        RTPSAckNackRunnerActionExecSpec sedpChatterPublicationSpec;
        RTPSAckNackRunnerActionExecSpec sedpRosDiscoverySubscriptionSpec;
        RTPSAckNackRunnerActionExecSpec rosDiscoveryInfoSpec;
        RTPSAckNackRunnerActionExecSpec chatterDataSpec;
        RaftROS* self = nullptr;
        const uint8_t* srcGuidPrefix = nullptr;
    } ctx;

    ctx.self = this;
    ctx.srcGuidPrefix = srcGuidPrefix;

    RTPSAckNackRunnerExecInitConfig execInit;
    execInit.discovered = &_discovered;
    execInit.metatrafficSock = _metatrafficSock;
    execInit.userDataSock = _userDataSock;
    execInit.sendBuf = _sendBuf;
    execInit.heartbeatCount = &_heartbeatCount;
    execInit.mutationPolicy.incrementHeartbeatOnSedpRetransmit = false;
    execInit.mutationPolicy.incrementHeartbeatOnUserDataRetransmit = true;
    execInit.dumpRosDiscoveryPayloadHex = false;
    RTPSRunnerAdapter_initAckExecContext(ctx.exec, execInit);

    ctx.sedpSequenceContext =
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::makeAckSedpSequenceContext(
            _sedpSeqNum,
            _sedpSubSeqNum,
            _chatterSedpSeqNum,
            false);
    ctx.userDataSequenceContext =
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::makeAckUserDataSequenceContext(
            _rosDiscSeqNum,
            _chatterSeqNum,
            false);

    RTPSAckNackRunnerCallbackInitConfig callbackInit;
    callbackInit.logParsed = [](void*, const RTPSAckNackFields& fields, RTPSAckNackWriterKind writerKind)
    {
        LOG_I(MODULE_PREFIX, "  ACKNACK readerEID=%02X%02X%02X%02X writerEID=%02X%02X%02X%02X (%s) base=%u numBits=%u",
              fields.readerEID[0], fields.readerEID[1], fields.readerEID[2], fields.readerEID[3],
              fields.writerEID[0], fields.writerEID[1], fields.writerEID[2], fields.writerEID[3],
              RTPSAckNack_writerKindToStr(writerKind),
              fields.bitmapBaseLow, fields.numBits);
    };
    callbackInit.unknownRemote = [](void*)
    {
        LOG_W(MODULE_PREFIX, "  ACKNACK from unknown participant");
    };
    callbackInit.getChatterSeq = [](void* userCtx) -> uint64_t
    {
        return static_cast<AckExecCtx*>(userCtx)->self->_chatterSeqNum;
    };

    ctx.sedpRosDiscoveryPublicationSpec.sendChannel = RTPSAckNackRunnerSendChannel::Metatraffic;
    ctx.sedpRosDiscoveryPublicationSpec.buildMessage = [](void* actionCtx,
                                                          const uint8_t* srcGuidPrefix,
                                                          uint64_t) -> uint32_t
    {
        AckExecCtx* p = static_cast<AckExecCtx*>(actionCtx);
        RaftROS* self = p->self;
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionSedpPlan plan;
        if (!RaftROS::RTPS::Runtime::ReliabilityAndWriterState::getAckActionSedpPlan(
                RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication,
                p->sedpSequenceContext,
                plan))
            return 0;
        return self->_sedpHandler.buildPublicationMessage(
            self->_sendBuf, sizeof(self->_sendBuf),
            self->_participant, srcGuidPrefix,
            plan.endpoint.entityId,
            plan.endpoint.topicName,
            plan.endpoint.typeName,
            plan.endpoint.reliabilityKind,
            plan.endpoint.durabilityKind,
            plan.sequenceNumber, self->_myIpAddr);
    };

    ctx.sedpChatterPublicationSpec.sendChannel = RTPSAckNackRunnerSendChannel::Metatraffic;
    ctx.sedpChatterPublicationSpec.buildMessage = nullptr;

    ctx.sedpRosDiscoverySubscriptionSpec.sendChannel = RTPSAckNackRunnerSendChannel::Metatraffic;
    ctx.sedpRosDiscoverySubscriptionSpec.buildMessage = [](void* actionCtx,
                                                           const uint8_t* srcGuidPrefix,
                                                           uint64_t) -> uint32_t
    {
        AckExecCtx* p = static_cast<AckExecCtx*>(actionCtx);
        RaftROS* self = p->self;
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionSedpPlan plan;
        if (!RaftROS::RTPS::Runtime::ReliabilityAndWriterState::getAckActionSedpPlan(
                RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction::RetransmitSedpRosDiscoverySubscription,
                p->sedpSequenceContext,
                plan))
            return 0;
        return self->_sedpHandler.buildSubscriptionMessage(
            self->_sendBuf, sizeof(self->_sendBuf),
            self->_participant, srcGuidPrefix,
            plan.endpoint.entityId,
            plan.endpoint.topicName,
            plan.endpoint.typeName,
            plan.endpoint.reliabilityKind,
            plan.endpoint.durabilityKind,
            plan.sequenceNumber, self->_myIpAddr);
    };

    ctx.rosDiscoveryInfoSpec.sendChannel = RTPSAckNackRunnerSendChannel::UserData;
    ctx.rosDiscoveryInfoSpec.buildMessage = [](void* actionCtx,
                                               const uint8_t* srcGuidPrefix,
                                               uint64_t) -> uint32_t
    {
        AckExecCtx* p = static_cast<AckExecCtx*>(actionCtx);
        RaftROS* self = p->self;
        uint8_t rosDiscPayload[256];
        uint32_t rosDiscLen = self->buildRosDiscInfoWithGids(rosDiscPayload, sizeof(rosDiscPayload));
        if (rosDiscLen == 0)
            return 0;
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionUserDataPlan plan;
        if (!RaftROS::RTPS::Runtime::ReliabilityAndWriterState::getAckActionUserDataPlan(
                RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo,
                p->userDataSequenceContext,
                plan))
            return 0;
        if (RTPSRunnerAdapter_shouldDumpAckPayload(
                RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo,
                p->exec))
        {
            RTPSRunnerAdapter_logHexPayload("ROSDISC_PAYLOAD_HEX", rosDiscPayload, rosDiscLen);
        }
        return self->_sedpHandler.buildUserDataMessage(
            self->_sendBuf, sizeof(self->_sendBuf),
            self->_participant, srcGuidPrefix,
            plan.writerEntityId,
            rosDiscPayload, rosDiscLen,
            plan.sequenceNumber, self->_heartbeatCount);
    };

    ctx.chatterDataSpec.sendChannel = RTPSAckNackRunnerSendChannel::UserData;
    ctx.chatterDataSpec.buildMessage = [](void* actionCtx,
                                          const uint8_t* srcGuidPrefix,
                                          uint64_t chatterSeq) -> uint32_t
    {
        AckExecCtx* p = static_cast<AckExecCtx*>(actionCtx);
        RaftROS* self = p->self;
        p->userDataSequenceContext.chatterDataSeqNum = chatterSeq;
        char msgStr[64];
        snprintf(msgStr, sizeof(msgStr), "Hello from %s [%u]",
                 self->_nodeName.c_str(), self->_chatterMsgIndex > 0 ? self->_chatterMsgIndex - 1 : 0);
        uint8_t chatterPayload[256];
        uint32_t payloadLen = self->buildChatterPayload(chatterPayload, sizeof(chatterPayload), msgStr);
        if (payloadLen == 0)
            return 0;
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckActionUserDataPlan plan;
        if (!RaftROS::RTPS::Runtime::ReliabilityAndWriterState::getAckActionUserDataPlan(
                RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction::RetransmitChatterData,
                p->userDataSequenceContext,
                plan))
            return 0;
        if (plan.hasFirstSNOverride)
        {
            return self->_sedpHandler.buildUserDataMessage(
                self->_sendBuf, sizeof(self->_sendBuf),
                self->_participant, srcGuidPrefix,
                plan.writerEntityId,
                chatterPayload, payloadLen,
                plan.sequenceNumber, self->_heartbeatCount,
                plan.firstSN);
        }

        return self->_sedpHandler.buildUserDataMessage(
            self->_sendBuf, sizeof(self->_sendBuf),
            self->_participant, srcGuidPrefix,
            plan.writerEntityId,
            chatterPayload, payloadLen,
            plan.sequenceNumber, self->_heartbeatCount);
    };

    callbackInit.executeAction = [](void* userCtx,
                                    RTPSAckNackRunnerAction action,
                                    const RTPSAckNackFields&,
                                    RTPSAckNackWriterKind writerKind)
    {
        (void)writerKind;
        AckExecCtx* p = static_cast<AckExecCtx*>(userCtx);
        const uint64_t chatterSeq = p->self->_chatterSeqNum;

        LOG_I(MODULE_PREFIX, "  ACKNACK -> %s", RTPSRunnerAdapter_ackActionLogLabel(action));
        if (action == RTPSAckNackRunnerAction::RetransmitChatterData)
        {
            LOG_I(MODULE_PREFIX, "  ACKNACK chatter seq=%llu", (unsigned long long)chatterSeq);
        }

        RTPSRunnerAdapter_executeAckAction(
            p->exec,
            action,
            p->srcGuidPrefix,
            chatterSeq,
            p->sedpRosDiscoveryPublicationSpec,
            p->sedpChatterPublicationSpec,
            p->sedpRosDiscoverySubscriptionSpec,
            p->rosDiscoveryInfoSpec,
            p->chatterDataSpec,
            p,
            [](void*, RTPSAckNackRunnerAction action, int sentBytes, uint32_t msgLen)
            {
                LOG_I(MODULE_PREFIX,
                      "  %s %d/%d bytes",
                      RTPSRunnerAdapter_ackActionResultLabel(action),
                      sentBytes,
                      (int)msgLen);
            });
    };

            RTPSAckNackRunnerCallbacks callbacks;
            RTPSRunnerAdapter_initAckCallbacks(callbacks, callbackInit);

    RTPSAckNackRunnerOptions options;
    options.publicationsIncludesChatterAnnouncement = false;
    options.requirePublicationSeq2GateForRetransmit = false;
    RTPSAckNackRunner_run(srcGuidPrefix, pContent, contentLen, options, callbacks, &ctx);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Purge stale discovered participants whose lease has expired
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::purgeStaleParticipants()
{
    uint32_t now = millis();
    for (auto it = _discovered.begin(); it != _discovered.end(); )
    {
        if (RaftROS::RTPS::Runtime::DiscoveryRuntime::isLeaseExpired(
            now, it->discoveredTimeMs, it->leaseDurationSec))
        {
            LOG_I(MODULE_PREFIX, "purgeStale removing participant lease expired (%d sec ago)",
                  (int)((now - it->discoveredTimeMs) / 1000));
            it = _discovered.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send periodic heartbeats for our writers + resend ros_discovery_info
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::sendWriterHeartbeats()
{
    RTPSWriterHeartbeatCounterState counters = {
        _sedpSeqNum,
        _sedpSubSeqNum,
        _chatterSedpSeqNum,
        _livelinessSeqNum,
        _rosDiscSeqNum,
        _heartbeatCount,
        false,
    };

    for (const auto& remote : _discovered)
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
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            const DiscoveredParticipant& remoteRef = *ctx->remote;
            LOG_I(MODULE_PREFIX, "sendWriterHB rosDisc %d/%d to port %d",
                  sentBytes, (int)payloadLen, (int)remoteRef.userDataPort);
        };

        RTPSWriterHeartbeatRunner_run(
            RTPSWriterHeartbeatRuntimeFlavor::EspStyle,
            counters,
            callbacks,
            &execCtx);
    }

    _heartbeatCount = counters.heartbeatCount;
    _livelinessSeqNum = counters.livelinessSeqNum;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle a newly discovered participant: send SEDP + ros_discovery_info
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr)
{
    const RTPSInitialAnnouncePlan announcePlan = RTPSInitialAnnouncePlan_default();
    const RTPSInitialAnnounceSequence announceSeq = RTPSInitialAnnouncePlan_buildSequence(
        announcePlan, RTPSInitialAnnounceRuntimeFlavor::EspStyle);

    struct ExecCtx
    {
        RaftROS* self = nullptr;
        const DiscoveredParticipant* remote = nullptr;
        const struct sockaddr_in* senderAddr = nullptr;
    } execCtx = { this, &remote, &senderAddr };

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
                    sedpSpec.entityId, sedpSpec.topicName, sedpSpec.typeName,
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
    };

    RTPSInitialAnnounceRunnerContext runCtx;
    runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::EspStyle;
    runCtx.seqCounters = {
        _spdpSeqNum,
        _sedpSeqNum,
        _sedpSubSeqNum,
        _chatterSedpSeqNum,
        _livelinessSeqNum,
        _rosDiscSeqNum,
    };
    runCtx.heartbeatCount = _heartbeatCount;
    runCtx.previousPayloadLen = 0;

    RTPSInitialAnnounceRunner_run(announceSeq, runCtx, callbacks, &execCtx);

    _spdpSeqNum = runCtx.seqCounters.spdpSeqNum;
    _heartbeatCount = runCtx.heartbeatCount;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: build ros_discovery_info payload with our writer GIDs included
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RaftROS::buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen)
{
    const uint8_t* writerIds[] = { ENTITYID_CHATTER_WRITER };
    return SPDPHandler::buildRosDiscoveryInfoPayload(
        pBuf, bufLen,
        _participant.getParticipantGuid(),
        _nodeName.c_str(),
        _nodeNamespace.c_str(),
        writerIds, 1,   // 1 writer: chatter
        nullptr, 0);    // 0 readers
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
    snprintf(msgStr, sizeof(msgStr), "Hello from %s [%u]", _nodeName.c_str(), _chatterMsgIndex++);

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
            _chatterSeqNum, _heartbeatCount);

        if (msgLen > 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(remote.userDataPort);
            dest.sin_addr.s_addr = remote.ipAddr;
            int sent = sendto(_userDataSock, _sendBuf, msgLen, 0,
                              (struct sockaddr*)&dest, sizeof(dest));
            LOG_I(MODULE_PREFIX, "publishChatter seq=%llu \"%s\" sent %d/%d to port %d",
                  (unsigned long long)_chatterSeqNum, msgStr, sent, (int)msgLen,
                  (int)remote.userDataPort);
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
