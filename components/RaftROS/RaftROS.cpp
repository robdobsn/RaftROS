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
        if (_lastSpdpSendMs == 0 || (now - _lastSpdpSendMs >= _spdpIntervalMs))
        {
            sendSPDP();
            _lastSpdpSendMs = now;
        }

        // Periodic writer heartbeats + ros_discovery_info resend
        if (_connState == ConnState::ACTIVE &&
            (now - _lastWriterHbMs >= WRITER_HB_INTERVAL_MS))
        {
            sendWriterHeartbeats();
            _lastWriterHbMs = now;
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
    // Check if already discovered — update lease timer if so
    for (auto& dp : _discovered)
    {
        if (memcmp(dp.guidPrefix, remote.guidPrefix, 12) == 0)
        {
            dp.discoveredTimeMs = millis();
            return;  // already known, lease refreshed
        }
    }

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

    // Add to list and send our endpoint info
    remote.discoveredTimeMs = millis();
    if (_discovered.size() < MAX_DISCOVERED)
    {
        _discovered.push_back(remote);
        handleNewParticipant(remote, fromAddr);
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

    // Parse RTPS header
    uint8_t srcGuidPrefix[12];
    uint32_t hdrLen = RTPSMessage::parseHeader(_recvBuf, (uint32_t)n, srcGuidPrefix);
    if (hdrLen == 0)
    {
        LOG_W(MODULE_PREFIX, "recvMetatraffic invalid RTPS header");
        return;
    }

    // Walk submessages — handle HEARTBEATs with ACKNACKs
    uint32_t offset = hdrLen;
    while (offset < (uint32_t)n)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            _recvBuf + offset, (uint32_t)n - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0)
            break;

        if (submsgId == SUBMSG_HEARTBEAT && contentLen >= 28)
        {
            // Parse HEARTBEAT: readerEID(4) + writerEID(4) + firstSN(8) + lastSN(8) + count(4)
            const uint8_t* readerEID = pContent;       // the writer's target reader (us)
            const uint8_t* writerEID = pContent + 4;    // the remote writer
            uint32_t lastSNHigh = RTPSMessage::readLE32(pContent + 20);
            uint32_t lastSNLow  = RTPSMessage::readLE32(pContent + 24);

            // Determine our matching reader for their writer
            const uint8_t* ourReaderEID = ENTITYID_UNKNOWN;
            if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER;
            else if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER;
            else if (memcmp(writerEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER;
            else
                ourReaderEID = readerEID;  // use whatever the HB says

            // Check if F flag (final) is NOT set — if not final, we MUST respond with ACKNACK
            bool isFinal = (flags & 0x02) != 0;
            if (!isFinal)
            {
                // Build ACKNACK: RTPS header + INFO_DST + ACKNACK
                uint8_t ackBuf[80];
                uint32_t pos = 0;
                pos += RTPSMessage::writeHeader(ackBuf + pos, sizeof(ackBuf) - pos, _participant.getGuidPrefix());
                pos += RTPSMessage::writeInfoDST(ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);
                // bitmapBase = lastSN + 1 (we acknowledge everything up to lastSN)
                uint32_t ackBaseLow = lastSNLow + 1;
                uint32_t ackBaseHigh = lastSNHigh + (ackBaseLow == 0 ? 1 : 0);  // handle overflow
                _acknackCount++;
                pos += RTPSMessage::writeAcknack(ackBuf + pos, sizeof(ackBuf) - pos,
                    ourReaderEID, writerEID,
                    (int32_t)ackBaseHigh, ackBaseLow, _acknackCount);

                int sent = sendto(_metatrafficSock, ackBuf, pos, 0,
                                  (struct sockaddr*)&fromAddr, sizeof(fromAddr));
                LOG_I(MODULE_PREFIX, "  HEARTBEAT writerEID=%02X%02X%02X%02X lastSN=%u -> ACKNACK %d bytes",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                      lastSNLow, sent);
            }
            else
            {
                LOG_I(MODULE_PREFIX, "  HEARTBEAT(final) writerEID=%02X%02X%02X%02X lastSN=%u",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow);
            }
        }
        else if (submsgId == SUBMSG_DATA && contentLen >= 24)
        {
            // Check if this is an SPDP DATA (writerEntityId = SPDP participant writer)
            // DATA content layout: extraFlags(2) + octetsToInlineQos(2) + readerEID(4) + writerEID(4) + ...
            const uint8_t* writerEID = pContent + 8;
            if (memcmp(writerEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
            {
                // This is a unicast SPDP reply — parse as full RTPS message from the beginning
                LOG_I(MODULE_PREFIX, "  SPDP DATA on metatraffic, parsing as participant announcement");
                DiscoveredParticipant remote;
                if (_spdpHandler.parseAnnouncementMessage(_recvBuf, (uint32_t)n, remote))
                {
                    if (memcmp(remote.guidPrefix, _participant.getGuidPrefix(), 12) != 0)
                    {
                        processDiscoveredParticipant(remote, fromAddr);
                    }
                }
            }
            else
            {
                LOG_I(MODULE_PREFIX, "  submsg: DATA writerEID=%02X%02X%02X%02X len=%d",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3], (int)contentLen);
            }
        }
        else
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
        }

        offset += submsgSize;
    }
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

    // Parse RTPS header
    uint8_t srcGuidPrefix[12];
    uint32_t hdrLen = RTPSMessage::parseHeader(_recvBuf, (uint32_t)n, srcGuidPrefix);
    if (hdrLen == 0)
        return;

    // Walk submessages — handle HEARTBEATs with ACKNACKs (same as metatraffic)
    uint32_t offset = hdrLen;
    while (offset < (uint32_t)n)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            _recvBuf + offset, (uint32_t)n - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0)
            break;

        if (submsgId == SUBMSG_HEARTBEAT && contentLen >= 28)
        {
            const uint8_t* readerEID = pContent;
            const uint8_t* writerEID = pContent + 4;
            uint32_t lastSNHigh = RTPSMessage::readLE32(pContent + 20);
            uint32_t lastSNLow  = RTPSMessage::readLE32(pContent + 24);

            bool isFinal = (flags & 0x02) != 0;
            if (!isFinal)
            {
                uint8_t ackBuf[80];
                uint32_t pos = 0;
                pos += RTPSMessage::writeHeader(ackBuf + pos, sizeof(ackBuf) - pos, _participant.getGuidPrefix());
                pos += RTPSMessage::writeInfoDST(ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);
                uint32_t ackBaseLow = lastSNLow + 1;
                uint32_t ackBaseHigh = lastSNHigh + (ackBaseLow == 0 ? 1 : 0);
                _acknackCount++;
                pos += RTPSMessage::writeAcknack(ackBuf + pos, sizeof(ackBuf) - pos,
                    readerEID, writerEID,
                    (int32_t)ackBaseHigh, ackBaseLow, _acknackCount);

                // Send ACKNACK back to sender's metatraffic port (ACKNACKs go on metatraffic)
                // Find the remote's metatraffic port from discovered list
                struct sockaddr_in ackDest = fromAddr;
                for (const auto& dp : _discovered)
                {
                    if (dp.ipAddr == fromAddr.sin_addr.s_addr)
                    {
                        ackDest.sin_port = htons(dp.metatrafficPort);
                        break;
                    }
                }
                int sent = sendto(_metatrafficSock, ackBuf, pos, 0,
                                  (struct sockaddr*)&ackDest, sizeof(ackDest));
                LOG_I(MODULE_PREFIX, "  UD HEARTBEAT writerEID=%02X%02X%02X%02X lastSN=%u -> ACKNACK %d bytes",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                      lastSNLow, sent);
            }
        }
        else
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
        }

        offset += submsgSize;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Purge stale discovered participants whose lease has expired
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::purgeStaleParticipants()
{
    uint32_t now = millis();
    for (auto it = _discovered.begin(); it != _discovered.end(); )
    {
        // Use 2x lease duration as grace period
        uint32_t leaseMs = it->leaseDurationSec * 1000 * 2;
        if (leaseMs == 0)
            leaseMs = 240000;  // default 4 minutes if lease unknown
        if ((now - it->discoveredTimeMs) > leaseMs)
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
    for (const auto& remote : _discovered)
    {
        // 1) Resend SEDP publication DATA + HB → remote metatraffic port
        {
            uint32_t sedpLen = _sedpHandler.buildPublicationMessage(
                _sendBuf, sizeof(_sendBuf),
                _participant, remote.guidPrefix,
                ENTITYID_ROS_DISC_INFO_WRITER,
                "ros_discovery_info",
                "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                RELIABILITY_RELIABLE,
                DURABILITY_TRANSIENT_LOCAL,
                _sedpSeqNum);  // resend same seqNum (not incrementing)

            if (sedpLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote.metatrafficPort);
                dest.sin_addr.s_addr = remote.ipAddr;
                sendto(_metatrafficSock, _sendBuf, sedpLen, 0,
                       (struct sockaddr*)&dest, sizeof(dest));
            }
        }

        // 2) Resend ros_discovery_info DATA + HB → remote user data port
        {
            uint8_t rosDiscPayload[256];
            uint32_t rosDiscLen = SPDPHandler::buildRosDiscoveryInfoPayload(
                rosDiscPayload, sizeof(rosDiscPayload),
                _participant.getParticipantGuid(),
                _nodeName.c_str(),
                _nodeNamespace.c_str());

            if (rosDiscLen > 0)
            {
                _heartbeatCount++;
                uint32_t msgLen = _sedpHandler.buildUserDataMessage(
                    _sendBuf, sizeof(_sendBuf),
                    _participant, remote.guidPrefix,
                    ENTITYID_ROS_DISC_INFO_WRITER,
                    rosDiscPayload, rosDiscLen,
                    _rosDiscSeqNum, _heartbeatCount);

                if (msgLen > 0)
                {
                    struct sockaddr_in dest = {};
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(remote.userDataPort);
                    dest.sin_addr.s_addr = remote.ipAddr;
                    int sent = sendto(_userDataSock, _sendBuf, msgLen, 0,
                           (struct sockaddr*)&dest, sizeof(dest));
                    LOG_I(MODULE_PREFIX, "sendWriterHB rosDisc %d/%d to port %d",
                          sent, (int)msgLen, (int)remote.userDataPort);
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle a newly discovered participant: send SEDP + ros_discovery_info
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr)
{
    // 0) Send our SPDP as unicast back to the sender's actual address:port
    //    This enables UDP hole punching through firewalls (e.g., WSL2)
    {
        _spdpSeqNum++;
        uint32_t spdpLen = _spdpHandler.buildAnnouncementMessage(
            _sendBuf, sizeof(_sendBuf),
            _participant, _myIpAddr,
            _leaseDurationSec, _spdpSeqNum);
        if (spdpLen > 0)
        {
            // Reply to the actual sender address:port (bridge/relay)
            int sent = sendto(_spdpSock, _sendBuf, spdpLen, 0,
                              (const struct sockaddr*)&senderAddr, sizeof(senderAddr));
            LOG_I(MODULE_PREFIX, "handleNewParticipant unicast SPDP sent %d/%d bytes to %s:%d",
                  sent, (int)spdpLen,
                  inet_ntoa(((struct sockaddr_in&)senderAddr).sin_addr),
                  (int)ntohs(senderAddr.sin_port));

            // Also send to port 7400 on the remote IP (for direct multicast listeners)
            struct sockaddr_in dest = senderAddr;
            dest.sin_port = htons(_participant.getSPDPMulticastPort());
            sendto(_spdpSock, _sendBuf, spdpLen, 0,
                   (struct sockaddr*)&dest, sizeof(dest));
        }
    }

    // 1) Send SEDP publication announcement for ros_discovery_info DataWriter
    uint32_t sedpLen = _sedpHandler.buildPublicationMessage(
        _sendBuf, sizeof(_sendBuf),
        _participant, remote.guidPrefix,
        ENTITYID_ROS_DISC_INFO_WRITER,
        "ros_discovery_info",
        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
        RELIABILITY_RELIABLE,
        DURABILITY_TRANSIENT_LOCAL,
        _sedpSeqNum);

    if (sedpLen > 0)
    {
        struct sockaddr_in dest = {};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(remote.metatrafficPort);
        dest.sin_addr.s_addr = remote.ipAddr;
        int sent = sendto(_metatrafficSock, _sendBuf, sedpLen, 0,
               (struct sockaddr*)&dest, sizeof(dest));
        LOG_I(MODULE_PREFIX, "handleNewParticipant SEDP pub sent %d/%d bytes to port %d",
              sent, (int)sedpLen, (int)remote.metatrafficPort);
    }

    // 2) ros_discovery_info will be sent by periodic sendWriterHeartbeats()
    //    (initial burst causes errno 12 = lwIP buffer exhaustion)

    _connState = ConnState::ACTIVE;
    _lastWriterHbMs = 0;  // trigger immediate writer HB on next loop
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
