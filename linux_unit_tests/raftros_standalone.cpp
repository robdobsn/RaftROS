/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS Linux Standalone — Native ROS 2 RTPS participant for testing
//
// Reuses all protocol-layer code (CDR, RTPS, SPDP, SEDP) but replaces
// Raft SysMod lifecycle with a simple main() loop using POSIX sockets.
//
// Build: make -C linux_unit_tests standalone
// Run:   ./linux_unit_tests/raftros_linux
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <poll.h>

#include "RTPSTypes.h"
#include "RTPSMessage.h"
#include "RTPSParticipant.h"
#include "SPDPHandler.h"
#include "SEDPHandler.h"
#include "Logger.h"

static const char* MODULE_PREFIX = "RaftROS";

static volatile bool g_running = true;
static void sigHandler(int) { g_running = false; }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const uint32_t DOMAIN_ID = 0;
static const char* NODE_NAME = "raft_linux";
static const char* NODE_NAMESPACE = "/";
static const uint32_t LEASE_DURATION_SEC = 120;
static const uint32_t SPDP_INTERVAL_MS = 5000;
static const uint32_t WRITER_HB_INTERVAL_MS = 1000;
static const uint32_t MAX_DISCOVERED = 8;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// State
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static RTPSParticipant participant;
static SPDPHandler spdpHandler;
static SEDPHandler sedpHandler;

static int spdpSock = -1;
static int metatrafficSock = -1;
static int userDataSock = -1;
static uint32_t myIpAddr = 0;

static uint64_t spdpSeqNum = 0;
static uint64_t sedpSeqNum = 1;
static uint64_t sedpSubSeqNum = 1;
static uint64_t rosDiscSeqNum = 1;
static uint64_t livelinessSeqNum = 1;
static uint32_t heartbeatCount = 0;
static uint32_t acknackCount = 0;

// Chatter topic (Phase 2)
static uint64_t chatterSeqNum = 0;
static uint32_t lastChatterSendMs = 0;
static const uint32_t CHATTER_PUBLISH_INTERVAL_MS = 1000;
static uint32_t chatterMsgIndex = 0;

static std::vector<DiscoveredParticipant> discovered;

static uint8_t sendBuf[1024];
static uint8_t recvBuf[2048];

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// millis() for Linux
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t millis()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get local IP — prefer interface with multicast route, or first non-loopback
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t getLocalIP(const char* ifName)
{
    struct ifaddrs* ifas;
    if (getifaddrs(&ifas) < 0)
        return 0;
    uint32_t result = 0;
    uint32_t fallback = 0;
    char fallbackName[IF_NAMESIZE] = {};
    for (struct ifaddrs* ifa = ifas; ifa; ifa = ifa->ifa_next)
    {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
            continue;
        if (ifa->ifa_flags & IFF_LOOPBACK)
            continue;
        if (ifName && ifName[0] && strcmp(ifa->ifa_name, ifName) != 0)
            continue;
        uint32_t addr = ((struct sockaddr_in*)ifa->ifa_addr)->sin_addr.s_addr;
        // Remember first non-loopback as fallback
        if (fallback == 0)
        {
            fallback = addr;
            strncpy(fallbackName, ifa->ifa_name, sizeof(fallbackName) - 1);
        }
        // Prefer MULTICAST-capable interfaces (skip point-to-point / Tailscale)
        if (ifa->ifa_flags & IFF_MULTICAST)
        {
            // Prefer interfaces with a standard subnet (not /32 point-to-point)
            struct sockaddr_in* mask = (struct sockaddr_in*)ifa->ifa_netmask;
            uint32_t netmask = mask ? ntohl(mask->sin_addr.s_addr) : 0;
            if (netmask != 0xFFFFFFFF || result == 0)
            {
                result = addr;
                LOG_I(MODULE_PREFIX, "Using interface %s IP %s", ifa->ifa_name,
                      inet_ntoa(((struct sockaddr_in*)ifa->ifa_addr)->sin_addr));
                if (netmask != 0xFFFFFFFF)
                    break;  // Good candidate, stop looking
            }
        }
    }
    freeifaddrs(ifas);
    if (result == 0 && fallback != 0)
    {
        result = fallback;
        struct in_addr a; a.s_addr = fallback;
        LOG_I(MODULE_PREFIX, "Using fallback interface %s IP %s", fallbackName, inet_ntoa(a));
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create sockets
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool createSockets()
{
    // ---- SPDP multicast socket ----
    spdpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (spdpSock < 0) { perror("spdp socket"); return false; }

    int reuse = 1;
    setsockopt(spdpSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(spdpSock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(participant.getSPDPMulticastPort());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(spdpSock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("spdp bind");
        return false;
    }

    // Join SPDP multicast group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(RTPS_DEFAULT_MULTICAST_ADDR);
    mreq.imr_interface.s_addr = myIpAddr;
    if (setsockopt(spdpSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        perror("igmp join");
        return false;
    }

    struct in_addr ifAddr;
    ifAddr.s_addr = myIpAddr;
    setsockopt(spdpSock, IPPROTO_IP, IP_MULTICAST_IF, &ifAddr, sizeof(ifAddr));

    // Enable multicast loopback for same-host testing (needed so other
    // processes on this machine can receive our SPDP multicast)
    uint8_t loop = 1;
    setsockopt(spdpSock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    fcntl(spdpSock, F_SETFL, fcntl(spdpSock, F_GETFL, 0) | O_NONBLOCK);

    // ---- Metatraffic unicast socket ----
    metatrafficSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (metatrafficSock < 0) { perror("metatraffic socket"); return false; }
    setsockopt(metatrafficSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(participant.getMetatrafficUnicastPort());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(metatrafficSock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("metatraffic bind");
        return false;
    }
    fcntl(metatrafficSock, F_SETFL, fcntl(metatrafficSock, F_GETFL, 0) | O_NONBLOCK);

    // ---- User data unicast socket ----
    userDataSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (userDataSock < 0) { perror("userdata socket"); return false; }
    setsockopt(userDataSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(participant.getUserUnicastPort());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(userDataSock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("userdata bind");
        return false;
    }
    fcntl(userDataSock, F_SETFL, fcntl(userDataSock, F_GETFL, 0) | O_NONBLOCK);

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Process discovered participant (shared by SPDP multicast and metatraffic unicast)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr);
static void handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr);
static void handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen, const struct sockaddr_in& fromAddr);

static void processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr)
{
    // Update lease timer if already known
    for (auto& dp : discovered)
    {
        if (memcmp(dp.guidPrefix, remote.guidPrefix, 12) == 0)
        {
            dp.discoveredTimeMs = millis();
            return;
        }
    }

    char srcIpStr[16], locIpStr[16];
    strncpy(srcIpStr, inet_ntoa(fromAddr.sin_addr), sizeof(srcIpStr));
    srcIpStr[sizeof(srcIpStr)-1] = '\0';
    struct in_addr locAddr;
    locAddr.s_addr = remote.ipAddr;
    strncpy(locIpStr, inet_ntoa(locAddr), sizeof(locIpStr));
    locIpStr[sizeof(locIpStr)-1] = '\0';
    LOG_I(MODULE_PREFIX, "discovered from %s:%d locator=%s metaPort=%d userPort=%d lease=%d",
          srcIpStr, (int)ntohs(fromAddr.sin_port), locIpStr,
          (int)remote.metatrafficPort, (int)remote.userDataPort, (int)remote.leaseDurationSec);

    remote.discoveredTimeMs = millis();
    if (discovered.size() < MAX_DISCOVERED)
    {
        discovered.push_back(remote);
        handleNewParticipant(remote, fromAddr);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: build ros_discovery_info payload with our writer GIDs included
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen)
{
    const uint8_t* writerIds[] = { ENTITYID_CHATTER_WRITER };
    return SPDPHandler::buildRosDiscoveryInfoPayload(
        pBuf, bufLen,
        participant.getParticipantGuid(),
        NODE_NAME, NODE_NAMESPACE,
        writerIds, 1,
        nullptr, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build a CDR-encoded std_msgs/String payload
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t buildChatterPayload(uint8_t* pBuf, uint32_t bufLen, const char* message)
{
    uint32_t msgLen = (uint32_t)strlen(message) + 1;
    uint32_t totalLen = 4 + 4 + ((msgLen + 3) & ~3u);
    if (bufLen < totalLen)
        return 0;

    uint32_t pos = 0;
    pBuf[pos++] = 0x00;
    pBuf[pos++] = 0x01;  // CDR_LE
    pBuf[pos++] = 0x00;
    pBuf[pos++] = 0x00;

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

static void publishChatter()
{
    char msgStr[64];
    snprintf(msgStr, sizeof(msgStr), "Hello from %s [%u]", NODE_NAME, chatterMsgIndex++);

    uint8_t chatterPayload[256];
    uint32_t payloadLen = buildChatterPayload(chatterPayload, sizeof(chatterPayload), msgStr);
    if (payloadLen == 0)
        return;

    chatterSeqNum++;

    for (const auto& remote : discovered)
    {
        heartbeatCount++;
        uint32_t msgLen = sedpHandler.buildUserDataMessage(
            sendBuf, sizeof(sendBuf),
            participant, remote.guidPrefix,
            ENTITYID_CHATTER_WRITER,
            chatterPayload, payloadLen,
            chatterSeqNum, heartbeatCount,
            chatterSeqNum);  // firstSN = current (volatile, no history)

        if (msgLen > 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(remote.userDataPort);
            dest.sin_addr.s_addr = remote.ipAddr;
            int sent = sendto(userDataSock, sendBuf, msgLen, 0,
                              (struct sockaddr*)&dest, sizeof(dest));
            LOG_I(MODULE_PREFIX, "publishChatter seq=%llu \"%s\" sent %d/%d to port %d",
                  (unsigned long long)chatterSeqNum, msgStr, sent, (int)msgLen,
                  (int)remote.userDataPort);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send SPDP
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void sendSPDP()
{
    spdpSeqNum++;
    uint32_t msgLen = spdpHandler.buildAnnouncementMessage(
        sendBuf, sizeof(sendBuf), participant, myIpAddr, LEASE_DURATION_SEC, spdpSeqNum);
    if (msgLen == 0) return;

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(participant.getSPDPMulticastPort());
    dest.sin_addr.s_addr = inet_addr(RTPS_DEFAULT_MULTICAST_ADDR);

    int sent = sendto(spdpSock, sendBuf, msgLen, 0, (struct sockaddr*)&dest, sizeof(dest));
    LOG_I(MODULE_PREFIX, "sendSPDP %d bytes seq %llu to %s:%d",
          sent, (unsigned long long)spdpSeqNum,
          inet_ntoa(dest.sin_addr), (int)ntohs(dest.sin_port));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive SPDP
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void recvSPDP()
{
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(spdpSock, recvBuf, sizeof(recvBuf), 0, (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0) return;

    LOG_I(MODULE_PREFIX, "recvSPDP %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));

    DiscoveredParticipant remote;
    if (!spdpHandler.parseAnnouncementMessage(recvBuf, (uint32_t)n, remote))
    {
        LOG_W(MODULE_PREFIX, "recvSPDP parse FAILED");
        return;
    }

    if (memcmp(remote.guidPrefix, participant.getGuidPrefix(), 12) == 0)
        return;

    processDiscoveredParticipant(remote, fromAddr);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle ACKNACK — retransmit requested data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen,
                          const struct sockaddr_in& fromAddr)
{
    if (contentLen < 24) return;

    const uint8_t* readerEID = pContent;
    const uint8_t* writerEID = pContent + 4;
    uint32_t bitmapBaseLow = RTPSMessage::readLE32(pContent + 12);
    uint32_t numBits = RTPSMessage::readLE32(pContent + 16);

    LOG_I(MODULE_PREFIX, "  ACKNACK readerEID=%02X%02X%02X%02X writerEID=%02X%02X%02X%02X base=%u numBits=%u",
          readerEID[0], readerEID[1], readerEID[2], readerEID[3],
          writerEID[0], writerEID[1], writerEID[2], writerEID[3],
          bitmapBaseLow, numBits);

    const DiscoveredParticipant* remote = nullptr;
    for (const auto& dp : discovered)
    {
        if (memcmp(dp.guidPrefix, srcGuidPrefix, 12) == 0)
        {
            remote = &dp;
            break;
        }
    }
    if (!remote) return;

    if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0 && bitmapBaseLow <= 2)
    {
        LOG_I(MODULE_PREFIX, "  ACKNACK -> retransmit SEDP publications (base=%u)", bitmapBaseLow);
        // Retransmit SN=1 (ros_discovery_info) if needed
        if (bitmapBaseLow <= 1)
        {
            heartbeatCount++;
            uint32_t sedpLen = sedpHandler.buildPublicationMessage(
                sendBuf, sizeof(sendBuf), participant, srcGuidPrefix,
                ENTITYID_ROS_DISC_INFO_WRITER, "ros_discovery_info",
                "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sedpSeqNum, myIpAddr,
                heartbeatCount);
            if (sedpLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote->metatrafficPort);
                dest.sin_addr.s_addr = remote->ipAddr;
                int sent = sendto(metatrafficSock, sendBuf, sedpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
                LOG_I(MODULE_PREFIX, "  SEDP retransmit rosDisc %d/%d bytes", sent, (int)sedpLen);
            }
        }
        // Retransmit SN=2 (/chatter) if needed
        if (bitmapBaseLow <= 2)
        {
            heartbeatCount++;
            uint32_t chatterSedpLen = sedpHandler.buildPublicationMessage(
                sendBuf, sizeof(sendBuf), participant, srcGuidPrefix,
                ENTITYID_CHATTER_WRITER, CHATTER_DDS_TOPIC, CHATTER_DDS_TYPE,
                RELIABILITY_RELIABLE, DURABILITY_VOLATILE, sedpSeqNum + 1, myIpAddr,
                heartbeatCount);
            if (chatterSedpLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote->metatrafficPort);
                dest.sin_addr.s_addr = remote->ipAddr;
                int sent = sendto(metatrafficSock, sendBuf, chatterSedpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
                LOG_I(MODULE_PREFIX, "  SEDP retransmit chatter %d/%d bytes", sent, (int)chatterSedpLen);
            }
        }
    }
    else if (memcmp(writerEID, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0 && bitmapBaseLow <= 1)
    {
        LOG_I(MODULE_PREFIX, "  ACKNACK -> retransmit ros_discovery_info");
        uint8_t rosDiscPayload[256];
        uint32_t rosDiscLen = buildRosDiscInfoWithGids(rosDiscPayload, sizeof(rosDiscPayload));
        if (rosDiscLen > 0)
        {
            // Hex dump the CDR payload
            fprintf(stderr, "ROSDISC_PAYLOAD_HEX (%u bytes):", rosDiscLen);
            for (uint32_t i = 0; i < rosDiscLen; i++) fprintf(stderr, " %02x", rosDiscPayload[i]);
            fprintf(stderr, "\n");
            heartbeatCount++;
            uint32_t msgLen = sedpHandler.buildUserDataMessage(
                sendBuf, sizeof(sendBuf), participant, srcGuidPrefix,
                ENTITYID_ROS_DISC_INFO_WRITER, rosDiscPayload, rosDiscLen,
                rosDiscSeqNum, heartbeatCount);
            if (msgLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote->userDataPort);
                dest.sin_addr.s_addr = remote->ipAddr;
                int sent = sendto(userDataSock, sendBuf, msgLen, 0, (struct sockaddr*)&dest, sizeof(dest));
                LOG_I(MODULE_PREFIX, "  rosDisc retransmit %d/%d bytes", sent, (int)msgLen);
            }
        }
    }
    else if (memcmp(writerEID, ENTITYID_CHATTER_WRITER, 4) == 0)
    {
        if (chatterSeqNum > 0 && bitmapBaseLow <= chatterSeqNum)
        {
            LOG_I(MODULE_PREFIX, "  ACKNACK -> retransmit chatter seq=%llu",
                  (unsigned long long)chatterSeqNum);
            char msgStr[64];
            snprintf(msgStr, sizeof(msgStr), "Hello from %s [%u]",
                     NODE_NAME, chatterMsgIndex > 0 ? chatterMsgIndex - 1 : 0);
            uint8_t chatterPayload[256];
            uint32_t payloadLen = buildChatterPayload(chatterPayload, sizeof(chatterPayload), msgStr);
            if (payloadLen > 0)
            {
                heartbeatCount++;
                uint32_t msgLen = sedpHandler.buildUserDataMessage(
                    sendBuf, sizeof(sendBuf), participant, srcGuidPrefix,
                    ENTITYID_CHATTER_WRITER, chatterPayload, payloadLen,
                    chatterSeqNum, heartbeatCount,
                    chatterSeqNum);  // firstSN = current (volatile)
                if (msgLen > 0)
                {
                    struct sockaddr_in dest = {};
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(remote->userDataPort);
                    dest.sin_addr.s_addr = remote->ipAddr;
                    int sent = sendto(userDataSock, sendBuf, msgLen, 0, (struct sockaddr*)&dest, sizeof(dest));
                    LOG_I(MODULE_PREFIX, "  chatter retransmit %d/%d bytes", sent, (int)msgLen);
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive metatraffic
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void recvMetatraffic()
{
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(metatrafficSock, recvBuf, sizeof(recvBuf), 0, (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0) return;

    LOG_I(MODULE_PREFIX, "recvMetatraffic %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));

    uint8_t srcGuidPrefix[12];
    uint32_t hdrLen = RTPSMessage::parseHeader(recvBuf, (uint32_t)n, srcGuidPrefix);
    if (hdrLen == 0) return;

    uint32_t offset = hdrLen;
    while (offset < (uint32_t)n)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            recvBuf + offset, (uint32_t)n - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0) break;

        if (submsgId == SUBMSG_HEARTBEAT && contentLen >= 28)
        {
            const uint8_t* readerEID = pContent;
            const uint8_t* writerEID = pContent + 4;
            uint32_t lastSNLow = RTPSMessage::readLE32(pContent + 24);

            const uint8_t* ourReaderEID = ENTITYID_UNKNOWN;
            if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER;
            else if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER;
            else if (memcmp(writerEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER;
            else if (memcmp(writerEID, ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER, 4) == 0)
                ourReaderEID = ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER;
            else
                ourReaderEID = readerEID;

            bool isFinal = (flags & 0x02) != 0;
            if (!isFinal)
            {
                uint8_t ackBuf[80];
                uint32_t pos = 0;
                pos += RTPSMessage::writeHeader(ackBuf + pos, sizeof(ackBuf) - pos, participant.getGuidPrefix());
                pos += RTPSMessage::writeInfoDST(ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);
                uint32_t lastSNHigh = RTPSMessage::readLE32(pContent + 20);
                uint32_t ackBaseLow = lastSNLow + 1;
                uint32_t ackBaseHigh = lastSNHigh + (ackBaseLow == 0 ? 1 : 0);
                acknackCount++;
                pos += RTPSMessage::writeAcknack(ackBuf + pos, sizeof(ackBuf) - pos,
                    ourReaderEID, writerEID, (int32_t)ackBaseHigh, ackBaseLow, acknackCount);

                int sent = sendto(metatrafficSock, ackBuf, pos, 0, (struct sockaddr*)&fromAddr, sizeof(fromAddr));
                LOG_I(MODULE_PREFIX, "  HB writerEID=%02X%02X%02X%02X SN=%u -> ACKNACK %d bytes",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow, sent);
            }
            else
            {
                LOG_I(MODULE_PREFIX, "  HB(final) writerEID=%02X%02X%02X%02X SN=%u",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow);
            }
        }
        else if (submsgId == SUBMSG_DATA && contentLen >= 24)
        {
            const uint8_t* writerEID = pContent + 8;
            if (memcmp(writerEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
            {
                LOG_I(MODULE_PREFIX, "  SPDP DATA on metatraffic");
                DiscoveredParticipant remote;
                if (spdpHandler.parseAnnouncementMessage(recvBuf, (uint32_t)n, remote))
                {
                    if (memcmp(remote.guidPrefix, participant.getGuidPrefix(), 12) != 0)
                        processDiscoveredParticipant(remote, fromAddr);
                }
            }
            else
            {
                LOG_I(MODULE_PREFIX, "  DATA writerEID=%02X%02X%02X%02X len=%d",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3], (int)contentLen);
                // Hex dump incoming SEDP DATA for debugging
                if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0 ||
                    memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
                {
                    // Dump the serialized payload (after DATA header: extraFlags(2)+octetsToInlineQoS(2)+readerEID(4)+writerEID(4)+SN(8)=20)
                    if (contentLen > 20)
                    {
                        const uint8_t* payload = pContent + 20;
                        uint32_t payloadLen = contentLen - 20;
                        fprintf(stderr, "SEDP_INCOMING_%s_HEX (%d bytes):",
                                memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0 ? "PUB" : "SUB",
                                (int)payloadLen);
                        for (uint32_t i = 0; i < payloadLen && i < 512; i++)
                            fprintf(stderr, " %02x", payload[i]);
                        fprintf(stderr, "\n");
                    }
                }
            }
        }
        else if (submsgId == SUBMSG_ACKNACK && contentLen >= 24)
        {
            handleAcknack(srcGuidPrefix, pContent, contentLen, fromAddr);
        }
        else
        {
            const char* name = "?";
            switch (submsgId) {
                case SUBMSG_DATA:      name = "DATA"; break;
                case SUBMSG_HEARTBEAT: name = "HB"; break;
                case SUBMSG_ACKNACK:   name = "AN"; break;
                case SUBMSG_INFO_DST:  name = "DST"; break;
                case SUBMSG_INFO_TS:   name = "TS"; break;
                default: break;
            }
            LOG_I(MODULE_PREFIX, "  %s (0x%02X) flags=0x%02X len=%d",
                  name, (int)submsgId, (int)flags, (int)contentLen);
        }

        offset += submsgSize;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receive user data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void recvUserData()
{
    struct sockaddr_in fromAddr;
    socklen_t fromLen = sizeof(fromAddr);
    int n = recvfrom(userDataSock, recvBuf, sizeof(recvBuf), 0, (struct sockaddr*)&fromAddr, &fromLen);
    if (n <= 0) return;

    LOG_I(MODULE_PREFIX, "recvUserData %d bytes from %s:%d",
          n, inet_ntoa(fromAddr.sin_addr), (int)ntohs(fromAddr.sin_port));

    uint8_t srcGuidPrefix[12];
    uint32_t hdrLen = RTPSMessage::parseHeader(recvBuf, (uint32_t)n, srcGuidPrefix);
    if (hdrLen == 0) return;

    uint32_t offset = hdrLen;
    while (offset < (uint32_t)n)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            recvBuf + offset, (uint32_t)n - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0) break;

        if (submsgId == SUBMSG_HEARTBEAT && contentLen >= 28)
        {
            const uint8_t* readerEID = pContent;
            const uint8_t* writerEID = pContent + 4;
            uint32_t lastSNLow = RTPSMessage::readLE32(pContent + 24);
            uint32_t lastSNHigh = RTPSMessage::readLE32(pContent + 20);

            bool isFinal = (flags & 0x02) != 0;
            if (!isFinal)
            {
                uint8_t ackBuf[80];
                uint32_t pos = 0;
                pos += RTPSMessage::writeHeader(ackBuf + pos, sizeof(ackBuf) - pos, participant.getGuidPrefix());
                pos += RTPSMessage::writeInfoDST(ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);
                uint32_t ackBaseLow = lastSNLow + 1;
                uint32_t ackBaseHigh = lastSNHigh + (ackBaseLow == 0 ? 1 : 0);
                acknackCount++;
                pos += RTPSMessage::writeAcknack(ackBuf + pos, sizeof(ackBuf) - pos,
                    readerEID, writerEID, (int32_t)ackBaseHigh, ackBaseLow, acknackCount);

                // ACKNACKs go to metatraffic port
                struct sockaddr_in ackDest = fromAddr;
                for (const auto& dp : discovered)
                {
                    if (dp.ipAddr == fromAddr.sin_addr.s_addr)
                    {
                        ackDest.sin_port = htons(dp.metatrafficPort);
                        break;
                    }
                }
                int sent = sendto(metatrafficSock, ackBuf, pos, 0, (struct sockaddr*)&ackDest, sizeof(ackDest));
                LOG_I(MODULE_PREFIX, "  UD HB writerEID=%02X%02X%02X%02X SN=%u -> ACKNACK %d",
                      writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow, sent);
            }
        }
        else if (submsgId == SUBMSG_ACKNACK && contentLen >= 24)
        {
            handleAcknack(srcGuidPrefix, pContent, contentLen, fromAddr);
        }
        else
        {
            const char* name = "?";
            switch (submsgId) {
                case SUBMSG_DATA:      name = "DATA"; break;
                case SUBMSG_HEARTBEAT: name = "HB"; break;
                case SUBMSG_ACKNACK:   name = "AN"; break;
                case SUBMSG_INFO_DST:  name = "DST"; break;
                case SUBMSG_INFO_TS:   name = "TS"; break;
                default: break;
            }
            LOG_I(MODULE_PREFIX, "  UD %s (0x%02X) flags=0x%02X len=%d",
                  name, (int)submsgId, (int)flags, (int)contentLen);
        }
        offset += submsgSize;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Purge stale participants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void purgeStaleParticipants()
{
    uint32_t now = millis();
    for (auto it = discovered.begin(); it != discovered.end(); )
    {
        uint32_t leaseMs = it->leaseDurationSec * 1000 * 2;
        if (leaseMs == 0) leaseMs = 240000;
        if ((now - it->discoveredTimeMs) > leaseMs)
        {
            LOG_I(MODULE_PREFIX, "purge stale participant (expired %d sec ago)",
                  (int)((now - it->discoveredTimeMs) / 1000));
            it = discovered.erase(it);
        }
        else
            ++it;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send writer heartbeats + resend data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void sendWriterHeartbeats()
{
    for (const auto& remote : discovered)
    {
        // SEDP publication DATA + HB
        {
            heartbeatCount++;
            uint32_t sedpLen = sedpHandler.buildPublicationMessage(
                sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
                ENTITYID_ROS_DISC_INFO_WRITER, "ros_discovery_info",
                "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sedpSeqNum, myIpAddr,
                heartbeatCount);
            if (sedpLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote.metatrafficPort);
                dest.sin_addr.s_addr = remote.ipAddr;
                sendto(metatrafficSock, sendBuf, sedpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
            }
        }

        // SEDP subscription DATA + HB
        {
            heartbeatCount++;
            uint32_t sedpSubLen = sedpHandler.buildSubscriptionMessage(
                sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
                ENTITYID_ROS_DISC_INFO_READER, "ros_discovery_info",
                "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sedpSubSeqNum, myIpAddr,
                heartbeatCount);
            if (sedpSubLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote.metatrafficPort);
                dest.sin_addr.s_addr = remote.ipAddr;
                sendto(metatrafficSock, sendBuf, sedpSubLen, 0, (struct sockaddr*)&dest, sizeof(dest));
            }
        }

        // SEDP publication for /chatter DataWriter
        {
            heartbeatCount++;
            uint32_t chatterSedpLen = sedpHandler.buildPublicationMessage(
                sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
                ENTITYID_CHATTER_WRITER, CHATTER_DDS_TOPIC, CHATTER_DDS_TYPE,
                RELIABILITY_RELIABLE, DURABILITY_VOLATILE, sedpSeqNum + 1, myIpAddr,
                heartbeatCount);
            if (chatterSedpLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote.metatrafficPort);
                dest.sin_addr.s_addr = remote.ipAddr;
                sendto(metatrafficSock, sendBuf, chatterSedpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
            }
        }

        // Participant message data (liveliness)
        {
            heartbeatCount++;
            livelinessSeqNum++;
            uint32_t pmdLen = sedpHandler.buildParticipantMessageData(
                sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
                livelinessSeqNum, heartbeatCount);
            if (pmdLen > 0)
            {
                struct sockaddr_in dest = {};
                dest.sin_family = AF_INET;
                dest.sin_port = htons(remote.metatrafficPort);
                dest.sin_addr.s_addr = remote.ipAddr;
                sendto(metatrafficSock, sendBuf, pmdLen, 0, (struct sockaddr*)&dest, sizeof(dest));
            }
        }

        // ros_discovery_info DATA + HB
        {
            uint8_t rosDiscPayload[256];
            uint32_t rosDiscLen = buildRosDiscInfoWithGids(
                rosDiscPayload, sizeof(rosDiscPayload));
            if (rosDiscLen > 0)
            {
                // Hex dump payload on first send
                static bool dumpedOnce = false;
                if (!dumpedOnce) {
                    dumpedOnce = true;
                    fprintf(stderr, "ROSDISC_CDR (%u bytes):", rosDiscLen);
                    for (uint32_t i = 0; i < rosDiscLen; i++) fprintf(stderr, " %02x", rosDiscPayload[i]);
                    fprintf(stderr, "\n");
                }
                // Don't increment rosDiscSeqNum - content is static, always SN=1
                heartbeatCount++;
                uint32_t msgLen = sedpHandler.buildUserDataMessage(
                    sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
                    ENTITYID_ROS_DISC_INFO_WRITER, rosDiscPayload, rosDiscLen,
                    rosDiscSeqNum, heartbeatCount);
                if (msgLen > 0)
                {
                    struct sockaddr_in dest = {};
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(remote.userDataPort);
                    dest.sin_addr.s_addr = remote.ipAddr;
                    int sent = sendto(userDataSock, sendBuf, msgLen, 0, (struct sockaddr*)&dest, sizeof(dest));
                    LOG_I(MODULE_PREFIX, "sendWriterHB rosDisc %d/%d to port %d", sent, (int)msgLen, (int)remote.userDataPort);
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle newly discovered participant
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr)
{
    // Unicast SPDP reply
    spdpSeqNum++;
    uint32_t spdpLen = spdpHandler.buildAnnouncementMessage(
        sendBuf, sizeof(sendBuf), participant, myIpAddr, LEASE_DURATION_SEC, spdpSeqNum);
    if (spdpLen > 0)
    {
        int sent = sendto(spdpSock, sendBuf, spdpLen, 0,
                          (const struct sockaddr*)&senderAddr, sizeof(senderAddr));
        LOG_I(MODULE_PREFIX, "handleNewParticipant unicast SPDP %d/%d to %s:%d",
              sent, (int)spdpLen, inet_ntoa(((struct sockaddr_in&)senderAddr).sin_addr),
              (int)ntohs(senderAddr.sin_port));

        struct sockaddr_in dest = senderAddr;
        dest.sin_port = htons(participant.getSPDPMulticastPort());
        sendto(spdpSock, sendBuf, spdpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
    }

    // SEDP publication
    heartbeatCount++;
    uint32_t sedpLen = sedpHandler.buildPublicationMessage(
        sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
        ENTITYID_ROS_DISC_INFO_WRITER, "ros_discovery_info",
        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
        RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sedpSeqNum, myIpAddr,
        heartbeatCount);
    if (sedpLen > 0)
    {
        // Hex dump SEDP publication for debugging
        fprintf(stderr, "SEDP_PUB_HEX (%d bytes):", (int)sedpLen);
        for (uint32_t i = 0; i < sedpLen; i++)
            fprintf(stderr, " %02x", sendBuf[i]);
        fprintf(stderr, "\n");

        struct sockaddr_in dest = {};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(remote.metatrafficPort);
        dest.sin_addr.s_addr = remote.ipAddr;
        int sent = sendto(metatrafficSock, sendBuf, sedpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
        LOG_I(MODULE_PREFIX, "handleNewParticipant SEDP pub %d/%d to port %d",
              sent, (int)sedpLen, (int)remote.metatrafficPort);
    }

    // SEDP subscription announcement (we subscribe to ros_discovery_info)
    heartbeatCount++;
    uint32_t sedpSubLen = sedpHandler.buildSubscriptionMessage(
        sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
        ENTITYID_ROS_DISC_INFO_READER, "ros_discovery_info",
        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
        RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sedpSubSeqNum, myIpAddr,
        heartbeatCount);
    if (sedpSubLen > 0)
    {
        struct sockaddr_in dest = {};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(remote.metatrafficPort);
        dest.sin_addr.s_addr = remote.ipAddr;
        int sent = sendto(metatrafficSock, sendBuf, sedpSubLen, 0, (struct sockaddr*)&dest, sizeof(dest));
        LOG_I(MODULE_PREFIX, "handleNewParticipant SEDP sub %d/%d to port %d",
              sent, (int)sedpSubLen, (int)remote.metatrafficPort);
    }

    // SEDP publication announcement for /chatter DataWriter
    {
        heartbeatCount++;
        uint32_t chatterSedpLen = sedpHandler.buildPublicationMessage(
            sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
            ENTITYID_CHATTER_WRITER, CHATTER_DDS_TOPIC, CHATTER_DDS_TYPE,
            RELIABILITY_RELIABLE, DURABILITY_VOLATILE, sedpSeqNum + 1, myIpAddr,
            heartbeatCount);
        if (chatterSedpLen > 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(remote.metatrafficPort);
            dest.sin_addr.s_addr = remote.ipAddr;
            int sent = sendto(metatrafficSock, sendBuf, chatterSedpLen, 0, (struct sockaddr*)&dest, sizeof(dest));
            LOG_I(MODULE_PREFIX, "handleNewParticipant SEDP chatter pub %d/%d to port %d",
                  sent, (int)chatterSedpLen, (int)remote.metatrafficPort);
        }
    }

    // Participant message data (liveliness assertion)
    {
        heartbeatCount++;
        uint32_t pmdLen = sedpHandler.buildParticipantMessageData(
            sendBuf, sizeof(sendBuf), participant, remote.guidPrefix,
            livelinessSeqNum, heartbeatCount);
        if (pmdLen > 0)
        {
            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_port = htons(remote.metatrafficPort);
            dest.sin_addr.s_addr = remote.ipAddr;
            int sent = sendto(metatrafficSock, sendBuf, pmdLen, 0, (struct sockaddr*)&dest, sizeof(dest));
            LOG_I(MODULE_PREFIX, "handleNewParticipant liveliness %d/%d to port %d",
                  sent, (int)pmdLen, (int)remote.metatrafficPort);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    // Parse args
    const char* ifName = "";
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc)
            ifName = argv[++i];
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            printf("Usage: %s [-i <interface>]\n", argv[0]);
            printf("  -i <interface>  Network interface (e.g. eth0). Default: first non-loopback\n");
            return 0;
        }
    }

    printf("=== RaftROS Linux Standalone ===\n");
    printf("Node: %s%s\n", NODE_NAMESPACE, NODE_NAME);
    printf("Domain: %d\n", DOMAIN_ID);

    // Get local IP
    myIpAddr = getLocalIP(ifName);
    if (myIpAddr == 0)
    {
        fprintf(stderr, "No non-loopback IPv4 address found\n");
        return 1;
    }

    // Set GUID from a fake MAC (use IP bytes + pid for uniqueness)
    uint8_t fakeMac[6];
    uint32_t pid = (uint32_t)getpid();
    memcpy(fakeMac, &myIpAddr, 4);
    fakeMac[4] = (uint8_t)(pid >> 8);
    fakeMac[5] = (uint8_t)(pid & 0xFF);
    participant.init(DOMAIN_ID, NODE_NAME, 3);  // participantId=3 to avoid port conflicts with ros2 daemon
    participant.setGuidPrefixFromMAC(fakeMac);

    printf("GUID prefix: ");
    const uint8_t* gp = participant.getGuidPrefix();
    for (int i = 0; i < 12; i++) printf("%02x", gp[i]);
    printf("\n");
    printf("SPDP port: %d  Meta port: %d  User port: %d\n",
           participant.getSPDPMulticastPort(),
           participant.getMetatrafficUnicastPort(),
           participant.getUserUnicastPort());

    // Create sockets
    if (!createSockets())
    {
        fprintf(stderr, "Failed to create sockets\n");
        return 1;
    }

    printf("Sockets created. Waiting for discovery...\n");
    printf("Press Ctrl+C to stop.\n\n");

    // Send initial SPDP
    sendSPDP();

    uint32_t lastSpdpMs = millis();
    uint32_t lastHbMs = 0;
    bool active = false;

    // Main loop using poll() for efficiency
    struct pollfd fds[3];
    fds[0].fd = spdpSock;        fds[0].events = POLLIN;
    fds[1].fd = metatrafficSock; fds[1].events = POLLIN;
    fds[2].fd = userDataSock;    fds[2].events = POLLIN;

    while (g_running)
    {
        int ret = poll(fds, 3, 100);  // 100ms timeout

        uint32_t now = millis();

        // Periodic SPDP
        if (now - lastSpdpMs >= SPDP_INTERVAL_MS)
        {
            sendSPDP();
            lastSpdpMs = now;
        }

        // Periodic heartbeats (once active)
        if (active && (now - lastHbMs >= WRITER_HB_INTERVAL_MS))
        {
            sendWriterHeartbeats();
            lastHbMs = now;
        }

        // Periodic chatter publishing (once active)
        if (active && (now - lastChatterSendMs >= CHATTER_PUBLISH_INTERVAL_MS))
        {
            publishChatter();
            lastChatterSendMs = now;
        }

        purgeStaleParticipants();

        // Handle incoming
        if (ret > 0)
        {
            if (fds[0].revents & POLLIN) recvSPDP();
            if (fds[1].revents & POLLIN) recvMetatraffic();
            if (fds[2].revents & POLLIN) recvUserData();
        }

        // Transition to active on first discovery
        if (!active && !discovered.empty())
        {
            active = true;
            lastHbMs = 0;  // immediate HB
        }
    }

    printf("\nShutting down...\n");
    close(spdpSock);
    close(metatrafficSock);
    close(userDataSock);
    return 0;
}
