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
#include "runtime/core/RTPSParticipant.h"
#include "runtime/discovery/SPDPHandler.h"
#include "runtime/announce/SEDPHandler.h"
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
    uint32_t nowMs = millis();

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

    RaftRuntime::RTPS::Runtime::DiscoveryRuntime::MergeResult mergeResult =
        RaftRuntime::RTPS::Runtime::DiscoveryRuntime::mergeParticipant(
            discovered, remote, nowMs, MAX_DISCOVERED);
    if (mergeResult == RaftRuntime::RTPS::Runtime::DiscoveryRuntime::MergeResult::AddedNew)
    {
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
    snprintf(msgStr, sizeof(msgStr), "Hello from %s [%lu]", NODE_NAME, (unsigned long)chatterMsgIndex++);

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

static uint32_t acknackBuildRosDiscInfoPayload(void*, uint8_t* buf, uint32_t bufLen)
{
    return buildRosDiscInfoWithGids(buf, bufLen);
}

static uint32_t acknackBuildChatterPayload(void*, uint8_t* buf, uint32_t bufLen)
{
    char msgStr[64];
    snprintf(msgStr, sizeof(msgStr), "Hello from %s [%lu]",
             NODE_NAME,
             (unsigned long)(chatterMsgIndex > 0 ? chatterMsgIndex - 1 : 0));
    return buildChatterPayload(buf, bufLen, msgStr);
}

static void handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen,
                          const struct sockaddr_in& fromAddr)
{
    (void)fromAddr;

    using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

    RTPSAckActionStandardCtx ctx;
    ctx.srcGuidPrefix = srcGuidPrefix;

    RTPSAckActionStandardInitConfig initConfig;
    initConfig.discovered = &discovered;
    initConfig.metatrafficSock = metatrafficSock;
    initConfig.userDataSock = userDataSock;
    initConfig.sendBuf = sendBuf;
    initConfig.sendBufLen = sizeof(sendBuf);
    initConfig.heartbeatCount = &heartbeatCount;
    initConfig.dumpRosDiscoveryPayloadHex = true;
    initConfig.runtimeFlavor = RTPSAckNackRuntimeFlavor::LinuxStandalone;
    initConfig.rosDiscoveryPublicationSeqNum = sedpSeqNum;
    initConfig.rosDiscoverySubscriptionSeqNum = sedpSubSeqNum;
    initConfig.chatterPublicationSeqNum = 0;
    initConfig.rosDiscoveryInfoSeqNum = rosDiscSeqNum;
    initConfig.chatterDataSeqNum = chatterSeqNum;
    initConfig.sedpHandler = &sedpHandler;
    initConfig.participant = &participant;
    initConfig.myIpAddr = myIpAddr;
    initConfig.sedpBuildHeartbeatPolicy =
        RTPSAckActionSedpBuildHeartbeatPolicy::PassLiveHeartbeat;
    initConfig.buildRosDiscInfoPayload = &acknackBuildRosDiscInfoPayload;
    initConfig.buildChatterPayload = &acknackBuildChatterPayload;
    initConfig.payloadCtx = nullptr;

    RTPSRunnerAdapter_initStandardAckActionCtx(ctx, initConfig);

    RTPSAckNackRunnerCallbackInitConfig callbackInit;
    callbackInit.logParsed = [](void*, const RTPSAckNackFields& fields, RTPSAckNackWriterKind writerKind)
    {
        LOG_I(MODULE_PREFIX, "  ACKNACK readerEID=%02X%02X%02X%02X writerEID=%02X%02X%02X%02X (%s) base=%u numBits=%u",
              fields.readerEID[0], fields.readerEID[1], fields.readerEID[2], fields.readerEID[3],
              fields.writerEID[0], fields.writerEID[1], fields.writerEID[2], fields.writerEID[3],
              writerKindToStr(writerKind),
              fields.bitmapBaseLow, fields.numBits);
    };
    callbackInit.unknownRemote = nullptr;
    callbackInit.getChatterSeq = &RTPSRunnerAdapter_standardGetChatterSeq;
    callbackInit.executeAction = &RTPSRunnerAdapter_standardExecuteAction;

    RTPSAckNackRunnerCallbacks callbacks;
    RTPSRunnerAdapter_initAckCallbacks(callbacks, callbackInit);

    const auto decisionOptions =
        RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::makeAckNackDecisionOptionsForFlavor(
            RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackRuntimeFlavor::LinuxStandalone);
    RTPSAckNackRunner_run(srcGuidPrefix, pContent, contentLen, decisionOptions, callbacks, &ctx);
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

    struct RxCtx
    {
        RTPSRxRunnerAdapterBaseCtx base;
    } rxCtx;

    rxCtx.base.localGuidPrefix = participant.getGuidPrefix();
    rxCtx.base.discovered = &discovered;
    rxCtx.base.ackSendSock = metatrafficSock;
    rxCtx.base.readerPolicy = RTPSRxAdapterReaderPolicy::BuiltinEndpointMap;
    rxCtx.base.ackDestPolicy = RTPSRxAdapterAckDestPolicy::ReplyToSender;

    RTPSRxSubmessageRunnerCallbacks callbacks;
    RTPSRunnerAdapter_applyRxBaseCallbacks(callbacks);
    callbacks.onHeartbeat = [](void*, RTPSRxChannel, const uint8_t* writerEID, uint32_t lastSNLow, bool responded, int sentBytes)
    {
        if (responded)
        {
            LOG_I(MODULE_PREFIX, "  HB writerEID=%02X%02X%02X%02X SN=%u -> ACKNACK %d bytes",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow, sentBytes);
        }
        else
        {
            LOG_I(MODULE_PREFIX, "  HB(final) writerEID=%02X%02X%02X%02X SN=%u",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow);
        }
    };
    callbacks.onData = [](void*,
                          RTPSRxChannel,
                          const uint8_t* packet,
                          uint32_t packetLen,
                          const uint8_t*,
                          const struct sockaddr_in& from,
                          const uint8_t* pContent,
                          uint32_t contentLen)
    {
        const uint8_t* writerEID = pContent + 8;
        if (memcmp(writerEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
        {
            LOG_I(MODULE_PREFIX, "  SPDP DATA on metatraffic");
            DiscoveredParticipant remote;
            if (spdpHandler.parseAnnouncementMessage(packet, packetLen, remote))
            {
                if (memcmp(remote.guidPrefix, participant.getGuidPrefix(), 12) != 0)
                    processDiscoveredParticipant(remote, from);
            }
        }
        else
        {
            LOG_I(MODULE_PREFIX, "  DATA writerEID=%02X%02X%02X%02X len=%d",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], (int)contentLen);
            if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0 ||
                memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
            {
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
    };
    callbacks.onAckNack = [](void*,
                             RTPSRxChannel,
                             const uint8_t* srcGuidPrefix,
                             const uint8_t* pContent,
                             uint32_t contentLen,
                             const struct sockaddr_in& from)
    {
        handleAcknack(srcGuidPrefix, pContent, contentLen, from);
    };
    callbacks.onOther = [](void*, RTPSRxChannel, RTPSSubmessageId submsgId, uint8_t flags, uint32_t contentLen)
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
    };

    RTPSRxSubmessageRunner_run(
        recvBuf, (uint32_t)n, fromAddr,
        RTPSRxChannel::Metatraffic,
        acknackCount,
        callbacks,
        &rxCtx);
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

    struct RxCtx
    {
        RTPSRxRunnerAdapterBaseCtx base;
    } rxCtx;

    rxCtx.base.localGuidPrefix = participant.getGuidPrefix();
    rxCtx.base.discovered = &discovered;
    rxCtx.base.ackSendSock = metatrafficSock;
    rxCtx.base.readerPolicy = RTPSRxAdapterReaderPolicy::UseHeartbeatReader;
    rxCtx.base.ackDestPolicy = RTPSRxAdapterAckDestPolicy::RouteToDiscoveredMetatraffic;

    RTPSRxSubmessageRunnerCallbacks callbacks;
    RTPSRunnerAdapter_applyRxBaseCallbacks(callbacks);
    callbacks.onHeartbeat = [](void*, RTPSRxChannel, const uint8_t* writerEID, uint32_t lastSNLow, bool responded, int sentBytes)
    {
        if (responded)
        {
            LOG_I(MODULE_PREFIX, "  UD HB writerEID=%02X%02X%02X%02X SN=%u -> ACKNACK %d",
                  writerEID[0], writerEID[1], writerEID[2], writerEID[3], lastSNLow, sentBytes);
        }
    };
    callbacks.onData = nullptr;
    callbacks.onAckNack = [](void*,
                             RTPSRxChannel,
                             const uint8_t* srcGuidPrefix,
                             const uint8_t* pContent,
                             uint32_t contentLen,
                             const struct sockaddr_in& from)
    {
        handleAcknack(srcGuidPrefix, pContent, contentLen, from);
    };
    callbacks.onOther = [](void*, RTPSRxChannel, RTPSSubmessageId submsgId, uint8_t flags, uint32_t contentLen)
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
    };

    RTPSRxSubmessageRunner_run(
        recvBuf, (uint32_t)n, fromAddr,
        RTPSRxChannel::UserData,
        acknackCount,
        callbacks,
        &rxCtx);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Purge stale participants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void purgeStaleParticipants()
{
    uint32_t now = millis();
    for (auto it = discovered.begin(); it != discovered.end(); )
    {
        if (RaftRuntime::RTPS::Runtime::DiscoveryRuntime::isLeaseExpired(
            now, it->discoveredTimeMs, it->leaseDurationSec))
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
    static bool dumpedRosDiscOnce = false;
    RTPSWriterHeartbeatCounterState counters = {
        sedpSeqNum,
        sedpSubSeqNum,
        sedpSeqNum + 1,
        livelinessSeqNum,
        rosDiscSeqNum,
        heartbeatCount,
        dumpedRosDiscOnce,
    };

    for (const auto& remote : discovered)
    {
        struct ExecCtx
        {
            const DiscoveredParticipant* remote = nullptr;
        } execCtx = { &remote };

        RTPSWriterHeartbeatRunnerCallbacks callbacks;
        callbacks.buildPayload = [](void* userCtx,
                                    RTPSWriterHeartbeatAction action,
                                    uint64_t sequenceNumber,
                                    uint32_t heartbeatCountCb) -> uint32_t
        {
            ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
            const DiscoveredParticipant& remoteRef = *ctx->remote;

            switch (action)
            {
                case RTPSWriterHeartbeatAction::SedpRosDiscoveryPublication:
                    return sedpHandler.buildPublicationMessage(
                        sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_WRITER, "ros_discovery_info",
                        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                        RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sequenceNumber, myIpAddr,
                        heartbeatCountCb);
                case RTPSWriterHeartbeatAction::SedpRosDiscoverySubscription:
                    return sedpHandler.buildSubscriptionMessage(
                        sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_READER, "ros_discovery_info",
                        "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
                        RELIABILITY_RELIABLE, DURABILITY_TRANSIENT_LOCAL, sequenceNumber, myIpAddr,
                        heartbeatCountCb);
                case RTPSWriterHeartbeatAction::SedpChatterPublication:
                    return sedpHandler.buildPublicationMessage(
                        sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                        ENTITYID_CHATTER_WRITER, CHATTER_DDS_TOPIC, CHATTER_DDS_TYPE,
                        RELIABILITY_RELIABLE, DURABILITY_VOLATILE, sequenceNumber, myIpAddr,
                        heartbeatCountCb);
                case RTPSWriterHeartbeatAction::ParticipantMessageData:
                    return sedpHandler.buildParticipantMessageData(
                        sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                        sequenceNumber, heartbeatCountCb);
                case RTPSWriterHeartbeatAction::RosDiscoveryInfoData:
                {
                    uint8_t rosDiscPayload[256];
                    uint32_t rosDiscLen = buildRosDiscInfoWithGids(
                        rosDiscPayload, sizeof(rosDiscPayload));
                    if (rosDiscLen == 0)
                        return 0;
                    return sedpHandler.buildUserDataMessage(
                        sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                        ENTITYID_ROS_DISC_INFO_WRITER, rosDiscPayload, rosDiscLen,
                        sequenceNumber, heartbeatCountCb);
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
            const DiscoveredParticipant& remoteRef = *ctx->remote;

            struct sockaddr_in dest = {};
            dest.sin_family = AF_INET;
            dest.sin_addr.s_addr = remoteRef.ipAddr;
            int sock = -1;
            if (sendTarget == RTPSWriterHeartbeatSendTarget::Metatraffic)
            {
                sock = metatrafficSock;
                dest.sin_port = htons(remoteRef.metatrafficPort);
            }
            else
            {
                sock = userDataSock;
                dest.sin_port = htons(remoteRef.userDataPort);
            }
            return sendto(sock, sendBuf, payloadLen, 0, (struct sockaddr*)&dest, sizeof(dest));
        };

        callbacks.debugPayload = [](void*, RTPSWriterHeartbeatAction action, uint32_t payloadLen)
        {
            if (action != RTPSWriterHeartbeatAction::RosDiscoveryInfoData)
                return;
            fprintf(stderr, "ROSDISC_CDR (%u bytes):", payloadLen);
            for (uint32_t i = 0; i < payloadLen; i++) fprintf(stderr, " %02x", sendBuf[i]);
            fprintf(stderr, "\n");
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
            LOG_I(MODULE_PREFIX, "sendWriterHB rosDisc %d/%d to port %d", sentBytes, (int)payloadLen, (int)remoteRef.userDataPort);
        };

        RTPSWriterHeartbeatRunner_run(
            RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle,
            counters,
            callbacks,
            &execCtx);
    }

    heartbeatCount = counters.heartbeatCount;
    livelinessSeqNum = counters.livelinessSeqNum;
    dumpedRosDiscOnce = counters.rosDiscDebugDumped;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle newly discovered participant
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr)
{
    const RTPSInitialAnnouncePlan announcePlan = RTPSInitialAnnouncePlan_default();
    const RTPSInitialAnnounceSequence announceSeq = RTPSInitialAnnouncePlan_buildSequence(
        announcePlan, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle);

    struct ExecCtx
    {
        const DiscoveredParticipant* remote = nullptr;
        const struct sockaddr_in* senderAddr = nullptr;
    } execCtx = { &remote, &senderAddr };

    RTPSInitialAnnounceRunnerCallbacks callbacks;

    callbacks.buildPayload = [](void* userCtx,
                                const RTPSInitialAnnounceStep& step,
                                uint64_t sequenceNumber,
                                uint32_t heartbeatCount,
                                uint32_t previousPayloadLen) -> uint32_t
    {
        ExecCtx* ctx = static_cast<ExecCtx*>(userCtx);
        const DiscoveredParticipant& remoteRef = *ctx->remote;

        const RTPSInitialAnnounceBuildSpec buildSpec = RTPSInitialAnnouncePlan_getBuildSpec(step.action);
        switch (buildSpec.buildKind)
        {
            case RTPSInitialAnnounceBuildKind::SpdpAnnouncement:
                return spdpHandler.buildAnnouncementMessage(
                    sendBuf, sizeof(sendBuf), participant, myIpAddr, LEASE_DURATION_SEC, sequenceNumber);
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
                    return sedpHandler.buildPublicationMessage(
                        sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                        sedpSpec.entityId, sedpSpec.topicName, sedpSpec.typeName,
                        sedpSpec.reliabilityKind, sedpSpec.durabilityKind,
                        sequenceNumber, myIpAddr, heartbeatCount);
                }
                return sedpHandler.buildSubscriptionMessage(
                    sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
                    sedpSpec.entityId, sedpSpec.topicName, sedpSpec.typeName,
                    sedpSpec.reliabilityKind, sedpSpec.durabilityKind,
                    sequenceNumber, myIpAddr, heartbeatCount);
            }
            case RTPSInitialAnnounceBuildKind::ParticipantMessageData:
                return sedpHandler.buildParticipantMessageData(
                    sendBuf, sizeof(sendBuf), participant, remoteRef.guidPrefix,
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
        const DiscoveredParticipant& remoteRef = *ctx->remote;
        const struct sockaddr_in& senderAddrRef = *ctx->senderAddr;

        if (payloadLen == 0)
            return -1;
        const RTPSInitialAnnounceSendTarget target = RTPSInitialAnnouncePlan_getSendTarget(action);

        int sock = -1;
        switch (target.socket)
        {
            case RTPSInitialAnnounceSocket::Spdp:
                sock = spdpSock;
                break;
            case RTPSInitialAnnounceSocket::Metatraffic:
                sock = metatrafficSock;
                break;
            case RTPSInitialAnnounceSocket::UserData:
                sock = userDataSock;
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
                dest.sin_port = htons(participant.getSPDPMulticastPort());
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

        return sendto(sock, sendBuf, payloadLen, 0, (struct sockaddr*)&dest, sizeof(dest));
    };

    callbacks.debugPayload = [](void*, const RTPSInitialAnnounceStep&, uint32_t payloadLen)
    {
        // Hex dump SEDP publication for debugging
        fprintf(stderr, "SEDP_PUB_HEX (%d bytes):", (int)payloadLen);
        for (uint32_t i = 0; i < payloadLen; i++)
            fprintf(stderr, " %02x", sendBuf[i]);
        fprintf(stderr, "\n");
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
            LOG_I(MODULE_PREFIX, "handleNewParticipant %s%s %d/%d to %s:%d",
                  statusPrefix,
                  logSpec.actionLabel,
                  sentBytes, (int)payloadLen,
                  inet_ntoa(((struct sockaddr_in&)senderAddrRef).sin_addr),
                  (int)ntohs(senderAddrRef.sin_port));
        }
        else
        {
            LOG_I(MODULE_PREFIX, "handleNewParticipant %s%s %d/%d to port %d",
                  statusPrefix,
                  logSpec.actionLabel,
                  sentBytes, (int)payloadLen, (int)remoteRef.metatrafficPort);
        }
    };

    RTPSInitialAnnounceRunnerContext runCtx;
    runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::LinuxStyle;
    runCtx.seqCounters = {
        spdpSeqNum,
        sedpSeqNum,
        sedpSubSeqNum,
        sedpSeqNum + 1,
        livelinessSeqNum,
        rosDiscSeqNum,
    };
    runCtx.heartbeatCount = heartbeatCount;
    runCtx.previousPayloadLen = 0;

    RTPSInitialAnnounceRunner_run(announceSeq, runCtx, callbacks, &execCtx);

    spdpSeqNum = runCtx.seqCounters.spdpSeqNum;
    heartbeatCount = runCtx.heartbeatCount;
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
        if (RTPSRuntimeSchedule_isPeriodicDue(now, lastSpdpMs, SPDP_INTERVAL_MS, false))
        {
            sendSPDP();
            lastSpdpMs = now;
        }

        // Periodic heartbeats (once active)
        if (active && RTPSRuntimeSchedule_isPeriodicDue(now, lastHbMs, WRITER_HB_INTERVAL_MS, true))
        {
            sendWriterHeartbeats();
            lastHbMs = now;
        }

        // Periodic chatter publishing (once active)
        if (active && RTPSRuntimeSchedule_isPeriodicDue(now, lastChatterSendMs, CHATTER_PUBLISH_INTERVAL_MS, true))
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
        RTPSParticipantSetPolicyResult setPolicy =
            RaftRuntime::RTPS::Runtime::DiscoveryRuntime::applyParticipantSetPolicy(
                active,
                (uint32_t)discovered.size(),
                false,
                false);
        active = setPolicy.shouldBeActive;
        if (setPolicy.triggerImmediateWriterHeartbeat)
            RaftRuntime::RTPS::Runtime::DiscoveryRuntime::onActivated(lastHbMs);
    }

    printf("\nShutting down...\n");
    close(spdpSock);
    close(metatrafficSock);
    close(userDataSock);
    return 0;
}
