/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Linux unit tests for RaftROS
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <type_traits>
#include <vector>
#include <string>
#include <arpa/inet.h>
#include "utils.h"
#include "CDREncoder.h"
#include "CDRDecoder.h"
#include "runtime/wire/RTPSMessage.h"
#include "RTPSTypes.h"
#include "runtime/core/RTPSParticipant.h"
#include "runtime/discovery/SPDPHandler.h"
#include "runtime/announce/SEDPHandler.h"
#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"
#include "runtime/reliability/RTPSReaderRuntime.h"
#include "runtime/reliability/RTPSReaderRunner.h"
#include "runtime/reliability/RTPSReaderStateMap.h"
#include "runtime/receive/RTPSRxSubmessageRunner.h"
#include "runtime/announce/RTPSInitialAnnouncePlan.h"
#include "runtime/announce/RTPSInitialAnnounceRunner.h"
#include "runtime/announce/RTPSWriterHeartbeatRunner.h"
#include "runtime/dispatch/RTPSUserDispatch.h"
#include "runtime/dispatch/RTPSSubscriptionRegistry.h"
#include "runtime/dispatch/RTPSRemotePublicationMap.h"
#include "runtime/dispatch/RTPSSEDPPublicationParser.h"
#include "runtime/autopub/RTPSDynamicWriterRegistry.h"
#include "runtime/autopub/RTPSAutoPubLifecycle.h"
#include "runtime/autopub/RTPSAutoPubQoSProfile.h"
#include "runtime/autopub/RTPSAutoPubTopicNaming.h"
#include "runtime/autopub/RTPSAutoPubClassMap.h"
#include "runtime/autopub/RTPSAutoPubCDRSerializer.h"
#include "runtime/autopub/RTPSAutoPubBackend.h"
#include "AutoPub/AutoPubServiceRegistry.h"
#include "AutoPub/AutoPubParamCodec.h"
#include "AutoPub/AutoPubParameterStore.h"
#include <fstream>
#include "AutoPub/AutoPubSampleRunner.h"
#include "runtime/autopub/RTPSAutoPubSampleEmitter.h"
#include "AutoPub/AutoPubPublisherPool.h"
#include "AutoPub/AutoPubAttachPlan.h"
#include "autopub_pool_test_lock.h"

#define TEST_ASSERT(cond, msg) if (!(cond)) { printf("  FAIL: %s\n", msg); failCount++; } else { passCount++; }

bool isApprox(float a, float b, float tol = 0.0001f)
{
    return std::fabs(a - b) < tol;
}

int main()
{
    int failCount = 0;
    int passCount = 0;

    //=================================================================
    // CDR Encoder/Decoder round-trip
    //=================================================================
    {
        printf("Test: CDR Encoder/Decoder round-trip\n");

        uint8_t encBuf[256];
        CDREncoder encoder;
        encoder.reset(encBuf, sizeof(encBuf));
        encoder.writeEncapsulationHeader();
        encoder.writeBool(true);
        encoder.writeUint8(42);
        encoder.writeUint16(1234);
        encoder.writeUint32(56789);
        encoder.writeInt32(-12345);
        encoder.writeFloat32(3.14159f);
        encoder.writeString("hello");
        encoder.writeUint64(0x123456789ABCDEF0ULL);
        encoder.writeFloat64(2.718281828);

        CDRDecoder decoder;
        decoder.init(encBuf, encoder.getPos());
        decoder.readEncapsulationHeader();

        bool boolVal;
        TEST_ASSERT(decoder.readBool(boolVal) && boolVal == true, "CDR bool round-trip");

        uint8_t u8Val;
        TEST_ASSERT(decoder.readUint8(u8Val) && u8Val == 42, "CDR uint8 round-trip");

        uint16_t u16Val;
        TEST_ASSERT(decoder.readUint16(u16Val) && u16Val == 1234, "CDR uint16 round-trip");

        uint32_t u32Val;
        TEST_ASSERT(decoder.readUint32(u32Val) && u32Val == 56789, "CDR uint32 round-trip");

        int32_t i32Val;
        TEST_ASSERT(decoder.readInt32(i32Val) && i32Val == -12345, "CDR int32 round-trip");

        float f32Val;
        TEST_ASSERT(decoder.readFloat32(f32Val) && isApprox(f32Val, 3.14159f), "CDR float32 round-trip");

        char strBuf[64];
        uint32_t strLen;
        TEST_ASSERT(decoder.readString(strBuf, sizeof(strBuf), strLen) && strcmp(strBuf, "hello") == 0, "CDR string round-trip");

        uint64_t u64Val;
        TEST_ASSERT(decoder.readUint64(u64Val) && u64Val == 0x123456789ABCDEF0ULL, "CDR uint64 round-trip");

        double f64Val;
        TEST_ASSERT(decoder.readFloat64(f64Val) && std::fabs(f64Val - 2.718281828) < 1e-9, "CDR float64 round-trip");
    }

    // CDR alignment
    {
        printf("Test: CDR alignment\n");

        uint8_t encBuf[64];
        CDREncoder encoder;
        encoder.reset(encBuf, sizeof(encBuf));
        encoder.writeUint8(0x01);
        encoder.writeUint32(0x12345678);
        TEST_ASSERT(encoder.getPos() == 8, "CDR alignment: uint8 then uint32 = 8 bytes");
    }

    //=================================================================
    // RTPS Message Header
    //=================================================================
    {
        printf("Test: RTPS message header\n");

        uint8_t buf[32];
        uint8_t guidPrefix[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
        uint32_t n = RTPSMessage::writeHeader(buf, sizeof(buf), guidPrefix);
        TEST_ASSERT(n == 20, "RTPS header size = 20");
        TEST_ASSERT(buf[0]=='R' && buf[1]=='T' && buf[2]=='P' && buf[3]=='S', "RTPS header magic");
        TEST_ASSERT(buf[4]==2 && buf[5]==2, "RTPS header version 2.2");
        TEST_ASSERT(memcmp(buf+8, guidPrefix, 12)==0, "RTPS header guidPrefix");

        uint8_t parsedGuid[12];
        uint32_t off = RTPSMessage::parseHeader(buf, n, parsedGuid);
        TEST_ASSERT(off == 20, "RTPS parseHeader returns 20");
        TEST_ASSERT(memcmp(guidPrefix, parsedGuid, 12)==0, "RTPS parseHeader guidPrefix match");
    }

    //=================================================================
    // RTPS DATA Submessage
    //=================================================================
    {
        printf("Test: RTPS DATA submessage\n");

        uint8_t buf[128];
        uint8_t payload[] = {0xAA, 0xBB, 0xCC, 0xDD};
        uint8_t reader[4] = {0,0,0,0};
        uint8_t writer[4] = {0,1,0,0xC2};
        uint32_t n = RTPSMessage::writeDataSubmessage(buf, sizeof(buf),
            reader, writer, 0, 1, payload, 4);

        // Submessage header
        TEST_ASSERT(buf[0] == 0x15, "DATA submsgId = 0x15");
        TEST_ASSERT(buf[1] == 0x05, "DATA flags = 0x05 (LE|D)");

        // Content size = 2+2+4+4+8+4 = 24
        uint16_t octets = buf[2] | (buf[3]<<8);
        TEST_ASSERT(octets == 24, "DATA octetsToNextHeader = 24");

        // Total = 4 + 24 = 28
        TEST_ASSERT(n == 28, "DATA total size = 28");

        // Verify reader/writer entity IDs at correct offsets
        // Content starts at offset 4: extraFlags(2) + octetsToInlineQoS(2) = 8, then readerId at offset 8
        TEST_ASSERT(memcmp(buf+8, reader, 4)==0, "DATA reader entity ID");
        TEST_ASSERT(memcmp(buf+12, writer, 4)==0, "DATA writer entity ID");

        // Sequence number at offset 16: high=0, low=1
        uint32_t seqHigh = buf[16] | (buf[17]<<8) | (buf[18]<<16) | (buf[19]<<24);
        uint32_t seqLow  = buf[20] | (buf[21]<<8) | (buf[22]<<16) | (buf[23]<<24);
        TEST_ASSERT(seqHigh == 0 && seqLow == 1, "DATA sequence number 0:1");

        // Payload at offset 24
        TEST_ASSERT(memcmp(buf+24, payload, 4)==0, "DATA payload bytes");
    }

    //=================================================================
    // RTPS INFO_TS Submessage
    //=================================================================
    {
        printf("Test: RTPS INFO_TS submessage\n");

        uint8_t buf[16];
        uint32_t n = RTPSMessage::writeInfoTS(buf, sizeof(buf), 12345, 0);
        TEST_ASSERT(n == 12, "INFO_TS size = 12");
        TEST_ASSERT(buf[0] == 0x09, "INFO_TS submsgId");
        TEST_ASSERT(buf[1] == 0x01, "INFO_TS flags LE");
    }

    //=================================================================
    // RTPS INFO_DST Submessage
    //=================================================================
    {
        printf("Test: RTPS INFO_DST submessage\n");

        uint8_t buf[20];
        uint8_t gp[12] = {10,20,30,40,50,60,70,80,90,100,110,120};
        uint32_t n = RTPSMessage::writeInfoDST(buf, sizeof(buf), gp);
        TEST_ASSERT(n == 16, "INFO_DST size = 16");
        TEST_ASSERT(buf[0] == 0x0E, "INFO_DST submsgId");
        TEST_ASSERT(memcmp(buf+4, gp, 12)==0, "INFO_DST guidPrefix");
    }

    //=================================================================
    // RTPS HEARTBEAT Submessage
    //=================================================================
    {
        printf("Test: RTPS HEARTBEAT submessage\n");

        uint8_t buf[40];
        uint8_t reader[4] = {0,0,0,0};
        uint8_t writer[4] = {0,0,1,2};
        uint32_t n = RTPSMessage::writeHeartbeat(buf, sizeof(buf),
            reader, writer, 0, 1, 0, 5, 3);
        TEST_ASSERT(n == 32, "HEARTBEAT size = 32");
        TEST_ASSERT(buf[0] == 0x07, "HEARTBEAT submsgId");
        TEST_ASSERT((buf[1] & 0x02) == 0, "HEARTBEAT FinalFlag not set (expects ACKNACK)");
    }

    //=================================================================
    // Submessage Parsing
    //=================================================================
    {
        printf("Test: Submessage parsing\n");

        uint8_t buf[64];
        uint8_t gp[12] = {};
        uint32_t pos = RTPSMessage::writeHeader(buf, sizeof(buf), gp);
        pos += RTPSMessage::writeInfoTS(buf+pos, sizeof(buf)-pos, 100, 0);
        uint8_t payload[] = {1,2,3,4};
        pos += RTPSMessage::writeDataSubmessage(buf+pos, sizeof(buf)-pos,
            ENTITYID_UNKNOWN, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
            0, 1, payload, 4);

        // Parse header
        uint8_t parsedGP[12];
        uint32_t off = RTPSMessage::parseHeader(buf, pos, parsedGP);
        TEST_ASSERT(off == 20, "Parse: header offset 20");

        // Parse first submessage (INFO_TS)
        RTPSSubmessageId subId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t subSz = RTPSMessage::parseSubmessage(buf+off, pos-off,
            subId, flags, pContent, contentLen);
        TEST_ASSERT(subSz > 0, "Parse: INFO_TS submessage found");
        TEST_ASSERT(subId == SUBMSG_INFO_TS, "Parse: submsgId is INFO_TS");
        off += subSz;

        // Parse second submessage (DATA)
        subSz = RTPSMessage::parseSubmessage(buf+off, pos-off,
            subId, flags, pContent, contentLen);
        TEST_ASSERT(subSz > 0, "Parse: DATA submessage found");
        TEST_ASSERT(subId == SUBMSG_DATA, "Parse: submsgId is DATA");
    }

    //=================================================================
    // Port Calculation
    //=================================================================
    {
        printf("Test: Port calculation\n");

        TEST_ASSERT(rtpsDiscoveryMulticastPort(0) == 7400, "Domain 0 SPDP multicast port = 7400");
        TEST_ASSERT(rtpsDiscoveryUnicastPort(0, 0) == 7410, "Domain 0 participant 0 metatraffic = 7410");
        TEST_ASSERT(rtpsUserUnicastPort(0, 0) == 7411, "Domain 0 participant 0 user = 7411");
        TEST_ASSERT(rtpsDiscoveryMulticastPort(1) == 7650, "Domain 1 SPDP multicast port = 7650");
        TEST_ASSERT(rtpsDiscoveryUnicastPort(0, 1) == 7412, "Domain 0 participant 1 metatraffic = 7412");
    }

    //=================================================================
    // RTPSParticipant - GUID from MAC
    //=================================================================
    {
        printf("Test: RTPSParticipant GUID from MAC\n");

        RTPSParticipant part;
        part.init(0, "test_node", 0, nullptr);
        uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
        part.setGuidPrefixFromMAC(mac);

        const uint8_t* gp = part.getGuidPrefix();
        TEST_ASSERT(gp[0]==0x01 && gp[1]==0x0F, "GUID prefix vendor bytes");
        TEST_ASSERT(memcmp(gp+2, mac, 6)==0, "GUID prefix contains MAC");
        TEST_ASSERT(gp[8]==0 && gp[9]==0 && gp[10]==0 && gp[11]==0,
                    "GUID prefix participant discriminator = 0");

        const uint8_t* guid = part.getParticipantGuid();
        TEST_ASSERT(memcmp(guid+12, ENTITYID_PARTICIPANT, 4)==0,
                    "Participant GUID ends with ENTITYID_PARTICIPANT");

        TEST_ASSERT(part.getSPDPMulticastPort() == 7400, "Participant SPDP port = 7400");
        TEST_ASSERT(part.getMetatrafficUnicastPort() == 7410, "Participant metatraffic port = 7410");
        TEST_ASSERT(part.getUserUnicastPort() == 7411, "Participant user port = 7411");
    }

    //=================================================================
    // RTPSParticipant - Locator builder
    //=================================================================
    {
        printf("Test: Locator UDPv4 builder\n");

        uint8_t loc[24];
        // IP 192.168.1.100 in network byte order = 0xC0A80164
        uint32_t ip = 0x6401A8C0; // network byte order for 192.168.1.100 on LE machine
        RTPSParticipant::buildLocatorUDPv4(loc, ip, 7410);

        TEST_ASSERT(loc[0]==1 && loc[1]==0 && loc[2]==0 && loc[3]==0, "Locator kind = UDPv4");
        // Port 7410 LE: 0xF2 0x1C
        TEST_ASSERT(loc[4]==(7410&0xFF) && loc[5]==((7410>>8)&0xFF), "Locator port LE");
        // IP at offset 20
        TEST_ASSERT(memcmp(loc+20, &ip, 4)==0, "Locator IPv4 address at offset 20");
    }

    //=================================================================
    // SPDP Announcement Message Build + Parse Round-Trip
    //=================================================================
    {
        printf("Test: SPDP announcement build + parse\n");

        RTPSParticipant part;
        uint8_t mac[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
        part.init(0, "spdp_test", 0, nullptr);
        part.setGuidPrefixFromMAC(mac);

        // IP 10.0.0.50 in network byte order
        uint32_t ip = htonl(0x0A000032);

        SPDPHandler spdp;
        uint8_t msgBuf[512];
        uint32_t msgLen = spdp.buildAnnouncementMessage(
            msgBuf, sizeof(msgBuf), part, ip, 120, 1);

        TEST_ASSERT(msgLen > 0, "SPDP build returns non-zero length");
        TEST_ASSERT(msgLen < 300, "SPDP message under 300 bytes");

        // Verify RTPS header
        TEST_ASSERT(msgBuf[0]=='R' && msgBuf[1]=='T' && msgBuf[2]=='P' && msgBuf[3]=='S',
                    "SPDP msg has RTPS magic");
        TEST_ASSERT(memcmp(msgBuf+8, part.getGuidPrefix(), 12)==0,
                    "SPDP msg guidPrefix matches");

        // Parse it back
        DiscoveredParticipant parsed;
        bool ok = spdp.parseAnnouncementMessage(msgBuf, msgLen, parsed);
        TEST_ASSERT(ok, "SPDP parse succeeds");
        TEST_ASSERT(parsed.valid, "SPDP parsed participant is valid");
        TEST_ASSERT(memcmp(parsed.guidPrefix, part.getGuidPrefix(), 12)==0,
                    "SPDP parsed guidPrefix matches");
        TEST_ASSERT(parsed.metatrafficPort == 7410, "SPDP parsed metatraffic port = 7410");
        TEST_ASSERT(parsed.userDataPort == 7411, "SPDP parsed user data port = 7411");
        TEST_ASSERT(parsed.leaseDurationSec == 120, "SPDP parsed lease duration = 120");
        TEST_ASSERT(parsed.ipAddr == ip, "SPDP parsed IP address matches");

        // Participants commonly announce non-UDPv4 locators too (e.g. FastDDS announces a
        // shared-memory locator alongside UDPv4) - those must not supply ports/addresses
        uint8_t shmBuf[512];
        memcpy(shmBuf, msgBuf, msgLen);
        uint32_t locatorParamsPatched = 0;
        for (uint32_t i = 0; i + 4 <= msgLen; i++)
        {
            const uint16_t pid = shmBuf[i] | (shmBuf[i + 1] << 8);
            const uint16_t plen = shmBuf[i + 2] | (shmBuf[i + 3] << 8);
            if (((pid == PID_DEFAULT_UNICAST_LOCATOR) || (pid == PID_METATRAFFIC_UNICAST_LOCATOR)) &&
                (plen == 24) && (i + 4 + plen <= msgLen) && (shmBuf[i + 4] == LOCATOR_KIND_UDPv4))
            {
                shmBuf[i + 4] = 16;  // LOCATOR_KIND_SHM
                locatorParamsPatched++;
            }
        }
        DiscoveredParticipant shmParsed;
        const bool shmOk = spdp.parseAnnouncementMessage(shmBuf, msgLen, shmParsed);
        TEST_ASSERT(locatorParamsPatched == 2, "SPDP test patched both locator params to SHM");
        TEST_ASSERT(shmOk && shmParsed.valid, "SPDP parse succeeds with non-UDPv4 locators");
        TEST_ASSERT(memcmp(shmParsed.guidPrefix, part.getGuidPrefix(), 12)==0,
                    "SPDP non-UDPv4 locators still yield guidPrefix");
        TEST_ASSERT(shmParsed.metatrafficPort == 0 && shmParsed.userDataPort == 0 &&
                        shmParsed.ipAddr == 0,
                    "SPDP ignores ports/address from non-UDPv4 locators");
    }

    //=================================================================
    // SEDP Publication Message Build
    //=================================================================
    {
        printf("Test: SEDP publication message build\n");

        RTPSParticipant part;
        uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
        part.init(0, "sedp_test", 0, nullptr);
        part.setGuidPrefixFromMAC(mac);

        uint8_t destGP[12] = {10,20,30,40,50,60,70,80,90,100,110,120};

        SEDPHandler sedp;
        uint8_t msgBuf[1024];
        uint32_t msgLen = sedp.buildPublicationMessage(
            msgBuf, sizeof(msgBuf),
            part, destGP,
            ENTITYID_ROS_DISC_INFO_WRITER,
            "ros_discovery_info",
            "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
            RELIABILITY_RELIABLE,
            DURABILITY_TRANSIENT_LOCAL,
            1);

        TEST_ASSERT(msgLen > 0, "SEDP pub build returns non-zero");
        TEST_ASSERT(msgLen < 1024, "SEDP pub message fits in buffer");

        // Verify RTPS header
        TEST_ASSERT(msgBuf[0]=='R' && msgBuf[1]=='T' && msgBuf[2]=='P' && msgBuf[3]=='S',
                    "SEDP msg has RTPS magic");

        // Should contain INFO_DST, INFO_TS, DATA, HEARTBEAT submessages
        uint8_t parsedGP[12];
        uint32_t off = RTPSMessage::parseHeader(msgBuf, msgLen, parsedGP);
        TEST_ASSERT(off == 20, "SEDP parse header ok");

        // Count submessages
        int submsgCount = 0;
        bool foundData = false;
        bool foundHeartbeat = false;
        bool foundInfoDst = false;
        while (off < msgLen)
        {
            RTPSSubmessageId subId;
            uint8_t flags;
            const uint8_t* pContent;
            uint32_t contentLen;
            uint32_t sz = RTPSMessage::parseSubmessage(msgBuf+off, msgLen-off,
                subId, flags, pContent, contentLen);
            if (sz == 0) break;
            submsgCount++;
            if (subId == SUBMSG_DATA) foundData = true;
            if (subId == SUBMSG_HEARTBEAT) foundHeartbeat = true;
            if (subId == SUBMSG_INFO_DST) foundInfoDst = true;
            off += sz;
        }
        TEST_ASSERT(submsgCount >= 3, "SEDP msg has >= 3 submessages");
        TEST_ASSERT(foundData, "SEDP msg contains DATA");
        TEST_ASSERT(foundHeartbeat, "SEDP msg contains HEARTBEAT");
        TEST_ASSERT(foundInfoDst, "SEDP msg contains INFO_DST");
    }

    //=================================================================
    // ros_discovery_info CDR Payload
    //=================================================================
    {
        printf("Test: ros_discovery_info CDR payload\n");

        uint8_t guid[16] = {1,2,3,4,5,6,7,8,9,10,11,12, 0,0,1,0xC1};
        uint8_t buf[256];
        uint32_t len = SPDPHandler::buildRosDiscoveryInfoPayload(
            buf, sizeof(buf), guid, "my_node", "/my_ns");

        TEST_ASSERT(len > 0, "ros_disc_info payload non-zero");

        // CDR encapsulation header
        TEST_ASSERT(buf[0]==0x00 && buf[1]==0x01, "ros_disc_info CDR_LE header");

        // Gid at offset 4: 16 bytes (per rmw_dds_common::msg::Gid `char[16] data`)
        TEST_ASSERT(memcmp(buf+4, guid, 16)==0, "ros_disc_info gid matches guid");

        // Sequence length at offset 20 = 1
        uint32_t seqLen = buf[20] | (buf[21]<<8) | (buf[22]<<16) | (buf[23]<<24);
        TEST_ASSERT(seqLen == 1, "ros_disc_info seq length = 1");

        // node_namespace string at offset 24 (first in CDR order): length=7 ("/my_ns\0")
        uint32_t nsLen = buf[24] | (buf[25]<<8) | (buf[26]<<16) | (buf[27]<<24);
        TEST_ASSERT(nsLen == 7, "ros_disc_info node_namespace length = 7");
        TEST_ASSERT(memcmp(buf+28, "/my_ns", 6)==0, "ros_disc_info node_namespace content");

        // node_name string follows at offset 24+4+8(padded)=36: length=8 ("my_node\0")
        uint32_t nameOff = 24 + 4 + ((nsLen + 3) & ~3u);
        uint32_t nameLen = buf[nameOff] | (buf[nameOff+1]<<8) | (buf[nameOff+2]<<16) | (buf[nameOff+3]<<24);
        TEST_ASSERT(nameLen == 8, "ros_disc_info node_name length = 8");
        TEST_ASSERT(memcmp(buf+nameOff+4, "my_node", 7)==0, "ros_disc_info node_name content");
    }

    //=================================================================
    // User DATA Message Build (ros_discovery_info delivery)
    //=================================================================
    {
        printf("Test: User DATA message build\n");

        RTPSParticipant part;
        part.init(0, "data_test", 0, nullptr);
        uint8_t mac[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
        part.setGuidPrefixFromMAC(mac);

        uint8_t destGP[12] = {0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB};
        uint8_t payload[32] = {1,2,3,4,5,6,7,8};

        SEDPHandler sedp;
        uint8_t msgBuf[512];
        uint32_t msgLen = sedp.buildUserDataMessage(
            msgBuf, sizeof(msgBuf),
            part, destGP,
            ENTITYID_ROS_DISC_INFO_WRITER,
            payload, 8, 1, 1);

        TEST_ASSERT(msgLen > 0, "User DATA build returns non-zero");
        TEST_ASSERT(msgBuf[0]=='R' && msgBuf[1]=='T' && msgBuf[2]=='P' && msgBuf[3]=='S',
                    "User DATA msg has RTPS magic");

        // Parse and verify submessage structure
        uint8_t gp[12];
        uint32_t off = RTPSMessage::parseHeader(msgBuf, msgLen, gp);
        int foundHB = 0, foundDATA = 0;
        while (off < msgLen) {
            RTPSSubmessageId id; uint8_t fl; const uint8_t* pc; uint32_t cl;
            uint32_t sz = RTPSMessage::parseSubmessage(msgBuf+off, msgLen-off, id, fl, pc, cl);
            if (sz == 0) break;
            if (id == SUBMSG_DATA) foundDATA++;
            if (id == SUBMSG_HEARTBEAT) foundHB++;
            off += sz;
        }
        TEST_ASSERT(foundDATA == 1, "User DATA msg has 1 DATA submessage");
        TEST_ASSERT(foundHB == 1, "User DATA msg has 1 HEARTBEAT submessage");
    }

    //=================================================================
    // Initial announce: no step may reuse the send buffer's previous contents.
    // The sequence drains one step per loop pass and other senders share the
    // buffer in between, so a ReusePrevious step sent the previous step's
    // length over whatever was there by then - a truncated SEDP announcement.
    //=================================================================
    {
        printf("Test: initial-announce steps never reuse the previous buffer contents\n");
        RTPSInitialAnnouncePlan plan;
        bool anyReuse = false;
        for (auto flavor : {RTPSInitialAnnounceRuntimeFlavor::EspStyle, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle})
        {
            RTPSInitialAnnounceSequence seq = RTPSInitialAnnouncePlan_buildSequence(plan, flavor);
            for (uint8_t i = 0; i < seq.numSteps; i++)
            {
                const auto spec = RTPSInitialAnnouncePlan_getBuildSpec(seq.steps[i].action);
                if (spec.buildKind == RTPSInitialAnnounceBuildKind::ReusePrevious)
                    anyReuse = true;
            }
        }
        TEST_ASSERT(!anyReuse, "no initial-announce step relies on the send buffer being untouched between passes");
        TEST_ASSERT(RTPSInitialAnnouncePlan_getBuildSpec(RTPSInitialAnnounceAction::SpdpDiscoveryPortCopy).buildKind
                        == RTPSInitialAnnounceBuildKind::SpdpAnnouncement,
                    "the discovery-port copy of the SPDP reply is built afresh");
    }

    //=================================================================
    // Services: codec for the std_srvs types, and the registry that holds a
    // request from acceptance to the final without ever blocking the loop.
    //=================================================================
    {
        printf("Test: parameter codec against the ros2 param capture\n");
        {
            using namespace RaftRuntime::AutoPub;
            auto fixture = [](const char* name) {
                std::ifstream in(std::string("fixtures/") + name);
                std::string line, hex;
                while (std::getline(in, line)) if (!line.empty() && line[0] != '#') hex = line;
                std::vector<uint8_t> bytes;
                for (size_t i = 0; i + 1 < hex.size(); i += 2)
                    bytes.push_back((uint8_t)std::stoul(hex.substr(i, 2), nullptr, 16));
                return bytes;
            };
            uint8_t out[1024];
            char names[8][AUTOPUB_PARAM_NAME_MAX];
            uint32_t count = 0;
            uint64_t depth = 9;
            // FastCDR (what rclpy serialises with) leaves alignment padding
            // uninitialised, so a multi-string capture differs from ours only
            // where we wrote a padding zero.  Equal means: same length, and
            // every differing byte is one of our zeros - few of them.
            auto sameButPadding = [](const uint8_t* ours, const std::vector<uint8_t>& captured, uint32_t n) {
                if (n != captured.size()) return false;
                uint32_t differing = 0;
                for (uint32_t i = 0; i < n; ++i)
                    if (ours[i] != captured[i]) { if (ours[i] != 0) return false; ++differing; }
                return differing <= n / 16;
            };

            // ros2 param list: ListParameters {prefixes: [], depth: 0}
            auto req = fixture("zenoh_param_list_parameters_req_0.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readListRequest(req.data(), req.size(), names, 8, count, depth) &&
                        count == 0 && depth == 0, "list request: no prefixes, depth 0");
            TEST_ASSERT(!AutoPubParamCodec_readListRequest(req.data(), req.size() - 1, names, 8, count, depth),
                        "list request: truncated depth is rejected");
            const char* listNames[] = {"use_sim_time", "start_type_description_service", "chatterEnable",
                                       "chatterPeriodMs", "rangeScale", "routerHost"};
            auto resp = fixture("zenoh_param_list_parameters_resp_0.cdr.hex");
            uint32_t n = AutoPubParamCodec_writeListResult(out, sizeof(out), listNames, 6, nullptr, 0);
            TEST_ASSERT(sameButPadding(out, resp, n) && n == 140,
                        "list response: six names, no prefixes, byte for byte but padding");

            // ros2 param get: GetParameters {names: [x]} -> ParameterValue[]
            req = fixture("zenoh_param_get_parameters_req_0.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readNames(req.data(), req.size(), names, 8, count) && count == 1 &&
                        std::strcmp(names[0], "chatterEnable") == 0, "get request: one name");
            AutoPubParamValue v;
            v.type = AutoPubParamType::Bool; v.boolValue = true;
            resp = fixture("zenoh_param_get_parameters_resp_0.cdr.hex");
            n = AutoPubParamCodec_writeValues(out, sizeof(out), &v, 1);
            TEST_ASSERT(n == resp.size() && n == 56 && std::memcmp(out, resp.data(), n) == 0,
                        "get response: bool true is 52 body bytes, byte for byte");
            v = AutoPubParamValue(); v.type = AutoPubParamType::Integer; v.integerValue = 1000;
            resp = fixture("zenoh_param_get_parameters_resp_1.cdr.hex");
            n = AutoPubParamCodec_writeValues(out, sizeof(out), &v, 1);
            TEST_ASSERT(n == resp.size() && std::memcmp(out, resp.data(), n) == 0, "get response: int64 1000");
            v = AutoPubParamValue(); v.type = AutoPubParamType::Double; v.doubleValue = 1.5;
            resp = fixture("zenoh_param_get_parameters_resp_2.cdr.hex");
            n = AutoPubParamCodec_writeValues(out, sizeof(out), &v, 1);
            TEST_ASSERT(n == resp.size() && std::memcmp(out, resp.data(), n) == 0, "get response: double 1.5");
            v = AutoPubParamValue(); v.type = AutoPubParamType::String; std::strcpy(v.stringValue, "192.168.86.192");
            resp = fixture("zenoh_param_get_parameters_resp_3.cdr.hex");
            n = AutoPubParamCodec_writeValues(out, sizeof(out), &v, 1);
            TEST_ASSERT(n == resp.size() && std::memcmp(out, resp.data(), n) == 0, "get response: string");

            // ros2 param dump: one GetParameters with every name, sorted
            req = fixture("zenoh_param_get_parameters_req_4.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readNames(req.data(), req.size(), names, 8, count) && count == 6 &&
                        std::strcmp(names[0], "chatterEnable") == 0 && std::strcmp(names[5], "use_sim_time") == 0,
                        "dump request: six sorted names");
            TEST_ASSERT(AutoPubParamCodec_readNames(req.data(), req.size(), names, 2, count) && count == 6 &&
                        std::strcmp(names[1], "chatterPeriodMs") == 0, "names past the caller's slots are counted, not stored");
            AutoPubParamValue six[6];
            six[0].type = AutoPubParamType::Bool;
            six[1].type = AutoPubParamType::Integer; six[1].integerValue = 500;
            six[2].type = AutoPubParamType::Double; six[2].doubleValue = 2.25;
            six[3].type = AutoPubParamType::String; std::strcpy(six[3].stringValue, "192.168.86.192");
            six[4].type = AutoPubParamType::Bool; six[4].boolValue = true;
            six[5].type = AutoPubParamType::Bool;
            resp = fixture("zenoh_param_get_parameters_resp_4.cdr.hex");
            n = AutoPubParamCodec_writeValues(out, sizeof(out), six, 6);
            TEST_ASSERT(sameButPadding(out, resp, n) && n == 312,
                        "dump response: six values, 312 bytes, byte for byte but padding");

            // ros2 param set: SetParameters {parameters: [{name, value}]}
            AutoPubParamEntry entries[4];
            req = fixture("zenoh_param_set_parameters_req_0.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readSetRequest(req.data(), req.size(), entries, 4, count) && count == 1 &&
                        std::strcmp(entries[0].name, "chatterEnable") == 0 && entries[0].value.type == AutoPubParamType::Bool &&
                        !entries[0].value.boolValue && !entries[0].value.hasArrayData, "set request: bool false");
            req = fixture("zenoh_param_set_parameters_req_1.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readSetRequest(req.data(), req.size(), entries, 4, count) && count == 1 &&
                        entries[0].value.type == AutoPubParamType::Integer && entries[0].value.integerValue == 500,
                        "set request: int64 500");
            req = fixture("zenoh_param_set_parameters_req_2.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readSetRequest(req.data(), req.size(), entries, 4, count) && count == 1 &&
                        entries[0].value.type == AutoPubParamType::Double && entries[0].value.doubleValue == 2.25,
                        "set request: double 2.25");
            req = fixture("zenoh_param_set_parameters_req_3.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readSetRequest(req.data(), req.size(), entries, 4, count) && count == 1 &&
                        std::strcmp(entries[0].name, "chatterPeriodMs") == 0 && entries[0].value.type == AutoPubParamType::String &&
                        std::strcmp(entries[0].value.stringValue, "abc") == 0, "set request: a string where an int is held");
            for (size_t cut = 5; cut < req.size(); cut += 7)
                TEST_ASSERT(!AutoPubParamCodec_readSetRequest(req.data(), cut, entries, 4, count),
                            "set request: every truncation is rejected");
            // A set carrying array data is noted, its contents stepped over
            {
                uint8_t arr[128]; CDREncoder e; e.reset(arr, sizeof(arr));
                e.writeEncapsulationHeader(); e.writeSequenceLength(1); e.writeString("xs");
                e.writeUint8(7); e.writeBool(false); e.writeInt64(0); e.writeFloat64(0); e.writeString("");
                e.writeSequenceLength(0); e.writeSequenceLength(0);
                e.writeSequenceLength(2); e.writeInt64(1); e.writeInt64(2);
                e.writeSequenceLength(0); e.writeSequenceLength(1); e.writeString("s");
                TEST_ASSERT(AutoPubParamCodec_readSetRequest(arr, e.getPos(), entries, 4, count) && count == 1 &&
                            entries[0].value.type == AutoPubParamType::IntegerArray && entries[0].value.hasArrayData,
                            "set request: array values are recognised and skipped");
            }
            // Results
            AutoPubParamResult ok{true, ""};
            resp = fixture("zenoh_param_set_parameters_resp_0.cdr.hex");
            n = AutoPubParamCodec_writeResults(out, sizeof(out), &ok, 1);
            TEST_ASSERT(n == resp.size() && n == 17 && std::memcmp(out, resp.data(), n) == 0, "set response: success");
            AutoPubParamResult bad{false, "Wrong parameter type, expected 'Type.INTEGER' got 'Type.STRING'"};
            resp = fixture("zenoh_param_set_parameters_resp_1.cdr.hex");
            n = AutoPubParamCodec_writeResults(out, sizeof(out), &bad, 1);
            TEST_ASSERT(n == resp.size() && std::memcmp(out, resp.data(), n) == 0, "set response: failure with reason");
            n = AutoPubParamCodec_writeSingleResult(out, sizeof(out), bad);
            TEST_ASSERT(n == resp.size() - 4 && std::memcmp(out + 4, resp.data() + 8, n - 4) == 0,
                        "atomic set response: the same result without the sequence length");
            TEST_ASSERT(AutoPubParamCodec_writeResults(out, 12, &bad, 1) == 0, "a result that does not fit is refused");

            // ros2 param describe
            req = fixture("zenoh_param_describe_parameters_req_0.cdr.hex");
            TEST_ASSERT(AutoPubParamCodec_readNames(req.data(), req.size(), names, 8, count) && count == 1 &&
                        std::strcmp(names[0], "chatterPeriodMs") == 0, "describe request: one name");
            AutoPubParamDescriptor d{"chatterPeriodMs", AutoPubParamType::Integer, "Period of /chatter in ms", false};
            resp = fixture("zenoh_param_describe_parameters_resp_0.cdr.hex");
            n = AutoPubParamCodec_writeDescriptors(out, sizeof(out), &d, 1);
            TEST_ASSERT(n == resp.size() && std::memcmp(out, resp.data(), n) == 0, "describe response: byte for byte");

            // GetParameterTypes has no capture; its shape is uint8[]
            const AutoPubParamType types[] = {AutoPubParamType::Bool, AutoPubParamType::String};
            n = AutoPubParamCodec_writeTypes(out, sizeof(out), types, 2);
            TEST_ASSERT(n == 10 && out[4] == 2 && out[8] == 1 && out[9] == 4, "types response: sequence of uint8");
        }

        printf("Test: parameter store serves list/get/types/describe/set/set_atomically\n");
        {
            using namespace RaftRuntime::AutoPub;
            AutoPubParameterStore<8> store;
            auto boolV = [](bool b) { AutoPubParamValue v; v.type = AutoPubParamType::Bool; v.boolValue = b; return v; };
            auto intV = [](int64_t i) { AutoPubParamValue v; v.type = AutoPubParamType::Integer; v.integerValue = i; return v; };
            auto strV = [](const char* s) { AutoPubParamValue v; v.type = AutoPubParamType::String; std::strncpy(v.stringValue, s, sizeof(v.stringValue) - 1); return v; };
            int applied = 0;
            TEST_ASSERT(store.declare("use_sim_time", boolV(false), "", true) == 0, "declare read-only");
            TEST_ASSERT(store.declare("chatterEnable", boolV(true), "Publish /chatter", false,
                            [&](const AutoPubParamValue& v, const char*&) { applied += v.boolValue ? 1 : 10; return true; }) == 1,
                        "declare with a callback");
            TEST_ASSERT(store.declare("chatterPeriodMs", intV(1000), "Period of /chatter in ms") == 2, "declare int");
            TEST_ASSERT(store.declare("routerHost", strV("192.168.86.192"), "", false,
                            [](const AutoPubParamValue& v, const char*& reason) {
                                if (std::strchr(v.stringValue, '.') == nullptr) { reason = "not an IPv4 address"; return false; }
                                return true; }) == 3, "declare string with a validating callback");
            TEST_ASSERT(store.declare("a.b", intV(1)) == 4 && store.declare("a.c", intV(2)) == 5, "declare nested names");
            TEST_ASSERT(store.declare("chatterEnable", boolV(true)) == store.INVALID_SLOT, "no duplicate names");
            AutoPubParamValue arr; arr.type = AutoPubParamType::IntegerArray;
            TEST_ASSERT(store.declare("arr", arr) == store.INVALID_SLOT, "arrays cannot be declared");
            TEST_ASSERT(store.count() == 6 && store.value("chatterPeriodMs")->integerValue == 1000, "count and value");

            // Build requests with the codec's own writers' inverse: small hand encoders
            uint8_t req[512], out[1024];
            auto namesReq = [&](std::initializer_list<const char*> names) {
                CDREncoder e; e.reset(req, sizeof(req)); e.writeEncapsulationHeader(); e.writeSequenceLength((uint32_t)names.size());
                for (const char* n : names) e.writeString(n);
                return e.getPos(); };
            auto listReq = [&](std::initializer_list<const char*> prefixes, uint64_t depth) {
                CDREncoder e; e.reset(req, sizeof(req)); e.writeEncapsulationHeader(); e.writeSequenceLength((uint32_t)prefixes.size());
                for (const char* p : prefixes) e.writeString(p);
                e.writeUint64(depth); return e.getPos(); };
            auto setReq = [&](std::initializer_list<AutoPubParamEntry> entries) {
                CDREncoder e; e.reset(req, sizeof(req)); e.writeEncapsulationHeader(); e.writeSequenceLength((uint32_t)entries.size());
                for (const auto& en : entries) { e.writeString(en.name); AutoPubParamCodec_writeValue(e, en.value); }
                return e.getPos(); };
            auto entry = [](const char* name, AutoPubParamValue v) { AutoPubParamEntry en; std::strcpy(en.name, name); en.value = v; return en; };
            // Decode helpers for the responses
            auto readValues = [&](uint32_t n, std::vector<AutoPubParamValue>& values) {
                CDRDecoder d; d.init(out, n); uint32_t c = 0; values.clear();
                if (!d.readEncapsulationHeader() || !d.readSequenceLength(c)) return false;
                for (uint32_t i = 0; i < c; ++i) { AutoPubParamValue v; if (!AutoPubParamCodec_readValue(d, v)) return false; values.push_back(v); }
                return true; };
            auto readResults = [&](uint32_t n, bool single, std::vector<std::pair<bool, std::string>>& results) {
                CDRDecoder d; d.init(out, n); uint32_t c = 1; results.clear();
                if (!d.readEncapsulationHeader() || (!single && !d.readSequenceLength(c))) return false;
                for (uint32_t i = 0; i < c; ++i) { bool ok; char r[128]; uint32_t l; if (!d.readBool(ok) || !d.readString(r, sizeof(r), l)) return false; results.push_back({ok, r}); }
                return true; };
            auto readNamesResp = [&](uint32_t n, std::vector<std::string>& names, std::vector<std::string>& prefixes) {
                CDRDecoder d; d.init(out, n); uint32_t c = 0; names.clear(); prefixes.clear(); char s[64]; uint32_t l;
                if (!d.readEncapsulationHeader() || !d.readSequenceLength(c)) return false;
                for (uint32_t i = 0; i < c; ++i) { if (!d.readString(s, sizeof(s), l)) return false; names.push_back(s); }
                if (!d.readSequenceLength(c)) return false;
                for (uint32_t i = 0; i < c; ++i) { if (!d.readString(s, sizeof(s), l)) return false; prefixes.push_back(s); }
                return true; };

            std::vector<std::string> names, prefixes;
            uint32_t n = store.list(req, listReq({}, 0), out, sizeof(out));
            TEST_ASSERT(n && readNamesResp(n, names, prefixes) && names.size() == 6 && names[1] == "chatterEnable" &&
                        prefixes.size() == 1 && prefixes[0] == "a", "list: everything, with the one nested prefix");
            n = store.list(req, listReq({"a"}, 0), out, sizeof(out));
            TEST_ASSERT(n && readNamesResp(n, names, prefixes) && names.size() == 2 && names[0] == "a.b", "list: by prefix");
            n = store.list(req, listReq({}, 1), out, sizeof(out));
            TEST_ASSERT(n && readNamesResp(n, names, prefixes) && names.size() == 4 && prefixes.empty(), "list: depth 1 hides nested names");
            n = store.list(req, listReq({"nosuch"}, 0), out, sizeof(out));
            TEST_ASSERT(n && readNamesResp(n, names, prefixes) && names.empty(), "list: unknown prefix matches nothing");

            std::vector<AutoPubParamValue> values;
            n = store.get(req, namesReq({"chatterPeriodMs", "nosuch", "routerHost"}), out, sizeof(out));
            TEST_ASSERT(n && readValues(n, values) && values.size() == 3 && values[0].integerValue == 1000 &&
                        values[1].type == AutoPubParamType::NotSet && std::strcmp(values[2].stringValue, "192.168.86.192") == 0,
                        "get: values in request order, unknown as NotSet");
            n = store.getTypes(req, namesReq({"use_sim_time", "nosuch"}), out, sizeof(out));
            TEST_ASSERT(n == 10 && out[8] == 1 && out[9] == 0, "types: bool and NotSet");
            n = store.describe(req, namesReq({"chatterPeriodMs"}), out, sizeof(out));
            TEST_ASSERT(n > 0 && std::memcmp(out + 12, "chatterPeriodMs", 15) == 0, "describe: named descriptor");
            TEST_ASSERT(store.get(req, 3, out, sizeof(out)) == 0 && store.set(req, 3, out, sizeof(out)) == 0, "a truncated request builds nothing");

            std::vector<std::pair<bool, std::string>> results;
            n = store.set(req, setReq({entry("chatterEnable", boolV(false)), entry("chatterPeriodMs", strV("abc")),
                                       entry("use_sim_time", boolV(true)), entry("nosuch", intV(1)),
                                       entry("routerHost", strV("nodots"))}), out, sizeof(out));
            TEST_ASSERT(n && readResults(n, false, results) && results.size() == 5 &&
                        results[0].first && applied == 10 && !store.value("chatterEnable")->boolValue &&
                        !results[1].first && results[1].second == "Wrong parameter type, expected 'Type.INTEGER' got 'Type.STRING'" &&
                        !results[2].first && results[2].second == "Trying to set a read-only parameter: use_sim_time." &&
                        !results[3].first && results[3].second.rfind("Invalid access to undeclared", 0) == 0 &&
                        !results[4].first && results[4].second == "not an IPv4 address" &&
                        std::strcmp(store.value("routerHost")->stringValue, "192.168.86.192") == 0,
                        "set: each parameter judged on its own, with the node's reasons");
            n = store.set(req, setReq({entry("chatterEnable", boolV(true))}), out, sizeof(out));
            TEST_ASSERT(n && readResults(n, false, results) && results.size() == 1 && results[0].first && results[0].second.empty(),
                        "set: a success after refusals carries no stale reason");
            AutoPubParamValue arrSet; arrSet.type = AutoPubParamType::Integer; arrSet.hasArrayData = true;
            {
                // A set carrying array data on the wire
                CDREncoder e; e.reset(req, sizeof(req)); e.writeEncapsulationHeader(); e.writeSequenceLength(1); e.writeString("chatterPeriodMs");
                e.writeUint8(2); e.writeBool(false); e.writeInt64(5); e.writeFloat64(0); e.writeString("");
                e.writeSequenceLength(0); e.writeSequenceLength(0); e.writeSequenceLength(1); e.writeInt64(9); e.writeSequenceLength(0); e.writeSequenceLength(0);
                n = store.set(req, e.getPos(), out, sizeof(out));
                TEST_ASSERT(n && readResults(n, false, results) && !results[0].first && results[0].second.rfind("Array", 0) == 0 &&
                            store.value("chatterPeriodMs")->integerValue == 1000, "set: array data refused, value untouched");
            }
            n = store.setAtomically(req, setReq({entry("chatterPeriodMs", intV(250)), entry("use_sim_time", boolV(true))}), out, sizeof(out));
            TEST_ASSERT(n && readResults(n, true, results) && !results[0].first && store.value("chatterPeriodMs")->integerValue == 1000,
                        "atomic set: one refusal and nothing changes");
            n = store.setAtomically(req, setReq({entry("chatterPeriodMs", intV(250)), entry("chatterEnable", boolV(true))}), out, sizeof(out));
            TEST_ASSERT(n && readResults(n, true, results) && results[0].first && store.value("chatterPeriodMs")->integerValue == 250 &&
                        store.value("chatterEnable")->boolValue && applied == 12, "atomic set: all applied, callbacks ran");
            TEST_ASSERT(store.setLocal("chatterPeriodMs", intV(400)) && store.value("chatterPeriodMs")->integerValue == 400 &&
                        !store.setLocal("chatterPeriodMs", boolV(true)), "setLocal keeps the type");
        }

        printf("Test: a Raw service hands the CDR to its handler and copies its reply\n");
        {
            using namespace RaftRuntime::AutoPub;
            AutoPubServiceRegistry<2, 2, 64, 32> registry;
            static uint8_t seen[32]; static uint32_t seenLen = 0;
            static const uint8_t answer[] = {0, 1, 0, 0, 9, 8, 7};
            const uint8_t slot = registry.add("/raft/raw", AutoPubServiceKind::Raw,
                [](const AutoPubServiceRequest& req, AutoPubServiceReply& reply) {
                    seenLen = req.fields.rawLen; std::memcpy(seen, req.fields.raw, seenLen);
                    reply.fields.raw = answer; reply.fields.rawLen = sizeof(answer);
                    return AutoPubServiceOutcome::Replied;
                });
            const uint8_t request[] = {0, 1, 0, 0, 5, 6};
            TEST_ASSERT(registry.accept(slot, 1, request, sizeof(request), nullptr, 0, 0, 100), "a raw request is accepted");
            registry.service(100, 2);
            TEST_ASSERT(seenLen == sizeof(request) && std::memcmp(seen, request, seenLen) == 0, "the handler sees the request bytes");
            auto send = registry.next();
            TEST_ASSERT(send.kind == AutoPubServiceSend::Kind::Response && send.payloadLen == sizeof(answer) &&
                        std::memcmp(send.payload, answer, sizeof(answer)) == 0, "the reply is the handler's bytes");
            uint8_t big[40] = {};
            registry.sent(); registry.sent();
            TEST_ASSERT(!registry.accept(slot, 2, big, sizeof(big), nullptr, 0, 0, 100) &&
                        registry.next().kind == AutoPubServiceSend::Kind::Error, "a raw request over REQUEST_MAX is refused");
        }

        printf("Test: service codec encodes and decodes the std_srvs types\n");
        using namespace RaftRuntime::AutoPub;
        // The exact request payloads a real client sent (S0 fixtures)
        const uint8_t triggerReq[] = {0x00, 0x01, 0x00, 0x00, 0x00};
        const uint8_t setBoolReq[] = {0x00, 0x01, 0x00, 0x00, 0x01};
        AutoPubServiceRequestFields fields;
        TEST_ASSERT(AutoPubServiceCodec_decodeRequest(AutoPubServiceKind::Trigger, triggerReq, sizeof(triggerReq), fields) &&
                    fields.kind == AutoPubServiceKind::Trigger,
                    "an empty ROS request decodes: CDR header plus the one dummy byte");
        TEST_ASSERT(!AutoPubServiceCodec_decodeRequest(AutoPubServiceKind::Trigger, triggerReq, 4, fields),
                    "a Trigger request without its dummy byte is refused");
        TEST_ASSERT(AutoPubServiceCodec_decodeRequest(AutoPubServiceKind::SetBool, setBoolReq, sizeof(setBoolReq), fields) &&
                    fields.data == true, "SetBool{data:true} decodes");
        TEST_ASSERT(AutoPubServiceCodec_decodeRequest(AutoPubServiceKind::Empty, triggerReq, sizeof(triggerReq), fields),
                    "an Empty request has the same shape as a Trigger request");
        uint8_t out[64];
        const uint32_t n = AutoPubServiceCodec_encodeResponse(AutoPubServiceKind::Trigger, {true, "hello from server"}, out, sizeof(out));
        // What a real server sent for the same response (S0 fixture payload)
        const uint8_t expected[] = {0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00,
                                    'h','e','l','l','o',' ','f','r','o','m',' ','s','e','r','v','e','r', 0x00};
        TEST_ASSERT(n == sizeof(expected) && std::memcmp(out, expected, n) == 0,
                    "a Trigger response encodes byte for byte as a real rmw_zenoh server's");
        const uint32_t e = AutoPubServiceCodec_encodeResponse(AutoPubServiceKind::Empty, {}, out, sizeof(out));
        TEST_ASSERT(e == 5 && out[4] == 0, "an Empty response is the header plus the dummy byte");
        TEST_ASSERT(AutoPubServiceCodec_encodeResponse(AutoPubServiceKind::Trigger, {true, "x"}, out, 6) == 0,
                    "a response that does not fit is refused, not truncated");
        TEST_ASSERT(AutoPubServiceCodec_kindForWireType("std_srvs::srv::dds_::SetBool_") == AutoPubServiceKind::SetBool &&
                    AutoPubServiceCodec_kindForWireType("nope") == AutoPubServiceKind::Unknown,
                    "wire type names map to kinds");
    }
    {
        printf("Test: service registry answers now, later, or not at all - never blocking\n");
        using namespace RaftRuntime::AutoPub;
        using Registry = AutoPubServiceRegistry<4, 2, 128>;
        Registry registry;
        int calls = 0;
        const uint8_t slot = registry.add("/raft/trigger", AutoPubServiceKind::Trigger,
            [&](const AutoPubServiceRequest& req, AutoPubServiceReply& reply) {
                ++calls; reply.fields = {true, "now"}; return AutoPubServiceOutcome::Replied;
            });
        TEST_ASSERT(slot == 0 && registry.inUseCount() == 1, "a service takes the first slot");
        TEST_ASSERT(registry.add("/raft/trigger", AutoPubServiceKind::Trigger, [](const AutoPubServiceRequest&, AutoPubServiceReply&) {
                        return AutoPubServiceOutcome::Replied; }) == Registry::INVALID_SLOT,
                    "one server per service name");
        TEST_ASSERT(registry.add("/raft/x", AutoPubServiceKind::Unknown, [](const AutoPubServiceRequest&, AutoPubServiceReply&) {
                        return AutoPubServiceOutcome::Replied; }) == Registry::INVALID_SLOT,
                    "an unknown type cannot be served");

        const uint8_t req[] = {0x00, 0x01, 0x00, 0x00, 0x00};
        uint8_t att[33] = {0}; att[0] = 42; att[16] = 16;
        TEST_ASSERT(registry.accept(slot, 7, req, sizeof(req), att, sizeof(att), 600000, 1000),
                    "a well-formed request is accepted");
        TEST_ASSERT(registry.next().kind == AutoPubServiceSend::Kind::None && calls == 0,
                    "nothing is sent and no handler runs until service()");
        registry.service(1001, 2);
        TEST_ASSERT(calls == 1, "service() runs the handler");
        auto send = registry.next();
        TEST_ASSERT(send.kind == AutoPubServiceSend::Kind::Response && send.requestId == 7 && send.slot == slot &&
                    send.payloadLen > 4 && send.attachment && send.attachment[0] == 42,
                    "the reply is staged with the request id and the client's attachment echoed");
        registry.sent();
        send = registry.next();
        TEST_ASSERT(send.kind == AutoPubServiceSend::Kind::Final && send.requestId == 7,
                    "the final follows the response on the next pass");
        registry.sent();
        TEST_ASSERT(registry.next().kind == AutoPubServiceSend::Kind::None && registry.inflightCount() == 0 &&
                    registry.stats().completed == 1, "after the final the entry is free");

        // Deferred: the handler cannot answer now; the application completes later
        uint32_t token = 0;
        const uint8_t deferSlot = registry.add("/raft/later", AutoPubServiceKind::SetBool,
            [&](const AutoPubServiceRequest& r, AutoPubServiceReply&) { token = r.token; return AutoPubServiceOutcome::Deferred; });
        const uint8_t setReq[] = {0x00, 0x01, 0x00, 0x00, 0x01};
        registry.accept(deferSlot, 8, setReq, sizeof(setReq), att, sizeof(att), 0, 2000);
        registry.service(2001, 2);
        TEST_ASSERT(token != 0 && registry.next().kind == AutoPubServiceSend::Kind::None && registry.stats().deferred == 1,
                    "a deferred request sends nothing and hands the application a token");
        TEST_ASSERT(!registry.complete(token + 99, {}), "an unknown token is rejected");
        TEST_ASSERT(registry.complete(token, {{true, "later"}, ""}) && registry.next().kind == AutoPubServiceSend::Kind::Response,
                    "completing the token stages the reply");
        registry.sent(); registry.sent();

        // Timeout: a deferred request nobody completes is answered with an error
        registry.accept(deferSlot, 9, setReq, sizeof(setReq), att, sizeof(att), 0, 3000);
        registry.service(3001, 2);
        // The deadline runs from acceptance (3000), not from the first service()
        registry.service(3000 + Registry::DEFAULT_TIMEOUT_MS - 1, 2);
        TEST_ASSERT(registry.next().kind == AutoPubServiceSend::Kind::None, "not yet timed out");
        registry.service(3000 + Registry::DEFAULT_TIMEOUT_MS, 2);
        send = registry.next();
        TEST_ASSERT(send.kind == AutoPubServiceSend::Kind::Error && send.requestId == 9 &&
                    std::strcmp(send.reason, "timeout") == 0 && registry.stats().timedOut == 1,
                    "a deferred request that is never completed is refused with 'timeout'");
        registry.sent(); registry.sent();

        // Budget: three queued, budget two -> one waits for the next pass
        int budgetCalls = 0;
        const uint8_t bSlot = registry.add("/raft/b", AutoPubServiceKind::Empty,
            [&](const AutoPubServiceRequest&, AutoPubServiceReply&) { ++budgetCalls; return AutoPubServiceOutcome::Replied; });
        registry.remove(deferSlot);
        Registry big;   // the registry above holds 2 in flight; use a fresh one with the same limits to show the budget
        (void)bSlot;
        const uint8_t bs = big.add("/raft/b", AutoPubServiceKind::Empty,
            [&](const AutoPubServiceRequest&, AutoPubServiceReply&) { ++budgetCalls; return AutoPubServiceOutcome::Replied; });
        TEST_ASSERT(big.accept(bs, 1, req, sizeof(req), att, sizeof(att), 0, 10) &&
                    big.accept(bs, 2, req, sizeof(req), att, sizeof(att), 0, 10),
                    "the table holds INFLIGHT requests");
        TEST_ASSERT(!big.accept(bs, 3, req, sizeof(req), att, sizeof(att), 0, 10) && big.stats().refusedBusy == 1,
                    "a request beyond the table is refused at once");
        send = big.next();
        TEST_ASSERT(send.kind == AutoPubServiceSend::Kind::Error && send.requestId == 3 && std::strcmp(send.reason, "busy") == 0,
                    "the refusal is an error reply for the refused request, sent first");
        big.sent(); big.sent();
        big.service(11, 1);
        TEST_ASSERT(budgetCalls == 1, "a budget of one runs one handler per pass");
        big.service(12, 1);
        TEST_ASSERT(budgetCalls == 2, "the next pass runs the next");

        // Bad request: refused with an error, no handler runs
        Registry bad;
        int badCalls = 0;
        const uint8_t badSlot = bad.add("/raft/t", AutoPubServiceKind::Trigger,
            [&](const AutoPubServiceRequest&, AutoPubServiceReply&) { ++badCalls; return AutoPubServiceOutcome::Replied; });
        TEST_ASSERT(!bad.accept(badSlot, 5, req, 3, att, sizeof(att), 0, 0) && bad.stats().refusedBad == 1,
                    "an undecodable request is refused");
        bad.service(1, 2);
        TEST_ASSERT(badCalls == 0 && bad.next().kind == AutoPubServiceSend::Kind::Error &&
                    std::strcmp(bad.next().reason, "bad request") == 0,
                    "no handler runs for it and the client is told why");
    }

    //=================================================================
    // Liveliness (ParticipantMessageData): RTPS sequence numbers start at 1.
    // Sending sequence 0 put writerSN 0 in the DATA and firstSN/lastSN 0 in the
    // HEARTBEAT, and CycloneDDS discards that whole datagram as malformed - so
    // the first liveliness assertion to every participant was silently lost.
    //=================================================================
    {
        printf("Test: liveliness ParticipantMessageData rejects sequence 0\n");

        RTPSParticipant part;
        part.init(0, "liveliness_test", 0, nullptr);
        uint8_t mac[6] = {0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C};
        part.setGuidPrefixFromMAC(mac);
        uint8_t destGP[12] = {1,2,3,4,5,6,7,8,9,10,11,12};

        SEDPHandler sedp;
        uint8_t msgBuf[512];
        TEST_ASSERT(sedp.buildParticipantMessageData(msgBuf, sizeof(msgBuf), part, destGP,
                                                     /*sequenceNumber=*/0, 1) == 0,
                    "liveliness build refuses sequence 0 rather than sending an invalid sample");

        const uint32_t msgLen = sedp.buildParticipantMessageData(msgBuf, sizeof(msgBuf), part, destGP,
                                                                 /*sequenceNumber=*/1, 1);
        TEST_ASSERT(msgLen > 0, "liveliness build accepts sequence 1");

        // The HEARTBEAT must advertise the current sample, not an empty writer
        uint8_t gp[12];
        uint32_t off = RTPSMessage::parseHeader(msgBuf, msgLen, gp);
        bool checkedHeartbeat = false, checkedData = false;
        while (off < msgLen) {
            RTPSSubmessageId id; uint8_t fl; const uint8_t* pc; uint32_t cl;
            const uint32_t sz = RTPSMessage::parseSubmessage(msgBuf+off, msgLen-off, id, fl, pc, cl);
            if (sz == 0) break;
            if (id == SUBMSG_HEARTBEAT && cl >= 28) {
                // readerId, writerId, firstSN (high, low), lastSN (high, low)
                const uint32_t firstLow = (uint32_t)pc[12] | ((uint32_t)pc[13] << 8) |
                                         ((uint32_t)pc[14] << 16) | ((uint32_t)pc[15] << 24);
                const uint32_t lastLow  = (uint32_t)pc[20] | ((uint32_t)pc[21] << 8) |
                                         ((uint32_t)pc[22] << 16) | ((uint32_t)pc[23] << 24);
                TEST_ASSERT(firstLow == 1 && lastLow == 1,
                            "liveliness HEARTBEAT advertises the current sample, never sequence 0");
                checkedHeartbeat = true;
            }
            if (id == SUBMSG_DATA && cl >= 20) {
                // extraFlags, octetsToInlineQos, readerId, writerId, then writerSN
                const uint32_t writerSNLow = (uint32_t)pc[16] | ((uint32_t)pc[17] << 8) |
                                             ((uint32_t)pc[18] << 16) | ((uint32_t)pc[19] << 24);
                TEST_ASSERT(writerSNLow == 1, "liveliness DATA carries writerSN 1");
                checkedData = true;
            }
            off += sz;
        }
        TEST_ASSERT(checkedHeartbeat && checkedData,
                    "liveliness message contains both a DATA and a HEARTBEAT to check");
    }

    //=================================================================
    // ACKNACK action plan: VOLATILE /chatter HEARTBEAT firstSN must equal
    // the current sequence number (Fix 15 regression guard).
    // This ensures newly-matched VOLATILE subscribers do not NACK historical
    // samples after an ACKNACK-driven chatter retransmit.
    //=================================================================
    {
        printf("Test: ACKNACK chatter retransmit firstSN == currentSeq (VOLATILE invariant)\n");

        using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

        // ESP flavor: factory must opt in to chatterFirstSNMatchesSequence
        const auto espUserCtx = makeAckUserDataSequenceContextForFlavor(
            RTPSAckNackRuntimeFlavor::EspStyle,
            /*rosDiscoveryInfoSeqNum*/ 7,
            /*chatterDataSeqNum*/ 42);
        TEST_ASSERT(espUserCtx.chatterFirstSNMatchesSequence,
                    "ESP flavor sets chatterFirstSNMatchesSequence");

        // Linux flavor: factory must also opt in (VOLATILE invariant is flavor-agnostic)
        const auto linuxUserCtx = makeAckUserDataSequenceContextForFlavor(
            RTPSAckNackRuntimeFlavor::LinuxStandalone,
            /*rosDiscoveryInfoSeqNum*/ 9,
            /*chatterDataSeqNum*/ 100);
        TEST_ASSERT(linuxUserCtx.chatterFirstSNMatchesSequence,
                    "Linux flavor sets chatterFirstSNMatchesSequence");

        // Chatter retransmit plan must report firstSN override == sequenceNumber
        RTPSAckActionUserDataPlan chatterPlan = {};
        TEST_ASSERT(getAckActionUserDataPlan(
                        RTPSAckNackDecisionAction::RetransmitChatterData,
                        espUserCtx,
                        chatterPlan),
                    "Chatter retransmit plan produced");
        TEST_ASSERT(chatterPlan.sequenceNumber == 42,
                    "Chatter plan sequenceNumber == chatterDataSeqNum");
        TEST_ASSERT(chatterPlan.hasFirstSNOverride,
                    "Chatter plan hasFirstSNOverride=true for VOLATILE");
        TEST_ASSERT(chatterPlan.firstSN == chatterPlan.sequenceNumber,
                    "Chatter plan firstSN == sequenceNumber (VOLATILE invariant)");

        // ros_discovery_info: writer keeps only the LATEST sample (rebuilt
        // from current state on every emission).  firstSN==lastSN==current is
        // the only safe HEARTBEAT range -- a reader that joins after seq is
        // bumped would otherwise NACK historic SNs that we no longer hold,
        // never reaching the GraphCache update.  Plan must also request the
        // participant key hash on retransmits (keyed builtin topic).
        RTPSAckActionUserDataPlan rosDiscPlan = {};
        TEST_ASSERT(getAckActionUserDataPlan(
                        RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo,
                        espUserCtx,
                        rosDiscPlan),
                    "ros_discovery_info retransmit plan produced");
        TEST_ASSERT(rosDiscPlan.hasFirstSNOverride,
                    "ros_discovery_info plan overrides firstSN (writer holds only latest sample)");
        TEST_ASSERT(rosDiscPlan.firstSN == rosDiscPlan.sequenceNumber,
                    "ros_discovery_info plan firstSN == sequenceNumber");
        TEST_ASSERT(rosDiscPlan.useParticipantKeyHash,
                    "ros_discovery_info plan requests participant key hash on retransmits");
        TEST_ASSERT(rosDiscPlan.sequenceNumber == 7,
                    "ros_discovery_info plan sequenceNumber == rosDiscoveryInfoSeqNum");
    }

    //=================================================================
    // ACKNACK SEDP plan: two DataWriter announcements on the same
    // SEDP publications writer must use distinct sequence numbers
    // (Fix 16 regression guard).
    //=================================================================
    {
        printf("Test: ACKNACK SEDP publication retransmit uses distinct sequence numbers\n");

        using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

        // ESP flavor: ros_discovery_info SEDP pub at seq=1, chatter SEDP pub at seq=2.
        // The ESP path uses distinct chatterPublicationSeqNum (not derived).
        const auto espSedpCtx = makeAckSedpSequenceContextForFlavor(
            RTPSAckNackRuntimeFlavor::EspStyle,
            /*rosDiscoveryPublicationSeqNum*/ 1,
            /*rosDiscoverySubscriptionSeqNum*/ 1,
            /*chatterPublicationSeqNum*/ 2);
        TEST_ASSERT(!espSedpCtx.deriveChatterPublicationFromRosDiscoveryPublication,
                    "ESP flavor uses explicit chatterPublicationSeqNum");

        RTPSAckActionSedpPlan rosDiscSedpPlan = {};
        RTPSAckActionSedpPlan chatterSedpPlan = {};
        TEST_ASSERT(getAckActionSedpPlan(
                        RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication,
                        espSedpCtx,
                        rosDiscSedpPlan),
                    "ESP ros_discovery_info SEDP pub plan produced");
        TEST_ASSERT(getAckActionSedpPlan(
                        RTPSAckNackDecisionAction::RetransmitSedpChatterPublication,
                        espSedpCtx,
                        chatterSedpPlan),
                    "ESP chatter SEDP pub plan produced");

        TEST_ASSERT(rosDiscSedpPlan.sequenceNumber == 1,
                    "ESP ros_discovery_info SEDP pub seq=1");
        TEST_ASSERT(chatterSedpPlan.sequenceNumber == 2,
                    "ESP chatter SEDP pub seq=2");
        TEST_ASSERT(rosDiscSedpPlan.sequenceNumber != chatterSedpPlan.sequenceNumber,
                    "ESP two SEDP publications on same writer have distinct seq");

        // Endpoints must also be distinct (different entity IDs, topics, durability)
        TEST_ASSERT(rosDiscSedpPlan.endpoint.entityId != chatterSedpPlan.endpoint.entityId,
                    "ESP SEDP pub plans target distinct entity IDs");
        TEST_ASSERT(rosDiscSedpPlan.endpoint.durabilityKind == DURABILITY_TRANSIENT_LOCAL,
                    "ros_discovery_info SEDP pub is TRANSIENT_LOCAL");
        TEST_ASSERT(chatterSedpPlan.endpoint.durabilityKind == DURABILITY_VOLATILE,
                    "chatter SEDP pub is VOLATILE");

        // Linux flavor: derives chatter SEDP pub seq = rosDiscovery SEDP pub seq + 1.
        // Regardless of derivation strategy, the two seq numbers MUST differ.
        const auto linuxSedpCtx = makeAckSedpSequenceContextForFlavor(
            RTPSAckNackRuntimeFlavor::LinuxStandalone,
            /*rosDiscoveryPublicationSeqNum*/ 1,
            /*rosDiscoverySubscriptionSeqNum*/ 1,
            /*chatterPublicationSeqNum*/ 0);
        TEST_ASSERT(linuxSedpCtx.deriveChatterPublicationFromRosDiscoveryPublication,
                    "Linux flavor derives chatter SEDP pub from ros_discovery SEDP pub");

        RTPSAckActionSedpPlan linuxRosDiscPlan = {};
        RTPSAckActionSedpPlan linuxChatterPlan = {};
        TEST_ASSERT(getAckActionSedpPlan(
                        RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication,
                        linuxSedpCtx,
                        linuxRosDiscPlan),
                    "Linux ros_discovery_info SEDP pub plan produced");
        TEST_ASSERT(getAckActionSedpPlan(
                        RTPSAckNackDecisionAction::RetransmitSedpChatterPublication,
                        linuxSedpCtx,
                        linuxChatterPlan),
                    "Linux chatter SEDP pub plan produced");
        TEST_ASSERT(linuxRosDiscPlan.sequenceNumber != linuxChatterPlan.sequenceNumber,
                    "Linux two SEDP publications on same writer have distinct seq");
        TEST_ASSERT(linuxChatterPlan.sequenceNumber == linuxRosDiscPlan.sequenceNumber + 1,
                    "Linux derives chatter SEDP pub seq = rosDiscovery seq + 1");
    }

    //=================================================================
    // Reader runtime: incoming DATA accept/dedup/gap tracking
    //=================================================================
    {
        printf("Test: Reader DATA accept / dedup / gap handling\n");

        using namespace RaftRuntime::RTPS::Runtime::Reader;

        RTPSReaderWriterState st;
        st.reliabilityKind = RTPSReaderReliabilityKind::Reliable;

        // First DATA SN=1 is the next-expected sample -> Accept; contiguous advances.
        RTPSReaderDataFields d1; d1.writerSeqNum = 1;
        TEST_ASSERT(evaluateIncomingDataDecision(st, d1) == RTPSReaderDataAction::Accept,
                    "first DATA SN=1 accepted");
        applyAcceptedDataToReaderState(st, 1);
        TEST_ASSERT(st.highestContiguousSeq == 1, "contiguous advances to 1");
        TEST_ASSERT(st.receivedBitmap == 0, "bitmap empty after contiguous advance");

        // Re-delivery of SN=1 is dedup.
        TEST_ASSERT(evaluateIncomingDataDecision(st, d1) == RTPSReaderDataAction::Dedup,
                    "duplicate DATA SN=1 deduped");

        // SN=3 arrives before SN=2 -> Accept but held in bitmap; contiguous stays at 1.
        RTPSReaderDataFields d3; d3.writerSeqNum = 3;
        TEST_ASSERT(evaluateIncomingDataDecision(st, d3) == RTPSReaderDataAction::Accept,
                    "out-of-order DATA SN=3 accepted");
        applyAcceptedDataToReaderState(st, 3);
        TEST_ASSERT(st.highestContiguousSeq == 1, "contiguous remains 1 with gap at 2");
        TEST_ASSERT(st.highestSeenSeq == 3, "highestSeen advances to 3");
        TEST_ASSERT((st.receivedBitmap & 0x2ULL) != 0,
                    "bitmap marks SN=3 (offset 1 from base 2)");

        // Re-delivery of SN=3 while held -> dedup.
        TEST_ASSERT(evaluateIncomingDataDecision(st, d3) == RTPSReaderDataAction::Dedup,
                    "duplicate held DATA SN=3 deduped");

        // Missing SN=2 arrives -> Accept and window collapses up to SN=3.
        RTPSReaderDataFields d2; d2.writerSeqNum = 2;
        TEST_ASSERT(evaluateIncomingDataDecision(st, d2) == RTPSReaderDataAction::Accept,
                    "gap-filling DATA SN=2 accepted");
        applyAcceptedDataToReaderState(st, 2);
        TEST_ASSERT(st.highestContiguousSeq == 3,
                    "contiguous collapses through held SN=3");
        TEST_ASSERT(st.receivedBitmap == 0, "bitmap empty after collapse");
    }

    //=================================================================
    // Reader runtime: HEARTBEAT -> ACKNACK decision
    //=================================================================
    {
        printf("Test: Reader HEARTBEAT -> ACKNACK decision\n");

        using namespace RaftRuntime::RTPS::Runtime::Reader;

        RTPSReaderWriterState st;
        st.reliabilityKind = RTPSReaderReliabilityKind::Reliable;

        // Writer announces [1..5]; reader has nothing -> ACKNACK base=1, 5 missing bits all zero.
        RTPSReaderHeartbeatFields hb;
        hb.firstSN = 1;
        hb.lastSN = 5;
        hb.count = 1;
        hb.finalFlag = false;
        hb.livelinessFlag = false;
        auto d = evaluateIncomingHeartbeatDecision(st, hb);
        TEST_ASSERT(d.sendAckNack, "HB announces gap, reader must ACKNACK");
        TEST_ASSERT(d.ackNackBase == 1, "ACKNACK base = 1 (first missing)");
        TEST_ASSERT(d.ackNackNumBits == 5, "ACKNACK numBits covers [1..5]");
        TEST_ASSERT(d.ackNackBitmap == 0x1F, "no SNs received -> all NACK bits set (5 missing)");
        TEST_ASSERT(d.ackNackCount == 1, "outgoing count starts at 1");

        // Apply and record SN=1,2,3 received.
        applyHeartbeatProcessedToReaderState(st, hb, /*ackNackSent*/ true);
        applyAcceptedDataToReaderState(st, 1);
        applyAcceptedDataToReaderState(st, 2);
        applyAcceptedDataToReaderState(st, 3);
        TEST_ASSERT(st.highestContiguousSeq == 3, "contiguous after 1,2,3 = 3");

        // Same HB again (count unchanged, FINAL clear) -> still NACK [4..5].
        hb.count = 1;
        d = evaluateIncomingHeartbeatDecision(st, hb);
        TEST_ASSERT(d.sendAckNack, "HB without FINAL still requires ACKNACK");
        TEST_ASSERT(d.ackNackBase == 4, "ACKNACK base = 4 (next missing)");
        TEST_ASSERT(d.ackNackNumBits == 2, "numBits covers [4..5]");
        TEST_ASSERT(d.ackNackBitmap == 0x3, "SNs 4,5 missing -> both NACK bits set");

        // HB with FINAL and same count -> no reply (writer already knows our state).
        hb.finalFlag = true;
        d = evaluateIncomingHeartbeatDecision(st, hb);
        TEST_ASSERT(!d.sendAckNack,
                    "duplicate FINAL heartbeat requires no reply");

        // Reader catches up to SN=5; HB with FINAL+same-count still quiet.
        applyAcceptedDataToReaderState(st, 4);
        applyAcceptedDataToReaderState(st, 5);
        d = evaluateIncomingHeartbeatDecision(st, hb);
        TEST_ASSERT(!d.sendAckNack,
                    "caught-up reader: FINAL duplicate HB produces no ACKNACK");

        // Writer advances count (new HB) with FINAL clear while reader fully caught up ->
        // empty confirmation ACKNACK base = lastSN+1, numBits = 0.
        hb.count = 2;
        hb.finalFlag = false;
        d = evaluateIncomingHeartbeatDecision(st, hb);
        TEST_ASSERT(d.sendAckNack, "new HB without FINAL elicits confirmation ACKNACK");
        TEST_ASSERT(d.ackNackBase == 6 && d.ackNackNumBits == 0,
                    "confirmation ACKNACK base=lastSN+1 with numBits=0");

        // Best-effort reader never ACKNACKs.
        RTPSReaderWriterState be;
        be.reliabilityKind = RTPSReaderReliabilityKind::BestEffort;
        hb.finalFlag = false;
        hb.count = 1;
        d = evaluateIncomingHeartbeatDecision(be, hb);
        TEST_ASSERT(!d.sendAckNack,
                    "best-effort reader never emits ACKNACK");
    }

    //=================================================================
    // Reader runner: HEARTBEAT/DATA submessage parsing + orchestration
    //=================================================================
    {
        printf("Test: Reader runner HEARTBEAT submessage parse\n");

        // HEARTBEAT content layout: readerEID(4) writerEID(4) firstSN(8) lastSN(8) count(4)
        uint8_t hbContent[28] = {0};
        // readerEID
        hbContent[0] = 0x00; hbContent[1] = 0x00; hbContent[2] = 0x01; hbContent[3] = 0x07;
        // writerEID
        hbContent[4] = 0x00; hbContent[5] = 0x00; hbContent[6] = 0x01; hbContent[7] = 0x02;
        // firstSN = 3 (LE, high then low)
        hbContent[8] = 0; hbContent[9] = 0; hbContent[10] = 0; hbContent[11] = 0;
        hbContent[12] = 3; hbContent[13] = 0; hbContent[14] = 0; hbContent[15] = 0;
        // lastSN = 7
        hbContent[16] = 0; hbContent[17] = 0; hbContent[18] = 0; hbContent[19] = 0;
        hbContent[20] = 7; hbContent[21] = 0; hbContent[22] = 0; hbContent[23] = 0;
        // count = 42
        hbContent[24] = 42; hbContent[25] = 0; hbContent[26] = 0; hbContent[27] = 0;

        RTPSReaderHeartbeatParsed parsed;
        TEST_ASSERT(RTPSReaderRunner_parseHeartbeat(hbContent, sizeof(hbContent),
                                                     /*flags*/ 0x01, parsed),
                    "HEARTBEAT parse succeeds at min length");
        TEST_ASSERT(parsed.fields.firstSN == 3, "firstSN decoded");
        TEST_ASSERT(parsed.fields.lastSN == 7, "lastSN decoded");
        TEST_ASSERT(parsed.fields.count == 42, "count decoded");
        TEST_ASSERT(!parsed.fields.finalFlag, "no FINAL flag");
        TEST_ASSERT(!parsed.fields.livelinessFlag, "no LIVELINESS flag");

        // FINAL flag (bit 1) + LIVELINESS flag (bit 2).
        TEST_ASSERT(RTPSReaderRunner_parseHeartbeat(hbContent, sizeof(hbContent),
                                                     /*flags*/ 0x01 | 0x02 | 0x04, parsed),
                    "HEARTBEAT parse with FINAL+LIVELINESS");
        TEST_ASSERT(parsed.fields.finalFlag, "FINAL flag propagated");
        TEST_ASSERT(parsed.fields.livelinessFlag, "LIVELINESS flag propagated");

        // Short content rejected.
        TEST_ASSERT(!RTPSReaderRunner_parseHeartbeat(hbContent, 27, 0x01, parsed),
                    "short HEARTBEAT content rejected");
    }

    {
        printf("Test: Reader runner DATA submessage parse\n");

        // DATA content layout (D-flag set, no inline QoS):
        // extraFlags(2) octetsToInlineQos(2) readerEID(4) writerEID(4) writerSN(8) [payload]
        // octetsToInlineQos = 16 (from start of readerEID to start of payload when no inline QoS)
        const uint8_t payloadBytes[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
        uint8_t dataContent[20 + sizeof(payloadBytes)] = {0};
        // extraFlags = 0
        dataContent[0] = 0; dataContent[1] = 0;
        // octetsToInlineQos = 16
        dataContent[2] = 16; dataContent[3] = 0;
        // readerEID
        dataContent[4] = 0x00; dataContent[5] = 0x00; dataContent[6] = 0x00; dataContent[7] = 0x04;
        // writerEID
        dataContent[8] = 0x00; dataContent[9] = 0x00; dataContent[10] = 0x00; dataContent[11] = 0x03;
        // writerSN = 99
        dataContent[12] = 0; dataContent[13] = 0; dataContent[14] = 0; dataContent[15] = 0;
        dataContent[16] = 99; dataContent[17] = 0; dataContent[18] = 0; dataContent[19] = 0;
        // payload
        memcpy(dataContent + 20, payloadBytes, sizeof(payloadBytes));

        RTPSReaderDataParsed parsed;
        // flags: E=1, D=1 (0x01 | 0x04 = 0x05)
        TEST_ASSERT(RTPSReaderRunner_parseData(dataContent, sizeof(dataContent),
                                                /*flags*/ 0x05, parsed),
                    "DATA parse succeeds with D flag");
        TEST_ASSERT(parsed.fields.writerSeqNum == 99, "writerSN decoded");
        TEST_ASSERT(parsed.payload != nullptr, "payload pointer set");
        TEST_ASSERT(parsed.payloadLen == sizeof(payloadBytes), "payload length correct");
        TEST_ASSERT(parsed.payload[0] == 0xDE && parsed.payload[3] == 0xEF,
                    "payload bytes match source buffer");

        // No D flag -> no payload exposed.
        TEST_ASSERT(RTPSReaderRunner_parseData(dataContent, sizeof(dataContent),
                                                /*flags*/ 0x01, parsed),
                    "DATA parse without D flag still succeeds (header only)");
        TEST_ASSERT(parsed.payload == nullptr && parsed.payloadLen == 0,
                    "no D flag -> no payload");

        // Short content rejected.
        TEST_ASSERT(!RTPSReaderRunner_parseData(dataContent, 19, 0x05, parsed),
                    "short DATA content rejected");
    }

    {
        printf("Test: Reader runner orchestration - HEARTBEAT dispatches to state + sends ACKNACK\n");

        using namespace RaftRuntime::RTPS::Runtime::Reader;

        struct Ctx {
            RTPSReaderWriterState state;
            int resolveCalls = 0;
            int sendCalls = 0;
            int observerCalls = 0;
            RTPSReaderHeartbeatDecision lastDecision{};
            const uint8_t* lastWriterEID = nullptr;
        } ctx;
        ctx.state.reliabilityKind = RTPSReaderReliabilityKind::Reliable;

        RTPSReaderRunnerCallbacks cb;
        cb.resolveState = [](void* u, const uint8_t*, const uint8_t* wEID) -> RTPSReaderWriterState* {
            auto* c = static_cast<Ctx*>(u);
            c->resolveCalls++;
            c->lastWriterEID = wEID;
            return &c->state;
        };
        cb.sendAckNack = [](void* u, const uint8_t*, const uint8_t*, const uint8_t*,
                             const RTPSReaderHeartbeatDecision& d) {
            auto* c = static_cast<Ctx*>(u);
            c->sendCalls++;
            c->lastDecision = d;
        };
        cb.onHeartbeatParsed = [](void* u, const RTPSReaderHeartbeatParsed&,
                                   const RTPSReaderHeartbeatDecision&) {
            static_cast<Ctx*>(u)->observerCalls++;
        };

        // Build HB content announcing SNs [1..3], count=1, no FINAL.
        uint8_t hbContent[28] = {0};
        hbContent[6] = 0x01; hbContent[7] = 0x02; // writerEID marker
        hbContent[12] = 1;  // firstSN low = 1
        hbContent[20] = 3;  // lastSN low = 3
        hbContent[24] = 1;  // count = 1

        uint8_t srcPrefix[12] = {0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0, 0, 0, 0, 0};
        RTPSReaderRunner_onHeartbeat(srcPrefix, hbContent, sizeof(hbContent),
                                      /*flags*/ 0x01, cb, &ctx);

        TEST_ASSERT(ctx.resolveCalls == 1, "resolveState called once");
        TEST_ASSERT(ctx.sendCalls == 1, "ACKNACK send invoked (gap detected)");
        TEST_ASSERT(ctx.observerCalls == 1, "observer invoked");
        TEST_ASSERT(ctx.lastWriterEID == hbContent + 4, "writerEID points into content");
        TEST_ASSERT(ctx.lastDecision.ackNackBase == 1, "ACKNACK base = 1");
        TEST_ASSERT(ctx.lastDecision.ackNackNumBits == 3, "ACKNACK numBits = 3");
        TEST_ASSERT(ctx.state.lastHeartbeatCount == 1, "HB count recorded in state");
        TEST_ASSERT(ctx.state.outgoingAckNackCount == 1, "outgoing ACKNACK count advanced");

        // Same HB again with FINAL -> no send (duplicate), but state still touched.
        ctx.sendCalls = 0;
        hbContent[24] = 1; // count unchanged
        RTPSReaderRunner_onHeartbeat(srcPrefix, hbContent, sizeof(hbContent),
                                      /*flags*/ 0x01 | 0x02, cb, &ctx);
        TEST_ASSERT(ctx.sendCalls == 0, "duplicate FINAL HB -> no ACKNACK send");

        // Null resolver -> early exit, no send.
        ctx.resolveCalls = 0;
        ctx.sendCalls = 0;
        RTPSReaderRunnerCallbacks cb2 = cb;
        cb2.resolveState = [](void*, const uint8_t*, const uint8_t*) -> RTPSReaderWriterState* { return nullptr; };
        RTPSReaderRunner_onHeartbeat(srcPrefix, hbContent, sizeof(hbContent), 0x01, cb2, &ctx);
        TEST_ASSERT(ctx.sendCalls == 0, "null state -> no ACKNACK");
    }

    {
        printf("Test: Reader runner orchestration - DATA accept/dedup dispatches to user\n");

        using namespace RaftRuntime::RTPS::Runtime::Reader;

        struct Ctx {
            RTPSReaderWriterState state;
            int acceptDispatches = 0;
            RTPSReaderDataAction lastAction = RTPSReaderDataAction::Drop;
            uint64_t lastAcceptedSN = 0;
        } ctx;

        RTPSReaderRunnerCallbacks cb;
        cb.resolveState = [](void* u, const uint8_t*, const uint8_t*) -> RTPSReaderWriterState* {
            return &static_cast<Ctx*>(u)->state;
        };
        cb.dispatchData = [](void* u, const uint8_t*, const RTPSReaderDataParsed& p) {
            auto* c = static_cast<Ctx*>(u);
            c->acceptDispatches++;
            c->lastAcceptedSN = p.fields.writerSeqNum;
        };
        cb.onDataDecision = [](void* u, const RTPSReaderDataParsed&, RTPSReaderDataAction a) {
            static_cast<Ctx*>(u)->lastAction = a;
        };

        // Build DATA content for SN=1 with 4-byte payload.
        uint8_t dataContent[20 + 4] = {0};
        dataContent[2] = 16; // octetsToInlineQos
        dataContent[16] = 1; // writerSN low = 1
        dataContent[20] = 0xAB;
        dataContent[21] = 0xCD;
        dataContent[22] = 0xEF;
        dataContent[23] = 0x01;

        uint8_t srcPrefix[12] = {0};
        // flags: E | D
        RTPSReaderRunner_onData(srcPrefix, dataContent, sizeof(dataContent), 0x05, cb, &ctx);
        TEST_ASSERT(ctx.lastAction == RTPSReaderDataAction::Accept, "first DATA accepted");
        TEST_ASSERT(ctx.acceptDispatches == 1, "dispatchData called once");
        TEST_ASSERT(ctx.lastAcceptedSN == 1, "dispatched SN = 1");
        TEST_ASSERT(ctx.state.highestContiguousSeq == 1, "state advanced");

        // Re-send same SN -> dedup, no dispatch.
        RTPSReaderRunner_onData(srcPrefix, dataContent, sizeof(dataContent), 0x05, cb, &ctx);
        TEST_ASSERT(ctx.lastAction == RTPSReaderDataAction::Dedup, "replay -> Dedup");
        TEST_ASSERT(ctx.acceptDispatches == 1, "dedup does not re-dispatch");
    }

    //=================================================================
    // ACKNACK wire builder with explicit bitmap (DDSI-RTPS §9.4.2.7)
    //=================================================================
    {
        printf("Test: ACKNACK with bitmap - empty bitmap equivalence\n");

        const uint8_t readerEID[4] = {0x00, 0x00, 0x01, 0x07};
        const uint8_t writerEID[4] = {0x00, 0x00, 0x01, 0x02};

        uint8_t bufA[64] = {0};
        uint8_t bufB[64] = {0};

        // Existing empty-bitmap builder
        uint32_t lenA = RTPSMessage::writeAcknack(bufA, sizeof(bufA),
                                                   readerEID, writerEID,
                                                   /*baseHigh*/ 0, /*baseLow*/ 6,
                                                   /*count*/ 99);
        // New bitmap-capable builder with numBits=0, finalFlag=false
        uint32_t lenB = RTPSMessage::writeAcknackWithBitmap(bufB, sizeof(bufB),
                                                             readerEID, writerEID,
                                                             0, 6,
                                                             /*numBits*/ 0, nullptr,
                                                             /*count*/ 99,
                                                             /*finalFlag*/ false);
        TEST_ASSERT(lenA == 28, "legacy empty ACKNACK is 28 bytes");
        TEST_ASSERT(lenB == lenA, "bitmap-capable builder matches legacy length for numBits=0");
        TEST_ASSERT(memcmp(bufA, bufB, lenA) == 0, "byte-for-byte identical to legacy");
    }

    {
        printf("Test: ACKNACK with bitmap - payload contents and word packing\n");

        const uint8_t readerEID[4] = {0x00, 0x00, 0x00, 0x04};
        const uint8_t writerEID[4] = {0x00, 0x00, 0x00, 0x03};

        // numBits=5 so only one bitmap word is emitted. Pattern 0b10110 => NACK SN base+1,2,4.
        uint32_t bitmapWords[1] = { 0b10110u };

        uint8_t buf[64] = {0};
        uint32_t len = RTPSMessage::writeAcknackWithBitmap(buf, sizeof(buf),
                                                            readerEID, writerEID,
                                                            /*baseHigh*/ 0, /*baseLow*/ 10,
                                                            /*numBits*/ 5, bitmapWords,
                                                            /*count*/ 7,
                                                            /*finalFlag*/ false);
        // submsg hdr (4) + reader (4) + writer (4) + base (8) + numBits (4) + 1 word (4) + count (4) = 32
        TEST_ASSERT(len == 32, "numBits=5 -> one bitmap word -> 32 bytes");
        TEST_ASSERT(buf[0] == SUBMSG_ACKNACK, "submsg id = ACKNACK");
        TEST_ASSERT(buf[1] == 0x01, "E=1, F=0");
        // octetsToNextHeader (LE16) at offset 2..3 = 28
        TEST_ASSERT(buf[2] == 28 && buf[3] == 0, "octetsToNextHeader = 28");
        // Base LE64 at offset 12..19: high=0 @12, low=10 @16
        TEST_ASSERT(buf[12] == 0 && buf[16] == 10, "bitmap base encoded LE");
        // numBits LE32 at offset 20..23 = 5
        TEST_ASSERT(buf[20] == 5 && buf[21] == 0 && buf[22] == 0 && buf[23] == 0, "numBits = 5");
        // bitmap word at offset 24..27 = 0b10110 = 0x16
        TEST_ASSERT(buf[24] == 0x16 && buf[25] == 0 && buf[26] == 0 && buf[27] == 0,
                    "bitmap word packed little-endian");
        // count at offset 28..31 = 7
        TEST_ASSERT(buf[28] == 7 && buf[29] == 0, "count encoded LE");
    }

    {
        printf("Test: ACKNACK with bitmap - FINAL flag + multi-word bitmap\n");

        const uint8_t readerEID[4] = {0x00, 0x00, 0x00, 0x04};
        const uint8_t writerEID[4] = {0x00, 0x00, 0x00, 0x03};

        // numBits=33 forces two bitmap words.
        uint32_t bitmapWords[2] = { 0xAABBCCDDu, 0x00000001u };

        uint8_t buf[80] = {0};
        uint32_t len = RTPSMessage::writeAcknackWithBitmap(buf, sizeof(buf),
                                                            readerEID, writerEID,
                                                            0, 100,
                                                            /*numBits*/ 33, bitmapWords,
                                                            /*count*/ 1,
                                                            /*finalFlag*/ true);
        // 4 hdr + 4+4+8+4 + 2*4 + 4 = 36
        TEST_ASSERT(len == 36, "numBits=33 -> two bitmap words -> 36 bytes");
        TEST_ASSERT((buf[1] & 0x02) != 0, "FINAL flag (F=1) set");
        // First bitmap word at offset 24..27 = 0xAABBCCDD (LE)
        TEST_ASSERT(buf[24] == 0xDD && buf[25] == 0xCC && buf[26] == 0xBB && buf[27] == 0xAA,
                    "word 0 LE packing");
        // Second bitmap word at offset 28..31 = 0x00000001
        TEST_ASSERT(buf[28] == 0x01 && buf[29] == 0 && buf[30] == 0 && buf[31] == 0,
                    "word 1 LE packing");
    }

    {
        printf("Test: ACKNACK with bitmap - input validation\n");

        const uint8_t readerEID[4] = {0};
        const uint8_t writerEID[4] = {0};
        uint32_t word = 0;

        uint8_t small[20] = {0};
        TEST_ASSERT(RTPSMessage::writeAcknackWithBitmap(small, sizeof(small),
                                                         readerEID, writerEID,
                                                         0, 1, 0, nullptr, 1, false) == 0,
                    "buffer too small -> 0");

        uint8_t buf[64] = {0};
        TEST_ASSERT(RTPSMessage::writeAcknackWithBitmap(buf, sizeof(buf),
                                                         readerEID, writerEID,
                                                         0, 1, /*numBits*/ 257, &word, 1, false) == 0,
                    "numBits > 256 rejected (RTPS spec cap)");
        TEST_ASSERT(RTPSMessage::writeAcknackWithBitmap(buf, sizeof(buf),
                                                         readerEID, writerEID,
                                                         0, 1, /*numBits*/ 1, /*bitmap*/ nullptr,
                                                         1, false) == 0,
                    "null bitmap with numBits>0 rejected");
    }

    //=================================================================
    // RX submessage runner: opt-in reader-runner delegation path
    //=================================================================
    {
        printf("Test: RX runner opt-in HEARTBEAT delegates to reader runtime + sends bitmap ACKNACK\n");

        using namespace RaftRuntime::RTPS::Runtime::Reader;

        // Build a complete RTPS packet: header + HEARTBEAT submessage.
        uint8_t pkt[128] = {0};
        uint8_t srcPrefix[12];
        for (int i = 0; i < 12; ++i) srcPrefix[i] = (uint8_t)(0xA0 + i);
        uint32_t pktLen = RTPSMessage::writeHeader(pkt, sizeof(pkt), srcPrefix);

        // HEARTBEAT: firstSN=1, lastSN=3, count=1, flags=E (0x01), no FINAL.
        const uint8_t readerEIDbytes[4] = { 0x00, 0x00, 0x00, 0x00 };
        const uint8_t writerEIDbytes[4] = { 0x00, 0x00, 0x12, 0x02 };
        uint32_t hbLen = RTPSMessage::writeHeartbeat(
            pkt + pktLen, sizeof(pkt) - pktLen,
            readerEIDbytes, writerEIDbytes,
            /*firstSNHigh*/ 0, /*firstSNLow*/ 1,
            /*lastSNHigh*/ 0, /*lastSNLow*/ 3,
            /*count*/ 1);
        TEST_ASSERT(hbLen == 32, "HEARTBEAT submessage built");
        pktLen += hbLen;

        struct Ctx {
            RTPSReaderWriterState state;
            int resolveCalls = 0;
            int sendCalls = 0;
            std::vector<uint8_t> lastAck;
            uint8_t ourReader[4] = { 0x00, 0x00, 0x12, 0x07 };
            uint8_t localPrefix[12] = {0};
        } ctx;
        ctx.state.reliabilityKind = RTPSReaderReliabilityKind::Reliable;
        for (int i = 0; i < 12; ++i) ctx.localPrefix[i] = (uint8_t)(0x50 + i);

        RTPSRxSubmessageRunnerCallbacks cb{};
        cb.getLocalGuidPrefix = [](void* u) -> const uint8_t* {
            return static_cast<Ctx*>(u)->localPrefix;
        };
        cb.resolveReaderEID = [](void* u, RTPSRxChannel, const uint8_t*, const uint8_t*) -> const uint8_t* {
            return static_cast<Ctx*>(u)->ourReader;
        };
        cb.resolveAckDest = [](void*, RTPSRxChannel, const struct sockaddr_in&, struct sockaddr_in& out) -> bool {
            out = {};
            out.sin_family = AF_INET;
            return true;
        };
        cb.sendAck = [](void* u, const uint8_t* buf, uint32_t len, const struct sockaddr_in&) -> int {
            auto* c = static_cast<Ctx*>(u);
            c->sendCalls++;
            c->lastAck.assign(buf, buf + len);
            return (int)len;
        };
        cb.resolveReaderWriterState = [](void* u, RTPSRxChannel, const uint8_t*, const uint8_t*)
            -> RTPSReaderWriterState* {
            auto* c = static_cast<Ctx*>(u);
            c->resolveCalls++;
            return &c->state;
        };

        struct sockaddr_in from = {};
        from.sin_family = AF_INET;
        uint32_t ackCount = 0;
        bool ok = RTPSRxSubmessageRunner_run(pkt, pktLen, from,
                                              RTPSRxChannel::UserData,
                                              ackCount, cb, &ctx);
        TEST_ASSERT(ok, "packet parsed");
        TEST_ASSERT(ctx.resolveCalls == 1, "resolveReaderWriterState called once");
        TEST_ASSERT(ctx.sendCalls == 1, "ACKNACK sent via opt-in path");
        TEST_ASSERT(ackCount == 1, "acknackCount advanced to 1 from decision");
        TEST_ASSERT(ctx.state.lastHeartbeatCount == 1, "reader state HB count recorded");
        TEST_ASSERT(ctx.state.outgoingAckNackCount == 1, "outgoing count incremented in state");

        // Sanity-check the wire layout: opt-in path uses writeAcknackWithBitmap.
        // Packet = RTPS header (20) + INFO_DST (16) + ACKNACK with 3-bit bitmap.
        // ACKNACK header is 4 bytes then content; numBits=3 => 1 bitmap word.
        TEST_ASSERT(ctx.lastAck.size() >= 20 + 16 + 32, "ACK buffer includes hdr+INFO_DST+ACKNACK");
        const uint8_t* acksubmsg = ctx.lastAck.data() + 20 + 16;
        TEST_ASSERT(acksubmsg[0] == SUBMSG_ACKNACK, "submsg id = ACKNACK");
        // numBits at offset 4+4+4+8 = 20 of submsg (4-byte hdr + reader+writer+base)
        const uint8_t* content = acksubmsg + 4;
        uint32_t numBits = (uint32_t)content[16] | ((uint32_t)content[17] << 8) |
                           ((uint32_t)content[18] << 16) | ((uint32_t)content[19] << 24);
        TEST_ASSERT(numBits == 3, "ACKNACK numBits = 3 (covers [1..3] missing)");

        // Second call with the SAME HEARTBEAT (count=1) and now FINAL (flags bit 1) -> no send.
        // Re-encode HEARTBEAT flags byte (first byte after submsg id).
        // pkt + 20 is submsg id, pkt + 21 is flags byte.
        pkt[20 + 1] = 0x01 | 0x02;  // E | F
        ctx.sendCalls = 0;
        ok = RTPSRxSubmessageRunner_run(pkt, pktLen, from,
                                         RTPSRxChannel::UserData,
                                         ackCount, cb, &ctx);
        TEST_ASSERT(ok && ctx.sendCalls == 0,
                    "duplicate FINAL HB via opt-in path -> no ACKNACK emitted");
    }

    {
        printf("Test: RX runner legacy path still used when resolveReaderWriterState is null\n");

        // Build packet as before.
        uint8_t pkt[128] = {0};
        uint8_t srcPrefix[12]; for (int i = 0; i < 12; ++i) srcPrefix[i] = (uint8_t)(0x70 + i);
        uint32_t pktLen = RTPSMessage::writeHeader(pkt, sizeof(pkt), srcPrefix);
        const uint8_t r[4] = { 0 };
        const uint8_t w[4] = { 0x00, 0x00, 0x12, 0x02 };
        pktLen += RTPSMessage::writeHeartbeat(pkt + pktLen, sizeof(pkt) - pktLen,
                                                r, w, 0, 1, 0, 3, 1);

        struct Ctx {
            int sendCalls = 0;
            uint8_t ourReader[4] = { 0x00, 0x00, 0x12, 0x07 };
            uint8_t localPrefix[12] = {0};
        } ctx;

        RTPSRxSubmessageRunnerCallbacks cb{};
        cb.getLocalGuidPrefix = [](void* u) -> const uint8_t* {
            return static_cast<Ctx*>(u)->localPrefix;
        };
        cb.resolveReaderEID = [](void* u, RTPSRxChannel, const uint8_t*, const uint8_t*) -> const uint8_t* {
            return static_cast<Ctx*>(u)->ourReader;
        };
        cb.resolveAckDest = [](void*, RTPSRxChannel, const struct sockaddr_in&, struct sockaddr_in& out) -> bool {
            out = {};
            out.sin_family = AF_INET;
            return true;
        };
        cb.sendAck = [](void* u, const uint8_t*, uint32_t len, const struct sockaddr_in&) -> int {
            static_cast<Ctx*>(u)->sendCalls++;
            return (int)len;
        };
        // resolveReaderWriterState intentionally left null.

        struct sockaddr_in from = {};
        from.sin_family = AF_INET;
        uint32_t ackCount = 5;
        bool ok = RTPSRxSubmessageRunner_run(pkt, pktLen, from,
                                              RTPSRxChannel::UserData,
                                              ackCount, cb, &ctx);
        TEST_ASSERT(ok && ctx.sendCalls == 1, "legacy path emits ACKNACK as before");
        TEST_ASSERT(ackCount == 6, "legacy path post-increments acknackCount (was 5 -> 6)");
    }

    //=================================================================
    // RTPSReaderStateMap get-or-create and lookup semantics
    //=================================================================
    {
        printf("Test: RTPSReaderStateMap getOrCreate / find\n");
        using namespace RaftRuntime::RTPS::Runtime::Reader;
        RTPSReaderStateMap map;
        TEST_ASSERT(map.size() == 0, "map is empty on construction");

        uint8_t gpA[12] = { 1,2,3,4,5,6,7,8,9,10,11,12 };
        uint8_t gpB[12] = { 9,9,9,9,9,9,9,9,9,9,9,9 };
        uint8_t eidA[4] = { 0, 0, 0x12, 0x02 };
        uint8_t eidB[4] = { 0, 0, 0x34, 0x02 };

        TEST_ASSERT(map.find(gpA, eidA) == nullptr, "find returns null on miss");

        auto* s1 = map.getOrCreate(gpA, eidA);
        TEST_ASSERT(s1 != nullptr, "getOrCreate returns non-null");
        TEST_ASSERT(map.size() == 1, "size increments after create");
        s1->highestContiguousSeq = 42;

        auto* s1b = map.getOrCreate(gpA, eidA);
        TEST_ASSERT(s1b == s1, "getOrCreate returns same pointer for same key");
        TEST_ASSERT(s1b->highestContiguousSeq == 42, "existing state preserved on re-lookup");
        TEST_ASSERT(map.size() == 1, "size unchanged on re-lookup");

        auto* s2 = map.getOrCreate(gpA, eidB);
        TEST_ASSERT(s2 != s1 && map.size() == 2, "different writerEID creates new entry");

        auto* s3 = map.getOrCreate(gpB, eidA);
        TEST_ASSERT(s3 != s1 && s3 != s2 && map.size() == 3, "different guidPrefix creates new entry");

        TEST_ASSERT(map.find(gpA, eidA) == s1, "find returns inserted pointer");
        TEST_ASSERT(map.getOrCreate(nullptr, eidA) == nullptr, "null guidPrefix returns null");
        TEST_ASSERT(map.getOrCreate(gpA, nullptr) == nullptr, "null writerEID returns null");
    }

    //=================================================================
    // NACK bitmap wire encoding — LSB-internal to MSB-first wire
    // RTPS §9.4.2.7: bit i (SN bitmapBase+i) maps to bit (31 - i%32) of
    // bitmapWords[i/32], i.e. SN bitmapBase = MSB (0x80000000) of word 0.
    //=================================================================
    {
        printf("Test: RTPSRxSubmessageRunner NACK bitmap wire encoding\n");

        // Directly verify the bit-reversal by exercising evaluateIncomingHeartbeatDecision
        // (builds internal bitmap) then checking that after bit-reversal the expected
        // wire words come out correctly.
        //
        // Setup: reader has received nothing (highestContiguousSeq=0).
        // HB says firstSN=1, lastSN=3.  Reader should NACK SNs 1,2,3.
        // Internal bitmap (LSB-first): bits 0,1,2 set → 0x7 → 0x00000007
        // Wire (MSB-first): bits 0,1,2 → bit31,30,29 of word0 → 0xE0000000

        using namespace RaftRuntime::RTPS::Runtime::Reader;

        RTPSReaderWriterState state;
        state.reliabilityKind = RTPSReaderReliabilityKind::Reliable;

        RTPSReaderHeartbeatFields hb;
        hb.firstSN = 1;
        hb.lastSN  = 3;
        hb.count   = 1;
        hb.finalFlag = false;
        hb.livelinessFlag = false;

        auto decision = evaluateIncomingHeartbeatDecision(state, hb);
        TEST_ASSERT(decision.sendAckNack, "NACK bitmap: sendAckNack true");
        TEST_ASSERT(decision.ackNackBase == 1, "NACK bitmap: ackNackBase == 1");
        TEST_ASSERT(decision.ackNackNumBits == 3, "NACK bitmap: numBits == 3");
        TEST_ASSERT(decision.ackNackBitmap == 0x7u,
                    "NACK bitmap: internal bitmap == 0x7 (bits 0,1,2)");

        // Simulate the bit-reversal that emitReaderRunnerAckNack applies.
        auto bitrev32 = [](uint32_t x) -> uint32_t {
            x = ((x >>  1) & 0x55555555u) | ((x <<  1) & 0xAAAAAAAAu);
            x = ((x >>  2) & 0x33333333u) | ((x <<  2) & 0xCCCCCCCCu);
            x = ((x >>  4) & 0x0F0F0F0Fu) | ((x <<  4) & 0xF0F0F0F0u);
            x = ((x >>  8) & 0x00FF00FFu) | ((x <<  8) & 0xFF00FF00u);
            x = ( x >> 16              ) | ( x << 16              );
            return x;
        };
        uint32_t wireWord0 = bitrev32((uint32_t)(decision.ackNackBitmap & 0xFFFFFFFFu));
        TEST_ASSERT(wireWord0 == 0xE0000000u,
                    "NACK bitmap: wire word0 == 0xE0000000 (MSB-first, SNs 1,2,3)");

        // Also verify single-SN case: NACK only SN 1 → bitmapWords[0] = 0x80000000
        RTPSReaderHeartbeatFields hb1;
        hb1.firstSN = 1; hb1.lastSN = 1; hb1.count = 2;
        hb1.finalFlag = false; hb1.livelinessFlag = false;
        auto d1 = evaluateIncomingHeartbeatDecision(state, hb1);
        TEST_ASSERT(d1.ackNackBitmap == 0x1u, "NACK bitmap: single-SN internal bitmap == 1");
        TEST_ASSERT(bitrev32((uint32_t)d1.ackNackBitmap) == 0x80000000u,
                    "NACK bitmap: single-SN wire word0 == 0x80000000");
    }

    //=================================================================
    // RTPSInitialAnnouncePlan - ChatterReader SEDP subscription announce
    //=================================================================
    {
        printf("Test: RTPSInitialAnnouncePlan SedpChatterReader plumbing\n");

        // Default plan does NOT include ChatterReader - opt-in only.
        {
            RTPSInitialAnnouncePlan plan = RTPSInitialAnnouncePlan_default();
            TEST_ASSERT(!plan.sendSedpChatterReader, "default plan does not announce chatter reader");
            auto seq = RTPSInitialAnnouncePlan_buildSequence(
                plan, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle);
            bool seenReader = false;
            for (uint8_t i = 0; i < seq.numSteps; i++)
                if (seq.steps[i].action == RTPSInitialAnnounceAction::SedpChatterReader)
                    seenReader = true;
            TEST_ASSERT(!seenReader, "default sequence has no SedpChatterReader step");
        }

        // Opt-in: sequence includes ChatterReader step after ChatterWriter.
        {
            RTPSInitialAnnouncePlan plan = RTPSInitialAnnouncePlan_default();
            plan.sendSedpChatterReader = true;
            auto seq = RTPSInitialAnnouncePlan_buildSequence(
                plan, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle);
            int writerIdx = -1, readerIdx = -1;
            for (uint8_t i = 0; i < seq.numSteps; i++)
            {
                if (seq.steps[i].action == RTPSInitialAnnounceAction::SedpChatterWriter)
                    writerIdx = i;
                if (seq.steps[i].action == RTPSInitialAnnounceAction::SedpChatterReader)
                    readerIdx = i;
            }
            TEST_ASSERT(writerIdx >= 0 && readerIdx >= 0, "both chatter writer and reader scheduled");
            TEST_ASSERT(readerIdx == writerIdx + 1, "ChatterReader step immediately follows ChatterWriter");
            TEST_ASSERT(seq.steps[readerIdx].incrementHeartbeatBeforeSend,
                        "ChatterReader increments heartbeat before send on LinuxStyle");
            TEST_ASSERT(seq.steps[readerIdx].seqCounterHint == RTPSSeqCounterHint::SedpChatterReader,
                        "ChatterReader uses SedpChatterReader seq counter hint");
        }

        // Build-spec dispatch: ChatterReader -> SedpSubscription + ChatterReader profile.
        {
            auto spec = RTPSInitialAnnouncePlan_getBuildSpec(
                RTPSInitialAnnounceAction::SedpChatterReader);
            TEST_ASSERT(spec.buildKind == RTPSInitialAnnounceBuildKind::SedpSubscription,
                        "ChatterReader build kind is SedpSubscription");
            TEST_ASSERT(spec.sedpEndpointProfile == RTPSInitialAnnounceSedpEndpointProfile::ChatterReader,
                        "ChatterReader build spec references ChatterReader profile");
        }

        // Endpoint-spec dispatch: ChatterReader profile resolves to the correct entity/topic/type.
        {
            auto ep = RTPSInitialAnnouncePlan_getSedpEndpointSpec(
                RTPSInitialAnnounceSedpEndpointProfile::ChatterReader);
            TEST_ASSERT(ep.entityId != nullptr && memcmp(ep.entityId, ENTITYID_CHATTER_READER, 4) == 0,
                        "ChatterReader profile entity is ENTITYID_CHATTER_READER");
            TEST_ASSERT(ep.topicName != nullptr && strcmp(ep.topicName, CHATTER_IN_DDS_TOPIC) == 0,
                        "ChatterReader profile topic is rt/chatter_in");
            TEST_ASSERT(ep.typeName != nullptr && strcmp(ep.typeName, CHATTER_IN_DDS_TYPE) == 0,
                        "ChatterReader profile type is std_msgs::msg::dds_::String_");
            TEST_ASSERT(ep.reliabilityKind == RELIABILITY_RELIABLE,
                        "ChatterReader profile is RELIABLE");
        }

        // Send-target dispatch: ChatterReader uses metatraffic unicast (same as SEDP pub/sub).
        {
            auto tgt = RTPSInitialAnnouncePlan_getSendTarget(
                RTPSInitialAnnounceAction::SedpChatterReader);
            TEST_ASSERT(tgt.socket == RTPSInitialAnnounceSocket::Metatraffic,
                        "ChatterReader sends on metatraffic socket");
            TEST_ASSERT(tgt.addressing == RTPSInitialAnnounceAddressing::RemoteMetatrafficUnicast,
                        "ChatterReader addresses remote metatraffic unicast");
        }

        // Seq counter hint: ChatterReader uses its own counter (not chatter writer's).
        {
            RTPSInitialAnnounceStep step{};
            step.seqCounterHint = RTPSSeqCounterHint::SedpChatterReader;
            RTPSInitialAnnounceCounterState counters{};
            counters.sedpRosReaderSeqNum = 1;
            counters.sedpChatterReaderSeqNum = 7;
            counters.sedpChatterWriterSeqNum = 99;
            // EspStyle: dedicated chatter-reader counter.
            uint64_t seqEsp = RTPSInitialAnnouncePlan_applySequencePolicy(
                step, RTPSInitialAnnounceRuntimeFlavor::EspStyle, counters);
            TEST_ASSERT(seqEsp == 7, "ChatterReader seq counter returns sedpChatterReaderSeqNum on EspStyle");
            // LinuxStyle: mirrors chatter-writer policy - sample #(sedpRosReaderSeqNum+1) on SEDP subs writer.
            uint64_t seqLin = RTPSInitialAnnouncePlan_applySequencePolicy(
                step, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle, counters);
            TEST_ASSERT(seqLin == 2, "ChatterReader seq counter returns sedpRosReaderSeqNum+1 on LinuxStyle");
        }

        // Log-spec: ChatterReader has a distinct label from ChatterWriter.
        {
            auto ls = RTPSInitialAnnouncePlan_getLogSpec(
                RTPSInitialAnnounceAction::SedpChatterReader);
            TEST_ASSERT(ls.enabled && ls.actionLabel != nullptr, "ChatterReader has log label");
            TEST_ASSERT(strcmp(ls.actionLabel, "SEDP chatter sub") == 0,
                        "ChatterReader label is 'SEDP chatter sub'");
        }

        // Legacy default plan still produces the pre-existing ChatterWriter step (regression guard).
        {
            RTPSInitialAnnouncePlan plan = RTPSInitialAnnouncePlan_default();
            auto seq = RTPSInitialAnnouncePlan_buildSequence(
                plan, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle);
            bool seenWriter = false;
            for (uint8_t i = 0; i < seq.numSteps; i++)
                if (seq.steps[i].action == RTPSInitialAnnounceAction::SedpChatterWriter)
                    seenWriter = true;
            TEST_ASSERT(seenWriter, "default plan still schedules ChatterWriter");
        }
    }

    //=================================================================
    // RTPSInitialAnnounceRunner_runStep - per-step execution (for spreading
    // the initial-announce burst across multiple scheduler ticks).
    //=================================================================
    {
        printf("Test: RTPSInitialAnnounceRunner_runStep per-step execution\n");

        // Build a non-trivial sequence (default plan, LinuxStyle for deterministic counters).
        RTPSInitialAnnouncePlan plan = RTPSInitialAnnouncePlan_default();
        plan.sendSedpChatterReader = true;
        auto seq = RTPSInitialAnnouncePlan_buildSequence(
            plan, RTPSInitialAnnounceRuntimeFlavor::LinuxStyle);
        TEST_ASSERT(seq.numSteps > 1, "sequence has multiple steps to iterate");

        struct TestCtx
        {
            int sendCount = 0;
            int buildCount = 0;
            uint32_t lastPayloadLen = 0;
            uint8_t lastStepIdxSeen = 0xFF;
        };

        auto makeCallbacks = []()
        {
            RTPSInitialAnnounceRunnerCallbacks cb;
            cb.buildPayload = [](void* userCtx, const RTPSInitialAnnounceStep&,
                                 uint64_t, uint32_t, uint32_t) -> uint32_t
            {
                static_cast<TestCtx*>(userCtx)->buildCount++;
                return 64; // non-zero so send fires
            };
            cb.sendPayload = [](void* userCtx, RTPSInitialAnnounceAction, uint32_t payloadLen) -> int
            {
                auto* c = static_cast<TestCtx*>(userCtx);
                c->sendCount++;
                c->lastPayloadLen = payloadLen;
                return (int)payloadLen;
            };
            return cb;
        };

        // (a) runStep returns false when stepIdx >= numSteps.
        {
            TestCtx ctx;
            RTPSInitialAnnounceRunnerContext runCtx;
            runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::LinuxStyle;
            auto cb = makeCallbacks();
            bool inRange = RTPSInitialAnnounceRunner_runStep(seq, seq.numSteps, runCtx, cb, &ctx);
            TEST_ASSERT(!inRange, "runStep returns false when stepIdx out of range");
            TEST_ASSERT(ctx.sendCount == 0, "out-of-range runStep does not send");
        }

        // (b) runStep iterated over all indices produces the same send count as _run.
        int refSendCount = 0;
        {
            TestCtx ctx;
            RTPSInitialAnnounceRunnerContext runCtx;
            runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::LinuxStyle;
            auto cb = makeCallbacks();
            RTPSInitialAnnounceRunner_run(seq, runCtx, cb, &ctx);
            refSendCount = ctx.sendCount;
            TEST_ASSERT(refSendCount > 0, "reference _run sent at least one payload");
        }
        {
            TestCtx ctx;
            RTPSInitialAnnounceRunnerContext runCtx;
            runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::LinuxStyle;
            auto cb = makeCallbacks();
            for (uint8_t i = 0; i < seq.numSteps; i++)
                TEST_ASSERT(RTPSInitialAnnounceRunner_runStep(seq, i, runCtx, cb, &ctx),
                            "runStep returns true for in-range index");
            TEST_ASSERT(ctx.sendCount == refSendCount,
                        "runStep iteration sends same total as _run");
        }

        // (c) runCtx.previousPayloadLen is updated between runStep calls
        //     (important for ReusePrevious-kind steps in the sequence).
        {
            TestCtx ctx;
            RTPSInitialAnnounceRunnerContext runCtx;
            runCtx.runtimeFlavor = RTPSInitialAnnounceRuntimeFlavor::LinuxStyle;
            auto cb = makeCallbacks();
            TEST_ASSERT(runCtx.previousPayloadLen == 0, "previousPayloadLen starts at 0");
            RTPSInitialAnnounceRunner_runStep(seq, 0, runCtx, cb, &ctx);
            TEST_ASSERT(runCtx.previousPayloadLen == 64,
                        "previousPayloadLen reflects last buildPayload return across runStep calls");
        }
    }

    //=================================================================
    // RTPSWriterHeartbeatRunner_runStep - per-step execution for spreading
    // the periodic writer-heartbeat pass across multiple scheduler ticks.
    //=================================================================
    {
        printf("Test: RTPSWriterHeartbeatRunner_runStep per-step execution\n");

        const auto seq = RTPSWriterHeartbeatRunner_buildSequence(
            RTPSWriterHeartbeatRuntimeFlavor::EspStyle);
        TEST_ASSERT(seq.numSteps > 1, "HB sequence has multiple steps to iterate");

        struct HbCtx
        {
            int buildCount = 0;
            int sendCount = 0;
            uint32_t lastHeartbeatCount = 0;
        };

        auto makeCallbacks = []()
        {
            RTPSWriterHeartbeatRunnerCallbacks cb;
            cb.buildPayload = [](void* userCtx, RTPSWriterHeartbeatAction,
                                 uint64_t, uint32_t heartbeatCount) -> uint32_t
            {
                auto* c = static_cast<HbCtx*>(userCtx);
                c->buildCount++;
                c->lastHeartbeatCount = heartbeatCount;
                return 48;
            };
            cb.sendPayload = [](void* userCtx, RTPSWriterHeartbeatSendTarget, uint32_t payloadLen) -> int
            {
                static_cast<HbCtx*>(userCtx)->sendCount++;
                return (int)payloadLen;
            };
            return cb;
        };

        // (a) Out-of-range returns false and does not call buildPayload/sendPayload.
        {
            HbCtx ctx;
            RTPSWriterHeartbeatCounterState counters;
            auto cb = makeCallbacks();
            bool inRange = RTPSWriterHeartbeatRunner_runStep(
                RTPSWriterHeartbeatRuntimeFlavor::EspStyle,
                seq, seq.numSteps, counters, cb, &ctx);
            TEST_ASSERT(!inRange, "runStep returns false when stepIdx out of range");
            TEST_ASSERT(ctx.buildCount == 0 && ctx.sendCount == 0,
                        "out-of-range runStep does not invoke callbacks");
        }

        // (b) Iterated runStep matches _run send/build totals.
        int refBuild = 0, refSend = 0;
        uint32_t refHb = 0, refLive = 0;
        {
            HbCtx ctx;
            RTPSWriterHeartbeatCounterState counters;
            auto cb = makeCallbacks();
            RTPSWriterHeartbeatRunner_run(
                RTPSWriterHeartbeatRuntimeFlavor::EspStyle, counters, cb, &ctx);
            refBuild = ctx.buildCount;
            refSend = ctx.sendCount;
            refHb = counters.heartbeatCount;
            refLive = (uint32_t)counters.livelinessSeqNum;
            TEST_ASSERT(refSend > 0, "reference _run sent at least one payload");
        }
        {
            HbCtx ctx;
            RTPSWriterHeartbeatCounterState counters;
            auto cb = makeCallbacks();
            for (uint8_t i = 0; i < seq.numSteps; i++)
                TEST_ASSERT(RTPSWriterHeartbeatRunner_runStep(
                                RTPSWriterHeartbeatRuntimeFlavor::EspStyle,
                                seq, i, counters, cb, &ctx),
                            "runStep returns true for in-range index");
            TEST_ASSERT(ctx.buildCount == refBuild,
                        "iterated runStep build count matches _run");
            TEST_ASSERT(ctx.sendCount == refSend,
                        "iterated runStep send count matches _run");
            TEST_ASSERT(counters.heartbeatCount == refHb,
                        "iterated runStep heartbeatCount matches _run");
            TEST_ASSERT((uint32_t)counters.livelinessSeqNum == refLive,
                        "iterated runStep livelinessSeqNum matches _run");
        }

        // (c) Counter mutations (heartbeatCount, livelinessSeqNum) are per-step:
        //     only steps that request the increment actually bump the counters.
        {
            HbCtx ctx;
            RTPSWriterHeartbeatCounterState counters;
            auto cb = makeCallbacks();
            uint32_t prevHb = counters.heartbeatCount;
            uint64_t prevLive = counters.livelinessSeqNum;
            for (uint8_t i = 0; i < seq.numSteps; i++)
            {
                const auto& step = seq.steps[i];
                const uint32_t expectedHb =
                    prevHb + (step.incrementHeartbeatBeforeBuild ? 1 : 0);
                const uint64_t expectedLive =
                    prevLive + (step.incrementLivelinessSeqBeforeBuild ? 1 : 0);
                RTPSWriterHeartbeatRunner_runStep(
                    RTPSWriterHeartbeatRuntimeFlavor::EspStyle,
                    seq, i, counters, cb, &ctx);
                TEST_ASSERT(counters.heartbeatCount == expectedHb,
                            "heartbeatCount bumped only for steps that request it");
                TEST_ASSERT(counters.livelinessSeqNum == expectedLive,
                            "livelinessSeqNum bumped only for steps that request it");
                prevHb = counters.heartbeatCount;
                prevLive = counters.livelinessSeqNum;
            }
        }
    }

    //=================================================================
    // decodeStdMsgsString: CDR-encapsulated std_msgs/String decode
    //=================================================================
    {
        printf("Test: RTPSUserDispatch::decodeStdMsgsString\n");
        using namespace RaftRuntime::RTPS::Runtime::UserDispatch;

        // Build a reference CDR-LE std_msgs/String payload: "hello"
        // Layout: 4-byte encap header (0x00 0x01 0x00 0x00) + uint32 LE length (incl null)
        //       + string bytes + null + pad to 4.
        uint8_t pkt[32] = {0};
        uint32_t pos = 0;
        pkt[pos++] = 0x00; pkt[pos++] = 0x01; pkt[pos++] = 0x00; pkt[pos++] = 0x00;
        const char* text = "hello";
        uint32_t textLenPlusNull = 6;
        pkt[pos++] = textLenPlusNull & 0xFF;
        pkt[pos++] = (textLenPlusNull >> 8) & 0xFF;
        pkt[pos++] = 0; pkt[pos++] = 0;
        memcpy(pkt + pos, text, 5); pos += 5;
        pkt[pos++] = 0x00; // null terminator

        char out[16] = {0};
        auto r = decodeStdMsgsString(pkt, pos, out, sizeof(out));
        TEST_ASSERT(r.success, "decode succeeds on well-formed payload");
        TEST_ASSERT(r.textLen == 5, "textLen excludes null terminator");
        TEST_ASSERT(strcmp(out, "hello") == 0, "decoded string matches input");

        // Null outBuf
        char* nullOut = nullptr;
        auto r2 = decodeStdMsgsString(pkt, pos, nullOut, 16);
        TEST_ASSERT(!r2.success, "null outBuf returns failure");

        // Truncated buffer: encap header only, no string follows
        auto r3 = decodeStdMsgsString(pkt, 4, out, sizeof(out));
        TEST_ASSERT(!r3.success, "truncated payload (no string length) returns failure");

        // Truncated buffer: string length says 6 bytes but only 3 present
        uint8_t truncated[10] = { 0x00, 0x01, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 'h', 'i' };
        auto r4 = decodeStdMsgsString(truncated, sizeof(truncated), out, sizeof(out));
        TEST_ASSERT(!r4.success, "string length exceeding buffer returns failure");

        // Output buffer smaller than text: truncates but succeeds and null-terminates.
        char tiny[3] = {'x', 'x', 'x'};
        auto r5 = decodeStdMsgsString(pkt, pos, tiny, sizeof(tiny));
        TEST_ASSERT(r5.success, "small outBuf still succeeds");
        TEST_ASSERT(tiny[2] == '\0', "small outBuf is null-terminated");
        TEST_ASSERT(strncmp(tiny, "he", 2) == 0, "small outBuf holds truncated prefix");
        TEST_ASSERT(r5.textLen == 5, "textLen still reports full source string length");

        // Empty string ("") = length 1 (just null), textLen 0.
        uint8_t emptyPkt[12] = { 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        char out2[4] = {0};
        auto r6 = decodeStdMsgsString(emptyPkt, 9, out2, sizeof(out2));
        TEST_ASSERT(r6.success && r6.textLen == 0 && out2[0] == '\0',
                    "empty string decodes to textLen=0 + null-terminated buf");

        // Null payload
        auto r7 = decodeStdMsgsString(nullptr, 10, out, sizeof(out));
        TEST_ASSERT(!r7.success, "null payload returns failure");
    }

    //=================================================================
    // RTPSWriterHeartbeatRunner: SedpChatterSubscription retransmit
    //=================================================================
    {
        printf("Test: RTPSWriterHeartbeatRunner SedpChatterSubscription step\n");

        // Sequence includes the new sub action right after ChatterPublication,
        // on both Linux and Esp flavors.
        for (auto flavor : { RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle,
                              RTPSWriterHeartbeatRuntimeFlavor::EspStyle })
        {
            auto seq = RTPSWriterHeartbeatRunner_buildSequence(flavor);
            int pubIdx = -1, subIdx = -1;
            for (uint8_t i = 0; i < seq.numSteps; i++)
            {
                if (seq.steps[i].action == RTPSWriterHeartbeatAction::SedpChatterPublication)
                    pubIdx = i;
                if (seq.steps[i].action == RTPSWriterHeartbeatAction::SedpChatterSubscription)
                    subIdx = i;
            }
            TEST_ASSERT(pubIdx >= 0 && subIdx >= 0,
                        "both chatter pub and sub scheduled in heartbeat sequence");
            TEST_ASSERT(subIdx == pubIdx + 1,
                        "SedpChatterSubscription immediately follows SedpChatterPublication");
        }

        // Send target: Metatraffic (SEDP endpoint).
        TEST_ASSERT(RTPSWriterHeartbeatRunner_sendTargetForAction(
                        RTPSWriterHeartbeatAction::SedpChatterSubscription)
                        == RTPSWriterHeartbeatSendTarget::Metatraffic,
                    "SedpChatterSubscription sends on metatraffic");

        // Sequence number selection:
        // - EspStyle -> chatterSedpSubSeqNum
        // - LinuxStyle -> sedpSubSeqNum + 1
        RTPSWriterHeartbeatCounterState counters{};
        counters.sedpSubSeqNum = 10;
        counters.chatterSedpSubSeqNum = 77;
        uint64_t seqEsp = RTPSWriterHeartbeatRunner_sequenceForAction(
            RTPSWriterHeartbeatAction::SedpChatterSubscription,
            RTPSWriterHeartbeatRuntimeFlavor::EspStyle, counters);
        TEST_ASSERT(seqEsp == 77, "ESP uses chatterSedpSubSeqNum for chatter subscription HB");
        uint64_t seqLin = RTPSWriterHeartbeatRunner_sequenceForAction(
            RTPSWriterHeartbeatAction::SedpChatterSubscription,
            RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle, counters);
        TEST_ASSERT(seqLin == 11, "Linux derives chatter sub SN = sedpSubSeqNum+1");

        // End-to-end: RTPSWriterHeartbeatRunner_run invokes buildPayload for the
        // SedpChatterSubscription step and routes it to sendPayload on metatraffic.
        struct Ctx {
            int subBuildCalls = 0;
            uint64_t lastSubSeq = 0;
            int subSendCalls = 0;
            RTPSWriterHeartbeatSendTarget lastTarget = RTPSWriterHeartbeatSendTarget::UserData;
        } ctx;

        RTPSWriterHeartbeatRunnerCallbacks cb;
        cb.buildPayload = [](void* u, RTPSWriterHeartbeatAction action,
                              uint64_t seq, uint32_t) -> uint32_t {
            if (action != RTPSWriterHeartbeatAction::SedpChatterSubscription)
                return 0;
            auto* c = static_cast<Ctx*>(u);
            c->subBuildCalls++;
            c->lastSubSeq = seq;
            return 42;
        };
        cb.sendPayload = [](void* u, RTPSWriterHeartbeatSendTarget tgt, uint32_t) -> int {
            auto* c = static_cast<Ctx*>(u);
            c->subSendCalls++;
            c->lastTarget = tgt;
            return 42;
        };

        RTPSWriterHeartbeatCounterState runCounters{};
        runCounters.sedpSubSeqNum = 4;
        runCounters.chatterSedpSubSeqNum = 5;
        RTPSWriterHeartbeatRunner_run(
            RTPSWriterHeartbeatRuntimeFlavor::EspStyle, runCounters, cb, &ctx);
        TEST_ASSERT(ctx.subBuildCalls == 1, "buildPayload called once for chatter sub on ESP");
        TEST_ASSERT(ctx.lastSubSeq == 5, "ESP SN for chatter sub = chatterSedpSubSeqNum");
        TEST_ASSERT(ctx.subSendCalls == 1, "sendPayload invoked for chatter sub");
        TEST_ASSERT(ctx.lastTarget == RTPSWriterHeartbeatSendTarget::Metatraffic,
                    "chatter sub routed to metatraffic");

        // LinuxStyle path picks sedpSubSeqNum + 1.
        ctx.subBuildCalls = 0; ctx.subSendCalls = 0; ctx.lastSubSeq = 0;
        RTPSWriterHeartbeatRunner_run(
            RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle, runCounters, cb, &ctx);
        TEST_ASSERT(ctx.subBuildCalls == 1 && ctx.lastSubSeq == 5,
                    "Linux run uses sedpSubSeqNum+1 (=4+1=5) for chatter sub SN");
    }

    //=================================================================
    // RTPSReliabilityAndWriterState: ACKNACK writerEID label map
    //=================================================================
    {
        printf("Test: classifyWriter / writerKindToStr label map\n");
        using namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState;

        TEST_ASSERT(classifyWriter(ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER)
                        == RTPSAckNackWriterKind::SedpPublications,
                    "SEDP publications writer classified");
        TEST_ASSERT(classifyWriter(ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER)
                        == RTPSAckNackWriterKind::SedpSubscriptions,
                    "SEDP subscriptions writer classified");
        TEST_ASSERT(classifyWriter(ENTITYID_ROS_DISC_INFO_WRITER)
                        == RTPSAckNackWriterKind::RosDiscoveryInfo,
                    "ros_discovery_info writer classified");
        TEST_ASSERT(classifyWriter(ENTITYID_CHATTER_WRITER)
                        == RTPSAckNackWriterKind::Chatter,
                    "chatter writer classified");
        TEST_ASSERT(classifyWriter(ENTITYID_CHATTER_READER)
                        == RTPSAckNackWriterKind::ChatterReader,
                    "chatter_in reader (as ACK target) classified");
        TEST_ASSERT(classifyWriter(ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER)
                        == RTPSAckNackWriterKind::ParticipantMessage,
                    "liveliness/participant-message writer classified (fixes '(unknown)')");

        TEST_ASSERT(strcmp(writerKindToStr(RTPSAckNackWriterKind::ParticipantMessage),
                            "participant_msg") == 0,
                    "participant_msg label present");
        TEST_ASSERT(strcmp(writerKindToStr(RTPSAckNackWriterKind::ChatterReader),
                            "chatter_in_reader") == 0,
                    "chatter_in_reader label present");
    }

    //=================================================================
    // RTPSSubscriptionRegistry: N-ary string subscription table
    //=================================================================
    {
        printf("Test: RTPSSubscriptionRegistry add/lookup/iterate\n");
        using namespace RaftRuntime::RTPS::Runtime::Dispatch;

        // Deterministic entity ID allocator.
        uint8_t eid[4];
        TEST_ASSERT(RTPSSubscriptionRegistry_allocateNextEntityId(0, eid)
                    && eid[0] == 0x00 && eid[1] == 0x01 && eid[2] == 0x02 && eid[3] == 0x04,
                    "slot 0 allocates ENTITYID_CHATTER_READER");
        TEST_ASSERT(RTPSSubscriptionRegistry_allocateNextEntityId(1, eid)
                    && eid[0] == 0x00 && eid[1] == 0x01 && eid[2] == 0x03 && eid[3] == 0x04,
                    "slot 1 increments the key byte");
        TEST_ASSERT(RTPSSubscriptionRegistry_allocateNextEntityId(7, eid)
                    && eid[2] == 0x09,
                    "slot 7 (last) is 0x00 0x01 0x09 0x04");
        TEST_ASSERT(!RTPSSubscriptionRegistry_allocateNextEntityId(RTPS_SUBSCRIPTION_REGISTRY_CAPACITY, eid),
                    "slot >= capacity fails");

        // Empty registry.
        RTPSSubscriptionRegistry reg;
        TEST_ASSERT(reg.count == 0, "registry starts empty");

        // Add returns slot index; capacity guard.
        TEST_ASSERT(reg.add("rt/chatter_in", "std_msgs::msg::dds_::String_") == 0,
                    "first add returns slot 0");
        TEST_ASSERT(reg.add("rt/cmd", "std_msgs::msg::dds_::String_") == 1,
                    "second add returns slot 1");
        TEST_ASSERT(reg.count == 2, "count tracks additions");

        // Null / empty guards.
        TEST_ASSERT(reg.add(nullptr, "x") == -1, "null topic rejected");
        TEST_ASSERT(reg.add("x", nullptr) == -1, "null type rejected");
        TEST_ASSERT(reg.add("", "x") == -1, "empty topic rejected");
        TEST_ASSERT(reg.count == 2, "failed adds do not bump count");

        // Entity IDs match the allocator policy.
        TEST_ASSERT(reg.entries[0].entityId[2] == 0x02
                    && reg.entries[1].entityId[2] == 0x03,
                    "per-slot entity ID follows allocator");
        TEST_ASSERT(strcmp(reg.entries[1].topic, "rt/cmd") == 0,
                    "topic stored non-owning");

        // Lookup by entity ID.
        const uint8_t chatterIn[4]  = {0x00, 0x01, 0x02, 0x04};
        const uint8_t cmd[4]        = {0x00, 0x01, 0x03, 0x04};
        const uint8_t unknown[4]    = {0x00, 0x01, 0x09, 0x04};
        const RTPSSubscriptionEntry* e0 = reg.findByEntityId(chatterIn);
        const RTPSSubscriptionEntry* e1 = reg.findByEntityId(cmd);
        const RTPSSubscriptionEntry* eN = reg.findByEntityId(unknown);
        TEST_ASSERT(e0 && strcmp(e0->topic, "rt/chatter_in") == 0,
                    "findByEntityId resolves slot 0");
        TEST_ASSERT(e1 && strcmp(e1->topic, "rt/cmd") == 0,
                    "findByEntityId resolves slot 1");
        TEST_ASSERT(eN == nullptr, "findByEntityId returns null for unknown EID");
        TEST_ASSERT(reg.findByEntityId(nullptr) == nullptr, "null EID safe");

        // readerEntityIds fills pointer array.
        const uint8_t* ids[RTPS_SUBSCRIPTION_REGISTRY_CAPACITY] = {nullptr};
        uint32_t n = reg.readerEntityIds(ids, RTPS_SUBSCRIPTION_REGISTRY_CAPACITY);
        TEST_ASSERT(n == 2, "readerEntityIds returns count");
        TEST_ASSERT(ids[0] == reg.entries[0].entityId
                    && ids[1] == reg.entries[1].entityId,
                    "readerEntityIds populates in slot order");
        // maxOut cap honoured.
        n = reg.readerEntityIds(ids, 1);
        TEST_ASSERT(n == 1, "readerEntityIds honours maxOut cap");

        // Capacity guard: add up to 8 then fail.
        RTPSSubscriptionRegistry full;
        for (int i = 0; i < RTPS_SUBSCRIPTION_REGISTRY_CAPACITY; i++)
            TEST_ASSERT(full.add("t", "y") == i, "filling registry in order");
        TEST_ASSERT(full.count == RTPS_SUBSCRIPTION_REGISTRY_CAPACITY,
                    "registry at capacity");
        TEST_ASSERT(full.add("overflow", "y") == -1,
                    "add past capacity fails");

        // setTopic on existing slot.
        TEST_ASSERT(reg.setTopic(0, "rt/new_topic", "new_type"),
                    "setTopic updates existing slot");
        TEST_ASSERT(strcmp(reg.entries[0].topic, "rt/new_topic") == 0,
                    "setTopic reflected in entry");
        TEST_ASSERT(!reg.setTopic(99, "x", "y"),
                    "setTopic rejects out-of-range slot");
        TEST_ASSERT(!reg.setTopic(0, nullptr, "y"),
                    "setTopic rejects null topic");
    }

    //=================================================================
    // RTPSSEDPPublicationParser: extract writerGuid + topic from an
    // SEDP BuiltinPublications DATA payload.
    //=================================================================
    {
        printf("Test: RTPSSEDPPublicationParser parse (clean build)\n");
        using namespace RaftRuntime::RTPS::Runtime::Dispatch;

        // Clean, exact-byte payload.
        uint8_t buf[80] = {0};
        uint32_t off = 0;
        buf[off++] = 0x00; buf[off++] = 0x03; buf[off++] = 0x00; buf[off++] = 0x00; // encap
        // PID_ENDPOINT_GUID (pid=0x005A, len=16)
        buf[off++] = 0x5A; buf[off++] = 0x00; buf[off++] = 0x10; buf[off++] = 0x00;
        const uint8_t refGuid[16] = {
            0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
            0xA8, 0xA9, 0xAA, 0xAB,       // 12 prefix
            0x00, 0x00, 0x02, 0x04        // 4 entityId (user-defined reader w/ key)
        };
        memcpy(buf + off, refGuid, 16); off += 16;
        // PID_TOPIC_NAME (pid=0x0005, len=16: 4 strLen + 7 str ("rt/foo\0") + 5 pad = 16)
        buf[off++] = 0x05; buf[off++] = 0x00; buf[off++] = 0x10; buf[off++] = 0x00;
        buf[off++] = 0x07; buf[off++] = 0x00; buf[off++] = 0x00; buf[off++] = 0x00; // strLen=7 (incl null)
        const char* topicStr = "rt/foo";
        memcpy(buf + off, topicStr, 6); off += 6;
        buf[off++] = 0x00;                                // null
        buf[off++] = 0x00; buf[off++] = 0x00; buf[off++] = 0x00; buf[off++] = 0x00; buf[off++] = 0x00; // 5 pad
        // PID_SENTINEL (pid=0x0001, len=0)
        buf[off++] = 0x01; buf[off++] = 0x00; buf[off++] = 0x00; buf[off++] = 0x00;

        RTPSParsedPublicationAnnounce parsed;
        const bool ok = RTPSSEDPPublicationParser_parse(buf, off, parsed);
        TEST_ASSERT(ok, "parser returns true on well-formed payload");
        TEST_ASSERT(parsed.hasWriterGuid, "parser extracts endpoint GUID");
        TEST_ASSERT(memcmp(parsed.writerGuid, refGuid, 16) == 0,
                    "parsed writerGuid matches source bytes");
        TEST_ASSERT(parsed.topic != nullptr, "topic pointer set");
        TEST_ASSERT(parsed.topicLen == 6, "topicLen excludes trailing null");
        TEST_ASSERT(memcmp(parsed.topic, "rt/foo", 6) == 0,
                    "topic contents match source");

        // Null + short buffer rejection.
        RTPSParsedPublicationAnnounce p2;
        TEST_ASSERT(!RTPSSEDPPublicationParser_parse(nullptr, 16, p2),
                    "null buffer returns false");
        TEST_ASSERT(!RTPSSEDPPublicationParser_parse(buf, 4, p2),
                    "payload smaller than a single PID entry returns false");

        // Missing PID_ENDPOINT_GUID returns false.
        uint8_t onlyTopic[24] = {
            0x00, 0x03, 0x00, 0x00,           // encap
            0x05, 0x00, 0x10, 0x00,           // PID_TOPIC_NAME, len=16
            0x07, 0x00, 0x00, 0x00,           // strLen
            'r','t','/','f','o','o',0x00, 0x00, 0x00, 0x00, 0x00, // str+null+pad (12)
        };
        RTPSParsedPublicationAnnounce p3;
        const bool ok3 = RTPSSEDPPublicationParser_parse(onlyTopic, sizeof(onlyTopic), p3);
        TEST_ASSERT(!ok3, "payload without PID_ENDPOINT_GUID returns false");
    }

    //=================================================================
    // RTPSRemotePublicationMap: upsert + findSlot
    //=================================================================
    {
        printf("Test: RTPSRemotePublicationMap upsert/findSlot\n");
        using namespace RaftRuntime::RTPS::Runtime::Dispatch;

        RTPSRemotePublicationMap map;
        const uint8_t g1[16] = {1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0x02,0x03};
        const uint8_t g2[16] = {2,2,2,2,2,2,2,2,2,2,2,2, 0,0,0x03,0x03};

        TEST_ASSERT(map.findSlot(g1) == -1, "initial findSlot returns -1");

        TEST_ASSERT(map.upsert(g1, 0), "first upsert reports new entry");
        TEST_ASSERT(map.count == 1, "count incremented");
        TEST_ASSERT(map.findSlot(g1) == 0, "findSlot returns assigned slot");

        TEST_ASSERT(!map.upsert(g1, 3), "upsert existing guid returns false (update)");
        TEST_ASSERT(map.count == 1, "count unchanged on update");
        TEST_ASSERT(map.findSlot(g1) == 3, "slot updated in place");

        TEST_ASSERT(map.upsert(g2, 5), "second distinct guid is new");
        TEST_ASSERT(map.count == 2, "count incremented again");

        // Split-key lookup.
        TEST_ASSERT(map.findSlot(g1, g1 + 12) == 3,
                    "split-key findSlot matches full-key findSlot");

        // Reject invalid inputs.
        TEST_ASSERT(!map.upsert(nullptr, 0), "null guid rejected");
        TEST_ASSERT(!map.upsert(g1, -1),     "negative slot rejected");

        // Fill to capacity + overflow wraps via writeCursor.
        RTPSRemotePublicationMap full;
        uint8_t seed[16] = {0};
        for (uint8_t i = 0; i < RTPS_REMOTE_PUBLICATION_MAP_CAPACITY; i++)
        {
            seed[0] = i + 1;
            TEST_ASSERT(full.upsert(seed, (int8_t)(i % 8)),
                        "fill registry to capacity");
        }
        TEST_ASSERT(full.count == RTPS_REMOTE_PUBLICATION_MAP_CAPACITY,
                    "map at capacity");
        // One more upsert: overwrites oldest (index 0) via round-robin cursor.
        seed[0] = 0xFF;
        TEST_ASSERT(full.upsert(seed, 7),
                    "overflow upsert returns true (treated as new entry)");
        TEST_ASSERT(full.count == RTPS_REMOTE_PUBLICATION_MAP_CAPACITY,
                    "count clamped at capacity");
        TEST_ASSERT(full.findSlot(seed) == 7,
                    "overflow entry findable post-upsert");
    }

    //=================================================================
    // RTPSData_getSerializedPayload: skip fixed prefix + inline QoS
    //
    // DATA submessage contentLayout (starting right after the 4-byte
    // submessage header):
    //   [0..19]    fixed 20-byte prefix (extraFlags, octetsToInlineQos,
    //              readerId, writerId, writerSN)
    //   [20..]     if Q flag (0x02) set: inline QoS ParameterList
    //              terminated by PID_SENTINEL (0x0001, len=0)
    //   [after]    serializedPayload (if D or K flag set)
    //
    // Regression guard for Phase-3 bug where the helper was missing and
    // `pContent + 20` landed inside inline-QoS on FastDDS dispose DATAs.
    //=================================================================
    {
        printf("Test: RTPSData_getSerializedPayload (inline-QoS skipping)\n");

        // --- Case A: Q=0, payload begins at offset 20 ------------------
        {
            uint8_t buf[32] = {0};
            // Fixed 20 bytes stay zero; payload = 12 bytes of 0xAB.
            memset(buf + 20, 0xAB, 12);
            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(buf, sizeof(buf), /*flags=*/0x05,
                                          pPayload, payloadLen);
            TEST_ASSERT(pPayload == buf + 20,
                        "Q=0: payload begins after 20-byte prefix");
            TEST_ASSERT(payloadLen == 12,
                        "Q=0: payload length = contentLen - 20");
        }

        // --- Case B: Q=1 with sentinel-only inline QoS -----------------
        // Layout: [20] PID_SENTINEL (0x0001) [22] len=0x0000, then payload.
        {
            uint8_t buf[32] = {0};
            buf[20] = 0x01; buf[21] = 0x00; // PID_SENTINEL (LE)
            buf[22] = 0x00; buf[23] = 0x00; // length = 0
            memset(buf + 24, 0xCD, 8);
            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(buf, sizeof(buf), /*flags=*/0x07,
                                          pPayload, payloadLen);
            TEST_ASSERT(pPayload == buf + 24,
                        "Q=1 sentinel-only: payload starts at offset 24");
            TEST_ASSERT(payloadLen == 8,
                        "Q=1 sentinel-only: payloadLen = contentLen - 24");
        }

        // --- Case C: Q=1, dispose-style (PID_STATUS_INFO + PID_KEY_HASH
        //     + PID_SENTINEL) with no trailing payload. This mirrors the
        //     52-byte DATAs FastDDS sends on pub disposal.
        {
            uint8_t buf[52] = {0};
            uint32_t o = 20;
            // PID_STATUS_INFO (0x0071) len=4
            buf[o++] = 0x71; buf[o++] = 0x00;
            buf[o++] = 0x04; buf[o++] = 0x00;
            o += 4; // four bytes of status payload
            // PID_KEY_HASH (0x0070) len=16
            buf[o++] = 0x70; buf[o++] = 0x00;
            buf[o++] = 0x10; buf[o++] = 0x00;
            o += 16;
            // PID_SENTINEL (0x0001) len=0
            buf[o++] = 0x01; buf[o++] = 0x00;
            buf[o++] = 0x00; buf[o++] = 0x00;
            TEST_ASSERT(o == 52, "dispose layout = 52 bytes");

            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(buf, sizeof(buf), /*flags=*/0x03,
                                          pPayload, payloadLen);
            // Helper succeeds, but there's nothing past the sentinel.
            TEST_ASSERT(pPayload == buf + 52,
                        "dispose-style: payload pointer past sentinel");
            TEST_ASSERT(payloadLen == 0,
                        "dispose-style: zero-length serialized payload");
        }

        // --- Case D: Q=1, malformed (no sentinel, parameter runs off
        //     end of buffer) -> helper returns {nullptr, 0}.
        {
            uint8_t buf[28] = {0};
            // Parameter PID=0x0050 len=0x0020 (32 bytes) but only 4 bytes
            // remain in the buffer -> must be rejected.
            buf[20] = 0x50; buf[21] = 0x00;
            buf[22] = 0x20; buf[23] = 0x00;
            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(buf, sizeof(buf), /*flags=*/0x03,
                                          pPayload, payloadLen);
            TEST_ASSERT(pPayload == nullptr && payloadLen == 0,
                        "malformed inline-QoS: returns {nullptr, 0}");
        }

        // --- Case E: contentLen < 20 -> refuse ------------------------
        {
            uint8_t buf[16] = {0};
            const uint8_t* pPayload = nullptr;
            uint32_t payloadLen = 0;
            RTPSData_getSerializedPayload(buf, sizeof(buf), /*flags=*/0x05,
                                          pPayload, payloadLen);
            TEST_ASSERT(pPayload == nullptr && payloadLen == 0,
                        "contentLen<20: rejected");
        }
    }

    //=================================================================
    // Phase 4 / Slice 4.1 — RTPSDynamicWriterRegistry skeleton
    //=================================================================
    {
        printf("Test: RTPSDynamicWriterRegistry (Phase 4 Slice 4.1)\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;

        // ---- allocateEntityId policy ---------------------------------
        {
            uint8_t eid[4] = {0};
            TEST_ASSERT(RTPSDynamicWriterRegistry_allocateEntityId(0, eid),
                        "allocateEntityId(slot=0) succeeds");
            TEST_ASSERT(eid[0] == 0x00 && eid[1] == 0x01 &&
                        eid[2] == DYNAMIC_WRITER_ENTITY_KEY_BASE && eid[3] == 0x03,
                        "slot-0 entityId = {00,01,0x10,03}");

            TEST_ASSERT(RTPSDynamicWriterRegistry_allocateEntityId(15, eid),
                        "allocateEntityId(slot=15) succeeds");
            TEST_ASSERT(eid[2] == (uint8_t)(DYNAMIC_WRITER_ENTITY_KEY_BASE + 15) &&
                        eid[3] == 0x03,
                        "slot-15 entityId = {00,01,0x1F,03}");

            TEST_ASSERT(!RTPSDynamicWriterRegistry_allocateEntityId(
                            DYNAMIC_WRITER_REGISTRY_CAPACITY, eid),
                        "allocateEntityId out-of-range rejected");
        }

        // ---- Key validity --------------------------------------------
        {
            RTPSDynamicWriterKey zero{};
            RTPSDynamicWriterKey good{1, 0x6A};
            RTPSDynamicWriterKey good2{1, 0x38};
            TEST_ASSERT(!zero.isValid(), "zero key is invalid");
            TEST_ASSERT(good.isValid(), "non-zero key is valid");
            TEST_ASSERT(good.equals({1, 0x6A}), "key equality");
            TEST_ASSERT(!good.equals(good2), "key inequality");
        }

        // ---- allocate / find / release round-trip --------------------
        {
            RTPSDynamicWriterRegistry reg;
            TEST_ASSERT(reg.inUseCount() == 0, "registry starts empty");
            TEST_ASSERT(reg.capacity() == DYNAMIC_WRITER_REGISTRY_CAPACITY,
                        "registry capacity reported");

            uint8_t eidA[4] = {0};
            int slotA = reg.allocate({1, 0x6A}, "rt/imu_1_6a",
                                     "sensor_msgs::msg::dds_::Imu_", eidA);
            TEST_ASSERT(slotA == 0, "first allocate returns slot 0");
            TEST_ASSERT(eidA[2] == DYNAMIC_WRITER_ENTITY_KEY_BASE,
                        "slot 0 entityId key byte = base");
            TEST_ASSERT(reg.inUseCount() == 1, "inUseCount=1 after allocate");

            uint8_t eidB[4] = {0};
            int slotB = reg.allocate({1, 0x38}, "rt/temp_1_38",
                                     "sensor_msgs::msg::dds_::Temperature_", eidB);
            TEST_ASSERT(slotB == 1, "second allocate returns slot 1");
            TEST_ASSERT(eidB[2] == (uint8_t)(DYNAMIC_WRITER_ENTITY_KEY_BASE + 1),
                        "slot 1 entityId key byte = base+1");
            TEST_ASSERT(reg.inUseCount() == 2, "inUseCount=2");

            // Find by key
            TEST_ASSERT(reg.find({1, 0x6A}) == 0, "find returns slot 0");
            TEST_ASSERT(reg.find({1, 0x38}) == 1, "find returns slot 1");
            TEST_ASSERT(reg.find({2, 0x6A}) == -1, "find unknown key -> -1");
            TEST_ASSERT(reg.find({}) == -1, "find invalid key -> -1");

            // Find by entityId
            TEST_ASSERT(reg.findByEntityId(eidA) == 0, "findByEntityId slot 0");
            TEST_ASSERT(reg.findByEntityId(eidB) == 1, "findByEntityId slot 1");
            uint8_t bogus[4] = {0xFF, 0xFF, 0xFF, 0xFF};
            TEST_ASSERT(reg.findByEntityId(bogus) == -1, "findByEntityId unknown -> -1");

            // Accessor fields
            const auto* eA = reg.get(0);
            TEST_ASSERT(eA && eA->inUse && eA->key.busNum == 1 && eA->key.address == 0x6A,
                        "get(slot 0) has correct key");
            TEST_ASSERT(eA && strcmp(eA->topic, "rt/imu_1_6a") == 0,
                        "get(slot 0) has correct topic");
            TEST_ASSERT(eA && strcmp(eA->type, "sensor_msgs::msg::dds_::Imu_") == 0,
                        "get(slot 0) has correct type");
            // sedpSeqNum is pre-seeded with the per-slot announce base seq
            // (AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + slot) so SEDP retransmits use
            // a stable sequence number across the slot's lifetime.
            TEST_ASSERT(eA && eA->seqNum == 0 &&
                            eA->sedpSeqNum == AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + 0,
                        "newly-allocated slot has expected initial seq counters");

            // Mutable accessor -> bump seqNum
            auto* mA = reg.getMutable(0);
            TEST_ASSERT(mA != nullptr, "getMutable(0) non-null");
            mA->seqNum = 7;
            TEST_ASSERT(reg.get(0)->seqNum == 7, "seqNum bump survives");

            // Release by key + post-release accessors
            TEST_ASSERT(reg.release({1, 0x6A}), "release slot 0 by key");
            TEST_ASSERT(reg.inUseCount() == 1, "inUseCount=1 after release");
            TEST_ASSERT(reg.find({1, 0x6A}) == -1, "find after release -> -1");
            TEST_ASSERT(reg.get(0) == nullptr, "get(freed slot) -> nullptr");
            TEST_ASSERT(reg.getMutable(0) == nullptr, "getMutable(freed slot) -> nullptr");

            // Release non-existent key is a no-op
            TEST_ASSERT(!reg.release({9, 0xAB}), "release unknown key -> false");
            TEST_ASSERT(!reg.release({}), "release invalid key -> false");

            // Slot reuse: next allocate should reclaim slot 0 (lowest free).
            uint8_t eidC[4] = {0};
            int slotC = reg.allocate({1, 0x29}, "rt/range_1_29",
                                     "sensor_msgs::msg::dds_::Range_", eidC);
            TEST_ASSERT(slotC == 0, "slot reuse picks lowest free (slot 0)");
            TEST_ASSERT(eidC[2] == DYNAMIC_WRITER_ENTITY_KEY_BASE,
                        "reused slot gets the same entityId as before");
            // The reused slot resets the user-data seq, but sedpSeqNum is
            // re-seeded to the per-slot announce base seq.
            TEST_ASSERT(reg.get(0)->seqNum == 0 &&
                            reg.get(0)->sedpSeqNum == AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + 0,
                        "reused slot has fresh sequence counters");
        }

        // ---- Reject duplicate / invalid inputs -----------------------
        {
            RTPSDynamicWriterRegistry reg;
            TEST_ASSERT(reg.allocate({}, "t", "T") == -1,
                        "allocate invalid key -> -1");
            TEST_ASSERT(reg.allocate({1, 1}, nullptr, "T") == -1,
                        "allocate null topic -> -1");
            TEST_ASSERT(reg.allocate({1, 1}, "t", nullptr) == -1,
                        "allocate null type -> -1");
            TEST_ASSERT(reg.allocate({1, 1}, "", "T") == -1,
                        "allocate empty topic -> -1");
            TEST_ASSERT(reg.allocate({1, 1}, "t", "") == -1,
                        "allocate empty type -> -1");

            TEST_ASSERT(reg.allocate({1, 1}, "rt/a", "A") >= 0,
                        "valid allocate succeeds");
            TEST_ASSERT(reg.allocate({1, 1}, "rt/a2", "A") == -1,
                        "duplicate key rejected");
        }

        // ---- Capacity invariants --------------------------------------
        {
            RTPSDynamicWriterRegistry reg;
            // Fill the registry to capacity.
            for (uint8_t i = 0; i < DYNAMIC_WRITER_REGISTRY_CAPACITY; i++)
            {
                int slot = reg.allocate({1, (uint32_t)(0x10 + i)}, "rt/x", "X");
                TEST_ASSERT(slot == (int)i, "sequential allocate fills slots in order");
            }
            TEST_ASSERT(reg.inUseCount() == DYNAMIC_WRITER_REGISTRY_CAPACITY,
                        "inUseCount == capacity when full");

            // One more must fail.
            TEST_ASSERT(reg.allocate({2, 1}, "rt/over", "O") == -1,
                        "allocate beyond capacity -> -1");

            // Release middle slot, next allocate should fill that gap.
            const uint8_t gap = 5;
            TEST_ASSERT(reg.releaseSlot(gap), "releaseSlot(5)");
            TEST_ASSERT(reg.inUseCount() == DYNAMIC_WRITER_REGISTRY_CAPACITY - 1,
                        "inUseCount = cap-1 after releaseSlot");
            int slotG = reg.allocate({2, 1}, "rt/refill", "R");
            TEST_ASSERT(slotG == gap, "allocate fills the released gap");

            // Idempotent release.
            TEST_ASSERT(!reg.releaseSlot(DYNAMIC_WRITER_REGISTRY_CAPACITY),
                        "releaseSlot(out-of-range) -> false");
            // release gap+release gap again = false on second call
            TEST_ASSERT(reg.releaseSlot(gap), "release same slot once");
            TEST_ASSERT(!reg.releaseSlot(gap), "release same slot again -> false");

            // clear() wipes everything.
            reg.clear();
            TEST_ASSERT(reg.inUseCount() == 0, "clear() empties registry");
            TEST_ASSERT(reg.get(0) == nullptr, "clear(): get(0) null");
        }

        // ---- 20 attach/detach cycles, no state bleed -----------------
        {
            RTPSDynamicWriterRegistry reg;
            for (int cycle = 0; cycle < 20; cycle++)
            {
                int s = reg.allocate({1, 0x44}, "rt/cycle", "C");
                TEST_ASSERT(s == 0, "cycle: allocate returns slot 0");
                TEST_ASSERT(reg.release({1, 0x44}), "cycle: release succeeds");
                TEST_ASSERT(reg.inUseCount() == 0, "cycle: inUseCount back to 0");
            }
        }

        // ---- Slice 4.10: composite subIndex disambiguation ----------
        // AHT20-like device publishes Temperature (subIndex=0) and
        // RelativeHumidity (subIndex=1) on the same (bus, addr).  The
        // registry must treat them as two distinct keys/slots.
        {
            RTPSDynamicWriterRegistry reg;
            RTPSDynamicWriterKey kPrim{1, 0x38, 0};
            RTPSDynamicWriterKey kSec {1, 0x38, 1};
            TEST_ASSERT(!kPrim.equals(kSec),
                        "composite: subIndex differentiates keys");
            TEST_ASSERT(kPrim.equals({1, 0x38, 0}),
                        "composite: primary key equality with explicit sub=0");
            TEST_ASSERT(kPrim.equals({1, 0x38}),
                        "composite: default sub=0 equals explicit sub=0 (compat)");

            uint8_t eid0[4] = {0};
            uint8_t eid1[4] = {0};
            int s0 = reg.allocate(kPrim, "rt/temp_1_38",
                                  "sensor_msgs::msg::dds_::Temperature_", eid0);
            int s1 = reg.allocate(kSec,  "rt/rh_1_38",
                                  "sensor_msgs::msg::dds_::RelativeHumidity_", eid1);
            TEST_ASSERT(s0 == 0 && s1 == 1,
                        "composite: dual slots allocated");
            TEST_ASSERT(eid0[2] != eid1[2],
                        "composite: distinct entityIds for pri/sec");
            TEST_ASSERT(reg.find(kPrim) == 0, "composite: find primary");
            TEST_ASSERT(reg.find(kSec)  == 1, "composite: find secondary");
            TEST_ASSERT(reg.find({1, 0x38, 2}) == -1,
                        "composite: unknown subIndex -> -1");

            // Duplicate-key protection still applies per-subIndex.
            TEST_ASSERT(reg.allocate(kPrim, "rt/dup", "D") == -1,
                        "composite: duplicate primary rejected");
            TEST_ASSERT(reg.allocate(kSec,  "rt/dup", "D") == -1,
                        "composite: duplicate secondary rejected");

            // Releasing primary must leave the secondary intact.
            TEST_ASSERT(reg.release(kPrim), "composite: release primary");
            TEST_ASSERT(reg.find(kPrim) == -1, "composite: primary gone");
            TEST_ASSERT(reg.find(kSec)  == 1,  "composite: secondary still there");
            TEST_ASSERT(reg.inUseCount() == 1, "composite: one slot still in use");
            TEST_ASSERT(reg.release(kSec), "composite: release secondary");
            TEST_ASSERT(reg.inUseCount() == 0, "composite: registry empty");
        }
    }

    //=================================================================
    // Phase 4 / Slice 4.2 — RTPSAutoPubLifecycle attach/detach
    //=================================================================
    {
        printf("Test: RTPSAutoPubLifecycle (Phase 4 Slice 4.2)\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;

        // ---- attach copies strings into owned storage ----------------
        {
            RTPSAutoPubLifecycle life;
            TEST_ASSERT(life.inUseCount() == 0, "lifecycle starts empty");

            // Build topic/type in volatile local buffers, pass pointers,
            // then mutate the source to prove storage is owned.
            char topicSrc[64];
            char typeSrc[64];
            std::strcpy(topicSrc, "rt/raft_esp32/imu_1_6a");
            std::strcpy(typeSrc,  "sensor_msgs::msg::dds_::Imu_");

            uint8_t eid[4] = {0};
            int slot = life.attach({1, 0x6A}, topicSrc, typeSrc, eid);
            TEST_ASSERT(slot == 0, "attach returns slot 0");
            TEST_ASSERT(eid[0] == 0x00 && eid[1] == 0x01 &&
                        eid[2] == DYNAMIC_WRITER_ENTITY_KEY_BASE && eid[3] == 0x03,
                        "attach entityId matches registry policy");

            // Clobber the source to verify the lifecycle has its own copy.
            std::memset(topicSrc, 0, sizeof(topicSrc));
            std::memset(typeSrc,  0, sizeof(typeSrc));

            const auto* e = life.get(0);
            TEST_ASSERT(e != nullptr, "get(0) non-null");
            TEST_ASSERT(e && std::strcmp(e->topic, "rt/raft_esp32/imu_1_6a") == 0,
                        "attach: owned topic survives source clobber");
            TEST_ASSERT(e && std::strcmp(e->type, "sensor_msgs::msg::dds_::Imu_") == 0,
                        "attach: owned type survives source clobber");
            TEST_ASSERT(e && e->key.busNum == 1 && e->key.address == 0x6A,
                        "attach: key stored");
            TEST_ASSERT(std::strcmp(life.topicForSlot(0), "rt/raft_esp32/imu_1_6a") == 0,
                        "topicForSlot accessor");
            TEST_ASSERT(std::strcmp(life.typeForSlot(0),  "sensor_msgs::msg::dds_::Imu_") == 0,
                        "typeForSlot accessor");
        }

        // ---- reject oversize / invalid inputs ------------------------
        {
            RTPSAutoPubLifecycle life;
            char bigTopic[AUTOPUB_TOPIC_BUF_LEN + 4];
            std::memset(bigTopic, 'A', sizeof(bigTopic) - 1);
            bigTopic[sizeof(bigTopic) - 1] = '\0';
            TEST_ASSERT(life.attach({1, 1}, bigTopic, "T") == -1,
                        "attach rejects oversize topic");

            char bigType[AUTOPUB_TYPE_BUF_LEN + 4];
            std::memset(bigType, 'B', sizeof(bigType) - 1);
            bigType[sizeof(bigType) - 1] = '\0';
            TEST_ASSERT(life.attach({1, 1}, "rt/x", bigType) == -1,
                        "attach rejects oversize type");

            TEST_ASSERT(life.attach({}, "rt/x", "X") == -1,
                        "attach invalid key rejected");
            TEST_ASSERT(life.attach({1, 1}, nullptr, "X") == -1,
                        "attach null topic rejected");
            TEST_ASSERT(life.attach({1, 1}, "rt/x", nullptr) == -1,
                        "attach null type rejected");
            TEST_ASSERT(life.attach({1, 1}, "", "X") == -1,
                        "attach empty topic rejected");
            TEST_ASSERT(life.attach({1, 1}, "rt/x", "") == -1,
                        "attach empty type rejected");
            TEST_ASSERT(life.inUseCount() == 0,
                        "failed attaches leave registry empty");

            // A duplicate attach must be rejected without clobbering the existing slot.
            TEST_ASSERT(life.attach({1, 1}, "rt/first", "F") == 0, "attach succeeds");
            TEST_ASSERT(life.attach({1, 1}, "rt/second", "S") == -1,
                        "attach duplicate key rejected");
            TEST_ASSERT(std::strcmp(life.get(0)->topic, "rt/first") == 0,
                        "duplicate attach leaves original unchanged");
        }

        // ---- detach clears storage and frees slot --------------------
        {
            RTPSAutoPubLifecycle life;
            TEST_ASSERT(life.attach({1, 0x20}, "rt/a", "A") == 0, "attach slot 0");
            TEST_ASSERT(life.attach({1, 0x21}, "rt/b", "B") == 1, "attach slot 1");
            TEST_ASSERT(life.inUseCount() == 2, "inUseCount=2");

            TEST_ASSERT(life.detach({1, 0x20}), "detach slot 0 by key");
            TEST_ASSERT(life.inUseCount() == 1, "inUseCount=1 after detach");
            TEST_ASSERT(life.get(0) == nullptr, "get(freed slot) null");
            TEST_ASSERT(life.topicForSlot(0)[0] == '\0',
                        "detach zeroes owned topic storage");
            TEST_ASSERT(life.typeForSlot(0)[0] == '\0',
                        "detach zeroes owned type storage");

            TEST_ASSERT(!life.detach({1, 0x20}),
                        "detach already-freed key returns false");
            TEST_ASSERT(!life.detach({}), "detach invalid key returns false");
            TEST_ASSERT(!life.detach({9, 9}), "detach unknown key returns false");

            // detachSlot by index
            TEST_ASSERT(life.detachSlot(1), "detachSlot(1)");
            TEST_ASSERT(!life.detachSlot(1), "detachSlot idempotent");
            TEST_ASSERT(!life.detachSlot(DYNAMIC_WRITER_REGISTRY_CAPACITY),
                        "detachSlot out-of-range rejected");
            TEST_ASSERT(life.inUseCount() == 0, "all slots freed");
        }

        // ---- slot reuse after detach: same entityId, fresh counters --
        {
            RTPSAutoPubLifecycle life;
            uint8_t eid1[4] = {0};
            int s1 = life.attach({1, 0x50}, "rt/first", "F", eid1);
            TEST_ASSERT(s1 == 0, "first attach -> slot 0");
            // Bump seq via registry's mutable accessor.
            life.getMutable(0)->seqNum = 42;
            TEST_ASSERT(life.detach({1, 0x50}), "detach slot 0");

            uint8_t eid2[4] = {0};
            int s2 = life.attach({1, 0x51}, "rt/second", "S", eid2);
            TEST_ASSERT(s2 == 0, "second attach reuses slot 0");
            TEST_ASSERT(std::memcmp(eid1, eid2, 4) == 0,
                        "reused slot same entityId");
            TEST_ASSERT(life.get(0)->seqNum == 0,
                        "reused slot has fresh seqNum");
            TEST_ASSERT(std::strcmp(life.get(0)->topic, "rt/second") == 0,
                        "reused slot has new topic string");
        }

        // ---- entityId lookup goes via registry -----------------------
        {
            RTPSAutoPubLifecycle life;
            uint8_t eid[4] = {0};
            int slot = life.attach({1, 0x70}, "rt/t", "T", eid);
            TEST_ASSERT(slot == 0, "attach");
            TEST_ASSERT(life.findByEntityId(eid) == slot,
                        "findByEntityId routes via registry");
            TEST_ASSERT(life.find({1, 0x70}) == slot,
                        "find routes via registry");
        }

        // ---- Fill to capacity, overflow rejected ---------------------
        {
            RTPSAutoPubLifecycle life;
            for (uint8_t i = 0; i < DYNAMIC_WRITER_REGISTRY_CAPACITY; i++)
            {
                char tbuf[32];
                std::snprintf(tbuf, sizeof(tbuf), "rt/fill_%u", (unsigned)i);
                int s = life.attach({1, (uint32_t)(0x80 + i)}, tbuf, "X");
                TEST_ASSERT(s == (int)i, "sequential attach fills slots in order");
            }
            TEST_ASSERT(life.inUseCount() == DYNAMIC_WRITER_REGISTRY_CAPACITY,
                        "full capacity reached");
            TEST_ASSERT(life.attach({2, 1}, "rt/over", "O") == -1,
                        "attach beyond capacity -> -1");

            life.clear();
            TEST_ASSERT(life.inUseCount() == 0, "clear() empties lifecycle");
            TEST_ASSERT(life.topicForSlot(0)[0] == '\0', "clear() wipes topic storage");
        }

        // ---- 10 attach/detach cycles simulate device churn -----------
        {
            RTPSAutoPubLifecycle life;
            for (int cycle = 0; cycle < 10; cycle++)
            {
                int s = life.attach({1, 0x66}, "rt/churn", "C");
                TEST_ASSERT(s == 0, "churn: attach slot 0");
                TEST_ASSERT(life.detach({1, 0x66}), "churn: detach");
                TEST_ASSERT(life.inUseCount() == 0, "churn: back to 0");
                TEST_ASSERT(life.topicForSlot(0)[0] == '\0',
                            "churn: storage wiped after detach");
            }
        }
    }

    //=================================================================
    // Phase 4 / Slice 4.3 — RTPSAutoPubTopicNaming fallback formatting
    //=================================================================
    {
        printf("Test: RTPSAutoPubTopicNaming (Phase 4 Slice 4.3)\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;

        // ---- topic formatting: basic cases ---------------------------
        {
            char buf[64];
            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackTopic(buf, sizeof(buf), 1, 0x38),
                        "topic format bus=1 addr=0x38 succeeds");
            TEST_ASSERT(std::strcmp(buf, "/raft/raw_1_38") == 0,
                        "topic format: I2C addr 0x38 -> '/raft/raw_1_38'");

            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackTopic(buf, sizeof(buf), 1, 0x6A),
                        "topic format bus=1 addr=0x6A succeeds");
            TEST_ASSERT(std::strcmp(buf, "/raft/raw_1_6a") == 0,
                        "topic format: LSM6DS @0x6A -> lowercase hex");

            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackTopic(buf, sizeof(buf), 2, 0x08),
                        "topic format bus=2 addr=0x08 succeeds");
            TEST_ASSERT(std::strcmp(buf, "/raft/raw_2_08") == 0,
                        "topic format: single-digit addr zero-padded to 2 hex");

            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackTopic(buf, sizeof(buf), 0, 0),
                        "topic format bus=0 addr=0 succeeds");
            TEST_ASSERT(std::strcmp(buf, "/raft/raw_0_00") == 0,
                        "topic format: edge case bus=0 addr=0");
        }

        // ---- address masked to low 8 bits (I2C 7-bit convention) -----
        {
            char buf[64];
            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackTopic(buf, sizeof(buf), 1, 0x1138),
                        "topic format: address masked to 8 bits");
            TEST_ASSERT(std::strcmp(buf, "/raft/raw_1_38") == 0,
                        "topic format: high bits of address stripped");
        }

        // ---- topic formatting: input validation ----------------------
        {
            char buf[64];
            TEST_ASSERT(!RTPSAutoPubTopicNaming_formatFallbackTopic(nullptr, 64, 1, 1),
                        "topic format: null buffer rejected");
            TEST_ASSERT(!RTPSAutoPubTopicNaming_formatFallbackTopic(buf, 0, 1, 1),
                        "topic format: zero-length buffer rejected");
        }

        // ---- topic formatting: buffer-too-small leaves empty string --
        {
            char tiny[6]; // less than "/raft/raw_1_38" + NUL = 17
            std::memset(tiny, 'X', sizeof(tiny));
            TEST_ASSERT(!RTPSAutoPubTopicNaming_formatFallbackTopic(tiny, sizeof(tiny), 1, 0x38),
                        "topic format: short buffer rejected");
            TEST_ASSERT(tiny[0] == '\0',
                        "topic format: short buffer cleared to empty string on failure");
        }

        // ---- type formatting ----------------------------------------
        {
            char buf[64];
            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackType(buf, sizeof(buf)),
                        "type format succeeds");
            TEST_ASSERT(std::strcmp(buf, "std_msgs::msg::dds_::String_") == 0,
                        "type format: std_msgs/String");
            TEST_ASSERT(std::strcmp(buf, RTPS_AUTOPUB_FALLBACK_TYPE) == 0,
                        "type format: matches exported constant");

            TEST_ASSERT(!RTPSAutoPubTopicNaming_formatFallbackType(nullptr, 64),
                        "type format: null buffer rejected");
            TEST_ASSERT(!RTPSAutoPubTopicNaming_formatFallbackType(buf, 0),
                        "type format: zero-length buffer rejected");

            char tiny[8]; // less than full type name
            std::memset(tiny, 'X', sizeof(tiny));
            TEST_ASSERT(!RTPSAutoPubTopicNaming_formatFallbackType(tiny, sizeof(tiny)),
                        "type format: short buffer rejected");
            TEST_ASSERT(tiny[0] == '\0',
                        "type format: short buffer cleared to empty string on failure");
        }

        // ---- end-to-end: fallback pair feeds RTPSAutoPubLifecycle ----
        {
            char topic[AUTOPUB_TOPIC_BUF_LEN];
            char type[AUTOPUB_TYPE_BUF_LEN];
            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackTopic(topic, sizeof(topic), 1, 0x6A),
                        "pipe: topic format ok");
            TEST_ASSERT(RTPSAutoPubTopicNaming_formatFallbackType(type, sizeof(type)),
                        "pipe: type format ok");

            RTPSAutoPubLifecycle life;
            int slot = life.attach({1, 0x6A}, topic, type);
            TEST_ASSERT(slot == 0,
                        "pipe: lifecycle accepts formatted fallback pair");
            TEST_ASSERT(std::strcmp(life.topicForSlot(0), "/raft/raw_1_6a") == 0,
                        "pipe: lifecycle stores formatted fallback topic");
            TEST_ASSERT(std::strcmp(life.typeForSlot(0),
                                    "std_msgs::msg::dds_::String_") == 0,
                        "pipe: lifecycle stores formatted fallback type");
        }

        // ---- the "rt" prefix is the RTPS backend's, not the shared layer's ----
        {
            using RaftRuntime::AutoPub::AutoPubEndpointDesc;
            using RaftRuntime::AutoPub::AutoPubMsgKind;
            RTPSAutoPubBackend backend;
            AutoPubEndpointDesc desc;
            desc.deviceId = {1, 0x29, 0};
            desc.msgKind = AutoPubMsgKind::Range;
            desc.setNames("/raft/range_1_29", "sensor_msgs::msg::dds_::Range_");
            const uint8_t slot = backend.createPublisher(desc);
            TEST_ASSERT(slot != RTPSAutoPubBackend::INVALID_SLOT,
                        "backend accepts a ROS topic name");
            TEST_ASSERT(slot != RTPSAutoPubBackend::INVALID_SLOT &&
                        std::strcmp(backend.lifecycle().topicForSlot(slot), "rt/raft/range_1_29") == 0,
                        "RTPS announces the DDS name: a ROS topic gains the 'rt' prefix here, "
                        "not in the descriptor a Zenoh image would reject");
            backend.destroyPublisher(slot);

            AutoPubEndpointDesc unprefixed;
            unprefixed.deviceId = {1, 0x2A, 0};
            unprefixed.msgKind = AutoPubMsgKind::Range;
            unprefixed.setNames("raft/range_1_2a", "sensor_msgs::msg::dds_::Range_");
            const uint8_t bare = backend.createPublisher(unprefixed);
            TEST_ASSERT(bare != RTPSAutoPubBackend::INVALID_SLOT &&
                        std::strcmp(backend.lifecycle().topicForSlot(bare), "rt/raft/range_1_2a") == 0,
                        "a topic without a leading slash still gets exactly one separator");
            backend.destroyPublisher(bare);
        }
    }

    //=================================================================
    // Phase 4 / Slice 4.4 — RTPSAutoPubClassMap class→message mapping
    //=================================================================
    {
        printf("Test: RTPSAutoPubClassMap (Phase 4 Slice 4.4)\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;

        // Helper for compact table-driven assertions: all device-type rows
        // from DeviceTypeRecords.json, validated against §5.2 of the design doc.
        auto checkPrimary = [&](const char* name,
                                const char* const* clas, size_t clasCount,
                                RTPSAutoPubMsgKind expectedKind,
                                const char* expectedSlug) {
            const auto m = RTPSAutoPubClassMap_lookup(clas, clasCount, name);
            char msg[128];
            std::snprintf(msg, sizeof(msg), "%s: primary kind", name);
            TEST_ASSERT(m.primaryKind == expectedKind, msg);
            std::snprintf(msg, sizeof(msg), "%s: primary slug", name);
            TEST_ASSERT(m.primaryTopicSlug && std::strcmp(m.primaryTopicSlug, expectedSlug) == 0, msg);
            std::snprintf(msg, sizeof(msg), "%s: not excluded", name);
            TEST_ASSERT(!m.excluded, msg);
        };

        // ---- type-name accessor ----------------------------------------
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Imu),
                                "sensor_msgs::msg::dds_::Imu_") == 0,
                    "typeName: Imu");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Accel),
                                "sensor_msgs::msg::dds_::Imu_") == 0,
                    "typeName: Accel reuses Imu type");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Temperature),
                                "sensor_msgs::msg::dds_::Temperature_") == 0,
                    "typeName: Temperature");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::RelativeHumidity),
                                "sensor_msgs::msg::dds_::RelativeHumidity_") == 0,
                    "typeName: RelativeHumidity");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::FluidPressure),
                                "sensor_msgs::msg::dds_::FluidPressure_") == 0,
                    "typeName: FluidPressure");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Illuminance),
                                "sensor_msgs::msg::dds_::Illuminance_") == 0,
                    "typeName: Illuminance");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Range),
                                "sensor_msgs::msg::dds_::Range_") == 0,
                    "typeName: Range");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Float32),
                                "std_msgs::msg::dds_::Float32_") == 0,
                    "typeName: Float32");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Int32),
                                "std_msgs::msg::dds_::Int32_") == 0,
                    "typeName: Int32");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Bool),
                                "std_msgs::msg::dds_::Bool_") == 0,
                    "typeName: Bool");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::ByteMultiArray),
                                "std_msgs::msg::dds_::ByteMultiArray_") == 0,
                    "typeName: ByteMultiArray");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Wrench),
                                "geometry_msgs::msg::dds_::Wrench_") == 0,
                    "typeName: Wrench");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Float32MultiArray),
                                "std_msgs::msg::dds_::Float32MultiArray_") == 0,
                    "typeName: Float32MultiArray");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Joy),
                                "sensor_msgs::msg::dds_::Joy_") == 0,
                    "typeName: Joy");
        TEST_ASSERT(std::strcmp(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::String),
                                "std_msgs::msg::dds_::String_") == 0,
                    "typeName: String");
        TEST_ASSERT(RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind::Unknown) == nullptr,
                    "typeName: Unknown returns nullptr");

        // ---- hasClas helper ----------------------------------------
        {
            const char* c[] = {"ACC", "GYRO"};
            TEST_ASSERT(RTPSAutoPubClassMap_hasClas(c, 2, "ACC"), "hasClas: ACC hit");
            TEST_ASSERT(RTPSAutoPubClassMap_hasClas(c, 2, "GYRO"), "hasClas: GYRO hit");
            TEST_ASSERT(!RTPSAutoPubClassMap_hasClas(c, 2, "TEMP"), "hasClas: miss");
            TEST_ASSERT(!RTPSAutoPubClassMap_hasClas(nullptr, 0, "ACC"),
                        "hasClas: null array safe");
            TEST_ASSERT(!RTPSAutoPubClassMap_hasClas(c, 2, nullptr),
                        "hasClas: null code safe");
            TEST_ASSERT(!RTPSAutoPubClassMap_hasClas(c, 2, "acc"),
                        "hasClas: case-sensitive");
        }

        // ---- composite: ACC+GYRO wins over single ACC (LSM6DS) -------
        {
            const char* c[] = {"ACC", "GYRO"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 2, "LSM6DS");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::Imu,
                        "LSM6DS: primary = Imu");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "imu") == 0,
                        "LSM6DS: slug = imu");
            TEST_ASSERT(!m.hasSecondary(), "LSM6DS: single-writer Imu");
            // Reverse order still triggers composite rule.
            const char* cRev[] = {"GYRO", "ACC"};
            const auto mRev = RTPSAutoPubClassMap_lookup(cRev, 2, "LSM6DS");
            TEST_ASSERT(mRev.primaryKind == RTPSAutoPubMsgKind::Imu,
                        "LSM6DS: composite order-independent");
        }

        // ---- composite: TEMP+RH → Temperature + RelativeHumidity -----
        {
            const char* c[] = {"TEMP", "RH"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 2, "AHT20");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::Temperature,
                        "AHT20: primary = Temperature");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "temperature") == 0,
                        "AHT20: primary slug = temperature");
            TEST_ASSERT(m.secondaryKind == RTPSAutoPubMsgKind::RelativeHumidity,
                        "AHT20: secondary = RelativeHumidity");
            TEST_ASSERT(std::strcmp(m.secondaryTopicSlug, "humidity") == 0,
                        "AHT20: secondary slug = humidity");
            TEST_ASSERT(m.hasSecondary(), "AHT20: two-writer composite");
        }

        // ---- composite: PRES+TEMP → FluidPressure + Temperature ------
        {
            const char* c[] = {"PRES", "TEMP"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 2, "LPS25");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::FluidPressure,
                        "LPS25: primary = FluidPressure");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "pressure") == 0,
                        "LPS25: primary slug = pressure");
            TEST_ASSERT(m.secondaryKind == RTPSAutoPubMsgKind::Temperature,
                        "LPS25: secondary = Temperature");
            TEST_ASSERT(std::strcmp(m.secondaryTopicSlug, "temperature") == 0,
                        "LPS25: secondary slug = temperature");
        }

        // ---- single-class device-type sweep (DeviceTypeRecords.json) --
        { const char* c[] = {"ACC"};       checkPrimary("ADXL313",   c, 1, RTPSAutoPubMsgKind::Accel, "accel"); }
        { const char* c[] = {"ACC"};       checkPrimary("MXC400xXC", c, 1, RTPSAutoPubMsgKind::Accel, "accel"); }
        { const char* c[] = {"ANG"};       checkPrimary("AS5600",    c, 1, RTPSAutoPubMsgKind::Float32, "angle"); }
        { const char* c[] = {"ANG"};       checkPrimary("MT6701",    c, 1, RTPSAutoPubMsgKind::Float32, "angle"); }
        { const char* c[] = {"ROT"};       checkPrimary("M5Encoder", c, 1, RTPSAutoPubMsgKind::Int32,   "encoder"); }
        { const char* c[] = {"DIST"};      checkPrimary("VL6180",    c, 1, RTPSAutoPubMsgKind::Range,   "range"); }
        { const char* c[] = {"DIST"};      checkPrimary("VL53L4CD",  c, 1, RTPSAutoPubMsgKind::Range,   "range"); }
        { const char* c[] = {"LGHT"};      checkPrimary("VEML7700",  c, 1, RTPSAutoPubMsgKind::Illuminance, "illuminance"); }
        { const char* c[] = {"TCH"};       checkPrimary("CAP1203",   c, 1, RTPSAutoPubMsgKind::ByteMultiArray, "touch"); }
        { const char* c[] = {"BTN"};       checkPrimary("QwiicButton", c, 1, RTPSAutoPubMsgKind::Bool,  "button"); }
        { const char* c[] = {"FRCE"};      checkPrimary("HX711",     c, 1, RTPSAutoPubMsgKind::Wrench,  "force"); }
        { const char* c[] = {"HRM"};       checkPrimary("MAX30101",  c, 1, RTPSAutoPubMsgKind::Float32MultiArray, "ppg"); }
        { const char* c[] = {"SOIL"};      checkPrimary("AdafruitSoilSensor", c, 1, RTPSAutoPubMsgKind::Float32, "soil_moisture"); }
        { const char* c[] = {"GAME"};      checkPrimary("AdafruitGamepad",    c, 1, RTPSAutoPubMsgKind::Joy,     "joy"); }

        // ---- VCNL4040: [PROX, LGHT] — PROX wins single-class ordering? ----
        // Design §5.2 splits VCNL4040 into two topics (illuminance + proximity)
        // via Slice 4.10 composite.  For now the single-class fallthrough
        // picks LGHT (illuminance) since LGHT is checked before PROX.
        {
            const char* c[] = {"PROX", "LGHT"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 2, "VCNL4040");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::Illuminance,
                        "VCNL4040: primary = Illuminance (LGHT ordering)");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "illuminance") == 0,
                        "VCNL4040: slug = illuminance");
        }

        // ---- Device-type overrides ---------------------------------------
        // MCP9808 is mis-tagged LGHT in the JSON; override forces TEMP.
        {
            const char* c[] = {"LGHT"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 1, "MCP9808");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::Temperature,
                        "MCP9808: device-name override forces Temperature");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "temperature") == 0,
                        "MCP9808: slug = temperature");
            // Without the override, LGHT would map to Illuminance — confirm.
            const auto mBase = RTPSAutoPubClassMap_lookup(c, 1, nullptr);
            TEST_ASSERT(mBase.primaryKind == RTPSAutoPubMsgKind::Illuminance,
                        "MCP9808: without override would be Illuminance");
        }
        // RoboticalLightSensor (4-attr array) → Float32MultiArray "light".
        {
            const char* c[] = {"LGHT"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 1, "RoboticalLightSensor");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::Float32MultiArray,
                        "RoboticalLightSensor: primary = Float32MultiArray");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "light") == 0,
                        "RoboticalLightSensor: slug = light");
        }

        // ---- Actuator exclusions ----------------------------------------
        {
            const char* cS[] = {"SRVO"};
            const auto m = RTPSAutoPubClassMap_lookup(cS, 1, "RoboticalServo");
            TEST_ASSERT(m.excluded, "RoboticalServo: excluded");
            TEST_ASSERT(!m.hasPrimary(), "RoboticalServo: no primary writer");
        }
        {
            const char* cP[] = {"PUMP"};
            const auto m = RTPSAutoPubClassMap_lookup(cP, 1, "RoboticalWaterPump");
            TEST_ASSERT(m.excluded, "RoboticalWaterPump: excluded");
        }
        {
            const char* cPx[] = {"PIX"};
            const auto m = RTPSAutoPubClassMap_lookup(cPx, 1, "QwiicLEDStick");
            TEST_ASSERT(m.excluded, "QwiicLEDStick: excluded");
        }

        // ---- Fallback / BTHome (pending Slice 4.10) ---------------------
        {
            const char* c[] = {"BTHM"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 1, "BLEBTHome");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::String,
                        "BLEBTHome: falls through to String (pending 4.10)");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "raw") == 0,
                        "BLEBTHome: slug = raw");
            TEST_ASSERT(!m.excluded, "BLEBTHome: still published as String");
        }
        {
            const char* c[] = {"ZZZ_UNKNOWN"};
            const auto m = RTPSAutoPubClassMap_lookup(c, 1, "SomeNewDevice");
            TEST_ASSERT(m.primaryKind == RTPSAutoPubMsgKind::String,
                        "unknown clas: fallback String");
            TEST_ASSERT(std::strcmp(m.primaryTopicSlug, "raw") == 0,
                        "unknown clas: slug = raw");
        }
        {
            // Empty / null clas array still yields a usable fallback.
            const auto mEmpty = RTPSAutoPubClassMap_lookup(nullptr, 0, nullptr);
            TEST_ASSERT(mEmpty.primaryKind == RTPSAutoPubMsgKind::String,
                        "empty clas: fallback String");
            const auto mNullArr = RTPSAutoPubClassMap_lookup(nullptr, 5, "X");
            TEST_ASSERT(mNullArr.primaryKind == RTPSAutoPubMsgKind::String,
                        "null clasArray: fallback String");
        }

        // ---- Every returned non-excluded kind has a valid type name -----
        const RTPSAutoPubMsgKind allKinds[] = {
            RTPSAutoPubMsgKind::Imu, RTPSAutoPubMsgKind::Accel,
            RTPSAutoPubMsgKind::Temperature, RTPSAutoPubMsgKind::RelativeHumidity,
            RTPSAutoPubMsgKind::FluidPressure, RTPSAutoPubMsgKind::Illuminance,
            RTPSAutoPubMsgKind::Range, RTPSAutoPubMsgKind::Float32,
            RTPSAutoPubMsgKind::Int32, RTPSAutoPubMsgKind::Bool,
            RTPSAutoPubMsgKind::ByteMultiArray, RTPSAutoPubMsgKind::Wrench,
            RTPSAutoPubMsgKind::Float32MultiArray, RTPSAutoPubMsgKind::Joy,
            RTPSAutoPubMsgKind::String,
        };
        for (auto k : allKinds)
        {
            const char* tn = RTPSAutoPubClassMap_typeName(k);
            TEST_ASSERT(tn != nullptr && tn[0] != '\0',
                        "every message kind has a non-empty type name");
        }
    }

    //=================================================================
    // Phase 4 / Slice 4.5 — RTPSAutoPubCDRSerializer
    //=================================================================
    {
        printf("Test: RTPSAutoPubCDRSerializer (Phase 4 Slice 4.5)\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;

        namespace Common = RaftRuntime::AutoPub;
        static_assert(std::is_same_v<RTPSAutoPubMsgKind, Common::AutoPubMsgKind>);
        static_assert(std::is_same_v<RTPSAutoPubClassMapping, Common::AutoPubClassMapping>);
        static_assert(std::is_same_v<RTPSAutoPubAttrType, Common::AutoPubAttrType>);
        static_assert(std::is_same_v<RTPSAutoPubAttrFieldDesc, Common::AutoPubAttrFieldDesc>);
        static_assert(std::is_same_v<RTPSAutoPubCDRContext, Common::AutoPubCDRContext>);

        {
            const char* classes[] = {"DIST"};
            const auto mapping = Common::AutoPubClassMap_lookup(classes, 1, "VL6180");
            TEST_ASSERT(mapping.primaryKind == RTPSAutoPubMsgKind::Range,
                        "shared mapping: Range kind usable by legacy callers");
            TEST_ASSERT(std::strcmp(mapping.primaryTopicSlug, "range") == 0,
                        "shared mapping: Range topic slug unchanged");
            const uint16_t distanceRaw = 368;
            const Common::AutoPubAttrFieldDesc field{
                "dist", 0, Common::AutoPubAttrType::Uint16, "%u", 2.0f, 0.0f
            };
            Common::AutoPubCDRContext context;
            context.pFieldDescs = &field;
            context.fieldCount = 1;
            context.pStruct = reinterpret_cast<const uint8_t*>(&distanceRaw);
            context.structSize = sizeof(distanceRaw);
            context.timestampMs = 1234;
            context.frameId = "raft_range_1_29";
            uint8_t commonBytes[128]{};
            uint8_t legacyBytes[128]{};
            uint32_t commonLength = 0;
            uint32_t legacyLength = 0;
            TEST_ASSERT(Common::AutoPubCDRSerializer_serialize(
                            mapping.primaryKind, context, commonBytes, sizeof(commonBytes), commonLength),
                        "shared serializer: Range payload succeeds");
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(
                            mapping.primaryKind, context, legacyBytes, sizeof(legacyBytes), legacyLength),
                        "legacy serializer: shared Range context accepted");
            TEST_ASSERT(commonLength == 56 && commonLength == legacyLength &&
                            std::memcmp(commonBytes, legacyBytes, commonLength) == 0,
                        "shared serializer: legacy forwarding preserves exact Range bytes");
            TEST_ASSERT(!Common::AutoPubCDRSerializer_serialize(
                            mapping.primaryKind, context, commonBytes, 8, commonLength),
                        "shared serializer: short output buffer rejected");
            TEST_ASSERT(!RTPSAutoPubCDRSerializer_serialize(
                            mapping.primaryKind, context, legacyBytes, 8, legacyLength),
                        "legacy serializer: short output buffer rejected");
            TEST_ASSERT(commonLength == 0 && legacyLength == 0,
                        "shared serializer: failure clears both output lengths");
        }

        // Generated-struct mirrors (layout matches
        // RaftI2C/linux_unit_tests/DevicePollRecords_generated.h).
        struct pollAHT20    { uint32_t timeMs; uint8_t status; float humidity; float temperature; };
        struct pollMCP9808  { uint32_t timeMs; float temperature; };
        struct pollLPS25   { uint32_t timeMs; uint8_t status; float pressure; float temperature; };
        struct pollVL6180  { uint32_t timeMs; bool valid; float dist; };
        struct pollVCNL    { uint32_t timeMs; uint16_t prox; float als; float white; };
        struct pollADXL    { uint32_t timeMs; float x; float y; float z; };
        struct pollBtn     { uint32_t timeMs; bool press; };
        struct pollAS5600  { uint32_t timeMs; float angle; };
        struct pollM5Enc   { uint32_t timeMs; int32_t rotation; bool press; };
        struct pollLSM     { uint32_t timeMs; float gx, gy, gz, ax, ay, az; };
        struct pollHX711   { uint32_t timeMs; bool valid; float force; };
        struct pollCAP1203 { uint32_t timeMs; bool A; bool B; bool C; uint16_t status; };
        struct pollGamepad { uint32_t timeMs; float x; float y; int32_t SELECT, B, Y, A, X, START; };
        struct pollMax30101{ uint32_t timeMs; uint32_t Red; uint32_t IR; };

        // Helper: XCDR1 little-endian decoder for test assertions.
        auto le32 = [](const uint8_t* p) -> uint32_t {
            return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
        };
        auto lef32 = [&](const uint8_t* p) -> float {
            uint32_t bits = le32(p); float f; std::memcpy(&f, &bits, 4); return f;
        };
        auto lef64 = [](const uint8_t* p) -> double {
            uint64_t bits = 0;
            for (int i = 0; i < 8; i++) bits |= (uint64_t)p[i] << (i*8);
            double d; std::memcpy(&d, &bits, 8); return d;
        };
        auto nearly = [](double a, double b, double eps) { return std::fabs(a-b) <= eps; };

        uint8_t buf[512];
        uint32_t written = 0;

        // ---- Encapsulation header is always CDR_LE + 0x0000 options ----
        {
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serializeString("", buf, sizeof(buf), written),
                        "serializeString: empty ok");
            TEST_ASSERT(buf[0] == 0x00 && buf[1] == 0x01 && buf[2] == 0x00 && buf[3] == 0x00,
                        "serializeString: encap header = 00 01 00 00 (CDR_LE)");
            TEST_ASSERT(written >= 4 + 4 + 1, "serializeString: encap+len+null present");
        }

        // ---- Field reader helpers ----------------------------------------
        {
            pollAHT20 s{12345, 0, 42.5f, 23.1f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs",      offsetof(pollAHT20, timeMs),      RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"status",      offsetof(pollAHT20, status),      RTPSAutoPubAttrType::Uint8,  "", 1.0f, 0.0f},
                {"humidity",    offsetof(pollAHT20, humidity),    RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"temperature", offsetof(pollAHT20, temperature), RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs;
            ctx.fieldCount  = 4;
            ctx.pStruct     = reinterpret_cast<const uint8_t*>(&s);
            ctx.structSize  = sizeof(s);
            double v = 0.0;
            TEST_ASSERT(RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "temperature", v),
                        "readFieldDouble: temp found");
            TEST_ASSERT(nearly(v, 23.1, 1e-4), "readFieldDouble: temp value");
            TEST_ASSERT(RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "humidity", v),
                        "readFieldDouble: humidity found");
            TEST_ASSERT(nearly(v, 42.5, 1e-4), "readFieldDouble: humidity value");
            TEST_ASSERT(!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "nope", v),
                        "readFieldDouble: missing field returns false");
            // Divisor / addend applied
            descs[2].divisor = 10.0f; descs[2].addend = 1.0f;
            TEST_ASSERT(RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "humidity", v, true),
                        "readFieldDouble: scaled");
            TEST_ASSERT(nearly(v, 42.5/10.0 + 1.0, 1e-4),
                        "readFieldDouble: divisor+addend applied");
        }

        // ---- std_msgs/Bool (QwiicButton) ---------------------------------
        {
            pollBtn s{0, true};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollBtn, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"press",  offsetof(pollBtn, press),  RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 2;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Bool, ctx, buf, sizeof(buf), written),
                        "Bool: serialize ok");
            TEST_ASSERT(written == 4 + 1, "Bool: 4-byte encap + 1-byte bool = 5");
            TEST_ASSERT(buf[4] == 1, "Bool: press=true encoded as 1");
            s.press = false;
            RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Bool, ctx, buf, sizeof(buf), written);
            TEST_ASSERT(buf[4] == 0, "Bool: press=false encoded as 0");
        }

        // ---- std_msgs/Int32 (M5Encoder) ----------------------------------
        {
            pollM5Enc s{0, -1234, false};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs",   offsetof(pollM5Enc, timeMs),   RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"rotation", offsetof(pollM5Enc, rotation), RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
                {"press",    offsetof(pollM5Enc, press),    RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 3;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Int32, ctx, buf, sizeof(buf), written),
                        "Int32: serialize ok");
            TEST_ASSERT(written == 4 + 4, "Int32: encap + int32 = 8");
            int32_t got = (int32_t)le32(buf + 4);
            TEST_ASSERT(got == -1234, "Int32: rotation value round-trips");
        }

        // ---- std_msgs/Float32 (AS5600 ANG) -------------------------------
        {
            pollAS5600 s{0, 123.456f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollAS5600, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"angle",  offsetof(pollAS5600, angle),  RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 2;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Float32, ctx, buf, sizeof(buf), written),
                        "Float32: serialize ok");
            TEST_ASSERT(written == 4 + 4, "Float32: encap + float32 = 8");
            TEST_ASSERT(nearly(lef32(buf + 4), 123.456, 1e-3), "Float32: angle value");
        }

        // ---- sensor_msgs/Temperature (MCP9808) ---------------------------
        {
            pollMCP9808 s{7000, 24.75f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs",      offsetof(pollMCP9808, timeMs),      RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"temperature", offsetof(pollMCP9808, temperature), RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 2;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 7000;
            ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Temperature, ctx, buf, sizeof(buf), written),
                        "Temperature: serialize ok");
            // Layout: encap(4) + sec(int32) + nanosec(uint32) + frame_id(string) + temp(double) + variance(double)
            //                  pos 4         pos 8              pos 12
            //  sec = 7, nanosec = 0, frame "raft" len=5 w/ null → strlen(5)+pad to 8
            //  After frame_id @ pos 12+4+5=21. Align 8 → 24. temp @24..32. var @32..40.
            int32_t sec = (int32_t)le32(buf + 4);
            uint32_t ns = le32(buf + 8);
            TEST_ASSERT(sec == 7, "Temperature: Header sec = 7");
            TEST_ASSERT(ns == 0, "Temperature: Header nanosec = 0");
            uint32_t frameLen = le32(buf + 12);
            TEST_ASSERT(frameLen == 5, "Temperature: frame_id length = 5 (raft + null)");
            TEST_ASSERT(std::strncmp((char*)buf + 16, "raft", 4) == 0,
                        "Temperature: frame_id content = raft");
            // Alignment is relative to the body after the 4-byte encapsulation
            // header (as ROS 2 serialises: captured 2026-09-28): frame_id ends at
            // body 17, temp aligns to body 24 = absolute 28, variance at 36.
            TEST_ASSERT(nearly(lef64(buf + 28), 24.75, 1e-6),
                        "Temperature: temperature = 24.75");
            TEST_ASSERT(nearly(lef64(buf + 36), 0.0, 1e-12),
                        "Temperature: variance = 0");
            TEST_ASSERT(written == 44, "Temperature: payload size = 44");
        }

        // ---- sensor_msgs/RelativeHumidity: % → 0..1 -----------------------
        {
            pollAHT20 s{0, 0, 55.0f, 23.0f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs",      offsetof(pollAHT20, timeMs),      RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"status",      offsetof(pollAHT20, status),      RTPSAutoPubAttrType::Uint8,  "", 1.0f, 0.0f},
                {"humidity",    offsetof(pollAHT20, humidity),    RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"temperature", offsetof(pollAHT20, temperature), RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 4;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 1000; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::RelativeHumidity, ctx, buf, sizeof(buf), written),
                        "RelativeHumidity: serialize ok");
            // Same layout offsets as Temperature.
            TEST_ASSERT(nearly(lef64(buf + 28), 0.55, 1e-6),
                        "RelativeHumidity: 55% → 0.55");
        }

        // ---- sensor_msgs/FluidPressure: hPa → Pa --------------------------
        {
            pollLPS25 s{0, 0, 1013.25f, 22.0f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs",      offsetof(pollLPS25, timeMs),      RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"status",      offsetof(pollLPS25, status),      RTPSAutoPubAttrType::Uint8,  "", 1.0f, 0.0f},
                {"pressure",    offsetof(pollLPS25, pressure),    RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"temperature", offsetof(pollLPS25, temperature), RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 4;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 0; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::FluidPressure, ctx, buf, sizeof(buf), written),
                        "FluidPressure: serialize ok");
            TEST_ASSERT(nearly(lef64(buf + 28), 101325.0, 1.0),
                        "FluidPressure: 1013.25 hPa → 101325 Pa");
        }

        // ---- sensor_msgs/Illuminance (VCNL4040 als) ----------------------
        {
            pollVCNL s{0, 0, 350.5f, 100.0f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollVCNL, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"prox",   offsetof(pollVCNL, prox),   RTPSAutoPubAttrType::Uint16, "", 1.0f, 0.0f},
                {"als",    offsetof(pollVCNL, als),    RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"white",  offsetof(pollVCNL, white),  RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 4;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 0; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Illuminance, ctx, buf, sizeof(buf), written),
                        "Illuminance: serialize ok");
            TEST_ASSERT(nearly(lef64(buf + 28), 350.5, 1e-3),
                        "Illuminance: als = 350.5 lux");
        }

        // ---- sensor_msgs/Range (VL6180 dist in mm → m) --------------------
        {
            pollVL6180 s{0, true, 250.0f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollVL6180, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"valid",  offsetof(pollVL6180, valid),  RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
                {"dist",   offsetof(pollVL6180, dist),   RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 3;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 0; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Range, ctx, buf, sizeof(buf), written),
                        "Range: serialize ok");
            // Layout: encap(4) + hdr(4 sec + 4 ns + strlen(5)+pad to 4 → 4+4+5=17)
            //         radiation_type(uint8) @ pos 4+8+4+5 = 21
            //         float32 fov/min/max/range/variance aligned 4 → 24, 28, 32, 36, 40.
            TEST_ASSERT(buf[21] == 1, "Range: radiation_type = INFRARED (1)");
            TEST_ASSERT(nearly(lef32(buf + 24), 0.0, 1e-6), "Range: field_of_view = 0");
            TEST_ASSERT(nearly(lef32(buf + 28), 0.0, 1e-6), "Range: min_range = 0");
            TEST_ASSERT(nearly(lef32(buf + 32), 2.0, 1e-6), "Range: max_range = 2 m (DIST default)");
            TEST_ASSERT(nearly(lef32(buf + 36), 0.25, 1e-4), "Range: dist 250mm → 0.25 m");
            TEST_ASSERT(nearly(lef32(buf + 40), 0.0, 1e-6), "Range: variance = 0 (unknown)");
            TEST_ASSERT(written == 44, "Range: payload size = 44 bytes (Jazzy adds variance)");
        }

        // ---- sensor_msgs/Imu standalone ACC (ADXL313) --------------------
        {
            pollADXL s{0, 1.0f, 0.0f, 0.0f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollADXL, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"x",      offsetof(pollADXL, x),      RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"y",      offsetof(pollADXL, y),      RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"z",      offsetof(pollADXL, z),      RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 4;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 0; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Accel, ctx, buf, sizeof(buf), written),
                        "Accel: serialize ok");
            // Imu layout: encap + header + orientation(4×f64) + orient_cov(9×f64)
            //          + angular_velocity(3×f64) + ang_cov(9×f64)
            //          + linear_acceleration(3×f64) + lin_cov(9×f64).
            // header size: sec(4) + ns(4) + string(len4 + "raft\0" = 5) + pad
            //              = 4 + 4 + 4 + 5 = 17 → align 8 → 24.  Pos at header end = 4(encap) + 24 = 28? 
            // encap=4, sec@4, ns@8, strlen@12, bytes@16..21; body-relative align 8
            // → orient.x at body 24 = absolute 28 (ROS 2 aligns after the header).
            // orientation: x=0 y=0 z=0 w=1.
            TEST_ASSERT(nearly(lef64(buf + 28),  0.0, 1e-12), "Accel: orient.x = 0");
            TEST_ASSERT(nearly(lef64(buf + 28 + 8*3), 1.0, 1e-12), "Accel: orient.w = 1");
            // orient_covariance[0] = -1 (unknown) at offset 28 + 32 = 60.
            TEST_ASSERT(nearly(lef64(buf + 60), -1.0, 1e-12), "Accel: orient_cov[0] = -1");
            // After orient_cov (9×8=72 bytes) → 60+72 = 132. angular_velocity.x at 132.
            TEST_ASSERT(nearly(lef64(buf + 132), 0.0, 1e-12), "Accel: gx = 0");
            // ang_cov starts at 132 + 24 = 156. ang_cov[0] = -1 (unknown for accel-only).
            TEST_ASSERT(nearly(lef64(buf + 156), -1.0, 1e-12),
                        "Accel: angular_velocity_cov[0] = -1 (unknown)");
            // lin_accel.x at 156 + 72 = 228. Value: 1.0 g × 9.80665.
            TEST_ASSERT(nearly(lef64(buf + 228), 9.80665, 1e-6),
                        "Accel: ax = 1g → 9.80665 m/s²");
            // lin_cov at 228 + 24 = 252. lin_cov[0] = 0 (valid).
            TEST_ASSERT(nearly(lef64(buf + 252), 0.0, 1e-12),
                        "Accel: linear_accel_cov[0] = 0 (valid)");
            TEST_ASSERT(written == 252 + 72, "Accel: payload size");
        }

        // ---- sensor_msgs/Imu composite (LSM6DS) --------------------------
        {
            pollLSM s{0, 10.0f, -20.0f, 30.0f,     // gx,gy,gz (°/s)
                          0.5f,  0.0f,  0.0f};    // ax,ay,az (g)
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollLSM, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"gx",     offsetof(pollLSM, gx),     RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"gy",     offsetof(pollLSM, gy),     RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"gz",     offsetof(pollLSM, gz),     RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"ax",     offsetof(pollLSM, ax),     RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"ay",     offsetof(pollLSM, ay),     RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"az",     offsetof(pollLSM, az),     RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 7;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 0; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Imu, ctx, buf, sizeof(buf), written),
                        "Imu: serialize ok");
            // ang_cov[0] = 0 (valid) for composite Imu.
            TEST_ASSERT(nearly(lef64(buf + 156), 0.0, 1e-12),
                        "Imu: angular_velocity_cov[0] = 0 (valid for composite)");
            // gx: 10 °/s → 10 × π/180 ≈ 0.1745329
            TEST_ASSERT(nearly(lef64(buf + 132), 10.0 * 0.017453292519943295, 1e-9),
                        "Imu: gx = 10°/s → 0.1745 rad/s");
            // ax: 0.5 g → 0.5 × 9.80665
            TEST_ASSERT(nearly(lef64(buf + 228), 0.5 * 9.80665, 1e-6),
                        "Imu: ax = 0.5g → 4.903 m/s²");
        }

        // ---- geometry_msgs/Wrench (HX711 force on z axis) ----------------
        {
            pollHX711 s{0, true, 5.25f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollHX711, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"valid",  offsetof(pollHX711, valid),  RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
                {"force",  offsetof(pollHX711, force),  RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 3;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Wrench, ctx, buf, sizeof(buf), written),
                        "Wrench: serialize ok");
            // encap(4) + 6×float64: the first double is at body 0 = absolute 4,
            // no padding (alignment is body-relative) → 52 bytes.
            TEST_ASSERT(written == 52, "Wrench: 4 encap + 48 = 52 bytes");
            // force.z is the 3rd double → offset 4 + 16 = 20.
            TEST_ASSERT(nearly(lef64(buf + 20), 5.25, 1e-6), "Wrench: force.z = 5.25 N");
            TEST_ASSERT(nearly(lef64(buf + 4), 0.0, 1e-12), "Wrench: force.x = 0");
            TEST_ASSERT(nearly(lef64(buf + 16), 0.0, 1e-12), "Wrench: force.y = 0");
        }

        // ---- std_msgs/ByteMultiArray (CAP1203 touch) ---------------------
        {
            pollCAP1203 s{0, true, false, true, 0};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollCAP1203, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"A",      offsetof(pollCAP1203, A),      RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
                {"B",      offsetof(pollCAP1203, B),      RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
                {"C",      offsetof(pollCAP1203, C),      RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
                {"status", offsetof(pollCAP1203, status), RTPSAutoPubAttrType::Uint16, "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 5;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::ByteMultiArray, ctx, buf, sizeof(buf), written),
                        "ByteMultiArray: serialize ok");
            // Layout: encap(4) + layout.dim_len(uint32)=0 + layout.data_offset(uint32)=0
            //                  + data_len(uint32)=3 + data bytes.
            TEST_ASSERT(le32(buf + 4) == 0, "ByteMultiArray: dim[] length = 0");
            TEST_ASSERT(le32(buf + 8) == 0, "ByteMultiArray: data_offset = 0");
            TEST_ASSERT(le32(buf + 12) == 3, "ByteMultiArray: data length = 3 (A/B/C only)");
            TEST_ASSERT(buf[16] == 1 && buf[17] == 0 && buf[18] == 1,
                        "ByteMultiArray: A/B/C = 1/0/1");
        }

        // ---- std_msgs/Float32MultiArray (MAX30101 PPG) --------------------
        {
            pollMax30101 s{0, 1234, 5678};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollMax30101, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"Red",    offsetof(pollMax30101, Red),    RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"IR",     offsetof(pollMax30101, IR),     RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 3;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Float32MultiArray, ctx, buf, sizeof(buf), written),
                        "Float32MultiArray: serialize ok");
            TEST_ASSERT(le32(buf + 4) == 0, "Float32MultiArray: dim[] length = 0");
            TEST_ASSERT(le32(buf + 8) == 0, "Float32MultiArray: data_offset = 0");
            TEST_ASSERT(le32(buf + 12) == 2, "Float32MultiArray: data length = 2 (Red, IR)");
            TEST_ASSERT(nearly(lef32(buf + 16), 1234.0f, 1.0), "Float32MultiArray: Red value");
            TEST_ASSERT(nearly(lef32(buf + 20), 5678.0f, 1.0), "Float32MultiArray: IR value");
        }

        // ---- sensor_msgs/Joy (AdafruitGamepad) ---------------------------
        {
            pollGamepad s{0, 0.5f, -0.5f, 0, 1, 0, 1, 0, 0};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollGamepad, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"x",      offsetof(pollGamepad, x),      RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"y",      offsetof(pollGamepad, y),      RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"SELECT", offsetof(pollGamepad, SELECT), RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
                {"B",      offsetof(pollGamepad, B),      RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
                {"Y",      offsetof(pollGamepad, Y),      RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
                {"A",      offsetof(pollGamepad, A),      RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
                {"X",      offsetof(pollGamepad, X),      RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
                {"START",  offsetof(pollGamepad, START),  RTPSAutoPubAttrType::Int32,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 9;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 0; ctx.frameId = "raft";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(RTPSAutoPubMsgKind::Joy, ctx, buf, sizeof(buf), written),
                        "Joy: serialize ok");
            // Header ends at pos 21 ("raft\0" after 4-byte string-length).
            // axes_seq_len (uint32, align 4) → pad to 24.
            TEST_ASSERT(le32(buf + 24) == 2, "Joy: axes length = 2");
            TEST_ASSERT(nearly(lef32(buf + 28),  0.5, 1e-4), "Joy: axes[0] = x = 0.5");
            TEST_ASSERT(nearly(lef32(buf + 32), -0.5, 1e-4), "Joy: axes[1] = y = -0.5");
            // buttons_seq_len at 36 = 6.
            TEST_ASSERT(le32(buf + 36) == 6, "Joy: buttons length = 6");
        }

        // ---- std_msgs/String fallback (Slice 4.9) ------------------------
        {
            // Use an AHT20-like struct with three typed fields — exercises
            // float, bool, and uint16 paths of the JSON body writer.
            struct pollFallback {
                uint32_t timeMs;
                float    temp;      // scaled via divisor
                uint16_t count;     // int path
                uint8_t  press;     // bool path (type Bool)
            } __attribute__((packed));
            pollFallback s{100, 21.5f, 7, 1};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs", offsetof(pollFallback, timeMs), RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"temp",   offsetof(pollFallback, temp),   RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
                {"count",  offsetof(pollFallback, count),  RTPSAutoPubAttrType::Uint16, "", 1.0f, 0.0f},
                {"press",  offsetof(pollFallback, press),  RTPSAutoPubAttrType::Bool,   "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 4;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            ctx.timestampMs = 42;
            ctx.frameId = "raft";

            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(
                            RTPSAutoPubMsgKind::String, ctx, buf, sizeof(buf), written),
                        "String: serialize ok");
            uint32_t slen = le32(buf + 4);
            TEST_ASSERT(slen > 0 && slen < 200, "String: length bounded");
            TEST_ASSERT(buf[4 + 4 + slen - 1] == 0, "String: null-terminated");
            const char* body = reinterpret_cast<const char*>(buf + 8);
            TEST_ASSERT(body[0] == '{', "String: JSON starts with {");
            TEST_ASSERT(body[slen - 2] == '}', "String: JSON ends with }");
            TEST_ASSERT(std::strstr(body, "\"ts\":42") != nullptr,
                        "String: includes ts=42");
            TEST_ASSERT(std::strstr(body, "\"temp\":21.5") != nullptr,
                        "String: includes temp=21.5");
            TEST_ASSERT(std::strstr(body, "\"count\":7") != nullptr,
                        "String: includes count=7");
            TEST_ASSERT(std::strstr(body, "\"press\":true") != nullptr,
                        "String: press renders as true");
            TEST_ASSERT(std::strstr(body, "timeMs") == nullptr,
                        "String: timeMs field is skipped (ts replaces it)");
        }

        // ---- std_msgs/String fallback: empty field list ------------------
        {
            RTPSAutoPubCDRContext ctx{};
            ctx.timestampMs = 1234;
            ctx.fieldCount = 0;
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serialize(
                            RTPSAutoPubMsgKind::String, ctx, buf, sizeof(buf), written),
                        "String: serialize empty ok");
            const char* body = reinterpret_cast<const char*>(buf + 8);
            TEST_ASSERT(std::strstr(body, "\"ts\":1234}") != nullptr,
                        "String: empty fields produces {\"ts\":1234}");
        }

        // ---- std_msgs/String helper: explicit body -----------------------
        {
            const char* payload = "{\"hello\":\"world\"}";
            TEST_ASSERT(RTPSAutoPubCDRSerializer_serializeString(
                            payload, buf, sizeof(buf), written),
                        "serializeString: ok");
            uint32_t slen = le32(buf + 4);
            TEST_ASSERT(slen == std::strlen(payload) + 1,
                        "serializeString: length matches");
            TEST_ASSERT(std::strcmp(reinterpret_cast<const char*>(buf + 8), payload) == 0,
                        "serializeString: body matches");
        }

        // ---- Unknown kind rejected ---------------------------------------
        {
            RTPSAutoPubCDRContext ctx{};
            TEST_ASSERT(!RTPSAutoPubCDRSerializer_serialize(
                            RTPSAutoPubMsgKind::Unknown, ctx, buf, sizeof(buf), written),
                        "Unknown kind returns false");
        }

        // ---- Buffer-too-small is rejected --------------------------------
        {
            pollMCP9808 s{0, 24.0f};
            RTPSAutoPubAttrFieldDesc descs[] = {
                {"timeMs",      offsetof(pollMCP9808, timeMs),      RTPSAutoPubAttrType::Uint32, "", 1.0f, 0.0f},
                {"temperature", offsetof(pollMCP9808, temperature), RTPSAutoPubAttrType::Float,  "", 1.0f, 0.0f},
            };
            RTPSAutoPubCDRContext ctx{};
            ctx.pFieldDescs = descs; ctx.fieldCount = 2;
            ctx.pStruct = reinterpret_cast<const uint8_t*>(&s); ctx.structSize = sizeof(s);
            uint8_t tiny[8];
            TEST_ASSERT(!RTPSAutoPubCDRSerializer_serialize(
                            RTPSAutoPubMsgKind::Temperature, ctx, tiny, sizeof(tiny), written),
                        "Temperature: rejects 8-byte buffer");
        }
    }

    {
        printf("Test: common decoded-sample runner and synchronous backend handoff\n");
        using namespace RaftRuntime::AutoPub;
        struct PollRecord { uint32_t timeMs; float temperature; float humidity; };
        const PollRecord records[] = {{1000, 10.0f, 20.0f}, {2345, 23.5f, 56.0f}};
        const AutoPubAttrFieldDesc fields[] = {
            {"temperature", offsetof(PollRecord, temperature), AutoPubAttrType::Float, "", 1.0f, 0.0f},
            {"humidity", offsetof(PollRecord, humidity), AutoPubAttrType::Float, "", 1.0f, 0.0f}
        };
        AutoPubDecodedBatch batch;
        batch.data = reinterpret_cast<const uint8_t*>(records);
        batch.capacity = sizeof(records);
        batch.recordSize = sizeof(PollRecord);
        batch.recordCount = 2;
        batch.fields = fields;
        batch.fieldCount = 2;
        uint8_t payloads[2][128]{};
        AutoPubSampleOutput outputs[] = {
            {AutoPubMsgKind::Temperature, payloads[0], sizeof(payloads[0])},
            {AutoPubMsgKind::RelativeHumidity, payloads[1], sizeof(payloads[1])}
        };
        AutoPubSampleResult results[2];
        unsigned calls = 0;
        std::vector<uint8_t> copiedPayloads[2];
        auto publish = [&](uint8_t outputIndex, const uint8_t* data, uint32_t length,
                           uint32_t timestampMs) {
            TEST_ASSERT(outputIndex == calls, "sample runner: primary precedes secondary");
            TEST_ASSERT(timestampMs == 2345, "sample runner: latest record timestamp");
            TEST_ASSERT(data == outputs[outputIndex].data, "sample runner: borrows caller output");
            copiedPayloads[outputIndex].assign(data, data + length);
            ++calls;
            return outputIndex == 0 ? AutoPubPublishResult::Accepted : AutoPubPublishResult::QueueFull;
        };
        TEST_ASSERT(AutoPubSampleRunner::run(batch, outputs, 2, results, publish),
                    "sample runner: valid composite batch");
        TEST_ASSERT(calls == 2 && results[0].serialized && results[1].serialized,
                    "sample runner: emits both composite outputs");
        TEST_ASSERT(results[0].publishResult == AutoPubPublishResult::Accepted &&
                        results[1].publishResult == AutoPubPublishResult::QueueFull,
                    "sample runner: preserves independent backend outcomes");
        AutoPubCDRContext expectedContext;
        expectedContext.pFieldDescs = fields;
        expectedContext.fieldCount = 2;
        expectedContext.pStruct = reinterpret_cast<const uint8_t*>(&records[1]);
        expectedContext.structSize = sizeof(PollRecord);
        expectedContext.timestampMs = 2345;
        uint8_t expected[128]{};
        uint32_t expectedLength = 0;
        for (uint8_t outputIndex = 0; outputIndex < 2; ++outputIndex)
        {
            TEST_ASSERT(AutoPubCDRSerializer_serialize(outputs[outputIndex].kind, expectedContext,
                            expected, sizeof(expected), expectedLength),
                        "sample runner: reference serialization succeeds");
            TEST_ASSERT(results[outputIndex].bytesWritten == expectedLength &&
                            copiedPayloads[outputIndex].size() == expectedLength &&
                            std::memcmp(copiedPayloads[outputIndex].data(), expected, expectedLength) == 0,
                        "sample runner: latest-record bytes match serializer");
            std::memset(payloads[outputIndex], 0xff, sizeof(payloads[outputIndex]));
            TEST_ASSERT(std::memcmp(copiedPayloads[outputIndex].data(), expected, expectedLength) == 0,
                        "sample runner: backend copy survives caller buffer reuse");
        }
        calls = 0;
        auto publishSecondary = [&](uint8_t outputIndex, const uint8_t*, uint32_t, uint32_t) {
            TEST_ASSERT(outputIndex == 1, "sample runner: failed primary is not published");
            ++calls;
            return AutoPubPublishResult::Disconnected;
        };
        outputs[0].capacity = 8;
        TEST_ASSERT(AutoPubSampleRunner::run(batch, outputs, 2, results, publishSecondary),
                    "sample runner: output failure does not invalidate batch");
        TEST_ASSERT(calls == 1 && !results[0].serialized && results[0].bytesWritten == 0 &&
                        results[0].publishResult == AutoPubPublishResult::NotAttempted &&
                        results[1].publishResult == AutoPubPublishResult::Disconnected,
                    "sample runner: secondary continues after primary serialization failure");
        auto noPublish = [&](uint8_t, const uint8_t*, uint32_t, uint32_t) {
            ++calls;
            return AutoPubPublishResult::Accepted;
        };
        calls = 0;
        const uint32_t invalidCounts[] = {0, 3, UINT32_MAX};
        for (uint32_t recordCount : invalidCounts)
        {
            batch.recordCount = recordCount;
            TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                        "sample runner: rejects empty or out-of-bounds decoded batch");
            TEST_ASSERT(results[0].bytesWritten == 0 && !results[1].serialized &&
                            results[1].publishResult == AutoPubPublishResult::NotAttempted,
                        "sample runner: invalid batch clears prior results");
        }
        batch.recordCount = 2;
        batch.recordSize = 0;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: rejects zero stride");
        batch.recordSize = UINT32_MAX;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: rejects oversized stride without multiplication overflow");
        batch.recordSize = sizeof(PollRecord);
        batch.capacity = sizeof(records) - 1;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: rejects truncated last record");
        batch.capacity = sizeof(records);
        batch.data = nullptr;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: rejects null decoded storage");
        batch.data = reinterpret_cast<const uint8_t*>(records);
        batch.fields = nullptr;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: rejects missing field descriptors");
        batch.fields = fields;
        batch.fieldCount = 0;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: rejects empty field descriptors");
        batch.fieldCount = 2;
        TEST_ASSERT(!AutoPubSampleRunner::run(batch, nullptr, 2, results, noPublish) &&
                        !AutoPubSampleRunner::run(batch, outputs, 2, nullptr, noPublish) &&
                        !AutoPubSampleRunner::run(batch, outputs, 0, results, noPublish) &&
                        !AutoPubSampleRunner::run(batch, outputs, 3, results, noPublish),
                    "sample runner: validates output and result bounds");
        outputs[0].kind = AutoPubMsgKind::Unknown;
        outputs[1].data = nullptr;
        TEST_ASSERT(AutoPubSampleRunner::run(batch, outputs, 2, results, noPublish),
                    "sample runner: disabled outputs are skipped");
        TEST_ASSERT(calls == 0 && results[0].publishResult == AutoPubPublishResult::NotAttempted &&
                        results[1].publishResult == AutoPubPublishResult::NotAttempted,
                    "sample runner: no publish on invalid batches or disabled outputs");
    }

    {
        printf("Test: RTPS sample emitter adapter\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;
        using RaftRuntime::AutoPub::AutoPubPublishResult;
        RTPSDynamicWriterRegistry registry;
        const int slot = registry.allocate({1, 0x29, 0}, "rt/raft/range", "sensor_msgs::msg::dds_::Range_");
        auto* entry = registry.getMutable(static_cast<uint8_t>(slot));
        TEST_ASSERT(entry != nullptr, "RTPS emitter: test writer allocated");
        if (entry)
        {
            const uint8_t payload[] = {0, 1, 0, 0, 42};
            const std::vector<unsigned> peers{0, 1, 2};
            unsigned calls = 0;
            auto sendToPeer = [&](unsigned peer, const RTPSDynamicWriterEntry& writer,
                                  const uint8_t* data, uint32_t length, uint64_t sequence) {
                TEST_ASSERT(peer == calls, "RTPS emitter: all peers attempted in order");
                TEST_ASSERT(&writer == entry && sequence == 1 && writer.seqNum == sequence,
                            "RTPS emitter: one sequence shared by all peer sends");
                TEST_ASSERT(data == payload && length == sizeof(payload),
                            "RTPS emitter: CDR bytes passed without modification");
                ++calls;
                return peer == 1 ? AutoPubPublishResult::Accepted : AutoPubPublishResult::SendFailed;
            };
            const auto emission = RTPSAutoPubSampleEmitter_emit(entry, payload, sizeof(payload), peers, sendToPeer);
            TEST_ASSERT(calls == 3 && emission.sequence == 1 && emission.peersSent == 1 &&
                            emission.result == AutoPubPublishResult::Accepted,
                        "RTPS emitter: partial peer success remains accepted");
            const auto noPeers = RTPSAutoPubSampleEmitter_emit(
                entry, payload, sizeof(payload), std::vector<unsigned>{}, sendToPeer);
            TEST_ASSERT(noPeers.sequence == 2 && noPeers.peersSent == 0 && entry->seqNum == 2 &&
                            noPeers.result == AutoPubPublishResult::Disconnected && calls == 3,
                        "RTPS emitter: no peers still advances sequence exactly once");
            const auto empty = RTPSAutoPubSampleEmitter_emit(entry, payload, 0, peers, sendToPeer);
            const auto nullPayload = RTPSAutoPubSampleEmitter_emit(entry, nullptr, sizeof(payload), peers, sendToPeer);
            TEST_ASSERT(empty.result == AutoPubPublishResult::NotAttempted &&
                            nullPayload.result == AutoPubPublishResult::NotAttempted && entry->seqNum == 2,
                        "RTPS emitter: empty payload does not consume sequence");
            for (auto failure : {AutoPubPublishResult::Oversized, AutoPubPublishResult::SendFailed})
            {
                const auto failed = RTPSAutoPubSampleEmitter_emit(entry, payload, sizeof(payload), peers,
                    [&](unsigned, const RTPSDynamicWriterEntry&, const uint8_t*, uint32_t, uint64_t) {
                        return failure;
                    });
                TEST_ASSERT(failed.result == failure && failed.peersSent == 0 &&
                                failed.sequence == entry->seqNum,
                            "RTPS emitter: failed sends report outcome without successful peers");
            }
            TEST_ASSERT(entry->seqNum == 4, "RTPS emitter: one sequence per sample, including failed sends");
            registry.releaseSlot(static_cast<uint8_t>(slot));
            const auto detached = RTPSAutoPubSampleEmitter_emit(entry, payload, sizeof(payload), peers, sendToPeer);
            const auto invalid = RTPSAutoPubSampleEmitter_emit(nullptr, payload, sizeof(payload), peers, sendToPeer);
            TEST_ASSERT(detached.result == AutoPubPublishResult::InvalidHandle &&
                            invalid.result == AutoPubPublishResult::InvalidHandle &&
                            detached.sequence == 0 && calls == 3,
                        "RTPS emitter: absent or released writer never sends");
        }
    }

    {
        printf("Test: publisher pool generation-safe handles and latest-only mailboxes\n");
        using namespace RaftRuntime::AutoPub;
        using Pool = AutoPubPublisherPool<AutoPubTestLock, 3, 8>;
        Pool pool;
        int users[4] = {};
        auto fillWith = [](uint8_t value, uint32_t length, int* pCalls = nullptr) {
            return [=](void*, uint8_t* scratch, uint32_t recordSize) -> uint32_t {
                if (pCalls)
                    ++*pCalls;
                std::memset(scratch, value, recordSize);
                return length;
            };
        };

        // Handle encoding
        AutoPubPublisherHandle none;
        TEST_ASSERT(!none.isValid() && none.toCallbackInfo() == nullptr &&
                        !AutoPubPublisherHandle::fromCallbackInfo(nullptr).isValid(),
                    "publisher pool: invalid handle encodes as null and null decodes invalid");

        // Acquire validation and capacity
        TEST_ASSERT(!pool.acquire(nullptr, 4).isValid() && !pool.acquire(&users[0], 0).isValid() &&
                        !pool.acquire(&users[0], 9).isValid() && pool.inUseCount() == 0,
                    "publisher pool: rejects null user, empty and oversized records");
        const AutoPubPublisherHandle h0 = pool.acquire(&users[0], 4);
        const AutoPubPublisherHandle h1 = pool.acquire(&users[1], 8);
        const AutoPubPublisherHandle h2 = pool.acquire(&users[2], 2);
        TEST_ASSERT(h0.isValid() && h1.isValid() && h2.isValid() &&
                        h0.slot != h1.slot && h1.slot != h2.slot && h0.slot != h2.slot &&
                        !pool.acquire(&users[3], 4).isValid() && pool.inUseCount() == 3,
                    "publisher pool: distinct slots up to capacity, then full");
        const AutoPubPublisherHandle h0Decoded = AutoPubPublisherHandle::fromCallbackInfo(h0.toCallbackInfo());
        TEST_ASSERT(h0.toCallbackInfo() != nullptr && h0Decoded.slot == h0.slot &&
                        h0Decoded.generation == h0.generation,
                    "publisher pool: handle round-trips through callback info");

        // Store, drain, latest-only overwrite
        struct Drained { AutoPubPublisherHandle handle; void* pUser; std::vector<uint8_t> bytes;
                         uint32_t produced; uint32_t overwritten; };
        std::vector<Drained> drained;
        auto collect = [&](const AutoPubDrainedSample& s) {
            drained.push_back({s.handle, s.pUser, std::vector<uint8_t>(s.record, s.record + s.length),
                               s.produced, s.overwritten});
        };
        TEST_ASSERT(pool.drain(collect) == 0 && drained.empty(), "publisher pool: nothing pending initially");
        TEST_ASSERT(pool.produce(h0, fillWith(0x11, 4), 2) == AutoPubProduceResult::Stored &&
                        pool.produce(h0, fillWith(0x22, 4), 2) == AutoPubProduceResult::Overwritten &&
                        pool.produce(h1, fillWith(0x33, 8), 2) == AutoPubProduceResult::Stored,
                    "publisher pool: store then overwrite undrained sample");
        TEST_ASSERT(pool.drain(collect) == 2 && drained.size() == 2, "publisher pool: drains every pending slot once");
        bool latestOnly = false;
        bool secondSlot = false;
        for (const auto& d : drained)
        {
            if (d.handle.slot == h0.slot)
                latestOnly = d.pUser == &users[0] && d.handle.generation == h0.generation &&
                             d.bytes == std::vector<uint8_t>(4, 0x22) && d.produced == 2 && d.overwritten == 1;
            if (d.handle.slot == h1.slot)
                secondSlot = d.pUser == &users[1] && d.bytes == std::vector<uint8_t>(8, 0x33) &&
                             d.produced == 1 && d.overwritten == 0;
        }
        TEST_ASSERT(latestOnly, "publisher pool: latest record, user and counters delivered");
        TEST_ASSERT(secondSlot, "publisher pool: independent slot delivered with its own record size");
        drained.clear();
        TEST_ASSERT(pool.drain(collect) == 0, "publisher pool: drained sample is not repeated");

        // Empty or oversized fills never disturb a pending sample
        TEST_ASSERT(pool.produce(h0, fillWith(0x44, 4), 2) == AutoPubProduceResult::Stored &&
                        pool.produce(h0, fillWith(0x55, 0), 2) == AutoPubProduceResult::Empty &&
                        pool.produce(h0, fillWith(0x66, 5), 2) == AutoPubProduceResult::Empty,
                    "publisher pool: empty and over-length fills are rejected");
        pool.drain(collect);
        TEST_ASSERT(drained.size() == 1 && drained[0].bytes == std::vector<uint8_t>(4, 0x44) &&
                        drained[0].produced == 3 && drained[0].overwritten == 1,
                    "publisher pool: rejected fill leaves pending record and counters intact");
        drained.clear();

        // A producer may run while the consumer handles a sample (lock not held)
        pool.produce(h2, fillWith(0x77, 2), 2);
        AutoPubProduceResult reentrant = AutoPubProduceResult::Busy;
        pool.drain([&](const AutoPubDrainedSample& s) {
            reentrant = pool.produce(s.handle, fillWith(0x78, 2), 2);
            collect(s);
        });
        TEST_ASSERT(reentrant == AutoPubProduceResult::Stored && drained.size() == 1 &&
                        drained[0].bytes == std::vector<uint8_t>(2, 0x77),
                    "publisher pool: consumer runs outside the lock on a stable copy");
        drained.clear();
        pool.drain(collect);
        TEST_ASSERT(drained.size() == 1 && drained[0].bytes == std::vector<uint8_t>(2, 0x78),
                    "publisher pool: sample stored during consume is drained next pass");
        drained.clear();

        // Busy lock drops the sample without calling fill
        int fillCalls = 0;
        AutoPubTestLock::failNextTimedLocks = 1;
        TEST_ASSERT(pool.produce(h0, fillWith(0x88, 4, &fillCalls), 2) == AutoPubProduceResult::Busy &&
                        fillCalls == 0 && pool.counters().busyDrops == 1,
                    "publisher pool: busy lock drops sample and counts it");

        // Drain never waits: a held lock skips the slot, keeping its sample
        TEST_ASSERT(h0.slot == 0 && pool.produce(h0, fillWith(0xDD, 4), 2) == AutoPubProduceResult::Stored,
                    "publisher pool: sample pending in first slot");
        AutoPubTestLock::failNextTimedLocks = 1;
        TEST_ASSERT(pool.drain(collect) == 0 && drained.empty() && pool.counters().drainBusySkips == 1,
                    "publisher pool: drain skips a busy slot without waiting");
        TEST_ASSERT(pool.drain(collect) == 1 && drained.size() == 1 &&
                        drained[0].bytes == std::vector<uint8_t>(4, 0xDD),
                    "publisher pool: skipped sample drained on the next pass");
        drained.clear();

        // Release invalidates in-flight/stale handles and discards pending data
        pool.produce(h0, fillWith(0x99, 4), 2);
        TEST_ASSERT(pool.release(h0) && !pool.release(h0) && pool.inUseCount() == 2,
                    "publisher pool: release succeeds once");
        TEST_ASSERT(pool.produce(h0, fillWith(0xAA, 4, &fillCalls), 2) == AutoPubProduceResult::Stale &&
                        fillCalls == 0,
                    "publisher pool: stale handle rejected before user state is reached");
        pool.drain(collect);
        TEST_ASSERT(drained.empty(), "publisher pool: release discards undrained sample");

        // Slot reuse gets a new generation; the old handle stays dead
        const AutoPubPublisherHandle h0Reused = pool.acquire(&users[3], 4);
        TEST_ASSERT(h0Reused.isValid() && h0Reused.slot == h0.slot &&
                        h0Reused.generation != h0.generation,
                    "publisher pool: reused slot gets a fresh generation");
        TEST_ASSERT(pool.produce(h0, fillWith(0xBB, 4, &fillCalls), 2) == AutoPubProduceResult::Stale &&
                        fillCalls == 0 && !pool.release(h0) && pool.inUseCount() == 3,
                    "publisher pool: old handle cannot produce into or release the new owner");
        void* reusedUser = nullptr;
        TEST_ASSERT(pool.produce(h0Reused, [&](void* pUser, uint8_t* scratch, uint32_t) -> uint32_t {
                            reusedUser = pUser;
                            std::memset(scratch, 0xCC, 4);
                            return 4;
                        }, 2) == AutoPubProduceResult::Stored && reusedUser == &users[3],
                    "publisher pool: new owner receives its own user pointer");
        pool.drain(collect);
        TEST_ASSERT(drained.size() == 1 && drained[0].pUser == &users[3] && drained[0].produced == 1,
                    "publisher pool: reused slot counters restart");
        drained.clear();

        AutoPubPublisherHandle forged;
        forged.slot = 7;
        forged.generation = 1;
        TEST_ASSERT(pool.produce(forged, fillWith(0, 1), 2) == AutoPubProduceResult::Stale &&
                        pool.produce(none, fillWith(0, 1), 2) == AutoPubProduceResult::Stale &&
                        !pool.release(forged),
                    "publisher pool: out-of-range and invalid handles rejected");
        const auto counters = pool.counters();
        TEST_ASSERT(counters.staleDrops == 4 && counters.busyDrops == 1 && counters.emptyFills == 2,
                    "publisher pool: diagnostic counters");
    }

    {
        printf("Test: shared auto-pub attach plan (device -> endpoints)\n");
        using namespace RaftRuntime::AutoPub;
        // Record what the plan asks about so override resolution is verifiable
        std::vector<std::string> askedAliases;
        auto resolveQoS = [&](const char* alias, const char* const*, size_t, const char*) {
            askedAliases.push_back(std::string(alias));
            return std::string(alias).rfind("humidity", 0) == 0 ? AutoPubQoSProfileId::SlowSensor
                                                                : AutoPubQoSProfileId::FastSensor;
        };
        const AutoPubDeviceId devId{1, 0x29, 0};

        // Single-endpoint device (DIST -> Range)
        const char* distClas[] = {"DIST"};
        auto plan = AutoPubAttachPlan_build(devId, distClas, 1, "VL6180", resolveQoS);
        TEST_ASSERT(!plan.excluded && plan.endpointCount == 1, "attach plan: single endpoint for DIST");
        TEST_ASSERT(std::string(plan.endpoints[0].topic) == "/raft/range_1_29" &&
                        std::string(plan.endpoints[0].type) == "sensor_msgs::msg::dds_::Range_" &&
                        plan.endpoints[0].msgKind == AutoPubMsgKind::Range,
                    "attach plan: DIST topic/type/kind");
        TEST_ASSERT(plan.endpoints[0].deviceId.busNum == 1 && plan.endpoints[0].deviceId.address == 0x29 &&
                        plan.endpoints[0].deviceId.subIndex == 0,
                    "attach plan: primary carries device identity with subIndex 0");
        TEST_ASSERT(plan.endpoints[0].qosProfileId == AutoPubQoSProfileId::FastSensor &&
                        askedAliases.size() == 1 && askedAliases[0] == std::string("range_1_29"),
                    "attach plan: QoS resolved using the topic alias");

        // Composite device (TEMP+RH -> Temperature + RelativeHumidity)
        askedAliases.clear();
        const char* compositeClas[] = {"TEMP", "RH"};
        const AutoPubDeviceId compositeId{1, 0x38, 0};
        auto composite = AutoPubAttachPlan_build(compositeId, compositeClas, 2, "AHT20", resolveQoS);
        TEST_ASSERT(composite.endpointCount == 2, "attach plan: composite yields two endpoints");
        TEST_ASSERT(composite.endpoints[0].msgKind == AutoPubMsgKind::Temperature &&
                        composite.endpoints[1].msgKind == AutoPubMsgKind::RelativeHumidity,
                    "attach plan: composite kinds in primary/secondary order");
        TEST_ASSERT(composite.endpoints[1].deviceId.subIndex == 1 &&
                        composite.endpoints[0].deviceId.subIndex == 0,
                    "attach plan: composite endpoints get distinct subIndex");
        TEST_ASSERT(std::string(composite.endpoints[0].topic) != std::string(composite.endpoints[1].topic),
                    "attach plan: composite endpoints get distinct topics");
        TEST_ASSERT(composite.endpoints[1].qosProfileId == AutoPubQoSProfileId::SlowSensor &&
                        askedAliases.size() == 2,
                    "attach plan: each endpoint resolves its own QoS");

        // Actuators publish nothing
        const char* actuatorClas[] = {"SRVO"};
        auto excluded = AutoPubAttachPlan_build(devId, actuatorClas, 1, "SERVO", resolveQoS);
        TEST_ASSERT(excluded.excluded && excluded.endpointCount == 0,
                    "attach plan: actuator excluded with no endpoints");

        // Unknown class falls back to std_msgs/String
        const char* unknownClas[] = {"ZZZZ"};
        auto fallback = AutoPubAttachPlan_build(devId, unknownClas, 1, "MysteryDev", resolveQoS);
        TEST_ASSERT(fallback.endpointCount == 1 &&
                        std::string(fallback.endpoints[0].type) == "std_msgs::msg::dds_::String_" &&
                        std::string(fallback.endpoints[0].topic) == "/raft/raw_1_29",
                    "attach plan: unknown class falls back to raw String topic");

        // No class tags at all still yields the fallback endpoint
        auto noClas = AutoPubAttachPlan_build(devId, nullptr, 0, nullptr, resolveQoS);
        TEST_ASSERT(noClas.endpointCount == 1 && noClas.endpoints[0].isValid(),
                    "attach plan: missing class tags still produce a valid endpoint");

        // Descriptor rejects names that would truncate
        AutoPubEndpointDesc desc;
        std::string longTopic(AUTOPUB_TOPIC_MAX_LEN + 4, 'x');
        TEST_ASSERT(!desc.setNames(longTopic.c_str(), "t") && !desc.isValid(),
                    "endpoint desc: rejects over-long topic");
        TEST_ASSERT(!desc.setNames(nullptr, nullptr) && !desc.setNames("", "t") &&
                        !desc.setNames("t", ""),
                    "endpoint desc: rejects null/empty names");
        TEST_ASSERT(desc.setNames("/raft/range_1_29", "sensor_msgs::msg::dds_::Range_") &&
                        desc.isValid() && std::string(desc.topic) == "/raft/range_1_29",
                    "endpoint desc: accepts and owns valid names");
    }

    //=================================================================
    // Phase 4 / Slice 4.11 — RTPSAutoPubQoSProfile
    //=================================================================
    {
        printf("Test: RTPSAutoPubQoSProfile (Phase 4 Slice 4.11)\n");
        using namespace RaftRuntime::RTPS::Runtime::AutoPub;

        // ---- Built-in profile table ---------------------------------
        {
            auto fast = RTPSAutoPubQoSProfile_get(RTPSAutoPubQoSProfileId::FastSensor);
            TEST_ASSERT(fast.id == RTPSAutoPubQoSProfileId::FastSensor,
                        "fast_sensor id");
            TEST_ASSERT(fast.reliability == RTPS_AUTOPUB_RELIABILITY_BEST_EFFORT,
                        "fast_sensor reliability=BEST_EFFORT");
            TEST_ASSERT(fast.durability == RTPS_AUTOPUB_DURABILITY_VOLATILE,
                        "fast_sensor durability=VOLATILE");
            TEST_ASSERT(fast.historyDepth == 10, "fast_sensor depth=10");

            auto slow = RTPSAutoPubQoSProfile_get(RTPSAutoPubQoSProfileId::SlowSensor);
            TEST_ASSERT(slow.reliability == RTPS_AUTOPUB_RELIABILITY_RELIABLE,
                        "slow_sensor reliability=RELIABLE");
            TEST_ASSERT(slow.durability == RTPS_AUTOPUB_DURABILITY_VOLATILE,
                        "slow_sensor durability=VOLATILE");
            TEST_ASSERT(slow.historyDepth == 5, "slow_sensor depth=5");

            auto ev = RTPSAutoPubQoSProfile_get(RTPSAutoPubQoSProfileId::Event);
            TEST_ASSERT(ev.durability == RTPS_AUTOPUB_DURABILITY_TRANSIENT_LOCAL,
                        "event durability=TRANSIENT_LOCAL");
            TEST_ASSERT(ev.historyDepth == 20, "event depth=20");

            auto fb = RTPSAutoPubQoSProfile_get(RTPSAutoPubQoSProfileId::FallbackString);
            TEST_ASSERT(fb.reliability == RTPS_AUTOPUB_RELIABILITY_RELIABLE,
                        "fallback_string reliability=RELIABLE");
            TEST_ASSERT(fb.historyDepth == 10, "fallback_string depth=10");
        }

        // ---- Name parsing round-trip --------------------------------
        {
            RTPSAutoPubQoSProfileId id = RTPSAutoPubQoSProfileId::FallbackString;
            TEST_ASSERT(RTPSAutoPubQoSProfile_parseName("fast_sensor", id) &&
                        id == RTPSAutoPubQoSProfileId::FastSensor,
                        "parseName fast_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_parseName("slow_sensor", id) &&
                        id == RTPSAutoPubQoSProfileId::SlowSensor,
                        "parseName slow_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_parseName("event", id) &&
                        id == RTPSAutoPubQoSProfileId::Event,
                        "parseName event");
            TEST_ASSERT(RTPSAutoPubQoSProfile_parseName("fallback_string", id) &&
                        id == RTPSAutoPubQoSProfileId::FallbackString,
                        "parseName fallback_string");
            TEST_ASSERT(!RTPSAutoPubQoSProfile_parseName("bogus", id),
                        "parseName rejects unknown");
            TEST_ASSERT(!RTPSAutoPubQoSProfile_parseName(nullptr, id),
                        "parseName rejects null");
            TEST_ASSERT(!RTPSAutoPubQoSProfile_parseName("", id),
                        "parseName rejects empty");

            TEST_ASSERT(std::strcmp(
                RTPSAutoPubQoSProfile_name(RTPSAutoPubQoSProfileId::FastSensor),
                "fast_sensor") == 0, "name(FastSensor)");
            TEST_ASSERT(std::strcmp(
                RTPSAutoPubQoSProfile_name(RTPSAutoPubQoSProfileId::Event),
                "event") == 0, "name(Event)");
        }

        // ---- Per-class default table --------------------------------
        {
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("ACC") ==
                        RTPSAutoPubQoSProfileId::FastSensor, "ACC -> fast_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("GYRO") ==
                        RTPSAutoPubQoSProfileId::FastSensor, "GYRO -> fast_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("LGHT") ==
                        RTPSAutoPubQoSProfileId::FastSensor, "LGHT -> fast_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("FRCE") ==
                        RTPSAutoPubQoSProfileId::FastSensor, "FRCE -> fast_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("TEMP") ==
                        RTPSAutoPubQoSProfileId::SlowSensor, "TEMP -> slow_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("RH") ==
                        RTPSAutoPubQoSProfileId::SlowSensor, "RH -> slow_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("PRES") ==
                        RTPSAutoPubQoSProfileId::SlowSensor, "PRES -> slow_sensor");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("BTN") ==
                        RTPSAutoPubQoSProfileId::Event, "BTN -> event");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("ROT") ==
                        RTPSAutoPubQoSProfileId::Event, "ROT -> event");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("GAME") ==
                        RTPSAutoPubQoSProfileId::Event, "GAME -> event");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("ZZZZ") ==
                        RTPSAutoPubQoSProfileId::FallbackString,
                        "unknown class -> fallback_string");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass(nullptr) ==
                        RTPSAutoPubQoSProfileId::FallbackString,
                        "null class -> fallback_string");
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClass("") ==
                        RTPSAutoPubQoSProfileId::FallbackString,
                        "empty class -> fallback_string");
        }

        // ---- defaultForClasses composite + first-wins ---------------
        {
            // Composite IMU (ACC+GYRO) must promote to fast_sensor even if
            // GYRO comes first and would map to fast_sensor alone — and even
            // if extra classes follow.
            const char* imuClas[] = {"ACC", "GYRO"};
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClasses(imuClas, 2) ==
                        RTPSAutoPubQoSProfileId::FastSensor,
                        "composite ACC+GYRO -> fast_sensor");

            const char* imuClasRev[] = {"GYRO", "ACC"};
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClasses(imuClasRev, 2) ==
                        RTPSAutoPubQoSProfileId::FastSensor,
                        "composite order-independent");

            // AHT20-like: TEMP+RH → first real hit wins (both slow_sensor).
            const char* aht[] = {"TEMP", "RH"};
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClasses(aht, 2) ==
                        RTPSAutoPubQoSProfileId::SlowSensor,
                        "TEMP+RH -> slow_sensor");

            // Unknown → fallback_string.
            const char* unknown[] = {"ZZZZ"};
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClasses(unknown, 1) ==
                        RTPSAutoPubQoSProfileId::FallbackString,
                        "unknown classes -> fallback_string");

            // Empty input.
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClasses(nullptr, 0) ==
                        RTPSAutoPubQoSProfileId::FallbackString,
                        "null classes -> fallback_string");

            // First-real-match-wins: {ZZZZ, BTN} should pick BTN → event.
            const char* mix[] = {"ZZZZ", "BTN"};
            TEST_ASSERT(RTPSAutoPubQoSProfile_defaultForClasses(mix, 2) ==
                        RTPSAutoPubQoSProfileId::Event,
                        "mixed [unknown, BTN] -> event");
        }

        // ---- Lifecycle + registry carry profile id through ----------
        {
            RTPSAutoPubLifecycle life;
            uint8_t eid[4] = {0};
            const uint8_t fastId =
                static_cast<uint8_t>(RTPSAutoPubQoSProfileId::FastSensor);
            const int slot = life.attach({1, 0x6A}, "rt/raft/imu_1_6a",
                                         "sensor_msgs::msg::dds_::Imu_",
                                         eid, fastId);
            TEST_ASSERT(slot == 0, "attach with qosProfileId -> slot 0");
            const auto* e = life.get(0);
            TEST_ASSERT(e && e->qosProfileId == fastId,
                        "registry stores qosProfileId");

            // Default (no profile id arg) -> FallbackString (=3).
            int slot2 = life.attach({1, 0x38}, "rt/raft/temp_1_38",
                                    "sensor_msgs::msg::dds_::Temperature_",
                                    nullptr);
            TEST_ASSERT(slot2 == 1, "second attach -> slot 1");
            TEST_ASSERT(life.get(1)->qosProfileId ==
                        static_cast<uint8_t>(RTPSAutoPubQoSProfileId::FallbackString),
                        "default qosProfileId = FallbackString (3)");
        }
    }

    //=================================================================
    // Summary
    //=================================================================
    printf("\n--- Results: %d passed, %d failed ---\n", passCount, failCount);
    if (failCount > 0)
        printf("FAILED %d tests\n", failCount);
    else
        printf("All tests passed\n");

    return failCount > 0 ? 1 : 0;
}
