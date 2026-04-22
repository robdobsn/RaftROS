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
#include <vector>
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
#include "runtime/announce/RTPSWriterHeartbeatRunner.h"
#include "runtime/dispatch/RTPSUserDispatch.h"

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
        uint8_t msgBuf[512];
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
        TEST_ASSERT(msgLen < 400, "SEDP pub message under 400 bytes");

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

        // Gid at offset 4: first 16 bytes = guid, next 8 = zeros
        TEST_ASSERT(memcmp(buf+4, guid, 16)==0, "ros_disc_info gid matches guid");
        uint8_t zeros[8] = {};
        TEST_ASSERT(memcmp(buf+20, zeros, 8)==0, "ros_disc_info gid padding zeros");

        // Sequence length at offset 28 = 1
        uint32_t seqLen = buf[28] | (buf[29]<<8) | (buf[30]<<16) | (buf[31]<<24);
        TEST_ASSERT(seqLen == 1, "ros_disc_info seq length = 1");

        // node_namespace string at offset 32 (first in CDR order): length=7 ("/my_ns\0")
        uint32_t nsLen = buf[32] | (buf[33]<<8) | (buf[34]<<16) | (buf[35]<<24);
        TEST_ASSERT(nsLen == 7, "ros_disc_info node_namespace length = 7");
        TEST_ASSERT(memcmp(buf+36, "/my_ns", 6)==0, "ros_disc_info node_namespace content");

        // node_name string follows at offset 32+4+8(padded)=44: length=8 ("my_node\0")
        uint32_t nameOff = 32 + 4 + ((nsLen + 3) & ~3u);
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

        // ros_discovery_info is TRANSIENT_LOCAL: firstSN override must NOT be set
        RTPSAckActionUserDataPlan rosDiscPlan = {};
        TEST_ASSERT(getAckActionUserDataPlan(
                        RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo,
                        espUserCtx,
                        rosDiscPlan),
                    "ros_discovery_info retransmit plan produced");
        TEST_ASSERT(!rosDiscPlan.hasFirstSNOverride,
                    "ros_discovery_info plan does not override firstSN (TRANSIENT_LOCAL)");
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
        TEST_ASSERT(d.ackNackBitmap == 0, "no SNs received -> all NACK bits clear");
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
        TEST_ASSERT(d.ackNackBitmap == 0, "no SNs in [4..5] received");

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
    // Summary
    //=================================================================
    printf("\n--- Results: %d passed, %d failed ---\n", passCount, failCount);
    if (failCount > 0)
        printf("FAILED %d tests\n", failCount);
    else
        printf("All tests passed\n");

    return failCount > 0 ? 1 : 0;
}
