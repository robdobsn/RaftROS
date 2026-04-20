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
#include "RTPSMessage.h"
#include "RTPSTypes.h"
#include "RTPSParticipant.h"
#include "SPDPHandler.h"
#include "SEDPHandler.h"

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
    // Summary
    //=================================================================
    printf("\n--- Results: %d passed, %d failed ---\n", passCount, failCount);
    if (failCount > 0)
        printf("FAILED %d tests\n", failCount);
    else
        printf("All tests passed\n");

    return failCount > 0 ? 1 : 0;
}
