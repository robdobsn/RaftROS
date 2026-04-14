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
#include "utils.h"
#include "CDREncoder.h"
#include "CDRDecoder.h"
#include "RTPSMessage.h"

#define TEST_ASSERT(cond, msg) if (!(cond)) { printf("TEST_ASSERT failed %s\n", msg); failCount++; }

bool isApprox(float a, float b, float tol = 0.0001f)
{
    return std::fabs(a - b) < tol;
}

int main()
{
    int failCount = 0;

    // Test CDR Encoder/Decoder round-trip
    {
        printf("Testing CDR Encoder/Decoder round-trip...\n");

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

    // Test CDR Encoder buffer management
    {
        printf("Testing CDR Encoder buffer management...\n");

        uint8_t encBuf[64];
        CDREncoder encoder;
        encoder.reset(encBuf, sizeof(encBuf));
        encoder.writeUint32(0xDEADBEEF);
        encoder.writeUint32(0xCAFEBABE);
        TEST_ASSERT(encoder.getPos() == 8, "CDR encoder size after two uint32s");

        // Little endian: 0xEF 0xBE 0xAD 0xDE
        TEST_ASSERT(encBuf[0] == 0xEF && encBuf[1] == 0xBE && encBuf[2] == 0xAD && encBuf[3] == 0xDE, "CDR encoder LE uint32 byte order");
    }

    // Test CDR alignment
    {
        printf("Testing CDR alignment...\n");

        uint8_t encBuf[64];
        CDREncoder encoder;
        encoder.reset(encBuf, sizeof(encBuf));
        encoder.writeUint8(0x01);       // pos 1
        encoder.writeUint32(0x12345678); // should align to pos 4
        TEST_ASSERT(encoder.getPos() == 8, "CDR alignment: uint8 then uint32 should be 8 bytes");
    }

    // Test RTPS message header
    {
        printf("Testing RTPS message header...\n");

        std::vector<uint8_t> buf(20, 0);
        uint8_t guidPrefix[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
        RTPSMessage::writeHeader(buf.data(), buf.size(), guidPrefix);

        // Verify RTPS magic
        TEST_ASSERT(buf[0] == 'R' && buf[1] == 'T' && buf[2] == 'P' && buf[3] == 'S', "RTPS header magic");

        // Verify version 2.2
        TEST_ASSERT(buf[4] == 2 && buf[5] == 2, "RTPS header version");

        // Parse it back
        uint8_t parsedGuid[12];
        uint32_t offset = RTPSMessage::parseHeader(buf.data(), buf.size(), parsedGuid);
        TEST_ASSERT(offset == 20, "RTPS parseHeader success");
        TEST_ASSERT(memcmp(guidPrefix, parsedGuid, 12) == 0, "RTPS parsed guidPrefix");
    }

    // Summary
    if (failCount > 0)
        printf("FAILED %d tests\n", failCount);
    else
        printf("All tests passed\n");

    return failCount > 0 ? 1 : 0;
}
