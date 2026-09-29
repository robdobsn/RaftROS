/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CDR Encoder - Common Data Representation serialization
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

/// CDR Encoder for DDS message serialization
/// Implements XCDR1 (plain CDR) encoding used by ROS 2 / DDS
/// Spec: OMG Common Data Representation (CDR) / DDSI-RTPS §10
class CDREncoder
{
public:
    CDREncoder();
    ~CDREncoder();

    // Reset encoder to start of buffer
    void reset(uint8_t* pBuf, uint32_t bufLen);

    // Write CDR encapsulation header (4 bytes)
    bool writeEncapsulationHeader(bool littleEndian = true);

    // Primitive types
    bool writeBool(bool val);
    bool writeUint8(uint8_t val);
    bool writeInt8(int8_t val);
    bool writeUint16(uint16_t val);
    bool writeInt16(int16_t val);
    bool writeUint32(uint32_t val);
    bool writeInt32(int32_t val);
    bool writeUint64(uint64_t val);
    bool writeInt64(int64_t val);
    bool writeFloat32(float val);
    bool writeFloat64(double val);

    // String (length-prefixed, null-terminated in CDR)
    bool writeString(const char* str);

    // Raw bytes (for sequences/arrays)
    bool writeBytes(const uint8_t* pData, uint32_t len);

    // Sequence header (uint32 length prefix)
    bool writeSequenceLength(uint32_t count);

    // Get current write position
    uint32_t getPos() const { return _pos; }

private:
    uint8_t* _pBuf = nullptr;
    uint32_t _bufLen = 0;
    uint32_t _pos = 0;
    // Alignment is relative to the start of the serialized body, which is
    // after the 4-byte encapsulation header when one was written (XCDR1):
    // an int64 following a uint8 at body offset 1 lands at body offset 8, so
    // absolute offset 12, as a real ROS 2 node serialises it (captured
    // 2026-09-28: a scalar rcl_interfaces/ParameterValue is 52 body bytes).
    uint32_t _origin = 0;
    bool _littleEndian = true;

    // Align write position to N-byte boundary
    bool align(uint32_t alignment);
};
