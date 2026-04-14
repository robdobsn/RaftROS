/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CDR Decoder - Common Data Representation deserialization
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

/// CDR Decoder for DDS message deserialization
/// Implements XCDR1 (plain CDR) decoding used by ROS 2 / DDS
class CDRDecoder
{
public:
    CDRDecoder();
    ~CDRDecoder();

    // Initialize decoder with buffer
    void init(const uint8_t* pBuf, uint32_t bufLen);

    // Read CDR encapsulation header
    bool readEncapsulationHeader();

    // Primitive types
    bool readBool(bool& val);
    bool readUint8(uint8_t& val);
    bool readInt8(int8_t& val);
    bool readUint16(uint16_t& val);
    bool readInt16(int16_t& val);
    bool readUint32(uint32_t& val);
    bool readInt32(int32_t& val);
    bool readUint64(uint64_t& val);
    bool readInt64(int64_t& val);
    bool readFloat32(float& val);
    bool readFloat64(double& val);

    // String
    bool readString(char* buf, uint32_t bufLen, uint32_t& strLen);

    // Sequence length
    bool readSequenceLength(uint32_t& count);

    // Skip bytes
    bool skip(uint32_t count);

    // Get current position
    uint32_t getPos() const { return _pos; }

    // Check if little endian
    bool isLittleEndian() const { return _littleEndian; }

private:
    const uint8_t* _pBuf = nullptr;
    uint32_t _bufLen = 0;
    uint32_t _pos = 0;
    bool _littleEndian = true;

    bool align(uint32_t alignment);
};
