/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CDR Encoder - Common Data Representation serialization
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CDREncoder.h"

CDREncoder::CDREncoder()
{
}

CDREncoder::~CDREncoder()
{
}

void CDREncoder::reset(uint8_t* pBuf, uint32_t bufLen)
{
    _pBuf = pBuf;
    _bufLen = bufLen;
    _pos = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Encapsulation header (4 bytes at start of CDR payload)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CDREncoder::writeEncapsulationHeader(bool littleEndian)
{
    _littleEndian = littleEndian;
    if (_pos + 4 > _bufLen)
        return false;
    // Encapsulation scheme (DDSI-RTPS §10.2)
    // Bytes 0-1: encapsulation kind
    //   0x00 0x01 = CDR_BE (big-endian)
    //   0x00 0x02 = CDR_LE (little-endian)
    _pBuf[_pos++] = 0x00;
    _pBuf[_pos++] = littleEndian ? 0x01 : 0x00;
    // Bytes 2-3: options (zero)
    _pBuf[_pos++] = 0x00;
    _pBuf[_pos++] = 0x00;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Alignment
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CDREncoder::align(uint32_t alignment)
{
    uint32_t remainder = _pos % alignment;
    if (remainder == 0)
        return true;
    uint32_t padding = alignment - remainder;
    if (_pos + padding > _bufLen)
        return false;
    memset(_pBuf + _pos, 0, padding);
    _pos += padding;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Primitive types
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CDREncoder::writeBool(bool val)
{
    if (_pos + 1 > _bufLen)
        return false;
    _pBuf[_pos++] = val ? 1 : 0;
    return true;
}

bool CDREncoder::writeUint8(uint8_t val)
{
    if (_pos + 1 > _bufLen)
        return false;
    _pBuf[_pos++] = val;
    return true;
}

bool CDREncoder::writeInt8(int8_t val)
{
    return writeUint8(static_cast<uint8_t>(val));
}

bool CDREncoder::writeUint16(uint16_t val)
{
    if (!align(2))
        return false;
    if (_pos + 2 > _bufLen)
        return false;
    if (_littleEndian)
    {
        _pBuf[_pos++] = val & 0xFF;
        _pBuf[_pos++] = (val >> 8) & 0xFF;
    }
    else
    {
        _pBuf[_pos++] = (val >> 8) & 0xFF;
        _pBuf[_pos++] = val & 0xFF;
    }
    return true;
}

bool CDREncoder::writeInt16(int16_t val)
{
    return writeUint16(static_cast<uint16_t>(val));
}

bool CDREncoder::writeUint32(uint32_t val)
{
    if (!align(4))
        return false;
    if (_pos + 4 > _bufLen)
        return false;
    if (_littleEndian)
    {
        _pBuf[_pos++] = val & 0xFF;
        _pBuf[_pos++] = (val >> 8) & 0xFF;
        _pBuf[_pos++] = (val >> 16) & 0xFF;
        _pBuf[_pos++] = (val >> 24) & 0xFF;
    }
    else
    {
        _pBuf[_pos++] = (val >> 24) & 0xFF;
        _pBuf[_pos++] = (val >> 16) & 0xFF;
        _pBuf[_pos++] = (val >> 8) & 0xFF;
        _pBuf[_pos++] = val & 0xFF;
    }
    return true;
}

bool CDREncoder::writeInt32(int32_t val)
{
    return writeUint32(static_cast<uint32_t>(val));
}

bool CDREncoder::writeUint64(uint64_t val)
{
    if (!align(8))
        return false;
    if (_pos + 8 > _bufLen)
        return false;
    if (_littleEndian)
    {
        for (int i = 0; i < 8; i++)
            _pBuf[_pos++] = (val >> (i * 8)) & 0xFF;
    }
    else
    {
        for (int i = 7; i >= 0; i--)
            _pBuf[_pos++] = (val >> (i * 8)) & 0xFF;
    }
    return true;
}

bool CDREncoder::writeInt64(int64_t val)
{
    return writeUint64(static_cast<uint64_t>(val));
}

bool CDREncoder::writeFloat32(float val)
{
    uint32_t bits;
    memcpy(&bits, &val, sizeof(bits));
    return writeUint32(bits);
}

bool CDREncoder::writeFloat64(double val)
{
    uint64_t bits;
    memcpy(&bits, &val, sizeof(bits));
    return writeUint64(bits);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// String
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CDREncoder::writeString(const char* str)
{
    uint32_t len = str ? (uint32_t)strlen(str) + 1 : 1; // Include null terminator
    if (!writeUint32(len))
        return false;
    if (str && len > 1)
    {
        if (_pos + len > _bufLen)
            return false;
        memcpy(_pBuf + _pos, str, len);
        _pos += len;
    }
    else
    {
        if (_pos + 1 > _bufLen)
            return false;
        _pBuf[_pos++] = 0;
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Raw bytes
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CDREncoder::writeBytes(const uint8_t* pData, uint32_t len)
{
    if (_pos + len > _bufLen)
        return false;
    memcpy(_pBuf + _pos, pData, len);
    _pos += len;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sequence length
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CDREncoder::writeSequenceLength(uint32_t count)
{
    return writeUint32(count);
}
