/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CDR Decoder - Common Data Representation deserialization
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CDRDecoder.h"

CDRDecoder::CDRDecoder()
{
}

CDRDecoder::~CDRDecoder()
{
}

void CDRDecoder::init(const uint8_t* pBuf, uint32_t bufLen)
{
    _pBuf = pBuf;
    _bufLen = bufLen;
    _pos = 0;
    _origin = 0;
    _littleEndian = true;
}

bool CDRDecoder::readEncapsulationHeader()
{
    // CDR encapsulation: 2 bytes options, then 2 bytes padding
    if (_pos + 4 > _bufLen)
        return false;
    // Byte 0: 0x00, Byte 1: 0x01 = LE, 0x00 = BE
    _littleEndian = (_pBuf[_pos + 1] == 0x01);
    _pos += 4;
    _origin = _pos;
    return true;
}

bool CDRDecoder::align(uint32_t alignment)
{
    uint32_t remainder = (_pos - _origin) % alignment;
    if (remainder != 0)
        _pos += alignment - remainder;
    return _pos <= _bufLen;
}

bool CDRDecoder::readBool(bool& val)
{
    if (_pos + 1 > _bufLen)
        return false;
    val = (_pBuf[_pos] != 0);
    _pos += 1;
    return true;
}

bool CDRDecoder::readUint8(uint8_t& val)
{
    if (_pos + 1 > _bufLen)
        return false;
    val = _pBuf[_pos];
    _pos += 1;
    return true;
}

bool CDRDecoder::readInt8(int8_t& val)
{
    if (_pos + 1 > _bufLen)
        return false;
    val = static_cast<int8_t>(_pBuf[_pos]);
    _pos += 1;
    return true;
}

bool CDRDecoder::readUint16(uint16_t& val)
{
    if (!align(2) || _pos + 2 > _bufLen)
        return false;
    if (_littleEndian)
        val = _pBuf[_pos] | (_pBuf[_pos + 1] << 8);
    else
        val = (_pBuf[_pos] << 8) | _pBuf[_pos + 1];
    _pos += 2;
    return true;
}

bool CDRDecoder::readInt16(int16_t& val)
{
    uint16_t uval;
    if (!readUint16(uval))
        return false;
    val = static_cast<int16_t>(uval);
    return true;
}

bool CDRDecoder::readUint32(uint32_t& val)
{
    if (!align(4) || _pos + 4 > _bufLen)
        return false;
    if (_littleEndian)
        val = _pBuf[_pos] | (_pBuf[_pos + 1] << 8) | (_pBuf[_pos + 2] << 16) | (_pBuf[_pos + 3] << 24);
    else
        val = (_pBuf[_pos] << 24) | (_pBuf[_pos + 1] << 16) | (_pBuf[_pos + 2] << 8) | _pBuf[_pos + 3];
    _pos += 4;
    return true;
}

bool CDRDecoder::readInt32(int32_t& val)
{
    uint32_t uval;
    if (!readUint32(uval))
        return false;
    val = static_cast<int32_t>(uval);
    return true;
}

bool CDRDecoder::readUint64(uint64_t& val)
{
    if (!align(8) || _pos + 8 > _bufLen)
        return false;
    if (_littleEndian)
    {
        val = static_cast<uint64_t>(_pBuf[_pos])
            | (static_cast<uint64_t>(_pBuf[_pos + 1]) << 8)
            | (static_cast<uint64_t>(_pBuf[_pos + 2]) << 16)
            | (static_cast<uint64_t>(_pBuf[_pos + 3]) << 24)
            | (static_cast<uint64_t>(_pBuf[_pos + 4]) << 32)
            | (static_cast<uint64_t>(_pBuf[_pos + 5]) << 40)
            | (static_cast<uint64_t>(_pBuf[_pos + 6]) << 48)
            | (static_cast<uint64_t>(_pBuf[_pos + 7]) << 56);
    }
    else
    {
        val = (static_cast<uint64_t>(_pBuf[_pos]) << 56)
            | (static_cast<uint64_t>(_pBuf[_pos + 1]) << 48)
            | (static_cast<uint64_t>(_pBuf[_pos + 2]) << 40)
            | (static_cast<uint64_t>(_pBuf[_pos + 3]) << 32)
            | (static_cast<uint64_t>(_pBuf[_pos + 4]) << 24)
            | (static_cast<uint64_t>(_pBuf[_pos + 5]) << 16)
            | (static_cast<uint64_t>(_pBuf[_pos + 6]) << 8)
            | static_cast<uint64_t>(_pBuf[_pos + 7]);
    }
    _pos += 8;
    return true;
}

bool CDRDecoder::readInt64(int64_t& val)
{
    uint64_t uval;
    if (!readUint64(uval))
        return false;
    val = static_cast<int64_t>(uval);
    return true;
}

bool CDRDecoder::readFloat32(float& val)
{
    uint32_t uval;
    if (!readUint32(uval))
        return false;
    memcpy(&val, &uval, sizeof(float));
    return true;
}

bool CDRDecoder::readFloat64(double& val)
{
    uint64_t uval;
    if (!readUint64(uval))
        return false;
    memcpy(&val, &uval, sizeof(double));
    return true;
}

bool CDRDecoder::readString(char* buf, uint32_t bufLen, uint32_t& strLen)
{
    // CDR string: uint32 length (including null), then chars + null
    uint32_t len;
    if (!readUint32(len))
        return false;
    strLen = (len > 0) ? len - 1 : 0;
    if (_pos + len > _bufLen)
        return false;
    uint32_t copyLen = (strLen < bufLen - 1) ? strLen : bufLen - 1;
    if (buf && bufLen > 0)
    {
        memcpy(buf, _pBuf + _pos, copyLen);
        buf[copyLen] = '\0';
    }
    _pos += len;
    return true;
}

bool CDRDecoder::readSequenceLength(uint32_t& count)
{
    return readUint32(count);
}

bool CDRDecoder::skip(uint32_t count)
{
    if (_pos + count > _bufLen)
        return false;
    _pos += count;
    return true;
}
