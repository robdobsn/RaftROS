#pragma once

// Zenoh serial link framing (z-serial), shared by zenoh and zenoh-pico.
//
// One Zenoh transport batch per frame.  Before COBS:
//   header (1)  I=0x01 init, A=0x02 ack, R=0x04 reset, 0 for data
//   length (2)  payload length, little-endian
//   payload     0..1500 bytes - one batch, no TCP length prefix
//   CRC-32 (4)  little-endian, over the payload only
// The whole frame is COBS-encoded and ended with a single 0x00.
//
// The CRC is z-serial's, not zlib's: a reflected table algorithm (shift right,
// init 0xFFFFFFFF, final inversion) built on the unreflected polynomial
// 0x04C11DB7.  tools/zenoh_serial_link.py holds the same code and the test
// vectors from z-serial.

#include <array>
#include <cstddef>
#include <cstdint>

namespace RaftRuntime::Zenoh
{

namespace ZenohSerialFrame
{
    static constexpr uint8_t FLAG_INIT = 0x01;
    static constexpr uint8_t FLAG_ACK = 0x02;
    static constexpr uint8_t FLAG_RESET = 0x04;
    static constexpr size_t MAX_PAYLOAD = 1500;
    static constexpr size_t OVERHEAD = 1 + 2 + 4;
    static constexpr size_t MAX_RAW = MAX_PAYLOAD + OVERHEAD;
    /// COBS adds a code byte per 254 and one at the start, plus the delimiter
    static constexpr size_t encodedSize(size_t payload) { return payload + OVERHEAD + (payload + OVERHEAD) / 254 + 2; }
    static constexpr size_t MAX_ENCODED = encodedSize(MAX_PAYLOAD);
    /// z-serial's own receive bound (COBS_BUF_SIZE), which the decoder honours
    static constexpr size_t MAX_ENCODED_RX = 1517;

    inline uint32_t crcTableEntry(uint32_t index)
    {
        uint32_t value = index;
        for (int bit = 0; bit < 8; ++bit)
            value = (value & 1) ? (0x04C11DB7u ^ (value >> 1)) : (value >> 1);
        return value;
    }

    inline const std::array<uint32_t, 256>& crcTable()
    {
        static const std::array<uint32_t, 256> table = []() {
            std::array<uint32_t, 256> entries{};
            for (uint32_t index = 0; index < 256; ++index)
                entries[index] = crcTableEntry(index);
            return entries;
        }();
        return table;
    }

    inline uint32_t crc32(const uint8_t* data, size_t length)
    {
        const auto& table = crcTable();
        uint32_t acc = 0xFFFFFFFFu;
        for (size_t index = 0; index < length; ++index)
            acc = (acc >> 8) ^ table[(acc & 0xFF) ^ data[index]];
        return ~acc;
    }

    /// @brief Build an encoded frame, delimiter included
    /// @return bytes written, or 0 if the payload is too big or out too small
    inline size_t encode(uint8_t header, const uint8_t* payload, size_t length, uint8_t* out, size_t capacity)
    {
        if (length > MAX_PAYLOAD || (length && !payload) || !out || capacity < encodedSize(length))
            return 0;
        const uint32_t crc = crc32(payload, length);
        size_t written = 1;
        size_t codeIndex = 0;
        uint8_t code = 1;
        auto put = [&](uint8_t byte) {
            if (byte == 0)
            {
                out[codeIndex] = code;
                codeIndex = written++;
                code = 1;
                return;
            }
            out[written++] = byte;
            if (++code == 0xFF)
            {
                out[codeIndex] = code;
                codeIndex = written++;
                code = 1;
            }
        };
        put(header);
        put(static_cast<uint8_t>(length));
        put(static_cast<uint8_t>(length >> 8));
        for (size_t index = 0; index < length; ++index)
            put(payload[index]);
        for (int shift = 0; shift < 32; shift += 8)
            put(static_cast<uint8_t>(crc >> shift));
        out[codeIndex] = code;
        out[written++] = 0;
        return written;
    }
}

/// @brief Byte-fed frame decoder.  Bounded, never allocates, and resynchronises
/// on the next 0x00 after anything it cannot decode.
class ZenohSerialDecoder
{
public:
    enum class Result { Incomplete, Frame, Error };

    Result push(uint8_t byte)
    {
        if (byte != 0)
        {
            if (_size < _encoded.size())
                _encoded[_size++] = byte;
            else
                _overflow = true;
            return Result::Incomplete;
        }
        const bool overflow = _overflow;
        const size_t size = _size;
        _size = 0;
        _overflow = false;
        if (size == 0)
            return Result::Incomplete;      // a lone delimiter is not a frame
        if (overflow)
        {
            ++_oversize;
            return Result::Error;
        }
        if (!decode(size))
        {
            ++_malformed;
            return Result::Error;
        }
        const size_t length = static_cast<size_t>(_raw[1]) | (static_cast<size_t>(_raw[2]) << 8);
        if (_rawSize != length + ZenohSerialFrame::OVERHEAD)
        {
            ++_malformed;
            return Result::Error;
        }
        const uint8_t* crcBytes = _raw.data() + 3 + length;
        const uint32_t crc = static_cast<uint32_t>(crcBytes[0]) | (static_cast<uint32_t>(crcBytes[1]) << 8) |
                             (static_cast<uint32_t>(crcBytes[2]) << 16) | (static_cast<uint32_t>(crcBytes[3]) << 24);
        if (crc != ZenohSerialFrame::crc32(_raw.data() + 3, length))
        {
            ++_crcErrors;
            return Result::Error;
        }
        ++_frames;
        _payloadSize = length;
        return Result::Frame;
    }

    void reset() { _size = 0; _overflow = false; }

    uint8_t header() const { return _raw[0]; }
    const uint8_t* payload() const { return _raw.data() + 3; }
    size_t payloadSize() const { return _payloadSize; }

    uint32_t frames() const { return _frames; }
    uint32_t crcErrors() const { return _crcErrors; }
    uint32_t malformed() const { return _malformed; }
    uint32_t oversize() const { return _oversize; }
    uint32_t errors() const { return _crcErrors + _malformed + _oversize; }

private:
    bool decode(size_t size)
    {
        size_t in = 0;
        _rawSize = 0;
        while (in < size)
        {
            const uint8_t code = _encoded[in++];
            const size_t end = in + code - 1;
            if (end > size || end - in > _raw.size() - _rawSize)
                return false;
            while (in < end)
                _raw[_rawSize++] = _encoded[in++];
            if (code != 0xFF && in < size)
            {
                if (_rawSize == _raw.size())
                    return false;
                _raw[_rawSize++] = 0;
            }
        }
        return _rawSize >= ZenohSerialFrame::OVERHEAD;
    }

    std::array<uint8_t, ZenohSerialFrame::MAX_ENCODED_RX> _encoded{};
    std::array<uint8_t, ZenohSerialFrame::MAX_RAW> _raw{};
    size_t _size = 0, _rawSize = 0, _payloadSize = 0;
    bool _overflow = false;
    uint32_t _frames = 0, _crcErrors = 0, _malformed = 0, _oversize = 0;
};

}
