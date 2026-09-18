#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace RaftRuntime::Zenoh
{

template<size_t Capacity>
class ZenohStreamFramer
{
public:
    static_assert(Capacity > 0 && Capacity <= 65535, "Zenoh TCP batch capacity must fit uint16");

    enum class Result
    {
        Incomplete,
        Complete,
        InvalidLength
    };

    Result push(uint8_t byte)
    {
        if (_invalid)
            return Result::InvalidLength;
        if (_headerBytes == 2 && _received == _length)
            reset();
        if (_headerBytes < 2)
        {
            _length |= static_cast<size_t>(byte) << (8 * _headerBytes);
            ++_headerBytes;
            if (_headerBytes == 2 && (_length == 0 || _length > Capacity))
            {
                _invalid = true;
                return Result::InvalidLength;
            }
            return Result::Incomplete;
        }
        _bytes[_received++] = byte;
        return _received == _length ? Result::Complete : Result::Incomplete;
    }

    void reset()
    {
        _length = 0;
        _received = 0;
        _headerBytes = 0;
        _invalid = false;
    }

    const uint8_t* data() const { return _bytes.data(); }
    size_t size() const { return _received; }

private:
    std::array<uint8_t, Capacity> _bytes{};
    size_t _length = 0;
    size_t _received = 0;
    uint8_t _headerBytes = 0;
    bool _invalid = false;
};

}