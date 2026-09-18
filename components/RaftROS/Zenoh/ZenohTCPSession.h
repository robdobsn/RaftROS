#pragma once

#include "ZenohStreamFramer.h"
#include "ZenohNetworkMessage.h"
#include <cstring>
#include <limits>

namespace RaftRuntime::Zenoh
{

class ZenohTCPSession
{
public:
    static constexpr size_t BATCH_CAPACITY = 4096;
    static constexpr size_t COOKIE_CAPACITY = 1024;
    static constexpr uint64_t HANDSHAKE_TIMEOUT_MS = 5000;
    static constexpr uint64_t LOCAL_LEASE_MS = 4000;
    static constexpr uint8_t PROTOCOL_VERSION = 9;

    enum class State { Idle, AwaitInitAck, AwaitOpenAck, Established, Closed, Failed };
    enum class Error { None, InvalidIdentity, InvalidBatch, MalformedMessage, UnexpectedMessage,
                       UnsupportedNegotiation, HandshakeTimeout, LeaseExpired, SendStalled, LinkLost,
                       SequenceMismatch, DiscoveryRejected };
    using DiscoveryCallback = bool (*)(void*, const ZenohNetworkMessage::DiscoveryMessage&);

    bool start(const std::array<uint8_t, 16>& identity, uint64_t nowMs)
    {
        if (_state == State::AwaitInitAck || _state == State::AwaitOpenAck || _state == State::Established)
            return false;
        _framer.reset();
        _outputLength = _outputOffset = 0;
        _state = State::Idle;
        _error = Error::None;
        _negotiatedBatch = BATCH_CAPACITY;
        _sequenceBits = 32;
        _closeReason = 0;
        _remoteLeaseMs = _remoteInitialSequence = 0;
        _keepAlivesReceived = _frameCount = _discoveryCount = 0;
        _rxSequence.fill(0);
        _txSequence = 0;
        _stageStartedMs = _lastRxMs = _lastTxMs = _queuedAtMs = nowMs;
        bool nonzero = false;
        for (const auto byte : identity)
            nonzero = nonzero || byte != 0;
        if (!nonzero)
            return fail(Error::InvalidIdentity);
        _identity = identity;
        beginOutput();
        appendByte(0x41);
        appendByte(PROTOCOL_VERSION);
        appendByte(0xf2);
        for (const auto byte : identity)
            appendByte(byte);
        appendByte(0x0a);
        appendByte(static_cast<uint8_t>(BATCH_CAPACITY));
        appendByte(static_cast<uint8_t>(BATCH_CAPACITY >> 8));
        finishOutput(nowMs);
        _state = State::AwaitInitAck;
        return true;
    }

    bool receive(const uint8_t* bytes, size_t length, uint64_t nowMs,
                 DiscoveryCallback onDiscovery = nullptr, void* context = nullptr)
    {
        service(nowMs);
        if (!active())
            return false;
        if (!bytes && length != 0)
            return fail(Error::MalformedMessage);
        for (size_t offset = 0; offset < length; ++offset)
        {
            const auto result = _framer.push(bytes[offset]);
            if (result == ZenohStreamFramer<BATCH_CAPACITY>::Result::InvalidLength)
                return fail(Error::InvalidBatch);
            if (result == ZenohStreamFramer<BATCH_CAPACITY>::Result::Complete)
            {
                if (_framer.size() > _negotiatedBatch || !processBatch(nowMs, onDiscovery, context))
                {
                    if (_state != State::Failed)
                        fail(Error::InvalidBatch);
                    return false;
                }
                _lastRxMs = nowMs;
                if (_state == State::Closed)
                    return true;
            }
        }
        return true;
    }

    void service(uint64_t nowMs)
    {
        if (!active())
            return;
        if (_state != State::Established)
        {
            if (nowMs - _stageStartedMs >= HANDSHAKE_TIMEOUT_MS)
                fail(Error::HandshakeTimeout);
            return;
        }
        if (nowMs - _lastRxMs >= _remoteLeaseMs)
        {
            fail(Error::LeaseExpired);
            return;
        }
        if (outputSize() != 0)
        {
            if (nowMs - _queuedAtMs >= LOCAL_LEASE_MS)
                fail(Error::SendStalled);
            return;
        }
        if (nowMs - _lastTxMs >= LOCAL_LEASE_MS / 4)
        {
            beginOutput();
            appendByte(0x04);
            finishOutput(nowMs);
        }
    }

    const uint8_t* outputData() const { return _output.data() + _outputOffset; }
    size_t outputSize() const { return _outputLength - _outputOffset; }

    bool sendNetworkMessage(const uint8_t* message, size_t length, uint64_t nowMs)
    {
        service(nowMs);
        if (_state != State::Established || outputSize() != 0 || !message || length == 0)
            return false;
        size_t sequenceSize = 1;
        for (uint32_t remainder = _txSequence >> 7; remainder; remainder >>= 7)
            ++sequenceSize;
        if (length > _negotiatedBatch - 1 - sequenceSize)
            return false;
        beginOutput();
        appendByte(0x25);
        appendInteger(_txSequence);
        std::memcpy(_output.data() + _outputLength, message, length);
        _outputLength += length;
        _txSequence = static_cast<uint32_t>((_txSequence + uint64_t{1}) & ((uint64_t{1} << _sequenceBits) - 1));
        finishOutput(nowMs);
        return true;
    }

    bool consumeOutput(size_t count, uint64_t nowMs)
    {
        if (count > outputSize())
            return false;
        if (count == 0)
            return true;
        _outputOffset += count;
        if (_outputOffset == _outputLength)
        {
            _lastTxMs = nowMs;
            _outputOffset = _outputLength = 0;
        }
        return true;
    }

    void linkLost() { if (active()) fail(Error::LinkLost); }
    State state() const { return _state; }
    Error error() const { return _error; }
    uint16_t negotiatedBatch() const { return _negotiatedBatch; }
    uint64_t remoteLeaseMs() const { return _remoteLeaseMs; }
    uint32_t remoteInitialSequence() const { return _remoteInitialSequence; }
    uint32_t keepAlivesReceived() const { return _keepAlivesReceived; }
    uint32_t frameCount() const { return _frameCount; }
    uint32_t discoveryCount() const { return _discoveryCount; }
    uint8_t closeReason() const { return _closeReason; }

private:
    class Reader
    {
    public:
        Reader(const uint8_t* bytes, size_t size) : _bytes(bytes), _size(size) {}
        bool byte(uint8_t& value)
        {
            if (_offset == _size)
                return false;
            value = _bytes[_offset++];
            return true;
        }
        bool integer(uint64_t& value)
        {
            value = 0;
            for (size_t index = 0; index < 9; ++index)
            {
                uint8_t part = 0;
                if (!byte(part))
                    return false;
                if (index == 8)
                {
                    value |= static_cast<uint64_t>(part) << 56;
                    return true;
                }
                value |= static_cast<uint64_t>(part & 0x7f) << (7 * index);
                if ((part & 0x80) == 0)
                    return true;
            }
            return false;
        }
        bool skip(size_t count)
        {
            if (count > remaining())
                return false;
            _offset += count;
            return true;
        }
        const uint8_t* current() const { return _bytes + _offset; }
        size_t remaining() const { return _size - _offset; }
    private:
        const uint8_t* _bytes;
        size_t _size;
        size_t _offset = 0;
    };

    bool active() const
    {
        return _state == State::AwaitInitAck || _state == State::AwaitOpenAck || _state == State::Established;
    }
    bool fail(Error error)
    {
        _state = State::Failed;
        _error = error;
        _outputOffset = _outputLength = 0;
        return false;
    }
    void beginOutput() { _outputOffset = 0; _outputLength = 2; }
    void appendByte(uint8_t value) { _output[_outputLength++] = value; }
    void appendInteger(uint64_t value)
    {
        do
        {
            uint8_t part = static_cast<uint8_t>(value & 0x7f);
            value >>= 7;
            appendByte(static_cast<uint8_t>(part | (value != 0 ? 0x80 : 0)));
        } while (value != 0);
    }
    void finishOutput(uint64_t nowMs)
    {
        const size_t size = _outputLength - 2;
        _output[0] = static_cast<uint8_t>(size);
        _output[1] = static_cast<uint8_t>(size >> 8);
        _queuedAtMs = nowMs;
    }

    bool extensions(Reader& reader, bool more, bool negotiation)
    {
        for (size_t count = 0; more; ++count)
        {
            uint8_t header = 0;
            if (count == 16 || !reader.byte(header))
                return fail(Error::MalformedMessage);
            if (header & 0x10)
                return fail(Error::UnsupportedNegotiation);
            const uint8_t id = header & 0x0f;
            if (negotiation && id >= 1 && id <= 6)
                return fail(Error::UnsupportedNegotiation);
            const uint8_t encoding = header & 0x60;
            uint64_t value = 0;
            if (encoding == 0x60 || (encoding != 0 && !reader.integer(value)))
                return fail(Error::MalformedMessage);
            if (encoding == 0x40 && (value > reader.remaining() || !reader.skip(static_cast<size_t>(value))))
                return fail(Error::MalformedMessage);
            if (negotiation && id == 7 && (encoding != 0x20 || value != 0))
                return fail(Error::UnsupportedNegotiation);
            more = (header & 0x80) != 0;
        }
        return true;
    }

    bool initAck(Reader& reader, uint8_t header, uint64_t nowMs)
    {
        uint8_t version = 0, identityInfo = 0;
        if (!reader.byte(version) || !reader.byte(identityInfo))
            return fail(Error::MalformedMessage);
        if (version != PROTOCOL_VERSION || (identityInfo & 0x03) > 1 || (identityInfo & 0x0c))
            return fail(Error::UnsupportedNegotiation);
        const size_t identitySize = 1 + (identityInfo >> 4);
        if (reader.remaining() < identitySize)
            return fail(Error::MalformedMessage);
        bool nonzero = false;
        for (size_t index = 0; index < identitySize; ++index)
            nonzero = nonzero || reader.current()[index] != 0;
        std::array<uint8_t, 16> remoteIdentity{};
        std::memcpy(remoteIdentity.data(), reader.current(), identitySize);
        if (!nonzero || remoteIdentity == _identity)
            return fail(Error::InvalidIdentity);
        reader.skip(identitySize);
        uint8_t resolution = 0x0a;
        uint16_t batch = 65535;
        if (header & 0x40)
        {
            uint8_t low = 0, high = 0;
            if (!reader.byte(resolution) || !reader.byte(low) || !reader.byte(high))
                return fail(Error::MalformedMessage);
            batch = static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
        }
        if (batch < 64 || batch > BATCH_CAPACITY || (resolution & 0xf0) != 0 ||
            (resolution & 3) > 2 || ((resolution >> 2) & 3) > 2)
            return fail(Error::UnsupportedNegotiation);
        uint64_t cookieSize = 0;
        if (!reader.integer(cookieSize) || cookieSize > COOKIE_CAPACITY || cookieSize > reader.remaining())
            return fail(Error::MalformedMessage);
        const uint8_t* cookie = reader.current();
        reader.skip(static_cast<size_t>(cookieSize));
        if (!extensions(reader, (header & 0x80) != 0, true))
            return false;
        if (reader.remaining() != 0 || outputSize() != 0)
            return fail(Error::UnexpectedMessage);
        _negotiatedBatch = batch;
        _sequenceBits = static_cast<uint8_t>(8u << (resolution & 3));
        beginOutput();
        appendByte(0x42);
        appendInteger(LOCAL_LEASE_MS / 1000);
        appendInteger(0);
        appendInteger(cookieSize);
        for (size_t index = 0; index < cookieSize; ++index)
            appendByte(cookie[index]);
        if (_outputLength - 2 > _negotiatedBatch)
            return fail(Error::UnsupportedNegotiation);
        finishOutput(nowMs);
        _state = State::AwaitOpenAck;
        _stageStartedMs = nowMs;
        return true;
    }

    bool openAck(Reader& reader, uint8_t header)
    {
        uint64_t lease = 0, sequence = 0;
        if (!reader.integer(lease) || !reader.integer(sequence))
            return fail(Error::MalformedMessage);
        if (header & 0x40)
        {
            if (lease > std::numeric_limits<uint64_t>::max() / 1000)
                return fail(Error::MalformedMessage);
            lease *= 1000;
        }
        if (lease == 0 || lease > 120000 || sequence >= (uint64_t{1} << _sequenceBits))
            return fail(Error::UnsupportedNegotiation);
        if (!extensions(reader, (header & 0x80) != 0, true))
            return false;
        if (reader.remaining() != 0 || outputSize() != 0)
            return fail(Error::UnexpectedMessage);
        _remoteLeaseMs = lease;
        _remoteInitialSequence = static_cast<uint32_t>(sequence);
        _rxSequence.fill(_remoteInitialSequence);
        _state = State::Established;
        return true;
    }

    bool processBatch(uint64_t nowMs, DiscoveryCallback onDiscovery, void* context)
    {
        Reader reader(_framer.data(), _framer.size());
        while (reader.remaining() != 0)
        {
            uint8_t header = 0;
            reader.byte(header);
            const uint8_t id = header & 0x1f;
            if (id == 3)
            {
                if ((header & 0x40) || !reader.byte(_closeReason))
                    return fail(Error::MalformedMessage);
                if (!extensions(reader, (header & 0x80) != 0, false))
                    return false;
                _state = State::Closed;
                _outputLength = _outputOffset = 0;
                return true;
            }
            if (_state == State::AwaitInitAck && id == 1 && (header & 0x20))
                return initAck(reader, header, nowMs);
            if (_state == State::AwaitOpenAck && id == 2 && (header & 0x20))
                return openAck(reader, header);
            if (_state != State::Established)
                return fail(Error::UnexpectedMessage);
            if (id == 4)
            {
                if ((header & 0x60) || !extensions(reader, (header & 0x80) != 0, false))
                    return fail(_error == Error::None ? Error::MalformedMessage : _error);
                ++_keepAlivesReceived;
            }
            else if (id == 5)
            {
                uint64_t sequence = 0;
                if ((header & 0x40) || !reader.integer(sequence) ||
                    sequence >= (uint64_t{1} << _sequenceBits))
                    return fail(Error::MalformedMessage);
                if (!extensions(reader, (header & 0x80) != 0, false))
                    return false;
                const size_t channel = (header & 0x20) ? 0 : 1;
                if (sequence != _rxSequence[channel])
                    return fail(Error::SequenceMismatch);
                _rxSequence[channel] = static_cast<uint32_t>((sequence + 1) & ((uint64_t{1} << _sequenceBits) - 1));
                ++_frameCount;
                bool hasMessage = false;
                while (reader.remaining() && (reader.current()[0] & 0x1f) >= 0x10)
                {
                    ZenohNetworkMessage::DiscoveryMessage message;
                    const size_t consumed = ZenohNetworkMessage::readDiscovery(reader.current(), reader.remaining(), message);
                    if (!consumed || !reader.skip(consumed))
                        return fail(Error::MalformedMessage);
                    hasMessage = true;
                    ++_discoveryCount;
                    if (onDiscovery && !onDiscovery(context, message))
                        return fail(Error::DiscoveryRejected);
                }
                if (!hasMessage)
                    return fail(Error::MalformedMessage);
            }
            else
                return fail(Error::UnexpectedMessage);
        }
        return true;
    }

    ZenohStreamFramer<BATCH_CAPACITY> _framer;
    std::array<uint8_t, BATCH_CAPACITY + 2> _output{};
    std::array<uint8_t, 16> _identity{};
    size_t _outputLength = 0, _outputOffset = 0;
    State _state = State::Idle;
    Error _error = Error::None;
    uint16_t _negotiatedBatch = BATCH_CAPACITY;
    uint8_t _sequenceBits = 32, _closeReason = 0;
    uint64_t _stageStartedMs = 0, _lastRxMs = 0, _lastTxMs = 0, _queuedAtMs = 0, _remoteLeaseMs = 0;
    uint32_t _remoteInitialSequence = 0, _keepAlivesReceived = 0, _frameCount = 0, _discoveryCount = 0;
    std::array<uint32_t, 2> _rxSequence{};
    uint32_t _txSequence = 0;
};

}