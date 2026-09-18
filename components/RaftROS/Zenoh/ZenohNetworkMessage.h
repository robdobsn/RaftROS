#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace RaftRuntime::Zenoh
{

class ZenohNetworkMessage
{
public:
    static constexpr size_t MAX_KEY_SIZE = 1536;
    static constexpr size_t MAX_PAYLOAD_SIZE = 2048;
    static constexpr size_t MAX_ATTACHMENT_SIZE = 256;

    struct DiscoveryMessage
    {
        bool isInterest = false;
        uint8_t mode = 0;
        uint8_t options = 0;
        uint8_t declaration = 0;
        uint32_t id = 0;
        bool hasInterestId = false;
        uint32_t interestId = 0;
        uint16_t scope = 0;
        bool senderMapping = false;
        std::string_view key;
    };

    static size_t readDiscovery(const uint8_t* input, size_t length, DiscoveryMessage& output)
    {
        Cursor reader(input, length);
        DiscoveryMessage message;
        uint8_t header = 0;
        if (!reader.byte(header))
            return 0;
        if ((header & 0x1f) == 0x19)
        {
            message.isInterest = true;
            message.mode = (header >> 5) & 3;
            if (!reader.number(message.id))
                return 0;
            if (message.mode != 0)
            {
                if (!reader.byte(message.options))
                    return 0;
                if (message.options & 0x10)
                {
                    if (!reader.key(message, message.options))
                        return 0;
                }
                else if (message.options & 0x60)
                    return 0;
            }
            if (!reader.extensions((header & 0x80) != 0, false))
                return 0;
        }
        else if ((header & 0x1f) == 0x1e)
        {
            message.hasInterestId = (header & 0x20) != 0;
            if ((header & 0x40) || (message.hasInterestId && !reader.number(message.interestId)) ||
                !reader.extensions((header & 0x80) != 0, false))
                return 0;
            uint8_t declaration = 0;
            if (!reader.byte(declaration))
                return 0;
            message.declaration = declaration & 0x1f;
            if (message.declaration <= 7)
            {
                if (!reader.number(message.id) || (message.declaration <= 1 && message.id > 65535))
                    return 0;
                if ((message.declaration & 1) == 0)
                {
                    if ((message.declaration == 0 && (declaration & 0x40)) || !reader.key(message, declaration))
                        return 0;
                }
                else if (declaration & 0x60)
                    return 0;
            }
            else if (message.declaration != 0x1a || (declaration & 0x60))
                return 0;
            if (!reader.extensions((declaration & 0x80) != 0, true))
                return 0;
        }
        else
            return 0;
        output = message;
        return reader.position();
    }

    static size_t declareToken(uint8_t* output, size_t capacity, uint32_t tokenId,
                               std::string_view key, const uint32_t* interestId = nullptr)
    {
        if (!validKey(key))
            return 0;
        Writer writer(output, capacity);
        writer.byte(interestId ? 0x3e : 0x1e);
        if (interestId)
            writer.integer(*interestId);
        writer.byte(0x26);
        writer.integer(tokenId);
        writer.byte(0);
        writer.string(key);
        return writer.size();
    }

    static size_t undeclareToken(uint8_t* output, size_t capacity, uint32_t tokenId)
    {
        Writer writer(output, capacity);
        writer.byte(0x1e);
        writer.byte(0x07);
        writer.integer(tokenId);
        return writer.size();
    }

    static size_t declareFinal(uint8_t* output, size_t capacity, const uint32_t* interestId = nullptr)
    {
        Writer writer(output, capacity);
        writer.byte(interestId ? 0x3e : 0x1e);
        if (interestId)
            writer.integer(*interestId);
        writer.byte(0x1a);
        return writer.size();
    }

    static size_t put(uint8_t* output, size_t capacity, std::string_view key,
                      const uint8_t* payload, size_t payloadSize,
                      const uint8_t* attachment, size_t attachmentSize)
    {
        if (!validKey(key) || payloadSize > MAX_PAYLOAD_SIZE || attachmentSize > MAX_ATTACHMENT_SIZE ||
            (!payload && payloadSize) || (!attachment && attachmentSize))
            return 0;
        Writer writer(output, capacity);
        writer.byte(0x3d);
        writer.byte(0);
        writer.string(key);
        writer.byte(attachmentSize ? 0x81 : 0x01);
        if (attachmentSize)
        {
            writer.byte(0x43);
            writer.integer(static_cast<uint32_t>(attachmentSize));
            writer.bytes(attachment, attachmentSize);
        }
        writer.integer(static_cast<uint32_t>(payloadSize));
        writer.bytes(payload, payloadSize);
        return writer.size();
    }

private:
    class Cursor
    {
    public:
        Cursor(const uint8_t* input, size_t length) : _input(input), _length(input ? length : 0) {}
        bool byte(uint8_t& value)
        {
            if (_offset >= _length)
                return false;
            value = _input[_offset++];
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
                if (!(part & 0x80))
                    return true;
            }
            return false;
        }
        bool number(uint32_t& value)
        {
            uint64_t decoded = 0;
            if (!integer(decoded) || decoded > UINT32_MAX)
                return false;
            value = static_cast<uint32_t>(decoded);
            return true;
        }
        bool key(DiscoveryMessage& message, uint8_t flags)
        {
            uint32_t scope = 0;
            if (!number(scope) || scope > 65535)
                return false;
            message.scope = static_cast<uint16_t>(scope);
            message.senderMapping = (flags & 0x40) != 0;
            if (flags & 0x20)
            {
                uint32_t size = 0;
                if (!number(size) || size == 0 || size > MAX_KEY_SIZE || size > _length - _offset)
                    return false;
                message.key = std::string_view(reinterpret_cast<const char*>(_input + _offset), size);
                for (const char character : message.key)
                {
                    if (static_cast<unsigned char>(character) < 32 || static_cast<unsigned char>(character) >= 127)
                        return false;
                }
                _offset += size;
            }
            return scope != 0 || !message.key.empty();
        }
        bool extensions(bool more, bool declaration)
        {
            for (size_t count = 0; more; ++count)
            {
                uint8_t header = 0;
                uint64_t value = 0;
                if (count == 16 || !byte(header) || (header & 0x60) == 0x60)
                    return false;
                if ((header & 0x60) && !integer(value))
                    return false;
                if (header & 0x10)
                {
                    if (!declaration && (header & 0x7f) == 0x33)
                    {
                        if (value != 0)
                            return false;
                    }
                    else
                        return false;
                }
                if ((header & 0x60) == 0x40)
                {
                    if (value > _length - _offset)
                        return false;
                    _offset += static_cast<size_t>(value);
                }
                more = (header & 0x80) != 0;
            }
            return true;
        }
        size_t position() const { return _offset; }
    private:
        const uint8_t* _input;
        size_t _length;
        size_t _offset = 0;
    };

    static bool validKey(std::string_view key)
    {
        if (key.empty() || key.size() > MAX_KEY_SIZE || key.front() == '/' || key.back() == '/')
            return false;
        char previous = 0;
        for (const char character : key)
        {
            if (static_cast<unsigned char>(character) <= 32 || static_cast<unsigned char>(character) >= 127 ||
                character == '*' || character == '$' || character == '?' || character == '#' ||
                (character == '/' && previous == '/'))
                return false;
            previous = character;
        }
        return true;
    }

    class Writer
    {
    public:
        Writer(uint8_t* output, size_t capacity) : _output(output), _capacity(capacity), _ok(output != nullptr) {}
        void byte(uint8_t value) { bytes(&value, 1); }
        void integer(uint32_t value)
        {
            do
            {
                const uint8_t part = static_cast<uint8_t>(value & 0x7f);
                value >>= 7;
                byte(static_cast<uint8_t>(part | (value ? 0x80 : 0)));
            } while (value);
        }
        void string(std::string_view text)
        {
            integer(static_cast<uint32_t>(text.size()));
            bytes(reinterpret_cast<const uint8_t*>(text.data()), text.size());
        }
        void bytes(const uint8_t* data, size_t count)
        {
            if (!_ok || count > _capacity - _offset)
            {
                _ok = false;
                return;
            }
            if (count)
                std::memcpy(_output + _offset, data, count);
            _offset += count;
        }
        size_t size() const { return _ok ? _offset : 0; }
    private:
        uint8_t* _output;
        size_t _capacity;
        size_t _offset = 0;
        bool _ok;
    };
};

}