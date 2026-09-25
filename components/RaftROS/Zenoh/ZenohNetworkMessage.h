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

    /// @brief Declare a subscriber, so the router forwards matching samples to
    /// this session.  Same shape as declareToken (body id 2 rather than 6),
    /// with the key given in full: a session that never declares key-expression
    /// ids has nothing for the router to resolve a numeric key against.
    static size_t declareSubscriber(uint8_t* output, size_t capacity, uint32_t subscriberId,
                                    std::string_view key, const uint32_t* interestId = nullptr)
    {
        if (!validKey(key, /*allowWildcards=*/true))
            return 0;
        Writer writer(output, capacity);
        writer.byte(interestId ? 0x3e : 0x1e);
        if (interestId)
            writer.integer(*interestId);
        writer.byte(0x22);
        writer.integer(subscriberId);
        writer.byte(0);
        writer.string(key);
        return writer.size();
    }

    /// @brief Withdraw a subscriber
    static size_t undeclareSubscriber(uint8_t* output, size_t capacity, uint32_t subscriberId)
    {
        Writer writer(output, capacity);
        writer.byte(0x1e);
        writer.byte(0x03);
        writer.integer(subscriberId);
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

    /// @brief One sample arriving from the router.
    ///
    /// A key can arrive two ways: in full, or as a key-expression id the peer
    /// declared earlier plus an optional suffix.  The id form is reported
    /// rather than resolved - the resolution table belongs to whoever declared
    /// the subscriptions, not to a message parser.
    struct SampleMessage
    {
        uint32_t keyId = 0;             ///< 0 when the key arrived in full
        std::string_view key;           ///< Full key, or the suffix after `keyId`
        std::string_view payload;
        std::string_view attachment;    ///< Empty when the sample carried none
    };

    /// @brief Parse one Put from a peer
    /// @return bytes consumed, or 0 if this is not a well-formed Put
    static size_t readSample(const uint8_t* input, size_t length, SampleMessage& output)
    {
        Cursor reader(input, length);
        uint8_t header = 0;
        if (!reader.byte(header) || (header & 0x1f) != 0x1d)
            return 0;
        SampleMessage message;
        uint32_t keyId = 0;
        if (!reader.number(keyId) || keyId > 65535)
            return 0;
        message.keyId = keyId;
        if (header & 0x20)
        {
            uint32_t size = 0;
            if (!reader.number(size) || size == 0 || size > MAX_KEY_SIZE)
                return 0;
            const uint8_t* keyStart = nullptr;
            if (!reader.bytes(size, keyStart))
                return 0;
            message.key = std::string_view(reinterpret_cast<const char*>(keyStart), size);
        }
        else if (keyId == 0)
        {
            // Neither a full key nor a declared id: nothing identifies the topic
            return 0;
        }
        std::string_view ignored;
        if (!reader.sampleExtensions((header & 0x80) != 0, ignored))
            return 0;

        uint8_t body = 0;
        if (!reader.byte(body) || (body & 0x1f) != 0x01)
            return 0;
        // The Put body's own fields come before its extensions: a timestamp
        // (flag T) is a varint clock reading followed by the publishing
        // session's id, and an encoding (flag E) is a varint that says in its
        // low bit whether a schema string follows.  Neither is used here, but
        // both have to be stepped over to reach the attachment and payload.
        if (body & 0x20)
        {
            uint64_t clock = 0;
            uint32_t idSize = 0;
            const uint8_t* idStart = nullptr;
            if (!reader.integer(clock) || !reader.number(idSize) || idSize > 16 ||
                !reader.bytes(idSize, idStart))
                return 0;
        }
        if (body & 0x40)
        {
            uint32_t encoding = 0;
            if (!reader.number(encoding))
                return 0;
            if (encoding & 1)
            {
                uint32_t schemaSize = 0;
                const uint8_t* schemaStart = nullptr;
                if (!reader.number(schemaSize) || schemaSize > MAX_KEY_SIZE ||
                    !reader.bytes(schemaSize, schemaStart))
                    return 0;
            }
        }
        if (!reader.sampleExtensions((body & 0x80) != 0, message.attachment))
            return 0;
        uint32_t payloadSize = 0;
        if (!reader.number(payloadSize) || payloadSize > MAX_PAYLOAD_SIZE)
            return 0;
        const uint8_t* payloadStart = nullptr;
        if (!reader.bytes(payloadSize, payloadStart))
            return 0;
        message.payload = std::string_view(reinterpret_cast<const char*>(payloadStart), payloadSize);
        output = message;
        return reader.position();
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
        /// @brief Walk the extensions on an inbound sample, capturing the
        /// attachment and skipping the rest.
        ///
        /// Zenoh encodes an extension's body three ways, chosen by two bits of
        /// its header: nothing at all, a varint, or a length-prefixed buffer -
        /// so an extension cannot be skipped without decoding which it is.
        /// Unknown extensions are skipped even when flagged mandatory: a
        /// consumer that only reads the payload and the attachment loses
        /// nothing by ignoring, say, a timestamp, and dropping the sample
        /// instead would lose data over a field we do not use.
        bool sampleExtensions(bool more, std::string_view& attachment)
        {
            for (size_t count = 0; more; ++count)
            {
                uint8_t header = 0;
                uint64_t value = 0;
                if (count == 16 || !byte(header) || (header & 0x60) == 0x60)
                    return false;
                if ((header & 0x60) && !integer(value))
                    return false;
                if ((header & 0x60) == 0x40)
                {
                    if (value > _length - _offset || value > MAX_ATTACHMENT_SIZE)
                        return false;
                    if ((header & 0x7f) == 0x43)
                        attachment = std::string_view(reinterpret_cast<const char*>(_input + _offset),
                                                      static_cast<size_t>(value));
                    _offset += static_cast<size_t>(value);
                }
                more = (header & 0x80) != 0;
            }
            return true;
        }

        /// @brief Borrow `size` bytes of the input, without copying
        bool bytes(size_t size, const uint8_t*& start)
        {
            if (size > _length - _offset)
                return false;
            start = _input + _offset;
            _offset += size;
            return true;
        }

        size_t position() const { return _offset; }
    private:
        const uint8_t* _input;
        size_t _length;
        size_t _offset = 0;
    };

    /// @brief Is this a usable key expression?
    /// @param allowWildcards a subscription may match a set of keys ("*",
    ///        "**", "$*"); a publication names exactly one, so wildcards in it
    ///        are a mistake rather than a pattern and are refused.
    static bool validKey(std::string_view key, bool allowWildcards = false)
    {
        if (key.empty() || key.size() > MAX_KEY_SIZE || key.front() == '/' || key.back() == '/')
            return false;
        char previous = 0;
        for (const char character : key)
        {
            const bool wildcard = character == '*' || character == '$';
            if (static_cast<unsigned char>(character) <= 32 || static_cast<unsigned char>(character) >= 127 ||
                character == '?' || character == '#' ||
                (wildcard && !allowWildcards) ||
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