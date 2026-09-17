#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>

namespace RaftRuntime::Zenoh
{

class ZenohROSCodec
{
public:
    static constexpr const char* RMW_VERSION = "0.2.11";
    static constexpr const char* RMW_COMMIT = "8c1fe8ef412bca5e6ac64f320468c70dcb03fc52";
    static constexpr size_t GID_SIZE = 16;
    static constexpr size_t ATTACHMENT_SIZE = 8 + 8 + 1 + GID_SIZE;
    static constexpr size_t MAX_ROS_NAME_SIZE = 255;
    static constexpr size_t MAX_TYPE_NAME_SIZE = 128;

    struct Attachment
    {
        int64_t sequenceNumber = 0;
        int64_t sourceTimestampNs = 0;
        std::array<uint8_t, GID_SIZE> publisherGid{};
    };

    static size_t encodeAttachment(uint8_t* output, size_t capacity,
                                  const Attachment& attachment)
    {
        if (!output || capacity < ATTACHMENT_SIZE)
            return 0;
        writeInt64LE(output, attachment.sequenceNumber);
        writeInt64LE(output + 8, attachment.sourceTimestampNs);
        output[16] = GID_SIZE;
        std::memcpy(output + 17, attachment.publisherGid.data(), GID_SIZE);
        return ATTACHMENT_SIZE;
    }

    static bool decodeAttachment(const uint8_t* input, size_t length,
                                 Attachment& output)
    {
        if (!input || length != ATTACHMENT_SIZE || input[16] != GID_SIZE)
            return false;
        Attachment decoded;
        decoded.sequenceNumber = readInt64LE(input);
        decoded.sourceTimestampNs = readInt64LE(input + 8);
        std::memcpy(decoded.publisherGid.data(), input + 17, GID_SIZE);
        output = decoded;
        return true;
    }

    static bool formatTopicKey(char* output, size_t capacity, uint32_t domainId,
                               std::string_view topic, std::string_view wireType,
                               std::string_view typeHash)
    {
        if (!output || capacity == 0)
            return false;
        output[0] = '\0';
        if (!isRosPath(topic, false) || !isMessageType(wireType) || !isTypeHash(typeHash))
            return false;
        const int written = std::snprintf(output, capacity, "%u/%.*s/%.*s/%.*s",
            static_cast<unsigned>(domainId),
            static_cast<int>(topic.size() - 1), topic.data() + 1,
            static_cast<int>(wireType.size()), wireType.data(),
            static_cast<int>(typeHash.size()), typeHash.data());
        return finishFormat(output, capacity, written);
    }

    static bool formatNodeToken(char* output, size_t capacity, uint32_t domainId,
                                std::string_view sessionId, uint64_t nodeId,
                                std::string_view enclave, std::string_view nodeNamespace,
                                std::string_view nodeName)
    {
        if (!output || capacity == 0)
            return false;
        output[0] = '\0';
        if (!isSessionId(sessionId) || !isIdentifier(nodeName) ||
            (!enclave.empty() && !isRosPath(enclave, true)) ||
            (!nodeNamespace.empty() && !isRosPath(nodeNamespace, true)))
            return false;
        const auto mangledEnclave = manglePath(enclave);
        const auto mangledNamespace = manglePath(nodeNamespace);
        const int written = std::snprintf(output, capacity,
            "@ros2_lv/%u/%.*s/%llu/%llu/NN/%s/%s/%.*s",
            static_cast<unsigned>(domainId),
            static_cast<int>(sessionId.size()), sessionId.data(),
            static_cast<unsigned long long>(nodeId), static_cast<unsigned long long>(nodeId),
            mangledEnclave.data(), mangledNamespace.data(),
            static_cast<int>(nodeName.size()), nodeName.data());
        return finishFormat(output, capacity, written);
    }

private:
    static bool isLetter(char character)
    {
        return (character >= 'a' && character <= 'z') ||
               (character >= 'A' && character <= 'Z') || character == '_';
    }

    static bool isIdentifier(std::string_view name)
    {
        if (name.empty() || name.size() > MAX_ROS_NAME_SIZE || !isLetter(name.front()))
            return false;
        for (const char character : name)
        {
            if (!isLetter(character) && !(character >= '0' && character <= '9'))
                return false;
        }
        return true;
    }

    static bool isRosPath(std::string_view path, bool allowRoot)
    {
        if (path.empty() || path.size() > MAX_ROS_NAME_SIZE || path.front() != '/')
            return false;
        if (path.size() == 1)
            return allowRoot;
        size_t start = 1;
        while (start < path.size())
        {
            const size_t separator = path.find('/', start);
            const size_t end = separator == std::string_view::npos ? path.size() : separator;
            if (!isIdentifier(path.substr(start, end - start)))
                return false;
            if (end == path.size())
                return true;
            start = end + 1;
        }
        return false;
    }

    static bool isMessageType(std::string_view name)
    {
        constexpr std::string_view separator = "::msg::dds_::";
        if (name.size() > MAX_TYPE_NAME_SIZE)
            return false;
        const size_t split = name.find(separator);
        if (split == std::string_view::npos || !isIdentifier(name.substr(0, split)))
            return false;
        const auto message = name.substr(split + separator.size());
        return message.size() > 1 && message.back() == '_' && isIdentifier(message);
    }

    static bool isLowerHex(char character)
    {
        return (character >= '0' && character <= '9') ||
               (character >= 'a' && character <= 'f');
    }

    static bool isTypeHash(std::string_view hash)
    {
        if (hash.size() != 71 || hash.substr(0, 7) != "RIHS01_")
            return false;
        for (const char character : hash.substr(7))
        {
            if (!isLowerHex(character))
                return false;
        }
        return true;
    }

    static bool isSessionId(std::string_view sessionId)
    {
        if (sessionId.empty() || sessionId.size() > 32)
            return false;
        bool nonzero = false;
        for (const char character : sessionId)
        {
            if (!isLowerHex(character))
                return false;
            nonzero = nonzero || character != '0';
        }
        return nonzero;
    }

    static std::array<char, MAX_ROS_NAME_SIZE + 1> manglePath(std::string_view path)
    {
        std::array<char, MAX_ROS_NAME_SIZE + 1> output{};
        if (path.empty())
            output[0] = '%';
        else
        {
            for (size_t offset = 0; offset < path.size(); ++offset)
                output[offset] = path[offset] == '/' ? '%' : path[offset];
        }
        return output;
    }

    static bool finishFormat(char* output, size_t capacity, int written)
    {
        if (written < 0 || static_cast<size_t>(written) >= capacity)
        {
            output[0] = '\0';
            return false;
        }
        return true;
    }

    static void writeInt64LE(uint8_t* output, int64_t value)
    {
        const uint64_t bits = static_cast<uint64_t>(value);
        for (size_t byteIndex = 0; byteIndex < 8; ++byteIndex)
            output[byteIndex] = static_cast<uint8_t>(bits >> (8 * byteIndex));
    }

    static int64_t readInt64LE(const uint8_t* input)
    {
        uint64_t bits = 0;
        for (size_t byteIndex = 0; byteIndex < 8; ++byteIndex)
            bits |= static_cast<uint64_t>(input[byteIndex]) << (8 * byteIndex);
        if (bits & (uint64_t{1} << 63))
            return -1 - static_cast<int64_t>(~bits);
        return static_cast<int64_t>(bits);
    }
};

}