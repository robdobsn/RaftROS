#include <array>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include "Zenoh/ZenohROSCodec.h"

#define TEST_ASSERT(cond, msg) do { if (!(cond)) { std::printf("  FAIL: %s\n", msg); ++failCount; } else { ++passCount; } } while (false)

int main()
{
    using RaftRuntime::Zenoh::ZenohROSCodec;
    int passCount = 0;
    int failCount = 0;

    const std::array<uint8_t, 33> golden = {
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,
        0x10,
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
    };
    ZenohROSCodec::Attachment attachment;
    attachment.sequenceNumber = 0x0102030405060708LL;
    attachment.sourceTimestampNs = 0x1112131415161718LL;
    attachment.publisherGid = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
    };

    std::printf("Test: pinned Jazzy attachment wire layout\n");
    std::array<uint8_t, 35> encoded{};
    encoded.fill(0xa5);
    TEST_ASSERT(ZenohROSCodec::encodeAttachment(encoded.data() + 1, 33, attachment) == 33,
                "attachment fits exact-size unaligned output");
    TEST_ASSERT(std::memcmp(encoded.data() + 1, golden.data(), golden.size()) == 0,
                "attachment matches independent little-endian fixture");
    TEST_ASSERT(encoded.front() == 0xa5 && encoded.back() == 0xa5,
                "attachment does not overwrite surrounding bytes");

    ZenohROSCodec::Attachment decoded;
    TEST_ASSERT(ZenohROSCodec::decodeAttachment(golden.data(), golden.size(), decoded),
                "decode independent golden attachment");
    TEST_ASSERT(decoded.sequenceNumber == attachment.sequenceNumber &&
                decoded.sourceTimestampNs == attachment.sourceTimestampNs &&
                decoded.publisherGid == attachment.publisherGid,
                "decode all fields without CDR padding");

    std::printf("Test: attachment bounds and invalid inputs\n");
    for (size_t capacity = 0; capacity < golden.size(); ++capacity)
    {
        encoded.fill(0xa5);
        const auto untouched = encoded;
        TEST_ASSERT(ZenohROSCodec::encodeAttachment(encoded.data(), capacity, attachment) == 0 &&
                    encoded == untouched, "short output fails without writing");
        decoded = attachment;
        TEST_ASSERT(!ZenohROSCodec::decodeAttachment(golden.data(), capacity, decoded) &&
                    decoded.sequenceNumber == attachment.sequenceNumber &&
                    decoded.sourceTimestampNs == attachment.sourceTimestampNs &&
                    decoded.publisherGid == attachment.publisherGid,
                    "truncated input fails without modifying result");
    }
    TEST_ASSERT(ZenohROSCodec::encodeAttachment(nullptr, 33, attachment) == 0,
                "null attachment output rejected");
    TEST_ASSERT(!ZenohROSCodec::decodeAttachment(nullptr, 33, decoded),
                "null attachment input rejected");
    TEST_ASSERT(!ZenohROSCodec::decodeAttachment(encoded.data(), encoded.size(), decoded),
                "trailing attachment bytes rejected for pinned profile");
    for (unsigned gidLength = 0; gidLength < 256; ++gidLength)
    {
        auto invalid = golden;
        invalid[16] = static_cast<uint8_t>(gidLength);
        TEST_ASSERT(ZenohROSCodec::decodeAttachment(invalid.data(), invalid.size(), decoded) ==
                    (gidLength == 16), "only 16-byte publisher GID accepted");
    }

    std::printf("Test: signed attachment integer boundaries\n");
    const int64_t signedValues[] = {
        0, 1, -1, std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::max()
    };
    for (const int64_t value : signedValues)
    {
        attachment.sequenceNumber = value;
        attachment.sourceTimestampNs = value;
        TEST_ASSERT(ZenohROSCodec::encodeAttachment(encoded.data() + 1, 33, attachment) == 33 &&
                    ZenohROSCodec::decodeAttachment(encoded.data() + 1, 33, decoded) &&
                    decoded.sequenceNumber == value && decoded.sourceTimestampNs == value,
                    "signed attachment values round-trip at unaligned addresses");
    }

    std::printf("Test: pinned Jazzy topic key examples\n");
    constexpr const char* stringType = "std_msgs::msg::dds_::String_";
    constexpr const char* stringHash =
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
    constexpr const char* chatterKey =
        "0/chatter/std_msgs::msg::dds_::String_/"
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
    char key[1024];
    TEST_ASSERT(ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, "/chatter", stringType, stringHash) &&
                std::strcmp(key, chatterKey) == 0, "topic key matches pinned design example");
    TEST_ASSERT(ZenohROSCodec::formatTopicKey(key, sizeof(key), 42, "/robot1/chatter", stringType, stringHash) &&
                std::string(key) == "42/robot1/chatter/" + std::string(stringType) + "/" + stringHash,
                "domain and namespace carried without DDS rt prefix");
    TEST_ASSERT(ZenohROSCodec::formatTopicKey(key, sizeof(key), UINT32_MAX, "/_sensor2", stringType, stringHash) &&
                std::string(key).find("4294967295/_sensor2/") == 0,
                "domain formatting is not limited to DDS UDP port ranges");

    const std::string_view invalidTopics[] = {
        "", "/", "chatter", "//chatter", "/chatter/", "/robot//chatter", "/0sensor",
        "/robot/0sensor", "/sensor/*", "/sensor/**", "/sensor/$*", "/sensor/#",
        "/sensor/%", "/sensor/?", "/sensor/with space", "/sensor/with-dash",
        "/sensor/{name}", "/~sensor", std::string_view("/sensor\0suffix", 14)
    };
    for (const auto topic : invalidTopics)
    {
        key[0] = 'x';
        TEST_ASSERT(!ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, topic, stringType, stringHash) &&
                    key[0] == '\0', "invalid topic cannot produce a routable key");
    }
    const std::string_view invalidTypes[] = {
        "", "std_msgs/msg/String", "std_msgs::msg::dds_::String", "::msg::dds_::String_",
        "std_msgs::msg::dds_::_", "std_msgs::srv::dds_::String_", "std_msgs::msg::dds_::Str*ing_",
        "std_msgs::msg::dds_::String_/other", "0pkg::msg::dds_::String_"
    };
    for (const auto type : invalidTypes)
    {
        TEST_ASSERT(!ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, "/chatter", type, stringHash) &&
                    key[0] == '\0', "unsupported or malformed wire type rejected");
    }
    for (const std::string& hash : {
        std::string(), std::string(stringHash).substr(0, 70), std::string(stringHash) + "0",
        "RIHS00_" + std::string(64, '0'), "RIHS01_" + std::string(64, 'g'),
        "RIHS01_" + std::string(64, 'A'), "RIHS01_" + std::string(63, 'a') + "/"
    })
    {
        TEST_ASSERT(!ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, "/chatter", stringType, hash),
                    "missing, unsupported or malformed type hash rejected");
    }
    const std::string maxTopic = "/" + std::string(254, 'a');
    TEST_ASSERT(ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, maxTopic, stringType, stringHash),
                "maximum supported ROS topic length accepted");
    TEST_ASSERT(!ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, maxTopic + "a", stringType, stringHash),
                "oversized ROS topic rejected");
    TEST_ASSERT(!ZenohROSCodec::formatTopicKey(key, sizeof(key), 0, "/chatter",
                std::string(129, 'a'), stringHash), "oversized wire type rejected");
    TEST_ASSERT(!ZenohROSCodec::formatTopicKey(nullptr, sizeof(key), 0, "/chatter", stringType, stringHash),
                "null topic-key output rejected");
    for (size_t capacity = 0; capacity <= std::strlen(chatterKey); ++capacity)
    {
        std::memset(key, 'x', sizeof(key));
        TEST_ASSERT(!ZenohROSCodec::formatTopicKey(key + 1, capacity, 0, "/chatter", stringType, stringHash) &&
                    key[0] == 'x' && key[capacity + 1] == 'x' &&
                    (capacity == 0 ? key[1] == 'x' : key[1] == '\0'),
                    "short key buffer cleared within caller bounds");
    }
    TEST_ASSERT(ZenohROSCodec::formatTopicKey(key, std::strlen(chatterKey) + 1, 0, "/chatter", stringType, stringHash) &&
                std::strcmp(key, chatterKey) == 0, "topic key fits exact capacity including terminator");

    std::printf("Test: pinned Jazzy node liveliness tokens\n");
    constexpr const char* sessionId = "aac3178e146ba6f1fc6e6a4085e77f21";
    constexpr const char* listenerToken =
        "@ros2_lv/0/aac3178e146ba6f1fc6e6a4085e77f21/0/0/NN/%/%/listener";
    TEST_ASSERT(ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, "", "", "listener") &&
                std::strcmp(key, listenerToken) == 0, "node token matches pinned design example");
    TEST_ASSERT(ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, "/", "/", "listener") &&
                std::strcmp(key, listenerToken) == 0, "root and default namespaces encode identically");
    TEST_ASSERT(ZenohROSCodec::formatNodeToken(key, sizeof(key), 42, "1", UINT64_MAX,
                "/fleet/private", "/robot1/sensors", "raft_esp32") &&
                std::strcmp(key, "@ros2_lv/42/1/18446744073709551615/18446744073709551615/NN/"
                                 "%fleet%private/%robot1%sensors/raft_esp32") == 0,
                "node IDs repeated in full and namespaces mangled without key separators");
    const std::string_view invalidSessions[] = {
        "", "0", "000000", "A", "gg", "1/2", "a*", "0123456789abcdef0123456789abcdef0aa"
    };
    for (const auto session : invalidSessions)
    {
        TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, session, 0, "", "", "listener") &&
                    key[0] == '\0', "invalid session identity rejected");
    }
    const std::string_view invalidNodeNames[] = {"", "0node", "node/child", "node%child", "node*", "node name"};
    for (const auto nodeName : invalidNodeNames)
    {
        TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, "", "", nodeName),
                    "invalid node identity rejected");
    }
    for (const auto path : invalidTopics)
    {
        if (path.empty() || path == "/")
            continue;
        TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, path, "", "listener"),
                    "invalid enclave path rejected");
        TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, "", path, "listener"),
                    "invalid namespace path rejected");
    }
    TEST_ASSERT(ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, maxTopic, maxTopic, "listener"),
                "maximum namespace and enclave fit bounded mangle storage");
    TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, maxTopic + "a", "", "listener"),
                "oversized enclave rejected before mangling");
    TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, "", maxTopic + "a", "listener"),
                "oversized namespace rejected before mangling");
    TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key, sizeof(key), 0, sessionId, 0, "", "", std::string(256, 'a')),
                "oversized node name rejected");
    TEST_ASSERT(!ZenohROSCodec::formatNodeToken(nullptr, sizeof(key), 0, sessionId, 0, "", "", "listener"),
                "null node-token output rejected");
    for (size_t capacity = 0; capacity <= std::strlen(listenerToken); ++capacity)
    {
        std::memset(key, 'x', sizeof(key));
        TEST_ASSERT(!ZenohROSCodec::formatNodeToken(key + 1, capacity, 0, sessionId, 0, "", "", "listener") &&
                    key[0] == 'x' && key[capacity + 1] == 'x' &&
                    (capacity == 0 ? key[1] == 'x' : key[1] == '\0'),
                    "short node-token buffer cleared within caller bounds");
    }
    TEST_ASSERT(ZenohROSCodec::formatNodeToken(key, std::strlen(listenerToken) + 1, 0, sessionId, 0, "", "", "listener") &&
                std::strcmp(key, listenerToken) == 0, "node token fits exact capacity including terminator");

    std::printf("Zenoh codec: %d passed, %d failed\n", passCount, failCount);
    return failCount == 0 ? 0 : 1;
}