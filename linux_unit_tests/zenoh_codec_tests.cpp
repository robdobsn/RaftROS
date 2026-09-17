#include <array>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include "Zenoh/ZenohROSCodec.h"
#include "Zenoh/ZenohROSIdentity.h"

#ifdef RAFTROS_ZENOH_GID_REFERENCE
#include "simplified_xxhash3.hpp"
#endif

#define TEST_ASSERT(cond, msg) do { if (!(cond)) { std::printf("  FAIL: %s\n", msg); ++failCount; } else { ++passCount; } } while (false)

static RaftRuntime::Zenoh::ZenohROSCodec::NodeIdentity fixtureNode(std::string_view sessionId)
{
    return {23, sessionId, 0, "/", "/raft_test", "raft_fixture"};
}

static RaftRuntime::Zenoh::ZenohROSCodec::Endpoint fixturePublisher()
{
    using RaftRuntime::Zenoh::ZenohROSCodec;
    return {1, ZenohROSCodec::EndpointKind::Publisher, "/raft_test/chatter", "std_msgs::msg::dds_::String_",
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
        {ZenohROSCodec::Reliability::BestEffort, ZenohROSCodec::Durability::Volatile, 5}};
}

static void printGid(const RaftRuntime::Zenoh::ZenohROSIdentity::Gid& gid)
{
    for (const auto byte : gid)
        std::printf("%02x", static_cast<unsigned>(byte));
    std::putchar('\n');
}

#ifdef RAFTROS_ZENOH_GID_REFERENCE
static RaftRuntime::Zenoh::ZenohROSIdentity::Gid referenceGid(std::string_view token)
{
    const auto reference = simplified_XXH3_128bits(token.data(), token.size());
    RaftRuntime::Zenoh::ZenohROSIdentity::Gid expected{};
    for (size_t byte = 0; byte < 8; ++byte)
    {
        expected[byte] = static_cast<uint8_t>(reference.low64 >> (8 * byte));
        expected[8 + byte] = static_cast<uint8_t>(reference.high64 >> (8 * byte));
    }
    return expected;
}
#endif

static int emitFixture(const char* sessionId)
{
    using RaftRuntime::Zenoh::ZenohROSCodec;
    using RaftRuntime::Zenoh::ZenohROSIdentity;
    const auto node = fixtureNode(sessionId);
    auto endpoint = fixturePublisher();
    ZenohROSIdentity::Gid publisherGid{}, subscriptionGid{};
    char nodeToken[1024], publisherToken[1024], subscriptionToken[1024], topicKey[1024];
    if (!ZenohROSCodec::formatNodeToken(nodeToken, sizeof(nodeToken), node.domainId, node.sessionId,
            node.nodeId, node.enclave, node.nodeNamespace, node.nodeName) ||
        !ZenohROSCodec::formatEndpointToken(publisherToken, sizeof(publisherToken), node, endpoint) ||
        !ZenohROSCodec::formatTopicKey(topicKey, sizeof(topicKey), node.domainId, endpoint.topic,
                           endpoint.wireType, endpoint.typeHash) ||
        !ZenohROSIdentity::deriveEndpointGid(node, endpoint, publisherGid))
        return 1;
    endpoint.entityId = 2;
    endpoint.kind = ZenohROSCodec::EndpointKind::Subscription;
    if (!ZenohROSCodec::formatEndpointToken(subscriptionToken, sizeof(subscriptionToken), node, endpoint) ||
        !ZenohROSIdentity::deriveEndpointGid(node, endpoint, subscriptionGid))
        return 1;
#ifdef RAFTROS_ZENOH_GID_REFERENCE
    if (publisherGid != referenceGid(publisherToken) || subscriptionGid != referenceGid(subscriptionToken))
        return 1;
#endif
    std::printf("%s\n%s\n%s\n%s\n%s\n", ZenohROSCodec::RMW_VERSION,
                nodeToken, publisherToken, subscriptionToken, topicKey);
    printGid(publisherGid);
    printGid(subscriptionGid);
    return 0;
}

static int emitAttachment(std::string_view sessionId, std::string_view sequence)
{
    using RaftRuntime::Zenoh::ZenohROSCodec;
    ZenohROSCodec::Attachment attachment;
    if (sequence.empty() || !RaftRuntime::Zenoh::ZenohROSIdentity::deriveEndpointGid(
            fixtureNode(sessionId), fixturePublisher(), attachment.publisherGid))
        return 1;
    const auto parsedSequence = std::from_chars(sequence.data(), sequence.data() + sequence.size(), attachment.sequenceNumber);
    if (parsedSequence.ec != std::errc{} || parsedSequence.ptr != sequence.data() + sequence.size() ||
        attachment.sequenceNumber <= 0)
        return 1;
    uint8_t output[ZenohROSCodec::ATTACHMENT_SIZE];
    if (ZenohROSCodec::encodeAttachment(output, sizeof(output), attachment) != sizeof(output))
        return 1;
    for (const auto byte : output)
        std::printf("%02x", static_cast<unsigned>(byte));
    std::putchar('\n');
    return 0;
}

int main(int argc, char** argv)
{
#ifdef RAFTROS_ZENOH_GID_REFERENCE
    if (argc == 3 && std::strcmp(argv[1], "--reference-gid") == 0)
    {
        printGid(referenceGid(argv[2]));
        return 0;
    }
#endif
    if (argc == 3 && std::strcmp(argv[1], "--fixture") == 0)
        return emitFixture(argv[2]);
    if (argc == 4 && std::strcmp(argv[1], "--attachment") == 0)
        return emitAttachment(argv[2], argv[3]);
    if (argc != 1)
        return 2;
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

    std::printf("Test: canonical pinned Jazzy QoS fields\n");
    using Reliability = ZenohROSCodec::Reliability;
    using Durability = ZenohROSCodec::Durability;
    struct QoSCase
    {
        ZenohROSCodec::QoS qos;
        const char* expected;
    };
    const QoSCase qosCases[] = {
        {{Reliability::Reliable, Durability::Volatile, 42}, "::," ":," ":," ":,,"},
        {{Reliability::Reliable, Durability::Volatile, 10}, "::,10" ":," ":," ":,,"},
        {{Reliability::Reliable, Durability::Volatile, 7}, "::,7" ":," ":," ":,,"},
        {{Reliability::BestEffort, Durability::Volatile, 1}, "2::,1" ":," ":," ":,,"},
        {{Reliability::BestEffort, Durability::Volatile, 42}, "2::," ":," ":," ":,,"},
        {{Reliability::Reliable, Durability::TransientLocal, 10}, ":1:,10" ":," ":," ":,,"},
        {{Reliability::BestEffort, Durability::TransientLocal, UINT32_MAX}, "2:1:,4294967295" ":," ":," ":,,"}
    };
    for (const auto& testCase : qosCases)
    {
        TEST_ASSERT(ZenohROSCodec::formatQoS(key, ZenohROSCodec::QOS_BUFFER_SIZE, testCase.qos) &&
                    std::strcmp(key, testCase.expected) == 0,
                    "QoS values use ROS enum numbers and omit only pinned defaults");
        TEST_ASSERT(ZenohROSCodec::formatQoS(key, std::strlen(testCase.expected) + 1, testCase.qos) &&
                    std::strcmp(key, testCase.expected) == 0,
                    "QoS fits exact capacity including terminator");
        for (size_t capacity = 0; capacity <= std::strlen(testCase.expected); ++capacity)
        {
            std::memset(key, 'x', sizeof(key));
            TEST_ASSERT(!ZenohROSCodec::formatQoS(key + 1, capacity, testCase.qos) &&
                        key[0] == 'x' && key[capacity + 1] == 'x' &&
                        (capacity == 0 ? key[1] == 'x' : key[1] == '\0'),
                        "short QoS buffer fails without exposing a truncated value");
        }
    }
    const ZenohROSCodec::QoS invalidQoS[] = {
        {Reliability::Reliable, Durability::Volatile, 0},
        {static_cast<Reliability>(0), Durability::Volatile, 10},
        {static_cast<Reliability>(255), Durability::Volatile, 10},
        {Reliability::Reliable, static_cast<Durability>(0), 10},
        {Reliability::Reliable, static_cast<Durability>(255), 10}
    };
    for (const auto& qos : invalidQoS)
    {
        key[0] = 'x';
        TEST_ASSERT(!ZenohROSCodec::formatQoS(key, sizeof(key), qos) && key[0] == '\0',
                    "unresolved or invalid QoS is rejected, not silently downgraded");
    }
    TEST_ASSERT(!ZenohROSCodec::formatQoS(nullptr, ZenohROSCodec::QOS_BUFFER_SIZE, {}),
                "null QoS output rejected");

    std::printf("Test: pinned publisher and subscription token examples\n");
    using EndpointKind = ZenohROSCodec::EndpointKind;
    ZenohROSCodec::NodeIdentity node{0, sessionId, 0, "", "", "listener"};
    ZenohROSCodec::Endpoint endpoint{1, EndpointKind::Subscription, "/chatter", stringType, stringHash,
                                   {Reliability::Reliable, Durability::Volatile, 10}};
    constexpr const char* subscriptionToken =
        "@ros2_lv/0/aac3178e146ba6f1fc6e6a4085e77f21/0/1/MS/%/%/listener/%chatter/"
        "std_msgs::msg::dds_::String_/"
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,10" ":," ":," ":,,";
    constexpr const char* publisherToken =
        "@ros2_lv/0/8b20917502ee955ac447e0266340d5c/0/10/MP/%/%/talker/%chatter/"
        "std_msgs::msg::dds_::String_/"
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7" ":," ":," ":,,";
    TEST_ASSERT(ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) &&
                std::strcmp(key, subscriptionToken) == 0,
                "subscription token matches pinned design example");
    node.sessionId = "8b20917502ee955ac447e0266340d5c";
    node.nodeName = "talker";
    endpoint.entityId = 10;
    endpoint.kind = EndpointKind::Publisher;
    endpoint.qos.depth = 7;
    TEST_ASSERT(ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) &&
                std::strcmp(key, publisherToken) == 0,
                "publisher token matches pinned design example");

    node = {23, "1", 12, "/fleet/private", "/robot1", "raft_esp32"};
    endpoint = {34, EndpointKind::Publisher, "/robot1/chatter", stringType, stringHash,
                {Reliability::BestEffort, Durability::Volatile, 1}};
    const std::string expectedPublisher =
        "@ros2_lv/23/1/12/34/MP/%fleet%private/%robot1/raft_esp32/%robot1%chatter/" +
        std::string(stringType) + "/" + stringHash + "/2::,1" ":," ":," ":,,";
    TEST_ASSERT(ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) && key == expectedPublisher,
                "publisher owns distinct node and entity IDs, nested names and sensor QoS");
    for (const auto kind : {EndpointKind::Publisher, EndpointKind::Subscription})
    {
        endpoint.kind = kind;
        TEST_ASSERT(ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                    "both endpoint kinds format successfully");
        const std::string expected = key;
        for (size_t capacity = 0; capacity <= expected.size(); ++capacity)
        {
            std::memset(key, 'x', sizeof(key));
            TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key + 1, capacity, node, endpoint) &&
                        key[0] == 'x' && key[capacity + 1] == 'x' &&
                        (capacity == 0 ? key[1] == 'x' : key[1] == '\0'),
                        "short endpoint-token output is bounded and unusable on failure");
        }
        TEST_ASSERT(ZenohROSCodec::formatEndpointToken(key, expected.size() + 1, node, endpoint) && key == expected,
                    "endpoint token fits exact capacity");
    }
    endpoint.kind = static_cast<EndpointKind>(255);
    TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) && key[0] == '\0',
                "invalid endpoint kind rejected");
    endpoint.kind = EndpointKind::Publisher;
    TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(nullptr, sizeof(key), node, endpoint),
                "null endpoint-token output rejected");
    for (const auto topic : invalidTopics)
    {
        endpoint.topic = topic;
        TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) && key[0] == '\0',
                    "endpoint validates fully qualified topic before mangling");
    }
    endpoint.topic = "/chatter";
    for (const auto type : invalidTypes)
    {
        endpoint.wireType = type;
        TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                    "endpoint validates wire type");
    }
    endpoint.wireType = stringType;
    endpoint.typeHash = "RIHS01_missing";
    TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                "endpoint validates type hash");
    endpoint.typeHash = stringHash;
    for (const auto& qos : invalidQoS)
    {
        endpoint.qos = qos;
        TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) && key[0] == '\0',
                    "endpoint never advertises invalid or unresolved QoS");
    }
    endpoint.qos = {};
    for (const auto session : invalidSessions)
    {
        node.sessionId = session;
        TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                    "endpoint validates session identity");
    }
    node.sessionId = sessionId;
    for (const auto nodeName : invalidNodeNames)
    {
        node.nodeName = nodeName;
        TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                    "endpoint validates node identity");
    }
    node.nodeName = "raft_esp32";
    node.nodeNamespace = "/robot/*";
    TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                "endpoint namespace cannot inject wildcards");
    node.nodeNamespace = "/";
    node.enclave = "/fleet/%";
    TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint),
                "endpoint enclave cannot inject mangled separators");

    char largeToken[2048];
    const std::string maxNodeName(255, 'a');
    const std::string maxWireType = std::string(112, 'a') + "::msg::dds_::AB_";
    node = {UINT32_MAX, "0123456789abcdef0123456789abcdef", UINT64_MAX, maxTopic, maxTopic, maxNodeName};
    endpoint = {UINT64_MAX, EndpointKind::Publisher, maxTopic, maxWireType, stringHash,
                {Reliability::BestEffort, Durability::TransientLocal, UINT32_MAX}};
    TEST_ASSERT(maxWireType.size() == ZenohROSCodec::MAX_TYPE_NAME_SIZE &&
                ZenohROSCodec::formatEndpointToken(largeToken, sizeof(largeToken), node, endpoint),
                "all maximum-length endpoint fields fit explicit caller storage");
    TEST_ASSERT(!ZenohROSCodec::formatEndpointToken(key, sizeof(key), node, endpoint) && key[0] == '\0',
                "large valid token is not truncated into the smaller common buffer");

    std::printf("Test: endpoint GID determinism, bounds and reference agreement\n");
    using RaftRuntime::Zenoh::ZenohROSIdentity;
    node = {0, "1", 0, "/", "/", "a"};
    endpoint = {0, EndpointKind::Publisher, "/b", "a::msg::dds_::B_", stringHash, {}};
    ZenohROSIdentity::Gid derived{}, repeated{};
    TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) &&
                ZenohROSIdentity::deriveEndpointGid(node, endpoint, repeated) && derived == repeated,
                "endpoint identity is deterministic");
    const ZenohROSIdentity::Gid minimumTokenGid = {
        0xd6, 0x94, 0x4d, 0x1f, 0x87, 0x8b, 0x2c, 0xcc,
        0xa9, 0x40, 0x73, 0x0c, 0x04, 0xa6, 0x73, 0x53
    };
    TEST_ASSERT(derived == minimumTokenGid, "minimum endpoint matches captured upstream GID vector");
    std::array<char, ZenohROSIdentity::TOKEN_BUFFER_SIZE> minimumToken{};
    TEST_ASSERT(ZenohROSCodec::formatEndpointToken(minimumToken.data(), minimumToken.size(), node, endpoint) &&
                std::strlen(minimumToken.data()) > 16 && std::strlen(minimumToken.data()) <= 128,
                "minimum valid endpoint exercises the first medium-input path");
    const auto originalGid = derived;
    endpoint.kind = EndpointKind::Subscription;
    TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived != originalGid,
                "publisher and subscription have different identities");
    endpoint.kind = EndpointKind::Publisher;
    endpoint.qos.depth = 0;
    const auto beforeInvalid = derived;
    TEST_ASSERT(!ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == beforeInvalid,
                "invalid descriptors leave output identity unchanged");
    endpoint.qos = {};

    std::array<bool, 1400> coveredLengths{};
    for (size_t extension = 0; extension < 3; ++extension)
    {
        const std::string shortName = "a" + std::string(extension, 'b');
        node.nodeName = shortName;
        std::array<char, ZenohROSIdentity::TOKEN_BUFFER_SIZE> token{};
        TEST_ASSERT(ZenohROSCodec::formatEndpointToken(token.data(), token.size(), node, endpoint) &&
                    ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived),
                    "shortest endpoint variants produce bounded identities");
        coveredLengths[std::strlen(token.data())] = true;
#ifdef RAFTROS_ZENOH_GID_REFERENCE
        TEST_ASSERT(derived == referenceGid(token.data()), "shortest tokens match pinned RMW hash");
#endif
    }
    for (size_t extra = 0; extra <= 1000; ++extra)
    {
        const size_t topicExtra = extra < 253 ? extra : 253;
        const size_t namespaceExtra = extra > 253 ? (extra - 253 < 253 ? extra - 253 : 253) : 0;
        const size_t enclaveExtra = extra > 506 ? (extra - 506 < 253 ? extra - 506 : 253) : 0;
        const size_t nodeExtra = extra > 759 ? extra - 759 : 0;
        const std::string topicName = "/b" + std::string(topicExtra, 'x');
        const std::string namespaceName = "/n" + std::string(namespaceExtra, 'y');
        const std::string enclaveName = "/e" + std::string(enclaveExtra, 'z');
        const std::string nodeName = "a" + std::string(nodeExtra, 'w');
        node.nodeName = nodeName;
        node.nodeNamespace = namespaceName;
        node.enclave = enclaveName;
        endpoint.topic = topicName;
        std::array<char, ZenohROSIdentity::TOKEN_BUFFER_SIZE> token{};
        const bool formatted = ZenohROSCodec::formatEndpointToken(token.data(), token.size(), node, endpoint);
        const size_t tokenLength = std::strlen(token.data());
        TEST_ASSERT(formatted && tokenLength < coveredLengths.size() &&
                    ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived),
                    "bounded token identity handles medium, stripe and block lengths");
        if (formatted && tokenLength < coveredLengths.size())
            coveredLengths[tokenLength] = true;
#ifdef RAFTROS_ZENOH_GID_REFERENCE
    TEST_ASSERT(derived == referenceGid(token.data()), "GID bytes match pinned RMW hash implementation");
#endif
    }
    TEST_ASSERT(coveredLengths[128] && coveredLengths[129] && coveredLengths[160] &&
                coveredLengths[192] && coveredLengths[224] &&
                coveredLengths[239] && coveredLengths[240] && coveredLengths[241] &&
                coveredLengths[255] && coveredLengths[256] && coveredLengths[257] &&
                coveredLengths[1023] && coveredLengths[1024] && coveredLengths[1025],
                "reference corpus covers medium, final stripe and block boundaries");

    std::printf("Test: endpoint identity changes and fixed golden vectors\n");
    node = fixtureNode(sessionId);
    endpoint = fixturePublisher();
    const ZenohROSIdentity::Gid publisherGidVector = {
        0xe4, 0xf5, 0x6c, 0x26, 0x79, 0x0a, 0x63, 0x26,
        0x53, 0x97, 0xa6, 0x13, 0x65, 0x7e, 0x3a, 0xab
    };
    const ZenohROSIdentity::Gid subscriptionGidVector = {
        0x37, 0x47, 0xc3, 0x89, 0xaa, 0x1c, 0x3b, 0x32,
        0xca, 0x6f, 0x32, 0xa9, 0x04, 0xff, 0x7c, 0x18
    };
    TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == publisherGidVector,
                "publisher GID matches captured pinned helper byte order");
    endpoint.entityId = 2;
    endpoint.kind = EndpointKind::Subscription;
    TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == subscriptionGidVector,
                "subscription GID matches captured pinned helper byte order");
    endpoint = fixturePublisher();

    std::array<ZenohROSCodec::NodeIdentity, 6> changedNodes;
    changedNodes.fill(node);
    changedNodes[0].domainId = 24;
    changedNodes[1].sessionId = "1";
    changedNodes[2].nodeId = UINT64_MAX;
    changedNodes[3].enclave = "/private";
    changedNodes[4].nodeNamespace = "/different";
    changedNodes[5].nodeName = "different";
    for (const auto& changedNode : changedNodes)
    {
        TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(changedNode, endpoint, derived) && derived != publisherGidVector,
                    "every node-identity field participates in endpoint GID");
    }
    std::array<ZenohROSCodec::Endpoint, 8> changedEndpoints;
    changedEndpoints.fill(endpoint);
    changedEndpoints[0].entityId = UINT64_MAX;
    changedEndpoints[1].kind = EndpointKind::Subscription;
    changedEndpoints[2].topic = "/different";
    changedEndpoints[3].wireType = "std_msgs::msg::dds_::Bool_";
    const std::string changedHash = "RIHS01_" + std::string(64, '0');
    changedEndpoints[4].typeHash = changedHash;
    changedEndpoints[5].qos.reliability = Reliability::Reliable;
    changedEndpoints[6].qos.durability = Durability::TransientLocal;
    changedEndpoints[7].qos.depth = 42;
    for (const auto& changedEndpoint : changedEndpoints)
    {
        TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(node, changedEndpoint, derived) && derived != publisherGidVector,
                    "every endpoint descriptor field participates in GID");
    }
    for (const auto invalidTopic : invalidTopics)
    {
        endpoint.topic = invalidTopic;
        derived = publisherGidVector;
        TEST_ASSERT(!ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == publisherGidVector,
                    "invalid endpoint topic cannot overwrite identity");
    }
    endpoint = fixturePublisher();
    for (const auto invalidSession : invalidSessions)
    {
        node.sessionId = invalidSession;
        derived = publisherGidVector;
        TEST_ASSERT(!ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == publisherGidVector,
                    "invalid session cannot overwrite identity");
    }
    node = {UINT32_MAX, "0123456789abcdef0123456789abcdef", UINT64_MAX, maxTopic, maxTopic, maxNodeName};
    endpoint = {UINT64_MAX, EndpointKind::Publisher, maxTopic, maxWireType, stringHash,
                {Reliability::BestEffort, Durability::TransientLocal, UINT32_MAX}};
    const ZenohROSIdentity::Gid maximumTokenGid = {
        0xb4, 0x8e, 0x63, 0x2e, 0x77, 0xef, 0x52, 0x02,
        0x82, 0xb4, 0x9b, 0xae, 0x60, 0x25, 0x34, 0x50
    };
    TEST_ASSERT(ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == maximumTokenGid,
                "maximum endpoint matches captured multi-block upstream vector");
    const std::string oversizedName(256, 'a');
    node.nodeName = oversizedName;
    TEST_ASSERT(!ZenohROSIdentity::deriveEndpointGid(node, endpoint, derived) && derived == maximumTokenGid,
                "oversized descriptor fails without hashing a truncated token");

    std::printf("Zenoh codec: %d passed, %d failed\n", passCount, failCount);
    return failCount == 0 ? 0 : 1;
}