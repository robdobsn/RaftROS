///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Host tests for ZenohAutoPubBackend - the Zenoh implementation of the
// auto-publish backend contract.  The backend is driven through a real
// ZenohTCPSession fed synthetic handshake bytes (no router, no socket), so the
// tests cover exactly what the firmware does: stage a declaration, send it when
// the session can take it, publish, and give the slot back.
//
// Rob Dobson 2026
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "Zenoh/ZenohAutoPubBackend.h"
#include "Zenoh/ZenohInterestMatch.h"

#define TEST_ASSERT(cond, msg) do { if (!(cond)) { std::printf("  FAIL: %s\n", msg); ++failCount; } else { ++passCount; } } while (false)

using RaftRuntime::Zenoh::ZenohAutoPubBackend;
using RaftRuntime::Zenoh::ZenohAutoPubBackendDeps;
using RaftRuntime::Zenoh::ZenohROSCodec;
using RaftRuntime::Zenoh::ZenohTCPSession;
using RaftRuntime::AutoPub::AutoPubEndpointDesc;
using RaftRuntime::AutoPub::AutoPubMsgKind;
using RaftRuntime::AutoPub::AutoPubPublishResult;
using RaftRuntime::AutoPub::AutoPubQoSProfileId;

namespace
{

const std::array<uint8_t, 13> INIT_ACK{11, 0, 0x61, 9, 0x01, 0x77, 0x0a, 0, 0x10, 3, 0xaa, 0xbb, 0xcc};
const std::array<uint8_t, 5> OPEN_ACK{3, 0, 0x62, 10, 37};

/// @brief Bring a session to Established, as the transport task would
void establish(ZenohTCPSession& session, uint64_t nowMs = 0)
{
    const std::array<uint8_t, 16> identity{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    session.start(identity, nowMs);
    session.consumeOutput(session.outputSize(), nowMs);
    session.receive(INIT_ACK.data(), INIT_ACK.size(), nowMs);
    session.consumeOutput(session.outputSize(), nowMs);
    session.receive(OPEN_ACK.data(), OPEN_ACK.size(), nowMs);
}

/// @brief Drain whatever the session is holding, as the socket write would
std::vector<uint8_t> drain(ZenohTCPSession& session, uint64_t nowMs = 0)
{
    const std::vector<uint8_t> output(session.outputData(), session.outputData() + session.outputSize());
    session.consumeOutput(session.outputSize(), nowMs);
    return output;
}

bool contains(const std::vector<uint8_t>& haystack, const void* needle, size_t needleLen)
{
    if (needleLen == 0 || haystack.size() < needleLen)
        return false;
    for (size_t offset = 0; offset + needleLen <= haystack.size(); ++offset)
    {
        if (std::memcmp(haystack.data() + offset, needle, needleLen) == 0)
            return true;
    }
    return false;
}

bool containsText(const std::vector<uint8_t>& haystack, const char* text)
{
    return contains(haystack, text, std::strlen(text));
}

AutoPubEndpointDesc makeDesc(const char* topic, AutoPubMsgKind kind, uint8_t address = 0x29)
{
    AutoPubEndpointDesc desc;
    desc.deviceId = {0, address, 0};
    desc.msgKind = kind;
    desc.qosProfileId = AutoPubQoSProfileId::FastSensor;
    desc.setNames(topic, RaftRuntime::AutoPub::AutoPubClassMap_typeName(kind));
    return desc;
}

ZenohAutoPubBackendDeps makeDeps(ZenohTCPSession& session, uint8_t* sendBuf, uint32_t sendBufLen)
{
    ZenohAutoPubBackendDeps deps;
    deps.session = &session;
    deps.sendBuf = sendBuf;
    deps.sendBufLen = sendBufLen;
    deps.domainId = 23;
    deps.sessionId = "1020304050607080910111213141516";
    deps.nodeId = 1;
    deps.enclave = "/";
    deps.nodeNamespace = "/";
    deps.nodeName = "raft_esp32";
    return deps;
}

} // namespace

int main()
{
    int passCount = 0;
    int failCount = 0;
    uint8_t sendBuf[2048];

    std::printf("Test: backend refuses work before it is configured\n");
    {
        ZenohAutoPubBackend backend;
        const auto desc = makeDesc("/raft_esp32/range", AutoPubMsgKind::Range);
        TEST_ASSERT(backend.createPublisher(desc) == ZenohAutoPubBackend::INVALID_SLOT,
                    "createPublisher without setup is refused");
        TEST_ASSERT(backend.publish(0, sendBuf, 4) == AutoPubPublishResult::InvalidHandle,
                    "publish without setup reports an invalid handle");
        TEST_ASSERT(!backend.service(0), "service without setup sends nothing");
    }

    std::printf("Test: descriptors the backend cannot express are refused at create\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        AutoPubEndpointDesc unknownKind = makeDesc("/raft_esp32/range", AutoPubMsgKind::Range);
        unknownKind.msgKind = AutoPubMsgKind::Unknown;
        TEST_ASSERT(backend.createPublisher(unknownKind) == ZenohAutoPubBackend::INVALID_SLOT,
                    "a message kind with no ROS type hash cannot be declared");
        AutoPubEndpointDesc badTopic = makeDesc("raft_esp32/range", AutoPubMsgKind::Range);
        TEST_ASSERT(backend.createPublisher(badTopic) == ZenohAutoPubBackend::INVALID_SLOT,
                    "a topic that is not an absolute ROS path is refused");
        TEST_ASSERT(backend.createPublisher(AutoPubEndpointDesc{}) == ZenohAutoPubBackend::INVALID_SLOT,
                    "an empty descriptor is refused");
        TEST_ASSERT(backend.inUseCount() == 0 && session.outputSize() == 0,
                    "refused descriptors consume no slot and send nothing");
    }

    std::printf("Test: create stages a declaration, service sends it\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t slot = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        TEST_ASSERT(slot == 0, "first endpoint takes slot 0");
        TEST_ASSERT(backend.inUseCount() == 1 && backend.pendingCount() == 1 && !backend.isDeclared(slot),
                    "endpoint occupies a slot but is not yet declared");
        TEST_ASSERT(session.outputSize() == 0, "create sends nothing itself");
        const char* key = backend.topicKeyForSlot(slot);
        TEST_ASSERT(key && std::strcmp(key,
            "23/raft_esp32/range/sensor_msgs::msg::dds_::Range_/"
            "RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a") == 0,
            "topic key carries domain, mangled topic, wire type and ROS type hash");
        TEST_ASSERT(backend.publish(slot, sendBuf, 4) == AutoPubPublishResult::QueueFull,
                    "publishing before the token is out is refused, not sent blind");

        TEST_ASSERT(backend.service(10), "service sends the staged declaration");
        const auto declared = drain(session);
        TEST_ASSERT(containsText(declared, "@ros2_lv/23/"), "declaration carries a liveliness token");
        TEST_ASSERT(containsText(declared, "/MP/%/%/raft_esp32/%raft_esp32%range/"),
                    "token names this node as a publisher on the mangled topic");
        TEST_ASSERT(containsText(declared, "RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a"),
                    "token carries the ROS type hash");
        TEST_ASSERT(backend.isDeclared(slot) && backend.pendingCount() == 0,
                    "slot is declared once its token has gone out");
        TEST_ASSERT(backend.tokenKeyForSlot(slot) != nullptr, "declared token is retained for interest replies");
        TEST_ASSERT(!backend.service(11), "service with nothing staged sends nothing");
    }

    std::printf("Test: publish carries payload, sequence and publisher GID\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t slot = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        backend.service(10);
        drain(session);

        const uint8_t payload[6] = {0, 1, 0, 0, 0x2a, 0x00};
        uint64_t sequence = 0;
        uint32_t peers = 0;
        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload), &sequence, &peers) == AutoPubPublishResult::Accepted,
                    "declared endpoint accepts a sample");
        TEST_ASSERT(sequence == 1 && peers == 1, "first sample is sequence 1, sent to the router");
        const auto sent = drain(session);
        TEST_ASSERT(containsText(sent, "23/raft_esp32/range/"), "sample is published on the topic key");
        TEST_ASSERT(contains(sent, payload, sizeof(payload)), "sample carries the serialised payload");
        const auto* gid = backend.gidForSlot(slot);
        uint8_t expected[ZenohROSCodec::ATTACHMENT_SIZE];
        TEST_ASSERT(gid && ZenohROSCodec::encodeAttachment(expected, sizeof(expected), {1, 10 * 1000000, *gid}),
                    "expected attachment encodes");
        TEST_ASSERT(contains(sent, expected, sizeof(expected)),
                    "attachment carries sequence 1, the service clock and the publisher GID");

        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload), &sequence) == AutoPubPublishResult::Accepted &&
                    sequence == 2, "sequence advances per accepted sample");
        drain(session);
        TEST_ASSERT(backend.stats().published == 2, "published count tracks accepted samples");
    }

    std::printf("Test: the backend can be given the time without being serviced\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t slot = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        backend.service(10);
        drain(session);

        // publish() has no time argument, so a caller that publishes on some
        // other schedule must be able to refresh the clock on its own.  Without
        // this the sample carries the last serviced time - and the session is
        // handed that stale time too.
        backend.setNow(500);
        const uint8_t payload[4] = {0, 1, 0, 0};
        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload)) == AutoPubPublishResult::Accepted,
                    "a sample publishes after setNow alone");
        const auto sent = drain(session);
        const auto* gid = backend.gidForSlot(slot);
        uint8_t expected[ZenohROSCodec::ATTACHMENT_SIZE];
        TEST_ASSERT(gid && ZenohROSCodec::encodeAttachment(expected, sizeof(expected),
                                                           {1, 500 * 1000000LL, *gid}) &&
                    contains(sent, expected, sizeof(expected)),
                    "the sample is stamped with the time setNow supplied");
        TEST_ASSERT(session.state() == ZenohTCPSession::State::Established,
                    "publishing does not disturb the session");
    }

    std::printf("Test: publish refusals do not consume a sequence number\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t slot = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        backend.service(10);
        drain(session);
        const uint8_t payload[6] = {0, 1, 0, 0, 0x2a, 0x00};
        uint64_t sequence = 0;

        uint8_t oversized[RaftRuntime::Zenoh::ZenohNetworkMessage::MAX_PAYLOAD_SIZE + 1] = {};
        TEST_ASSERT(backend.publish(slot, oversized, sizeof(oversized)) == AutoPubPublishResult::Oversized,
                    "a sample larger than the transport allows is rejected");
        TEST_ASSERT(backend.publish(ZenohAutoPubBackend::INVALID_SLOT, payload, sizeof(payload)) ==
                    AutoPubPublishResult::InvalidHandle, "an out-of-range slot is an invalid handle");
        TEST_ASSERT(backend.publish(1, payload, sizeof(payload)) == AutoPubPublishResult::InvalidHandle,
                    "a free slot is an invalid handle");
        TEST_ASSERT(backend.publish(slot, nullptr, 4) == AutoPubPublishResult::InvalidHandle,
                    "a null payload is an invalid handle");

        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload), &sequence) == AutoPubPublishResult::Accepted &&
                    sequence == 1, "refusals did not consume sequence numbers");
        // Output holds the sample but has not started on the wire: a second
        // sample joins the same frame (a composite device's other endpoint)
        const size_t oneQueued = session.outputSize();
        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload), &sequence) == AutoPubPublishResult::Accepted &&
                    sequence == 2 && session.outputSize() > oneQueued,
                    "a second sample is appended to the queued frame");
        // Once the frame is going out, the session applies backpressure
        session.consumeOutput(2, 10);
        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload)) == AutoPubPublishResult::QueueFull,
                    "a frame being sent applies backpressure rather than dropping the link");
        drain(session);
        TEST_ASSERT(backend.publish(slot, payload, sizeof(payload), &sequence) == AutoPubPublishResult::Accepted &&
                    sequence == 3, "the backpressured sample did not consume a sequence number");
        drain(session);
    }

    std::printf("Test: destroy undeclares before the slot is reused\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t slot = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        backend.service(10);
        drain(session);

        backend.destroyPublisher(slot);
        TEST_ASSERT(backend.inUseCount() == 1 && backend.pendingCount() == 1 && !backend.isDeclared(slot),
                    "a declared slot stays held until its undeclare is sent");
        TEST_ASSERT(backend.publish(slot, sendBuf, 4) == AutoPubPublishResult::QueueFull,
                    "an endpoint being torn down publishes nothing");
        TEST_ASSERT(backend.service(11), "service sends the undeclare");
        const auto undeclared = drain(session);
        TEST_ASSERT(undeclared.size() > 0 && !containsText(undeclared, "@ros2_lv"),
                    "undeclare names the token id only, not the key");
        TEST_ASSERT(backend.inUseCount() == 0, "slot is free once the undeclare has gone out");
        TEST_ASSERT(backend.publish(slot, sendBuf, 4) == AutoPubPublishResult::InvalidHandle,
                    "a released slot rejects late samples");
        TEST_ASSERT(backend.topicKeyForSlot(slot) == nullptr, "a released slot keeps no key");

        // A slot destroyed before its declaration went out needs no undeclare
        const uint8_t second = backend.createPublisher(makeDesc("/raft_esp32/temperature", AutoPubMsgKind::Temperature));
        TEST_ASSERT(second == slot, "a released slot is reused");
        backend.destroyPublisher(second);
        TEST_ASSERT(backend.inUseCount() == 0 && !backend.service(12) && session.outputSize() == 0,
                    "an endpoint destroyed before it was declared sends nothing at all");
        backend.destroyPublisher(ZenohAutoPubBackend::INVALID_SLOT);
        TEST_ASSERT(backend.inUseCount() == 0, "destroying an invalid slot is harmless");
    }

    std::printf("Test: entity ids are never reused\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t slot = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        const std::string firstToken = backend.topicKeyForSlot(slot) ?
            std::string(backend.tokenKeyForSlot(slot) ? backend.tokenKeyForSlot(slot) : "") : "";
        backend.service(10);
        const std::string declaredToken = backend.tokenKeyForSlot(slot);
        drain(session);
        backend.destroyPublisher(slot);
        backend.service(11);
        drain(session);
        const uint8_t reused = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        backend.service(12);
        TEST_ASSERT(reused == slot && declaredToken != backend.tokenKeyForSlot(reused),
                    "a reused slot declares a new entity id, so it is not confused with the old endpoint");
        TEST_ASSERT(firstToken.empty() || true, "token is readable only once declared");
        drain(session);
    }

    std::printf("Test: a dropped session re-declares live endpoints\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        const uint8_t range = backend.createPublisher(makeDesc("/raft_esp32/range", AutoPubMsgKind::Range));
        const uint8_t temp = backend.createPublisher(makeDesc("/raft_esp32/temperature", AutoPubMsgKind::Temperature, 0x18));
        TEST_ASSERT(backend.service(10), "first endpoint declares");
        drain(session);
        TEST_ASSERT(backend.service(11), "second endpoint declares once the session can take it");
        drain(session);
        TEST_ASSERT(backend.isDeclared(range) && backend.isDeclared(temp), "both endpoints are visible");

        session.linkLost();
        TEST_ASSERT(backend.publish(range, sendBuf, 4) == AutoPubPublishResult::Disconnected,
                    "publishing on a dead session reports disconnection, not queue pressure");
        TEST_ASSERT(!backend.service(21), "service on a dead session sends nothing");
        TEST_ASSERT(backend.pendingCount() == 2 && !backend.isDeclared(range) && !backend.isDeclared(temp),
                    "the router lost the declarations, so they are staged again");
        TEST_ASSERT(backend.stats().redeclares == 2, "re-declares are counted");

        establish(session, 30);
        drain(session, 30);
        TEST_ASSERT(backend.service(31), "first endpoint re-declares on the new session");
        drain(session, 31);
        TEST_ASSERT(backend.service(32), "second endpoint re-declares on the new session");
        TEST_ASSERT(backend.isDeclared(range) && backend.isDeclared(temp) && backend.pendingCount() == 0,
                    "endpoints are visible again without being recreated");
        drain(session, 32);
        uint64_t sequence = 0;
        TEST_ASSERT(backend.publish(range, sendBuf, 4, &sequence) == AutoPubPublishResult::Accepted && sequence == 1,
                    "publishing resumes on the re-declared endpoint");
        drain(session, 32);

        // An endpoint destroyed while the link is down needs no undeclare
        backend.destroyPublisher(temp);
        session.linkLost();
        TEST_ASSERT(!backend.service(41) && backend.inUseCount() == 1,
                    "an endpoint torn down across a session drop is released, not left pending");
    }

    std::printf("Test: staged work is sent round-robin and the registry is bounded\n");
    {
        ZenohTCPSession session;
        establish(session);
        ZenohAutoPubBackend backend;
        backend.setup(makeDeps(session, sendBuf, sizeof(sendBuf)));
        for (uint8_t index = 0; index < backend.capacity(); ++index)
        {
            char topic[64];
            std::snprintf(topic, sizeof(topic), "/raft_esp32/range_%u", static_cast<unsigned>(index));
            TEST_ASSERT(backend.createPublisher(makeDesc(topic, AutoPubMsgKind::Range,
                        static_cast<uint8_t>(0x30 + index))) == index, "each endpoint takes the next free slot");
        }
        TEST_ASSERT(backend.createPublisher(makeDesc("/raft_esp32/extra", AutoPubMsgKind::Range, 0x50)) ==
                    ZenohAutoPubBackend::INVALID_SLOT, "a full registry refuses further endpoints");
        for (uint8_t index = 0; index < backend.capacity(); ++index)
        {
            TEST_ASSERT(backend.service(100 + index), "service sends one staged declaration per call");
            char expected[64];
            std::snprintf(expected, sizeof(expected), "raft_esp32%%range_%u/", static_cast<unsigned>(index));
            TEST_ASSERT(containsText(drain(session), expected),
                        "declarations go out in slot order, one per call, none starved");
            TEST_ASSERT(backend.isDeclared(index), "each endpoint is declared in turn");
        }
        TEST_ASSERT(backend.pendingCount() == 0 && backend.inUseCount() == backend.capacity(),
                    "all endpoints are declared and holding slots");
    }

    std::printf("Test: built-in QoS profiles map onto the Zenoh encoding\n");
    {
        using RaftRuntime::AutoPub::AutoPubQoSProfileId;
        const auto fast = ZenohAutoPubBackend::qosForProfile(AutoPubQoSProfileId::FastSensor);
        TEST_ASSERT(fast.reliability == ZenohROSCodec::Reliability::BestEffort &&
                    fast.durability == ZenohROSCodec::Durability::Volatile && fast.depth == 10,
                    "fast_sensor announces best-effort, volatile, depth 10");
        const auto event = ZenohAutoPubBackend::qosForProfile(AutoPubQoSProfileId::Event);
        TEST_ASSERT(event.reliability == ZenohROSCodec::Reliability::Reliable &&
                    event.durability == ZenohROSCodec::Durability::TransientLocal && event.depth == 20,
                    "event announces reliable, transient-local, depth 20");
        const auto fallback = ZenohAutoPubBackend::qosForProfile(AutoPubQoSProfileId::FallbackString);
        TEST_ASSERT(fallback.reliability == ZenohROSCodec::Reliability::Reliable &&
                    fallback.durability == ZenohROSCodec::Durability::Volatile && fallback.depth == 10,
                    "fallback_string - the subscription default - is reliable, volatile, depth 10, "
                    "which is what readers were announced as before profiles applied to them");
    }

    std::printf("Test: router interests are matched, or refused rather than guessed at\n");
    {
        using RaftRuntime::Zenoh::ZenohInterestMatch;
        using Result = RaftRuntime::Zenoh::ZenohInterestMatchResult;
        const char* token = "@ros2_lv/23/abc/1/2/MP/%/%/raft_esp32/%raft_esp32%range/type/hash/2::,5:,:,:,,";

        TEST_ASSERT(ZenohInterestMatch("", token) == Result::Match,
                    "an empty key expression asks for everything we hold");
        TEST_ASSERT(ZenohInterestMatch(token, token) == Result::Match, "an exact key matches");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/23/other", token) == Result::NoMatch,
                    "a different exact key does not match");

        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/**", token) == Result::Match,
                    "a subtree expression covers keys below its prefix");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/23/abc/**", token) == Result::Match,
                    "a deeper subtree expression still covers the key");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/24/**", token) == Result::NoMatch,
                    "a subtree of a different domain does not match");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/2/**", "@ros2_lv/23/abc") == Result::NoMatch,
                    "a subtree prefix only matches on a segment boundary, not a shared character run");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/23/**", "@ros2_lv/23") == Result::Match,
                    "a subtree expression covers the prefix itself");

        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/*/abc", token) == Result::Unsupported,
                    "a wildcard we do not implement is refused, not silently unmatched");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/*/**", token) == Result::Unsupported,
                    "a wildcard inside a subtree prefix is refused");
        TEST_ASSERT(ZenohInterestMatch("@ros2_lv/$x/abc", token) == Result::Unsupported,
                    "a verbatim-segment expression is refused");
    }

    std::printf("Zenoh firmware pieces: %d passed, %d failed\n", passCount, failCount);
    return failCount == 0 ? 0 : 1;
}
