#include "Zenoh/ZenohInterestMatch.h"
#include "Zenoh/ZenohTCPSession.h"
#include "Zenoh/ZenohNetworkMessage.h"
#include "Zenoh/ZenohROSIdentity.h"
#include "CDR/CDREncoder.h"
#include "AutoPub/AutoPubSampleRunner.h"
#include <arpa/inet.h>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <poll.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <unistd.h>

using RaftRuntime::Zenoh::ZenohTCPSession;

static uint64_t nowMs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

static bool parseNumber(const char* text, unsigned& value)
{
    const char* end = text + std::strlen(text);
    const auto result = std::from_chars(text, end, value);
    return result.ec == std::errc{} && result.ptr == end;
}

static bool serializeRangeSample(uint32_t sequence, uint8_t* output, uint32_t capacity, uint32_t& written)
{
    using namespace RaftRuntime::AutoPub;
    written = 0;
    if (sequence == 0 || sequence > 64)
        return false;
    struct PollRecord
    {
        uint32_t timeMs;
        uint16_t distanceRaw;
    };
    constexpr uint16_t distancesMm[]{0, 184, 1000, 2000};
    const PollRecord record{1234 + (sequence - 1) * 250,
        static_cast<uint16_t>(2 * distancesMm[(sequence - 1) % 4])};
    const AutoPubAttrFieldDesc distanceField{
        "dist", static_cast<uint16_t>(offsetof(PollRecord, distanceRaw)), AutoPubAttrType::Uint16, "%u", 2.0f, 0.0f
    };
    AutoPubDecodedBatch batch;
    batch.data = reinterpret_cast<const uint8_t*>(&record);
    batch.capacity = sizeof(record);
    batch.recordSize = sizeof(record);
    batch.recordCount = 1;
    batch.fields = &distanceField;
    batch.fieldCount = 1;
    batch.frameId = "raft_range_1_29";
    const AutoPubSampleOutput sampleOutput{AutoPubMsgKind::Range, output, capacity};
    AutoPubSampleResult result;
    const bool validBatch = AutoPubSampleRunner::run(batch, &sampleOutput, 1, &result,
        [&](uint8_t, const uint8_t*, uint32_t length, uint32_t) {
            written = length;
            return AutoPubPublishResult::Accepted;
        });
    return validBatch && result.serialized;
}

struct Socket
{
    int descriptor = -1;
    ~Socket() { if (descriptor >= 0) ::close(descriptor); }
};

class PublicationProbe
{
public:
    bool init(const std::array<uint8_t, 16>& identity, bool publishRange)
    {
        using RaftRuntime::Zenoh::ZenohROSCodec;
        _publishRange = publishRange;
        char sessionId[33];
        for (size_t index = 0; index < identity.size(); ++index)
            std::snprintf(sessionId + index * 2, 3, "%02x", identity[15 - index]);
        const char* first = sessionId;
        while (*first == '0')
            ++first;
        ZenohROSCodec::NodeIdentity node{23, first, 0, "/", "/raft_test", "raft_fixture"};
        ZenohROSCodec::Endpoint endpoint{1, ZenohROSCodec::EndpointKind::Publisher, "/raft_test/chatter",
            "std_msgs::msg::dds_::String_", "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            {ZenohROSCodec::Reliability::BestEffort, ZenohROSCodec::Durability::Volatile, 5}};
        if (_publishRange)
        {
            using namespace RaftRuntime::AutoPub;
            endpoint.topic = "/raft_test/range";
            endpoint.wireType = AutoPubClassMap_typeName(AutoPubMsgKind::Range);
            endpoint.typeHash = "RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a";
        }
        if (!ZenohROSCodec::formatNodeToken(_nodeToken.data(), _nodeToken.size(), node.domainId, node.sessionId,
                node.nodeId, node.enclave, node.nodeNamespace, node.nodeName) ||
            !ZenohROSCodec::formatEndpointToken(_publisherToken.data(), _publisherToken.size(), node, endpoint) ||
            !ZenohROSCodec::formatTopicKey(_topicKey.data(), _topicKey.size(), node.domainId, endpoint.topic, endpoint.wireType, endpoint.typeHash) ||
            !RaftRuntime::Zenoh::ZenohROSIdentity::deriveEndpointGid(node, endpoint, _attachment.publisherGid))
            return false;
        std::printf("{\"publisher_gid\":\"");
        for (const auto byte : _attachment.publisherGid)
            std::printf("%02x", byte);
        std::printf("\",\"type_hash\":\"%.*s\"}\n", static_cast<int>(endpoint.typeHash.size()), endpoint.typeHash.data());
        return true;
    }

    bool onDiscovery(const RaftRuntime::Zenoh::ZenohNetworkMessage::DiscoveryMessage& message)
    {
        if (!message.isInterest)
            return true;
        if (message.mode == 0)
        {
            for (size_t index = 0; index < _replyCount;)
            {
                if (_replies[index].id == message.id)
                    _replies[index] = _replies[--_replyCount];
                else
                    ++index;
            }
            return true;
        }
        if (message.mode == 2)
            return true;
        if (_replyCount == _replies.size() || message.scope != 0)
            return false;
        Reply reply{message.id, 0, 0};
        if (message.options & 8)
        {
            for (uint8_t token = 0; token < 2; ++token)
            {
                const std::string_view key = token == 0 ? _nodeToken.data() : _publisherToken.data();
                // Shared with the firmware SysMod: an expression we do not
                // implement is refused rather than answered incompletely
                const auto match = RaftRuntime::Zenoh::ZenohInterestMatch(message.key, key);
                if (match == RaftRuntime::Zenoh::ZenohInterestMatchResult::Unsupported)
                    return false;
                const bool active = token == 0 ? _stage >= 2 && _stage < 6 : _stage >= 3 && _stage < 5;
                if (match == RaftRuntime::Zenoh::ZenohInterestMatchResult::Match && active)
                    reply.mask |= static_cast<uint8_t>(1u << token);
            }
        }
        _replies[_replyCount++] = reply;
        return true;
    }

    bool step(ZenohTCPSession& session, uint64_t elapsed, uint64_t now)
    {
        using RaftRuntime::Zenoh::ZenohNetworkMessage;
        if (session.outputSize())
            return true;
        if (_replyCount)
        {
            auto& reply = _replies[0];
            while (reply.next < 2 && !(reply.mask & (1u << reply.next)))
                ++reply.next;
            const size_t replySize = reply.next < 2 ?
                ZenohNetworkMessage::declareToken(_message.data(), _message.size(), reply.next + 1,
                    reply.next == 0 ? _nodeToken.data() : _publisherToken.data(), &reply.id) :
                ZenohNetworkMessage::declareFinal(_message.data(), _message.size(), &reply.id);
            if (!replySize)
                return false;
            if (session.sendNetworkMessage(_message.data(), replySize, now))
            {
                if (++reply.next == 3)
                {
                    for (size_t index = 1; index < _replyCount; ++index)
                        _replies[index - 1] = _replies[index];
                    --_replyCount;
                }
            }
            return true;
        }
        if (done())
            return true;
        size_t length = 0;
        if (_stage == 0)
            length = ZenohNetworkMessage::declareFinal(_message.data(), _message.size());
        else if (_stage == 1)
            length = ZenohNetworkMessage::declareToken(_message.data(), _message.size(), 1, _nodeToken.data());
        else if (_stage == 2)
            length = ZenohNetworkMessage::declareToken(_message.data(), _message.size(), 2, _publisherToken.data());
        else if (_stage == 3)
        {
            if (elapsed >= 6500)
            {
                _stage = 4;
                return true;
            }
            if (now - _lastPublish < 250)
                return true;
            uint8_t payload[128], metadata[RaftRuntime::Zenoh::ZenohROSCodec::ATTACHMENT_SIZE];
            uint32_t payloadSize = 0;
            if (_publishRange)
            {
                if (!serializeRangeSample(_samples + 1, payload, sizeof(payload), payloadSize))
                    return false;
            }
            else
            {
                CDREncoder encoder;
                encoder.reset(payload, sizeof(payload));
                if (!encoder.writeEncapsulationHeader() || !encoder.writeString("from_raft_tcp"))
                    return false;
                payloadSize = encoder.getPos();
            }
            _attachment.sequenceNumber = _samples + 1;
            if (!RaftRuntime::Zenoh::ZenohROSCodec::encodeAttachment(metadata, sizeof(metadata), _attachment))
                return false;
            length = ZenohNetworkMessage::put(_message.data(), _message.size(), _topicKey.data(),
                                              payload, payloadSize, metadata, sizeof(metadata));
        }
        else
            length = ZenohNetworkMessage::undeclareToken(_message.data(), _message.size(), _stage == 4 ? 2 : 1);
        if (!length)
            return false;
        if (session.sendNetworkMessage(_message.data(), length, now))
        {
            if (_stage == 3)
            {
                ++_samples;
                _lastPublish = now;
            }
            else
                ++_stage;
        }
        return true;
    }
    bool done() const { return _stage == 6; }
    uint32_t samples() const { return _samples; }
private:
    struct Reply { uint32_t id; uint8_t mask; uint8_t next; };
    std::array<Reply, 4> _replies{};
    size_t _replyCount = 0;
    std::array<char, 1536> _nodeToken{}, _publisherToken{}, _topicKey{};
    std::array<uint8_t, ZenohTCPSession::BATCH_CAPACITY> _message{};
    RaftRuntime::Zenoh::ZenohROSCodec::Attachment _attachment;
    unsigned _stage = 0;
    uint32_t _samples = 0;
    uint64_t _lastPublish = 0;
    bool _publishRange = false;
};

struct ProbeStorage
{
    static constexpr size_t RX_SCRATCH_SIZE = 2048;
    static constexpr size_t MAX_STORAGE_SIZE = 24 * 1024;

    ProbeStorage() = default;
    ProbeStorage(const ProbeStorage&) = delete;
    ProbeStorage& operator=(const ProbeStorage&) = delete;

    ZenohTCPSession session;
    PublicationProbe publication;
    std::array<uint8_t, RX_SCRATCH_SIZE> receiveBuffer{};
};

static_assert(sizeof(ProbeStorage) <= ProbeStorage::MAX_STORAGE_SIZE,
              "Probe session/publication workspace exceeds its fixed storage budget");

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "--resource-info") == 0)
    {
        std::printf("{\"pointer_bytes\":%zu,\"session_bytes\":%zu,\"publication_probe_bytes\":%zu,"
                    "\"socket_rx_scratch_bytes\":%zu,\"gid_token_scratch_bytes\":%zu,"
                    "\"owned_storage_bytes\":%zu,\"owned_storage_limit_bytes\":%zu,"
                    "\"storage_location\":\"single_heap_allocation\","
                    "\"batch_capacity\":%zu,\"cookie_capacity\":%zu}\n",
                    sizeof(void*), sizeof(ZenohTCPSession), sizeof(PublicationProbe),
                    ProbeStorage::RX_SCRATCH_SIZE,
                    RaftRuntime::Zenoh::ZenohROSIdentity::TOKEN_BUFFER_SIZE,
                    sizeof(ProbeStorage), ProbeStorage::MAX_STORAGE_SIZE,
                    ZenohTCPSession::BATCH_CAPACITY, ZenohTCPSession::COOKIE_CAPACITY);
        return 0;
    }
    if (argc == 3 && std::strcmp(argv[1], "--range-payload") == 0)
    {
        unsigned sequence = 0;
        uint8_t payload[128];
        uint32_t size = 0;
        if (!parseNumber(argv[2], sequence) || !serializeRangeSample(sequence, payload, sizeof(payload), size))
            return 2;
        for (uint32_t offset = 0; offset < size; ++offset)
            std::printf("%02x", static_cast<unsigned>(payload[offset]));
        std::putchar('\n');
        return 0;
    }
    unsigned port = 0, durationMs = 12000;
    const bool publishRange = argc == 5 && std::strcmp(argv[4], "--publish-range") == 0;
    const bool publish = publishRange || (argc == 5 && std::strcmp(argv[4], "--publish") == 0);
    if (argc < 3 || argc > 5 || (argc == 5 && !publish) || !parseNumber(argv[2], port) || port == 0 || port > 65535 ||
        (argc >= 4 && (!parseNumber(argv[3], durationMs) || durationMs < 12000 || durationMs > 120000)))
    {
        std::fprintf(stderr, "Usage: zenoh_session_probe IPv4 PORT [DURATION_MS=12000, max 120000] [--publish|--publish-range]\n");
        return 2;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, argv[1], &address.sin_addr) != 1)
        return 2;
    std::unique_ptr<ProbeStorage> storage(new (std::nothrow) ProbeStorage);
    if (!storage)
    {
        std::fprintf(stderr, "Unable to allocate %zu bytes of bounded probe storage\n", sizeof(ProbeStorage));
        return 1;
    }
    auto& session = storage->session;
    auto& publication = storage->publication;
    auto& receiveBuffer = storage->receiveBuffer;
    std::array<uint8_t, 16> identity{};
    size_t randomBytes = 0;
    while (randomBytes < identity.size())
    {
        const ssize_t count = getrandom(identity.data() + randomBytes, identity.size() - randomBytes, 0);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0)
        {
            std::perror("getrandom");
            return 1;
        }
        randomBytes += static_cast<size_t>(count);
    }
    Socket socket;
    socket.descriptor = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket.descriptor < 0)
    {
        std::perror("socket");
        return 1;
    }
    if (connect(socket.descriptor, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        if (errno != EINPROGRESS)
        {
            std::perror("connect");
            return 1;
        }
        const uint64_t deadline = nowMs() + 5000;
        pollfd connectPoll{socket.descriptor, POLLOUT, 0};
        int ready = 0;
        do
        {
            const uint64_t current = nowMs();
            const uint64_t remaining = deadline > current ? deadline - current : 0;
            ready = poll(&connectPoll, 1, static_cast<int>(remaining));
        } while (ready < 0 && errno == EINTR && nowMs() < deadline);
        int error = 0;
        socklen_t errorSize = sizeof(error);
        if (ready <= 0 || getsockopt(socket.descriptor, SOL_SOCKET, SO_ERROR, &error, &errorSize) < 0 || error != 0)
        {
            std::fprintf(stderr, "TCP connect failed or timed out: %d\n", error);
            return 1;
        }
    }
    const uint64_t startedAt = nowMs();
    if (!session.start(identity, startedAt))
        return 1;
    if (publish && !publication.init(identity, publishRange))
        return 1;
    bool established = false;
    uint64_t establishedAt = 0;
    uint64_t sentBytes = 0, receivedBytes = 0;
    while (nowMs() - startedAt < durationMs + 2 * ZenohTCPSession::HANDSHAKE_TIMEOUT_MS)
    {
        session.service(nowMs());
        if (session.state() == ZenohTCPSession::State::Failed || session.state() == ZenohTCPSession::State::Closed)
            break;
        pollfd socketPoll{socket.descriptor, static_cast<short>(POLLIN | (session.outputSize() ? POLLOUT : 0)), 0};
        const int ready = poll(&socketPoll, 1, 100);
        if (ready < 0 && errno == EINTR)
            continue;
        if (ready < 0 || (socketPoll.revents & (POLLERR | POLLNVAL)))
        {
            session.linkLost();
            break;
        }
        if (socketPoll.revents & POLLOUT)
        {
            const ssize_t sent = send(socket.descriptor, session.outputData(), session.outputSize(), MSG_NOSIGNAL);
            if (sent > 0)
            {
                session.consumeOutput(static_cast<size_t>(sent), nowMs());
                sentBytes += static_cast<uint64_t>(sent);
            }
            else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
                session.linkLost();
        }
        if (socketPoll.revents & (POLLIN | POLLHUP))
        {
            const ssize_t count = recv(socket.descriptor, receiveBuffer.data(), receiveBuffer.size(), 0);
            if (count > 0)
            {
                receivedBytes += static_cast<uint64_t>(count);
                if (!session.receive(receiveBuffer.data(), static_cast<size_t>(count), nowMs(),
                    publish ? [](void* context, const RaftRuntime::Zenoh::ZenohNetworkMessage::DiscoveryMessage& message) {
                        return static_cast<PublicationProbe*>(context)->onDiscovery(message);
                    } : static_cast<ZenohTCPSession::DiscoveryCallback>(nullptr), &publication))
                    break;
            }
            else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
                session.linkLost();
        }
        if (session.state() == ZenohTCPSession::State::Established)
        {
            if (!established)
            {
                established = true;
                establishedAt = nowMs();
                std::printf("Established Raft-owned TCP session: batch=%u remoteLeaseMs=%llu initialSN=%u\n",
                    static_cast<unsigned>(session.negotiatedBatch()),
                    static_cast<unsigned long long>(session.remoteLeaseMs()), session.remoteInitialSequence());
                std::fflush(stdout);
            }
            if (publish && !publication.step(session, nowMs() - establishedAt, nowMs()))
                break;
            if (nowMs() - establishedAt >= durationMs && session.keepAlivesReceived() >= 2 &&
                (!publish || (publication.done() && publication.samples() >= 5 && session.outputSize() == 0)))
            {
                if (publish)
                    std::printf("Raft TCP publications queued=%u, tokens withdrawn\n", publication.samples());
                std::printf("PASS: Raft-owned INIT/OPEN and keepalive for %u ms; rxKeepAlives=%u rxFrames=%u txBytes=%llu rxBytes=%llu\n",
                    durationMs, session.keepAlivesReceived(), session.frameCount(),
                    static_cast<unsigned long long>(sentBytes), static_cast<unsigned long long>(receivedBytes));
                return 0;
            }
        }
    }
    std::fprintf(stderr, "Session probe failed: state=%u error=%u closeReason=%u keepAlives=%u txBytes=%llu rxBytes=%llu\n",
        static_cast<unsigned>(session.state()), static_cast<unsigned>(session.error()),
        static_cast<unsigned>(session.closeReason()), session.keepAlivesReceived(),
        static_cast<unsigned long long>(sentBytes), static_cast<unsigned long long>(receivedBytes));
    return 1;
}