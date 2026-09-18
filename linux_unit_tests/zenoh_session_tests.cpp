#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "Zenoh/ZenohStreamFramer.h"
#include "Zenoh/ZenohTCPSession.h"
#include "Zenoh/ZenohNetworkMessage.h"

#define TEST_ASSERT(cond, msg) do { if (!(cond)) { std::printf("  FAIL: %s\n", msg); ++failCount; } else { ++passCount; } } while (false)

int main()
{
    int passCount = 0;
    int failCount = 0;
    using Framer = RaftRuntime::Zenoh::ZenohStreamFramer<256>;
    using Result = Framer::Result;
    std::printf("Test: bounded TCP batch framing\n");
    const std::array<uint8_t, 9> stream{3, 0, 0x01, 0x09, 0x02, 2, 0, 0x22, 0x01};
    for (size_t split = 0; split <= stream.size(); ++split)
    {
        Framer framer;
        size_t completeCount = 0;
        const auto consume = [&](size_t start, size_t end)
        {
            for (size_t offset = start; offset < end; ++offset)
            {
                const auto result = framer.push(stream[offset]);
                TEST_ASSERT(result != Result::InvalidLength, "valid batch accepted across split reads");
                if (result == Result::Complete)
                {
                    ++completeCount;
                    if (completeCount == 1)
                        TEST_ASSERT(framer.size() == 3 && std::memcmp(framer.data(), stream.data() + 2, 3) == 0,
                                    "first batch excludes length header");
                    else
                        TEST_ASSERT(framer.size() == 2 && std::memcmp(framer.data(), stream.data() + 7, 2) == 0,
                                    "coalesced next batch is independently framed");
                }
            }
        };
        consume(0, split);
        consume(split, stream.size());
        TEST_ASSERT(completeCount == 2, "split position does not change completed batch count");
    }
    for (const size_t length : {size_t{1}, size_t{255}, size_t{256}})
    {
        Framer framer;
        TEST_ASSERT(framer.push(static_cast<uint8_t>(length)) == Result::Incomplete &&
                    framer.push(static_cast<uint8_t>(length >> 8)) == Result::Incomplete,
                    "length header uses little-endian uint16");
        for (size_t offset = 0; offset < length; ++offset)
        {
            TEST_ASSERT(framer.push(static_cast<uint8_t>(offset)) ==
                        (offset + 1 == length ? Result::Complete : Result::Incomplete),
                        "completion occurs at exactly the advertised length");
        }
    }
    for (const size_t length : {size_t{0}, size_t{257}, size_t{65535}})
    {
        Framer framer;
        framer.push(static_cast<uint8_t>(length));
        TEST_ASSERT(framer.push(static_cast<uint8_t>(length >> 8)) == Result::InvalidLength,
                    "zero and over-capacity lengths are rejected before buffering");
        TEST_ASSERT(framer.push(1) == Result::InvalidLength, "invalid stream remains failed until reset");
        framer.reset();
        TEST_ASSERT(framer.push(1) == Result::Incomplete && framer.push(0) == Result::Incomplete &&
                    framer.push(4) == Result::Complete && framer.data()[0] == 4,
                    "explicit reset accepts a new transport stream");
    }
    Framer partial;
    partial.push(3);
    partial.push(0);
    partial.push(1);
    partial.reset();
    TEST_ASSERT(partial.push(1) == Result::Incomplete && partial.push(0) == Result::Incomplete &&
                partial.push(4) == Result::Complete, "reset discards partial payload from old connection");
    std::printf("Test: INIT/OPEN handshake and keepalive state\n");
    using Session = RaftRuntime::Zenoh::ZenohTCPSession;
    const std::array<uint8_t, 16> identity{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    const std::array<uint8_t, 13> initAck{11, 0, 0x61, 9, 0x01, 0x77, 0x0a, 0, 0x10, 3, 0xaa, 0xbb, 0xcc};
    const std::array<uint8_t, 5> openAck{3, 0, 0x62, 10, 37};
    for (size_t split = 0; split <= initAck.size(); ++split)
    {
        Session session;
        TEST_ASSERT(session.start(identity, 0), "valid identity starts session");
        const auto* initial = session.outputData();
        TEST_ASSERT(session.outputSize() == 24 && initial[0] == 22 && initial[1] == 0 &&
                    initial[2] == 0x41 && initial[3] == 9 && initial[4] == 0xf2 &&
                    std::memcmp(initial + 5, identity.data(), identity.size()) == 0 &&
                    initial[21] == 0x0a && initial[22] == 0 && initial[23] == 0x10,
                    "INIT SYN advertises client, version, resolutions and bounded batch");
        TEST_ASSERT(!session.consumeOutput(25, 0) && session.outputSize() == 24, "TX over-consume rejected");
        TEST_ASSERT(session.consumeOutput(1, 1) && session.outputSize() == 23 &&
                    session.consumeOutput(23, 2), "partial TCP writes preserve remaining output");
        TEST_ASSERT(session.receive(initAck.data(), split, 3) &&
                    session.receive(initAck.data() + split, initAck.size() - split, 4) &&
                    session.state() == Session::State::AwaitOpenAck, "split INIT ACK advances handshake");
        const uint8_t expectedOpen[]{7, 0, 0x42, 4, 0, 3, 0xaa, 0xbb, 0xcc};
        TEST_ASSERT(session.outputSize() == sizeof(expectedOpen) &&
                    std::memcmp(session.outputData(), expectedOpen, sizeof(expectedOpen)) == 0,
                    "OPEN SYN echoes opaque cookie and advertises four-second lease");
        session.consumeOutput(session.outputSize(), 5);
        TEST_ASSERT(session.receive(openAck.data(), openAck.size(), 6) &&
                    session.state() == Session::State::Established && session.remoteLeaseMs() == 10000 &&
                    session.remoteInitialSequence() == 37, "OPEN ACK establishes negotiated state");
        TEST_ASSERT(!session.start(identity, 7), "active session cannot be implicitly replaced");
        session.service(1004);
        TEST_ASSERT(session.outputSize() == 0, "keepalive not sent before cadence");
        session.service(1005);
        const uint8_t keepalive[]{1, 0, 4};
        TEST_ASSERT(session.outputSize() == 3 && std::memcmp(session.outputData(), keepalive, 3) == 0,
                    "keepalive sent at quarter of advertised local lease");
        session.consumeOutput(3, 1005);
        const uint8_t coalescedKeepalives[]{2, 0, 4, 4, 1, 0, 4};
        TEST_ASSERT(session.receive(coalescedKeepalives, sizeof(coalescedKeepalives), 8000) &&
                    session.keepAlivesReceived() == 3, "batched keepalives are parsed and counted");
        session.service(18000);
        TEST_ASSERT(session.state() == Session::State::Failed && session.error() == Session::Error::LeaseExpired,
                    "remote silence expires lease independently of local sends");
    }
    Session invalidIdentity;
    TEST_ASSERT(!invalidIdentity.start({}, 0) && invalidIdentity.outputSize() == 0,
                "all-zero session identity rejected");
    Session timeout;
    timeout.start(identity, 10);
    timeout.service(5010);
    TEST_ASSERT(timeout.error() == Session::Error::HandshakeTimeout && timeout.outputSize() == 0,
                "handshake and stalled initial send have finite timeout");
    TEST_ASSERT(timeout.start(identity, 6000), "explicit restart resets failed framing and handshake state");
    timeout.linkLost();
    TEST_ASSERT(timeout.error() == Session::Error::LinkLost, "EOF/socket error marks session failed");
    Session closed;
    closed.start(identity, 0);
    const uint8_t close[]{2, 0, 0x23, 1};
    TEST_ASSERT(closed.receive(close, sizeof(close), 1) && closed.state() == Session::State::Closed &&
                closed.closeReason() == 1 && closed.outputSize() == 0, "peer rejection is reported without retry loop");
    for (size_t index : {size_t{2}, size_t{3}, size_t{4}, size_t{6}, size_t{8}, size_t{9}})
    {
        Session invalid;
        invalid.start(identity, 0);
        invalid.consumeOutput(invalid.outputSize(), 0);
        auto malformed = initAck;
        malformed[index] = 0xff;
        TEST_ASSERT(!invalid.receive(malformed.data(), malformed.size(), 1) && invalid.state() == Session::State::Failed,
                    "invalid INIT flags, version, resolution, batch or cookie fails closed");
    }
    std::printf("Test: truncated and malformed handshake batches\n");
    const auto sendPayload = [](Session& session, const std::vector<uint8_t>& payload, uint64_t timeMs)
    {
        std::vector<uint8_t> wire{static_cast<uint8_t>(payload.size()), static_cast<uint8_t>(payload.size() >> 8)};
        wire.insert(wire.end(), payload.begin(), payload.end());
        return session.receive(wire.data(), wire.size(), timeMs);
    };
    const auto prepareInit = [&](Session& session)
    {
        session.start(identity, 0);
        session.consumeOutput(session.outputSize(), 0);
    };
    const auto prepareOpen = [&](Session& session)
    {
        prepareInit(session);
        session.receive(initAck.data(), initAck.size(), 1);
        session.consumeOutput(session.outputSize(), 1);
    };
    const auto establish = [&](Session& session)
    {
        prepareOpen(session);
        session.receive(openAck.data(), openAck.size(), 2);
    };
    const std::vector<uint8_t> initBody(initAck.begin() + 2, initAck.end());
    const std::vector<uint8_t> openBody(openAck.begin() + 2, openAck.end());
    for (size_t length = 0; length < initBody.size(); ++length)
    {
        Session session;
        prepareInit(session);
        TEST_ASSERT(!sendPayload(session, {initBody.begin(), initBody.begin() + length}, 1) &&
                    session.state() == Session::State::Failed && session.outputSize() == 0,
                    "every complete-but-truncated INIT ACK batch fails without TX");
    }
    for (size_t length = 0; length < openBody.size(); ++length)
    {
        Session session;
        prepareOpen(session);
        TEST_ASSERT(!sendPayload(session, {openBody.begin(), openBody.begin() + length}, 2) &&
                    session.state() == Session::State::Failed,
                    "every complete-but-truncated OPEN ACK batch fails");
    }
    for (const std::vector<uint8_t>& body : {
        std::vector<uint8_t>{0x62, 0, 0},
        std::vector<uint8_t>{0x62, 121, 0},
        std::vector<uint8_t>{0x62, 10, 0x80, 0x80, 0x80, 0x80, 0x10},
        std::vector<uint8_t>{0x62, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0},
        std::vector<uint8_t>{0x62, 10, 0x80},
        std::vector<uint8_t>{0x62, 10, 0, 0}
    })
    {
        Session session;
        prepareOpen(session);
        TEST_ASSERT(!sendPayload(session, body, 2), "invalid lease, sequence, unterminated integer or trailing OPEN data rejected");
    }
    Session milliseconds;
    prepareOpen(milliseconds);
    TEST_ASSERT(sendPayload(milliseconds, {0x22, 0xa0, 0x1f, 0xac, 0x02}, 2) &&
                milliseconds.remoteLeaseMs() == 4000 && milliseconds.remoteInitialSequence() == 300,
                "OPEN supports millisecond lease and multibyte sequence integer");

    std::printf("Test: bounded negotiation and extensions\n");
    for (const auto batchSize : {size_t{64}, size_t{256}, Session::BATCH_CAPACITY})
    {
        Session session;
        prepareInit(session);
        auto body = initBody;
        body[5] = static_cast<uint8_t>(batchSize);
        body[6] = static_cast<uint8_t>(batchSize >> 8);
        TEST_ASSERT(sendPayload(session, body, 1) && session.negotiatedBatch() == batchSize,
                    "peer may reduce negotiated batch capacity");
    }
    for (const uint8_t resolution : {uint8_t{0}, uint8_t{5}, uint8_t{10}})
    {
        Session session;
        prepareInit(session);
        auto body = initBody;
        body[4] = resolution;
        TEST_ASSERT(sendPayload(session, body, 1), "8/16/32-bit negotiated resolutions accepted");
        session.consumeOutput(session.outputSize(), 1);
        TEST_ASSERT(sendPayload(session, openBody, 2), "initial sequence fits negotiated resolution");
    }
    Session smallSequence;
    prepareInit(smallSequence);
    auto reduced = initBody;
    reduced[4] = 0;
    sendPayload(smallSequence, reduced, 1);
    smallSequence.consumeOutput(smallSequence.outputSize(), 1);
    TEST_ASSERT(!sendPayload(smallSequence, {0x62, 10, 0x80, 2}, 2), "OPEN sequence cannot exceed 8-bit negotiation");
    Session absentSizes;
    prepareInit(absentSizes);
    TEST_ASSERT(!sendPayload(absentSizes, {0x21, 9, 1, 0x77, 0}, 1) &&
                absentSizes.error() == Session::Error::UnsupportedNegotiation,
                "omitted size fields imply protocol defaults, not acceptance of small local batch");
    for (const size_t cookieLength : {size_t{0}, size_t{127}, size_t{128}, Session::COOKIE_CAPACITY})
    {
        Session session;
        prepareInit(session);
        std::vector<uint8_t> body{0x61, 9, 1, 0x77, 0x0a, 0, 0x10};
        if (cookieLength < 128)
            body.push_back(static_cast<uint8_t>(cookieLength));
        else
        {
            body.push_back(static_cast<uint8_t>((cookieLength & 0x7f) | 0x80));
            body.push_back(static_cast<uint8_t>(cookieLength >> 7));
        }
        body.resize(body.size() + cookieLength, 0xa5);
        TEST_ASSERT(sendPayload(session, body, 1) && session.outputSize() >= cookieLength + 6,
                    "cookie length boundaries fit fixed output storage");
        TEST_ASSERT(cookieLength == 0 || std::memcmp(session.outputData() + session.outputSize() - cookieLength,
                    body.data() + body.size() - cookieLength, cookieLength) == 0,
                    "cookie echoed byte-for-byte without interpretation");
    }
    Session tooMuchCookie;
    prepareInit(tooMuchCookie);
    TEST_ASSERT(!sendPayload(tooMuchCookie, {0x61, 9, 1, 0x77, 0x0a, 0, 0x10, 0x81, 8}, 1),
                "over-capacity cookie length rejected before reading cookie");
    for (const std::vector<uint8_t>& extension : {
        std::vector<uint8_t>{0x08},
        std::vector<uint8_t>{0x28, 0x80, 1},
        std::vector<uint8_t>{0x48, 2, 1, 2},
        std::vector<uint8_t>{0x88, 0x28, 1},
        std::vector<uint8_t>{0x27, 0},
        std::vector<uint8_t>{0x28, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
    })
    {
        Session session;
        prepareInit(session);
        auto body = initBody;
        body[0] |= 0x80;
        body.insert(body.end(), extension.begin(), extension.end());
        TEST_ASSERT(sendPayload(session, body, 1), "well-formed optional extensions are bounded and skipped");
    }
    for (const std::vector<uint8_t>& extension : {
        std::vector<uint8_t>{}, std::vector<uint8_t>{0x18}, std::vector<uint8_t>{0x68},
        std::vector<uint8_t>{0x28, 0x80}, std::vector<uint8_t>{0x48, 2, 1},
        std::vector<uint8_t>{0x88}, std::vector<uint8_t>{0x01}, std::vector<uint8_t>{0x05},
        std::vector<uint8_t>{0x06}, std::vector<uint8_t>{0x27, 1}, std::vector<uint8_t>(17, 0x88)
    })
    {
        Session session;
        prepareInit(session);
        auto body = initBody;
        body[0] |= 0x80;
        body.insert(body.end(), extension.begin(), extension.end());
        TEST_ASSERT(!sendPayload(session, body, 1), "mandatory, malformed or unrequested negotiation extensions fail closed");
    }
    Session self;
    std::array<uint8_t, 16> shortIdentity{0x77};
    self.start(shortIdentity, 0);
    self.consumeOutput(self.outputSize(), 0);
    TEST_ASSERT(!sendPayload(self, initBody, 1) && self.error() == Session::Error::InvalidIdentity,
                "short peer identity cannot disguise connection to self");

    std::printf("Test: late traffic and blocked writes cannot keep session alive\n");
    Session lateInit;
    prepareInit(lateInit);
    TEST_ASSERT(!sendPayload(lateInit, initBody, 5000) && lateInit.error() == Session::Error::HandshakeTimeout,
                "receive enforces deadline before accepting late INIT ACK");
    Session lateOpen;
    prepareOpen(lateOpen);
    TEST_ASSERT(!sendPayload(lateOpen, openBody, 5001) && lateOpen.error() == Session::Error::HandshakeTimeout,
                "OPEN ACK must arrive before its own handshake deadline");
    Session lateKeepalive;
    establish(lateKeepalive);
    TEST_ASSERT(!sendPayload(lateKeepalive, {4}, 10002) && lateKeepalive.error() == Session::Error::LeaseExpired,
                "receive cannot resurrect an expired session");
    Session stalled;
    establish(stalled);
    stalled.service(1001);
    stalled.consumeOutput(1, 1002);
    stalled.service(5001);
    TEST_ASSERT(stalled.error() == Session::Error::SendStalled && stalled.outputSize() == 0,
                "partial TX progress cannot extend a queued batch indefinitely");
    Session trickle;
    prepareInit(trickle);
    trickle.receive(initAck.data(), 3, 4999);
    trickle.service(5000);
    TEST_ASSERT(trickle.error() == Session::Error::HandshakeTimeout, "partial RX bytes do not reset handshake timeout");
    Session nullInput;
    prepareInit(nullInput);
    TEST_ASSERT(!nullInput.receive(nullptr, 1, 1) && nullInput.error() == Session::Error::MalformedMessage,
                "nonempty null receive input fails with diagnostic");
    Session frame;
    establish(frame);
    TEST_ASSERT(sendPayload(frame, {0x25, 37, 0x1e, 0x1a, 4}, 3) && frame.frameCount() == 1 &&
                frame.discoveryCount() == 1 && frame.keepAlivesReceived() == 1,
                "discovery boundary preserves following keepalive in the same TCP batch");
    TEST_ASSERT(!sendPayload(frame, {0x06}, 4), "fragment reassembly is explicitly unsupported");

    std::printf("Test: network declaration and PUT wire builders\n");
    using RaftRuntime::Zenoh::ZenohNetworkMessage;
    std::array<uint8_t, 64> message{};
    const uint8_t tokenWire[]{0x1e, 0x26, 0x81, 1, 0, 3, 'a', '/', 'b'};
    TEST_ASSERT(ZenohNetworkMessage::declareToken(message.data(), message.size(), 129, "a/b") == sizeof(tokenWire) &&
                std::memcmp(message.data(), tokenWire, sizeof(tokenWire)) == 0,
                "token declaration contains unscoped named key and z32 ID");
    const uint32_t interestId = 5;
    const uint8_t replyWire[]{0x3e, 5, 0x26, 1, 0, 1, 'a'};
    TEST_ASSERT(ZenohNetworkMessage::declareToken(message.data(), message.size(), 1, "a", &interestId) == sizeof(replyWire) &&
                std::memcmp(message.data(), replyWire, sizeof(replyWire)) == 0,
                "current-interest reply carries interest ID");
    const uint8_t finalWire[]{0x3e, 5, 0x1a};
    TEST_ASSERT(ZenohNetworkMessage::declareFinal(message.data(), message.size(), &interestId) == sizeof(finalWire) &&
                std::memcmp(message.data(), finalWire, sizeof(finalWire)) == 0, "declaration final closes current reply");
    const uint8_t removeWire[]{0x1e, 7, 0xac, 2};
    TEST_ASSERT(ZenohNetworkMessage::undeclareToken(message.data(), message.size(), 300) == sizeof(removeWire) &&
                std::memcmp(message.data(), removeWire, sizeof(removeWire)) == 0, "token removal carries original token ID");
    const uint8_t cdr[]{0, 1, 0, 0};
    const uint8_t attachment[]{1, 2, 3};
    const uint8_t putWire[]{0x3d, 0, 3, 'a', '/', 'b', 0x81, 0x43, 3, 1, 2, 3, 4, 0, 1, 0, 0};
    TEST_ASSERT(ZenohNetworkMessage::put(message.data(), message.size(), "a/b", cdr, sizeof(cdr), attachment, sizeof(attachment)) == sizeof(putWire) &&
                std::memcmp(message.data(), putWire, sizeof(putWire)) == 0,
                "PUT embeds attachment extension and length-prefixed CDR without RTPS envelope");
    for (size_t capacity = 0; capacity < sizeof(putWire); ++capacity)
    {
        message.fill(0xa5);
        TEST_ASSERT(ZenohNetworkMessage::put(message.data() + 1, capacity, "a/b", cdr, sizeof(cdr), attachment, sizeof(attachment)) == 0 &&
                    message[0] == 0xa5 && message[capacity + 1] == 0xa5, "short writer output never crosses capacity");
    }
    TEST_ASSERT(ZenohNetworkMessage::put(nullptr, 64, "a", cdr, 4, nullptr, 0) == 0 &&
                ZenohNetworkMessage::put(message.data(), 64, "a", nullptr, 4, nullptr, 0) == 0,
                "invalid output/payload pointers rejected");
    for (const char* key : {"", "/a", "a/", "a//b", "a/*", "a?b", "a b"})
        TEST_ASSERT(ZenohNetworkMessage::declareToken(message.data(), message.size(), 1, key) == 0,
                    "invalid concrete key rejected before encoding");
    Session sender;
    TEST_ASSERT(!sender.sendNetworkMessage(finalWire, sizeof(finalWire), 0), "network TX requires established session");
    establish(sender);
    TEST_ASSERT(sender.sendNetworkMessage(finalWire, sizeof(finalWire), 3), "network message queued on established TCP link");
    const uint8_t framed[]{5, 0, 0x25, 0, 0x3e, 5, 0x1a};
    TEST_ASSERT(sender.outputSize() == sizeof(framed) && std::memcmp(sender.outputData(), framed, sizeof(framed)) == 0,
                "reliable frame uses initial sequence and TCP batch prefix");
    TEST_ASSERT(!sender.sendNetworkMessage(finalWire, sizeof(finalWire), 3) &&
                std::memcmp(sender.outputData(), framed, sizeof(framed)) == 0, "busy send preserves queued bytes");
    sender.consumeOutput(sender.outputSize(), 3);
    TEST_ASSERT(sender.sendNetworkMessage(finalWire, sizeof(finalWire), 4) && sender.outputData()[3] == 1,
                "frame sequence advances only for accepted send");
    sender.consumeOutput(sender.outputSize(), 4);
    std::array<uint8_t, Session::BATCH_CAPACITY> oversized{};
    TEST_ASSERT(!sender.sendNetworkMessage(oversized.data(), oversized.size(), 5) && sender.outputSize() == 0,
                "frame envelope counts against negotiated batch size");

    std::printf("Test: discovery parsing and transport sequence continuity\n");
    ZenohNetworkMessage::DiscoveryMessage discovery;
    TEST_ASSERT(ZenohNetworkMessage::readDiscovery(tokenWire, sizeof(tokenWire), discovery) == sizeof(tokenWire) &&
                !discovery.isInterest && discovery.declaration == 6 && discovery.id == 129 && discovery.scope == 0 && discovery.key == "a/b",
                "token parser identifies its exact end and borrowed suffix");
    TEST_ASSERT(ZenohNetworkMessage::readDiscovery(finalWire, sizeof(finalWire), discovery) == sizeof(finalWire) &&
                discovery.declaration == 0x1a && discovery.hasInterestId && discovery.interestId == 5,
                "final parser retains interest correlation");
    const uint8_t interestWire[]{0x79, 3, 0x38, 0, 4, '@', '/', '*', '*'};
    TEST_ASSERT(ZenohNetworkMessage::readDiscovery(interestWire, sizeof(interestWire), discovery) == sizeof(interestWire) &&
                discovery.isInterest && discovery.mode == 3 && discovery.id == 3 && discovery.options == 0x38 && discovery.key == "@/**",
                "current/future token interest parses full restricted key");
    for (size_t length = 0; length < sizeof(interestWire); ++length)
    {
        discovery.id = 123;
        TEST_ASSERT(ZenohNetworkMessage::readDiscovery(interestWire, length, discovery) == 0 && discovery.id == 123,
                    "truncated discovery does not mutate output descriptor");
    }
    for (size_t length = 0; length < sizeof(tokenWire); ++length)
        TEST_ASSERT(ZenohNetworkMessage::readDiscovery(tokenWire, length, discovery) == 0, "truncated declaration rejected");
    Session ordered;
    establish(ordered);
    TEST_ASSERT(sendPayload(ordered, {0x25, 37, 0x1e, 0x1a, 0x25, 38, 0x19, 1, 4}, 3) &&
                ordered.frameCount() == 2 && ordered.discoveryCount() == 2 && ordered.keepAlivesReceived() == 1,
                "two ordered frames and a control message share one batch");
    TEST_ASSERT(!sendPayload(ordered, {0x25, 38, 0x1e, 0x1a}, 4) && ordered.error() == Session::Error::SequenceMismatch,
                "duplicate reliable sequence fails rather than replaying discovery callbacks");
    Session unknownNetwork;
    establish(unknownNetwork);
    TEST_ASSERT(!sendPayload(unknownNetwork, {0x25, 37, 0x1d, 0}, 3), "unsupported network kinds fail instead of discarding arbitrary batch tails");
    Session callbacks;
    establish(callbacks);
    const uint8_t receivedInterest[]{5, 0, 0x25, 37, 0x59, 1, 8};
    uint32_t observedId = 0;
    TEST_ASSERT(callbacks.receive(receivedInterest, sizeof(receivedInterest), 3,
        [](void* context, const ZenohNetworkMessage::DiscoveryMessage& parsed) {
            *static_cast<uint32_t*>(context) = parsed.id;
            return parsed.isInterest;
        }, &observedId) && observedId == 1, "session exposes discovery through synchronous borrowed callback");

    for (const std::vector<uint8_t>& invalid : {
        std::vector<uint8_t>{0x19}, std::vector<uint8_t>{0x39, 1, 0x60},
        std::vector<uint8_t>{0x39, 1, 0x38, 0, 5, 'x'},
        std::vector<uint8_t>{0x39, 1, 0x38, 0, 1, 0},
        std::vector<uint8_t>{0x1e, 0x08}, std::vector<uint8_t>{0x5e, 0x1a},
        std::vector<uint8_t>{0x1e, 0x47, 1}, std::vector<uint8_t>{0x1e, 0x06, 1, 0},
        std::vector<uint8_t>{0x1e, 0x26, 1, 0, 0},
        std::vector<uint8_t>{0x19, 0xff, 0xff, 0xff, 0xff, 0x10},
        std::vector<uint8_t>{0x1e, 0x20, 0x80, 0x80, 4, 0, 1, 'a'},
        std::vector<uint8_t>{0x9e, 0x33, 1, 0x1a},
        std::vector<uint8_t>{0x9e, 0x18, 0x1a},
        std::vector<uint8_t>{0x9e, 0x48, 3, 1, 2},
        std::vector<uint8_t>{0x9e, 0x68, 0x1a},
        std::vector<uint8_t>{0x1e, 0x87, 1, 0x5f, 2, 0, 1}
    })
    {
        discovery.id = 456;
        TEST_ASSERT(ZenohNetworkMessage::readDiscovery(invalid.data(), invalid.size(), discovery) == 0 && discovery.id == 456,
                    "malformed flags, key bounds, IDs and unsupported mandatory extensions leave event unchanged");
    }
    const uint8_t optionalExtensions[]{0x9e, 0xa8, 0xac, 2, 0x48, 2, 1, 2, 0x1a};
    TEST_ASSERT(ZenohNetworkMessage::readDiscovery(optionalExtensions, sizeof(optionalExtensions), discovery) == sizeof(optionalExtensions),
                "known-length optional network extensions can be skipped without losing message boundary");
    const uint8_t keyDeclaration[]{0x1e, 0x20, 1, 0, 3, 'a', '/', 'b'};
    TEST_ASSERT(ZenohNetworkMessage::readDiscovery(keyDeclaration, sizeof(keyDeclaration), discovery) == sizeof(keyDeclaration) &&
                discovery.declaration == 0 && discovery.id == 1, "key declaration is surfaced without inventing a scope table");
    const uint8_t scopedSubscriber[]{0x1e, 0x42, 2, 1};
    TEST_ASSERT(ZenohNetworkMessage::readDiscovery(scopedSubscriber, sizeof(scopedSubscriber), discovery) == sizeof(scopedSubscriber) &&
                discovery.declaration == 2 && discovery.scope == 1 && discovery.senderMapping,
                "scoped declaration retains mapping for caller handling");
    Session rejected;
    establish(rejected);
    TEST_ASSERT(!rejected.receive(receivedInterest, sizeof(receivedInterest), 3,
        [](void*, const ZenohNetworkMessage::DiscoveryMessage&) { return false; }) &&
        rejected.error() == Session::Error::DiscoveryRejected, "full or unsupported application discovery state fails explicitly");
    Session partialDiscovery;
    establish(partialDiscovery);
    TEST_ASSERT(!sendPayload(partialDiscovery, {0x25, 37, 0x1e, 0x26, 1, 0, 10, 'a', 4}, 3),
                "truncated network key does not consume following control as a valid message");
    Session channelSequence;
    establish(channelSequence);
    TEST_ASSERT(sendPayload(channelSequence, {0x25, 37, 0x1e, 0x1a, 0x05, 37, 0x1e, 0x1a}, 3),
                "reliable and best-effort sequence spaces start independently");
    TEST_ASSERT(!sendPayload(channelSequence, {0x05, 39, 0x1e, 0x1a}, 4),
                "unsupported receive sequence gaps do not silently skip discovery");
    Session wrap;
    prepareInit(wrap);
    reduced = initBody;
    reduced[4] = 0;
    sendPayload(wrap, reduced, 1);
    wrap.consumeOutput(wrap.outputSize(), 1);
    TEST_ASSERT(sendPayload(wrap, {0x62, 10, 0xff, 1}, 2) &&
                sendPayload(wrap, {0x25, 0xff, 1, 0x1e, 0x1a, 0x25, 0, 0x1e, 0x1a}, 3),
                "8-bit receive sequence wraps from 255 to zero");
    for (unsigned sequence = 0; sequence <= 257; ++sequence)
    {
        TEST_ASSERT(wrap.sendNetworkMessage(finalWire, sizeof(finalWire), 4) &&
                    wrap.outputData()[3] == ((sequence & 255) >= 128 ? ((sequence & 127) | 0x80) : (sequence & 255)),
                    "8-bit transmit sequence wraps without losing pending-write ordering");
        wrap.consumeOutput(wrap.outputSize(), 4);
    }
    std::array<uint8_t, 4096> largeMessage{};
    const std::string maxKey(ZenohNetworkMessage::MAX_KEY_SIZE, 'a');
    const std::string oversizedKey = maxKey + 'a';
    const std::array<uint8_t, ZenohNetworkMessage::MAX_PAYLOAD_SIZE> maxPayload{};
    const std::array<uint8_t, ZenohNetworkMessage::MAX_ATTACHMENT_SIZE> maxAttachment{};
    TEST_ASSERT(ZenohNetworkMessage::put(largeMessage.data(), largeMessage.size(), maxKey,
        maxPayload.data(), maxPayload.size(), maxAttachment.data(), maxAttachment.size()) > 0,
        "maximum payload, attachment and key remain bounded below batch size");
    TEST_ASSERT(!ZenohNetworkMessage::put(largeMessage.data(), largeMessage.size(), oversizedKey,
        maxPayload.data(), maxPayload.size(), nullptr, 0) &&
        !ZenohNetworkMessage::put(largeMessage.data(), largeMessage.size(), "a",
        maxPayload.data(), maxPayload.size() + 1, nullptr, 0) &&
        !ZenohNetworkMessage::put(largeMessage.data(), largeMessage.size(), "a",
        nullptr, 0, maxAttachment.data(), maxAttachment.size() + 1),
        "oversize lengths rejected before reading caller buffers");
    std::printf("Test: in-place session reset clears prior lifecycle state\n");
    Session restarted;
    establish(restarted);
    TEST_ASSERT(sendPayload(restarted, {0x25, 37, 0x1e, 0x1a, 4}, 3) &&
                restarted.frameCount() == 1 && restarted.keepAlivesReceived() == 1,
                "prior connection has receive and discovery state to reset");
    TEST_ASSERT(restarted.sendNetworkMessage(finalWire, sizeof(finalWire), 4) &&
                restarted.consumeOutput(1, 4), "prior connection has partially sent application output");
    const uint8_t partialLength = 9;
    TEST_ASSERT(restarted.receive(&partialLength, 1, 5), "prior connection has an incomplete batch header");
    restarted.linkLost();
    auto newIdentity = identity;
    newIdentity[0] = 99;
    TEST_ASSERT(restarted.start(newIdentity, 6000) && restarted.error() == Session::Error::None &&
                restarted.remoteLeaseMs() == 0 && restarted.remoteInitialSequence() == 0 &&
                restarted.frameCount() == 0 && restarted.discoveryCount() == 0 &&
                restarted.keepAlivesReceived() == 0 && restarted.closeReason() == 0,
                "in-place restart clears counters, peer state and failure diagnostics");
    Session fresh;
    fresh.start(newIdentity, 6000);
    TEST_ASSERT(restarted.outputSize() == fresh.outputSize() &&
                std::memcmp(restarted.outputData(), fresh.outputData(), fresh.outputSize()) == 0,
                "in-place restart emits only a fresh INIT, not stale output");
    restarted.consumeOutput(restarted.outputSize(), 6000);
    TEST_ASSERT(restarted.receive(initAck.data(), initAck.size(), 6001), "restart discards old framing prefix");
    restarted.consumeOutput(restarted.outputSize(), 6001);
    TEST_ASSERT(restarted.receive(openAck.data(), openAck.size(), 6002) &&
                restarted.sendNetworkMessage(finalWire, sizeof(finalWire), 6003) &&
                restarted.outputData()[3] == 0,
                "restart resets transmit sequence to the newly advertised initial value");
    restarted.consumeOutput(restarted.outputSize(), 6003);
    TEST_ASSERT(sendPayload(restarted, {0x25, 37, 0x1e, 0x1a}, 6004), "restart resets receive sequence from peer OPEN");
    TEST_ASSERT(sendPayload(restarted, {0x23, 1}, 6005) && restarted.closeReason() == 1 &&
                restarted.start(identity, 7000) && restarted.closeReason() == 0,
                "restart from remote CLOSE clears its reason");
    std::printf("Zenoh session: %d passed, %d failed\n", passCount, failCount);
    return failCount == 0 ? 0 : 1;
}