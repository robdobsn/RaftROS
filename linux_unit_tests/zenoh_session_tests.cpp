#include <array>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include "Zenoh/ZenohStreamFramer.h"
#include "Zenoh/ZenohTCPSession.h"
#include "Zenoh/ZenohNetworkMessage.h"
#include <cstring>

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
    std::printf("Test: subscriber declarations and inbound samples\n");
    {
        using Message = RaftRuntime::Zenoh::ZenohNetworkMessage;
        uint8_t buffer[512];

        // Declaring a subscriber is the same shape as declaring a token, with
        // body id 2; readDiscovery is the parser a peer would use on it
        const size_t declared = Message::declareSubscriber(buffer, sizeof(buffer), 5, "0/raft/range_1_29/**");
        Message::DiscoveryMessage parsed;
        TEST_ASSERT(declared > 0 && Message::readDiscovery(buffer, declared, parsed) == declared,
                    "subscriber declaration parses back");
        TEST_ASSERT(parsed.declaration == 2 && parsed.id == 5 && parsed.key == "0/raft/range_1_29/**",
                    "subscriber declaration carries body id 2, the subscriber id and the full key");
        TEST_ASSERT(!parsed.hasInterestId, "an unsolicited declaration carries no interest id");

        const uint32_t interestId = 9;
        const size_t replied = Message::declareSubscriber(buffer, sizeof(buffer), 5, "0/raft/x/**", &interestId);
        TEST_ASSERT(replied > 0 && Message::readDiscovery(buffer, replied, parsed) == replied &&
                    parsed.hasInterestId && parsed.interestId == 9,
                    "a subscriber declared in reply to an interest carries its id");

        TEST_ASSERT(Message::declareSubscriber(buffer, sizeof(buffer), 5, "") == 0 &&
                    Message::declareSubscriber(buffer, sizeof(buffer), 5, "trailing/") == 0,
                    "an unusable key is refused rather than sent");
        TEST_ASSERT(Message::declareSubscriber(buffer, 3, 5, "0/raft/x") == 0,
                    "a declaration that would not fit is refused");

        const size_t withdrawn = Message::undeclareSubscriber(buffer, sizeof(buffer), 5);
        TEST_ASSERT(withdrawn > 0 && Message::readDiscovery(buffer, withdrawn, parsed) == withdrawn &&
                    parsed.declaration == 3 && parsed.id == 5,
                    "withdrawing a subscriber names body id 3 and the subscriber id");

        // Inbound samples: what we send is what a peer sends us, so a put()
        // round-trips through readSample
        const uint8_t payload[] = {0x00, 0x01, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00};
        const uint8_t attachment[] = {1, 2, 3, 4};
        const size_t sent = Message::put(buffer, sizeof(buffer), "0/raft/range_1_29/type/hash",
                                         payload, sizeof(payload), attachment, sizeof(attachment));
        Message::SampleMessage sample;
        TEST_ASSERT(sent > 0 && Message::readSample(buffer, sent, sample) == sent,
                    "a sample parses back");
        TEST_ASSERT(sample.key == "0/raft/range_1_29/type/hash" && sample.keyId == 0,
                    "a sample sent with a full key reports no key id");
        TEST_ASSERT(sample.payload.size() == sizeof(payload) &&
                    std::memcmp(sample.payload.data(), payload, sizeof(payload)) == 0,
                    "sample payload is borrowed intact");
        TEST_ASSERT(sample.attachment.size() == sizeof(attachment) &&
                    std::memcmp(sample.attachment.data(), attachment, sizeof(attachment)) == 0,
                    "sample attachment is borrowed intact");

        uint8_t bareBuffer[128];
        const size_t bare = Message::put(bareBuffer, sizeof(bareBuffer), "0/raft/x", payload, sizeof(payload), nullptr, 0);
        TEST_ASSERT(bare > 0 && Message::readSample(bareBuffer, bare, sample) == bare && sample.attachment.empty(),
                    "a sample without an attachment parses with an empty one");

        bool everyTruncationRejected = true;
        for (size_t truncated = 1; truncated < sent; ++truncated)
            everyTruncationRejected &= Message::readSample(buffer, truncated, sample) == 0;
        TEST_ASSERT(everyTruncationRejected, "every truncation of a sample is rejected");
        const uint8_t notASample[] = {0x1e, 0x07, 0x01};
        TEST_ASSERT(Message::readSample(notASample, sizeof(notASample), sample) == 0,
                    "a declaration is not read as a sample");
        // Key expression id 0 with no literal key identifies nothing
        const uint8_t noKey[] = {0x1d, 0x00, 0x01, 0x01, 0x00};
        TEST_ASSERT(Message::readSample(noKey, sizeof(noKey), sample) == 0,
                    "a sample that names neither a key nor a declared id is refused");
    }

    std::printf("Test: a clock that goes backwards does not expire a live session\n");
    {
        Session session;
        establish(session);
        session.consumeOutput(session.outputSize(), 5000);
        // Received at 5000, then serviced with an earlier time: subtracting
        // would wrap to an enormous age and drop a session that is perfectly
        // healthy.  This cost a device its router connection on every publish
        // that ran before the clock was refreshed.
        session.service(0);
        TEST_ASSERT(session.state() == Session::State::Established,
                    "servicing with a time before the last receive keeps the session");
        const uint8_t message[] = {0x1e, 0x1a};
        TEST_ASSERT(session.sendNetworkMessage(message, sizeof(message), 0) &&
                    session.state() == Session::State::Established,
                    "sending with a stale clock still works");
        session.consumeOutput(session.outputSize(), 5000);
        session.service(5000 + 20000);
        TEST_ASSERT(session.state() == Session::State::Failed &&
                    session.error() == Session::Error::LeaseExpired,
                    "a genuinely expired lease still fails the session");
    }

    std::printf("Test: a sample captured from a real Zenoh router\n");
    {
        // Recorded off the wire from zenoh 1.8.0 publishing std_msgs/String to
        // a subscribing device.  A real Put carries extensions this parser does
        // not use (a timestamp among them), encoded three different ways -
        // reading them wrongly cost a session, so the exact bytes are the test.
        const uint8_t captured[] = {
        0x7d, 0x00, 0x71, 0x30, 0x2f, 0x63, 0x68, 0x61, 0x74, 0x74, 0x65, 0x72,
        0x5f, 0x69, 0x6e, 0x2f, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67, 0x73,
        0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a,
        0x3a, 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x5f, 0x2f, 0x52, 0x49, 0x48,
        0x53, 0x30, 0x31, 0x5f, 0x64, 0x66, 0x36, 0x36, 0x38, 0x63, 0x37, 0x34,
        0x30, 0x34, 0x38, 0x32, 0x62, 0x62, 0x64, 0x34, 0x38, 0x66, 0x62, 0x33,
        0x39, 0x64, 0x37, 0x36, 0x61, 0x37, 0x30, 0x64, 0x66, 0x64, 0x34, 0x62,
        0x64, 0x35, 0x39, 0x64, 0x62, 0x31, 0x32, 0x38, 0x38, 0x30, 0x32, 0x31,
        0x37, 0x34, 0x33, 0x35, 0x30, 0x33, 0x32, 0x35, 0x39, 0x65, 0x39, 0x34,
        0x38, 0x66, 0x36, 0x62, 0x31, 0x61, 0x31, 0x38, 0xa1, 0xb0, 0x97, 0x86,
        0xd4, 0xf4, 0xcc, 0xa6, 0xdb, 0x6a, 0x10, 0xa2, 0xe3, 0x62, 0x94, 0xfd,
        0x14, 0x31, 0xa2, 0x1f, 0x48, 0x34, 0x5f, 0x56, 0x0b, 0x61, 0xd9, 0x43,
        0x21, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c, 0x23,
        0xc6, 0xa2, 0x9b, 0xd8, 0x18, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x16, 0x00,
        0x01, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x68, 0x65, 0x6c, 0x6c, 0x6f,
        0x20, 0x7a, 0x65, 0x6e, 0x6f, 0x68, 0x20, 0x30, 0x00
        };
        RaftRuntime::Zenoh::ZenohNetworkMessage::SampleMessage sample;
        const size_t consumed = RaftRuntime::Zenoh::ZenohNetworkMessage::readSample(
            captured, sizeof(captured), sample);
        TEST_ASSERT(consumed == sizeof(captured), "a real router's sample parses completely");
        TEST_ASSERT(sample.key ==
            "0/chatter_in/std_msgs::msg::dds_::String_/"
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            "the sample's key survives the extensions around it");
        TEST_ASSERT(sample.attachment.size() == 33 &&
                    static_cast<uint8_t>(sample.attachment[0]) == 1 &&
                    static_cast<uint8_t>(sample.attachment[16]) == 16,
                    "the attachment is found among extensions of other encodings");
        // std_msgs/String CDR: encapsulation header, length including the NUL,
        // then the text.  Checked here rather than through the decoder so this
        // suite keeps its single dependency on the Zenoh headers.
        const char expectedPayload[] = "\x00\x01\x00\x00\x0e\x00\x00\x00hello zenoh 0";
        TEST_ASSERT(sample.payload.size() == 22 &&
                    std::memcmp(sample.payload.data(), expectedPayload, 21) == 0 &&
                    sample.payload.back() == '\0',
                    "the payload is the std_msgs/String that was published");
    }

    std::printf("Test: samples reach the session's handler\n");
    {
        uint8_t buffer[256];
        const uint8_t payload[] = {0x00, 0x01, 0x00, 0x00, 0x07};
        const size_t putLen = RaftRuntime::Zenoh::ZenohNetworkMessage::put(
            buffer, sizeof(buffer), "0/raft/range_1_29/type/hash", payload, sizeof(payload), nullptr, 0);
        std::vector<uint8_t> frame{0x25, 37};
        frame.insert(frame.end(), buffer, buffer + putLen);

        struct Capture { int count = 0; std::string key; bool accept = true; } capture;
        Session receiver;
        establish(receiver);
        receiver.consumeOutput(receiver.outputSize(), 3);
        std::vector<uint8_t> wire{static_cast<uint8_t>(frame.size()), static_cast<uint8_t>(frame.size() >> 8)};
        wire.insert(wire.end(), frame.begin(), frame.end());
        TEST_ASSERT(receiver.receive(wire.data(), wire.size(), 3, nullptr, &capture,
                    [](void* context, const RaftRuntime::Zenoh::ZenohNetworkMessage::SampleMessage& sample) {
                        auto* state = static_cast<Capture*>(context);
                        ++state->count;
                        state->key = std::string(sample.key);
                        return state->accept;
                    }),
                    "a frame carrying a sample is accepted");
        TEST_ASSERT(capture.count == 1 && capture.key == "0/raft/range_1_29/type/hash" &&
                    receiver.sampleCount() == 1, "the sample reaches the handler with its key");

        // A router may forward a sample we have no handler for; skipping it is
        // not the same as the batch being malformed
        Session ignoring;
        establish(ignoring);
        ignoring.consumeOutput(ignoring.outputSize(), 3);
        TEST_ASSERT(ignoring.receive(wire.data(), wire.size(), 3) &&
                    ignoring.state() == Session::State::Established && ignoring.sampleCount() == 1,
                    "a sample with no handler installed is skipped, not treated as malformed");

        Session refusing;
        establish(refusing);
        refusing.consumeOutput(refusing.outputSize(), 3);
        capture.accept = false;
        capture.count = 0;
        TEST_ASSERT(!refusing.receive(wire.data(), wire.size(), 3, nullptr, &capture,
                    [](void* context, const RaftRuntime::Zenoh::ZenohNetworkMessage::SampleMessage&) {
                        ++static_cast<Capture*>(context)->count;
                        return false;
                    }) && refusing.state() == Session::State::Failed,
                    "a handler that refuses a sample fails the session");
    }

    std::printf("Test: service declarations round-trip\n");
    {
        using Message = RaftRuntime::Zenoh::ZenohNetworkMessage;
        uint8_t buffer[512];
        Message::DiscoveryMessage parsed;
        const size_t ke = Message::declareKeyExpr(buffer, sizeof(buffer), 30, "0/raft_test/trigger/std_srvs::srv::dds_::Trigger_/RIHS01_x");
        TEST_ASSERT(ke > 0 && Message::readDiscovery(buffer, ke, parsed) == ke && parsed.declaration == 0 &&
                    parsed.id == 30 && parsed.key == "0/raft_test/trigger/std_srvs::srv::dds_::Trigger_/RIHS01_x",
                    "a key-expression declaration carries body id 0, the id and the full key");
        TEST_ASSERT(Message::declareKeyExpr(buffer, sizeof(buffer), 0, "0/raft/x") == 0,
                    "key-expression id 0 is reserved and refused");
        const size_t q = Message::declareQueryable(buffer, sizeof(buffer), 27, 30);
        TEST_ASSERT(q == 6 && buffer[1] == 0xc4 && buffer[2] == 27 && buffer[3] == 30 && buffer[4] == 0x21 && buffer[5] == 0x01,
                    "a queryable declaration is byte for byte what rmw_zenoh sends: c4 <id> <keyexpr> 21 01");
        TEST_ASSERT(Message::readDiscovery(buffer, q, parsed) == q && parsed.declaration == 4 && parsed.id == 27,
                    "a queryable declaration parses as body id 4");
        const size_t uq = Message::undeclareQueryable(buffer, sizeof(buffer), 27);
        TEST_ASSERT(uq > 0 && Message::readDiscovery(buffer, uq, parsed) == uq && parsed.declaration == 5 && parsed.id == 27,
                    "withdrawing a queryable names body id 5");
        const size_t uk = Message::undeclareKeyExpr(buffer, sizeof(buffer), 30);
        TEST_ASSERT(uk > 0 && Message::readDiscovery(buffer, uk, parsed) == uk && parsed.declaration == 1 && parsed.id == 30,
                    "withdrawing a key expression names body id 1");
    }

    std::printf("Test: requests captured from a real router parse completely\n");
    {
        using Message = RaftRuntime::Zenoh::ZenohNetworkMessage;
        const auto loadFixture = [](const char* path) {
            std::vector<std::vector<uint8_t>> messages;
            std::ifstream in(path);
            std::string line;
            while (std::getline(in, line))
            {
                if (line.empty() || line[0] == '#') continue;
                std::vector<uint8_t> bytes;
                for (size_t i = 0; i + 1 < line.size(); i += 2)
                    bytes.push_back(static_cast<uint8_t>(std::stoul(line.substr(i, 2), nullptr, 16)));
                messages.push_back(bytes);
            }
            return messages;
        };
        const auto trigger = loadFixture("fixtures/zenoh_service_trigger_request.hex");
        const auto setbool = loadFixture("fixtures/zenoh_service_setbool_request.hex");
        TEST_ASSERT(trigger.size() == 1 && setbool.size() == 1, "request fixtures load (run from linux_unit_tests/)");
        if (trigger.size() == 1 && setbool.size() == 1)
        {
            Message::RequestMessage request;
            const auto& t = trigger[0];
            TEST_ASSERT(Message::readRequest(t.data(), t.size(), request) == t.size(),
                        "the Trigger request the router sent a server parses completely");
            TEST_ASSERT(request.requestId == 1 && request.keyId == 30 && request.key.empty(),
                        "the router addresses the request by our declared key-expression id, with no suffix");
            TEST_ASSERT(request.timeoutMs == 600000, "the client's timeout (600 s from the CLI) is read");
            const uint8_t triggerCdr[] = {0x00, 0x01, 0x00, 0x00, 0x00};
            TEST_ASSERT(request.payload.size() == sizeof(triggerCdr) &&
                        std::memcmp(request.payload.data(), triggerCdr, sizeof(triggerCdr)) == 0,
                        "an empty ROS request is a CDR header plus the one dummy byte");
            TEST_ASSERT(request.attachment.size() == 33 && static_cast<uint8_t>(request.attachment[0]) == 1 &&
                        static_cast<uint8_t>(request.attachment[16]) == 16,
                        "the client's attachment (sequence 1, 16-byte GID) is found under extension id 5");
            const auto& b = setbool[0];
            TEST_ASSERT(Message::readRequest(b.data(), b.size(), request) == b.size() && request.requestId == 2 &&
                        request.keyId == 32 && request.payload.size() == 5 &&
                        static_cast<uint8_t>(request.payload[4]) == 1,
                        "the SetBool{data:true} request carries request id 2, key id 32 and a payload ending in 01");
            bool everyTruncationRejected = true;
            for (size_t n = 1; n < t.size(); ++n)
                everyTruncationRejected &= Message::readRequest(t.data(), n, request) == 0;
            TEST_ASSERT(everyTruncationRejected, "every truncation of a request is rejected");
        }
    }

    std::printf("Test: our reply has the shape of a real server's\n");
    {
        using Message = RaftRuntime::Zenoh::ZenohNetworkMessage;
        // A small reader for what a router would parse out of our RESPONSE
        struct Reply { uint32_t rid = 0; std::string key; std::string payload; std::string attachment; bool ok = false; };
        const auto readReply = [](const uint8_t* b, size_t n) {
            Reply r;
            size_t o = 0;
            auto varint = [&](uint32_t& v) {
                v = 0;
                for (int i = 0; i < 5 && o < n; ++i)
                {
                    const uint8_t x = b[o++];
                    v |= (x & 0x7f) << (7 * i);
                    if (x < 128)
                        return true;
                }
                return false;
            };
            auto blob = [&](std::string& out) {
                uint32_t v = 0;
                if (!varint(v) || o + v > n)
                    return false;
                out.assign(reinterpret_cast<const char*>(b + o), v);
                o += v;
                return true;
            };
            uint32_t v = 0;
            if (n < 2 || (b[o++] & 0x1f) != 0x1b)
                return r;
            if (!varint(v))
                return r;
            r.rid = v;
            if (!varint(v))                                   // scope
                return r;
            if (!blob(r.key))
                return r;
            if (o >= n || b[o++] != 0x04)                     // REPLY
                return r;
            if (o >= n)
                return r;
            const uint8_t put = b[o++];
            if (put & 0x80)
            {
                if (o >= n || b[o++] != 0x43 || !blob(r.attachment))
                    return r;
            }
            if (!blob(r.payload))
                return r;
            r.ok = (o == n);
            return r;
        };
        uint8_t buffer[512];
        const uint8_t cdr[] = {0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 'o', 'k', 0x00};
        uint8_t attachment[33] = {0}; attachment[0] = 7; attachment[16] = 16;
        const size_t n = Message::writeReply(buffer, sizeof(buffer), 9, "0/raft/svc/std_srvs::srv::dds_::Trigger_/RIHS01_x",
                                             cdr, sizeof(cdr), attachment, sizeof(attachment));
        const Reply reply = readReply(buffer, n);
        TEST_ASSERT(n > 0 && reply.ok && reply.rid == 9 && reply.key == "0/raft/svc/std_srvs::srv::dds_::Trigger_/RIHS01_x",
                    "a reply names the request id and the service key in full");
        TEST_ASSERT(reply.payload.size() == sizeof(cdr) && reply.attachment.size() == 33 &&
                    static_cast<uint8_t>(reply.attachment[0]) == 7,
                    "a reply carries the CDR response and the attachment that echoes the request's sequence");
        TEST_ASSERT(buffer[0] == 0x7b, "a reply is RESPONSE with the key named in full, as a real server sends it");
        const size_t f = Message::writeResponseFinal(buffer, sizeof(buffer), 9);
        TEST_ASSERT(f == 2 && buffer[0] == 0x1a && buffer[1] == 9, "the final is RESPONSE_FINAL with the request id");
        const size_t e = Message::writeReplyError(buffer, sizeof(buffer), 9, "0/raft/svc/t/h", "no such service");
        TEST_ASSERT(e > 0 && buffer[0] == 0x7b, "a refusal is a RESPONSE carrying ERR");
        {
            // After header, rid, keyexpr id 0 and the key comes the ERR body:
            // 0x45 = ERR | E (encoding present), encoding 0, then the reason as
            // a length-prefixed payload.  A router reads an ERR without E as an
            // empty payload followed by garbage and closes the session.
            const size_t keyLen = std::strlen("0/raft/svc/t/h");
            const size_t bodyAt = 1 + 1 + 1 + 1 + keyLen;
            TEST_ASSERT(buffer[bodyAt] == 0x45 && buffer[bodyAt + 1] == 0 &&
                        buffer[bodyAt + 2] == std::strlen("no such service") &&
                        std::memcmp(buffer + bodyAt + 3, "no such service", std::strlen("no such service")) == 0 &&
                        e == bodyAt + 3 + std::strlen("no such service"),
                        "ERR carries its encoding flag and the reason as the payload");
        }
        TEST_ASSERT(Message::writeReply(buffer, sizeof(buffer), 9, "", cdr, sizeof(cdr), nullptr, 0) == 0,
                    "a reply with no key is refused");
    }

    std::printf("Test: oversized payloads are stepped over, not fatal\n");
    {
        using Message = RaftRuntime::Zenoh::ZenohNetworkMessage;
        // A REQUEST whose QUERY body extension carries a 3000-byte payload:
        // 0x1c rid=9 keyId=300 | QUERY 0x83 (Z) | ext 0x43 (ZBuf id 3) len | encoding 0 | bytes
        std::vector<uint8_t> msg = {0x1c, 9, 0xac, 0x02, 0x83, 0x43};
        const uint32_t bodyLen = 1 + 3000;
        msg.push_back((uint8_t)(bodyLen & 0x7f) | 0x80); msg.push_back((uint8_t)(bodyLen >> 7));
        msg.push_back(0);
        msg.insert(msg.end(), 3000, 0x5a);
        Message::RequestMessage request;
        const size_t consumed = Message::readRequest(msg.data(), msg.size(), request);
        TEST_ASSERT(consumed == msg.size() && request.oversized && request.payload.empty() &&
                    request.requestId == 9 && request.keyId == 300,
                    "an oversized request is consumed whole and flagged, with no payload");
        std::vector<uint8_t> shortMsg(msg.begin(), msg.end() - 10);
        TEST_ASSERT(Message::readRequest(shortMsg.data(), shortMsg.size(), request) == 0,
                    "a payload longer than the batch is still a malformed message");

        // A PUT whose payload is 3000 bytes: 0x1d keyId=7 | PUT 0x01 | len | bytes
        std::vector<uint8_t> put = {0x1d, 7, 0x01};
        put.push_back((uint8_t)(3000 & 0x7f) | 0x80); put.push_back((uint8_t)(3000 >> 7));
        put.insert(put.end(), 3000, 0x5a);
        Message::SampleMessage sample;
        const size_t putConsumed = Message::readSample(put.data(), put.size(), sample);
        TEST_ASSERT(putConsumed == put.size() && sample.oversized && sample.payload.empty() && sample.keyId == 7,
                    "an oversized sample is consumed whole and flagged");
    }

    std::printf("Test: requests reach the session's handler\n");
    {
        using Message = RaftRuntime::Zenoh::ZenohNetworkMessage;
        std::ifstream in("fixtures/zenoh_service_trigger_request.hex");
        std::string line, hex;
        while (std::getline(in, line)) if (!line.empty() && line[0] != '#') hex = line;
        std::vector<uint8_t> request;
        for (size_t i = 0; i + 1 < hex.size(); i += 2)
            request.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
        std::vector<uint8_t> frame{0x25, 37};
        frame.insert(frame.end(), request.begin(), request.end());
        std::vector<uint8_t> wire{static_cast<uint8_t>(frame.size()), static_cast<uint8_t>(frame.size() >> 8)};
        wire.insert(wire.end(), frame.begin(), frame.end());

        struct Capture { int count = 0; uint32_t keyId = 0; size_t payload = 0; } capture;
        Session receiver;
        establish(receiver);
        receiver.consumeOutput(receiver.outputSize(), 3);
        TEST_ASSERT(receiver.receive(wire.data(), wire.size(), 3, nullptr, &capture, nullptr,
                    [](void* context, const Message::RequestMessage& r) {
                        auto* c = static_cast<Capture*>(context); ++c->count; c->keyId = r.keyId; c->payload = r.payload.size(); return true;
                    }),
                    "a frame carrying a request is accepted");
        TEST_ASSERT(capture.count == 1 && capture.keyId == 30 && capture.payload == 5 && receiver.requestCount() == 1,
                    "the request reaches the handler with its key id and payload");
        Session ignoring;
        establish(ignoring);
        ignoring.consumeOutput(ignoring.outputSize(), 3);
        TEST_ASSERT(ignoring.receive(wire.data(), wire.size(), 3) && ignoring.state() == Session::State::Established &&
                    ignoring.requestCount() == 1,
                    "a request with no handler installed is skipped, not treated as malformed");
    }

    std::printf("Zenoh session: %d passed, %d failed\n", passCount, failCount);
    return failCount == 0 ? 0 : 1;
}