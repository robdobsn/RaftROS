// Zenoh serial link tests: z-serial framing vectors, decoder robustness, the
// link handshake against a scripted port, and the session's datagram path.
//
//   zenoh_serial_tests                     unit tests
//   zenoh_serial_tests --serial PATH [S]   hold a real session for S seconds (default 20)
//                                          with a router listening on serial at the other
//                                          end of PATH (a socat pty, or a USB-UART)

#include "Zenoh/ZenohSerialFramer.h"
#include "Zenoh/ZenohSerialLink.h"
#include "Zenoh/ZenohTCPSession.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <poll.h>
#include <random>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>

using namespace RaftRuntime::Zenoh;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

static std::vector<uint8_t> encodeFrame(uint8_t header, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> out(ZenohSerialFrame::MAX_ENCODED);
    const size_t size = ZenohSerialFrame::encode(header, payload.data(), payload.size(), out.data(), out.size());
    out.resize(size);
    return out;
}

static void testVectors()
{
    // From z-serial's own tests
    const std::vector<uint8_t> payload1{0x00, 0x11, 0x00};
    const std::vector<uint8_t> payload2{0x11, 0x22, 0x00, 0x33};
    CHECK(ZenohSerialFrame::crc32(payload1.data(), payload1.size()) == 0xF975EC73u);
    CHECK(ZenohSerialFrame::crc32(payload2.data(), payload2.size()) == 0xFB6D038Du);
    CHECK(encodeFrame(0, payload1) == (std::vector<uint8_t>{0x01, 0x02, 0x03, 0x01, 0x02, 0x11, 0x05, 0x73, 0xEC, 0x75, 0xF9, 0x00}));
    CHECK(encodeFrame(0, payload2) == (std::vector<uint8_t>{0x01, 0x02, 0x04, 0x03, 0x11, 0x22, 0x06, 0x33, 0x8D, 0x03, 0x6D, 0xFB, 0x00}));
    // The empty init frame: header 1, length 0, CRC of nothing = 0
    CHECK(encodeFrame(ZenohSerialFrame::FLAG_INIT, {}) == (std::vector<uint8_t>{0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00}));
    // Too big, too small an output
    std::vector<uint8_t> big(ZenohSerialFrame::MAX_PAYLOAD + 1, 1);
    uint8_t out[16];
    CHECK(ZenohSerialFrame::encode(0, big.data(), big.size(), out, sizeof(out)) == 0);
    CHECK(ZenohSerialFrame::encode(0, payload1.data(), payload1.size(), out, 5) == 0);
}

static void testRoundTripAndFuzz()
{
    std::mt19937 rng(7);
    ZenohSerialDecoder decoder;
    uint32_t good = 0;
    for (size_t size = 0; size <= ZenohSerialFrame::MAX_PAYLOAD; size += (size < 600 ? 1 : 37))
    {
        std::vector<uint8_t> payload(size);
        for (auto& byte : payload)
        {
            const uint32_t pick = rng() % 5;
            byte = pick == 0 ? 0 : pick == 1 ? 0xFF : static_cast<uint8_t>(rng());
        }
        const auto frame = encodeFrame(0, payload);
        CHECK(frame.size() <= ZenohSerialFrame::MAX_ENCODED);
        CHECK(std::count(frame.begin(), frame.end() - 1, 0) == 0);
        int frames = 0;
        for (const auto byte : frame)
            if (decoder.push(byte) == ZenohSerialDecoder::Result::Frame)
            {
                ++frames;
                CHECK(decoder.payloadSize() == size);
                CHECK(std::equal(payload.begin(), payload.end(), decoder.payload()));
            }
        CHECK(frames == 1);
        ++good;
    }
    CHECK(decoder.frames() == good && decoder.errors() == 0);

    // Random corruption and junk: never crashes, never delivers a bad payload,
    // and a clean frame after the noise always decodes
    for (int round = 0; round < 3000; ++round)
    {
        std::vector<uint8_t> payload(rng() % 200 + 1);
        for (auto& byte : payload)
            byte = static_cast<uint8_t>(rng());
        auto frame = encodeFrame(0, payload);
        std::vector<uint8_t> noise(rng() % 2000);
        for (auto& byte : noise)
            byte = static_cast<uint8_t>(rng() % 8 == 0 ? 0 : rng());
        if (rng() % 2)
            frame[rng() % (frame.size() - 1)] ^= static_cast<uint8_t>(1 + rng() % 255);
        for (const auto byte : noise)
            if (decoder.push(byte) == ZenohSerialDecoder::Result::Frame)
                CHECK(decoder.payloadSize() <= ZenohSerialFrame::MAX_PAYLOAD);
        decoder.push(0);    // end whatever the noise left open
        for (const auto byte : frame)
            decoder.push(byte);
        const auto clean = encodeFrame(0, payload);
        bool delivered = false;
        for (const auto byte : clean)
            if (decoder.push(byte) == ZenohSerialDecoder::Result::Frame)
                delivered = decoder.payloadSize() == payload.size() &&
                            std::equal(payload.begin(), payload.end(), decoder.payload());
        CHECK(delivered);
    }
    CHECK(decoder.errors() > 0);

    // An unterminated run longer than any frame is dropped, then resyncs
    ZenohSerialDecoder overflow;
    for (int index = 0; index < 5000; ++index)
        overflow.push(0x41);
    CHECK(overflow.push(0) == ZenohSerialDecoder::Result::Error && overflow.oversize() == 1);
    const auto frame = encodeFrame(0, {1, 2, 3});
    ZenohSerialDecoder::Result last = ZenohSerialDecoder::Result::Incomplete;
    for (const auto byte : frame)
        last = overflow.push(byte);
    CHECK(last == ZenohSerialDecoder::Result::Frame);
}

/// Scripted port: what the device writes is captured; what the "router" sends is queued
struct FakePort
{
    std::deque<uint8_t> rx;
    std::vector<std::vector<uint8_t>> written;
    bool full = false;
    bool idle = true;
    bool txIdle() { return idle; }
    int read(uint8_t* buf, size_t len)
    {
        size_t count = 0;
        while (count < len && !rx.empty())
        {
            buf[count++] = rx.front();
            rx.pop_front();
        }
        return static_cast<int>(count);
    }
    bool write(const uint8_t* buf, size_t len)
    {
        if (full)
            return false;
        written.emplace_back(buf, buf + len);
        return true;
    }
    void routerSends(uint8_t header, const std::vector<uint8_t>& payload)
    {
        const auto frame = encodeFrame(header, payload);
        rx.insert(rx.end(), frame.begin(), frame.end());
    }
};

static uint8_t headerOf(const std::vector<uint8_t>& frame)
{
    ZenohSerialDecoder decoder;
    for (const auto byte : frame)
        if (decoder.push(byte) == ZenohSerialDecoder::Result::Frame)
            return decoder.header();
    return 0xEE;
}

static void testLinkHandshake()
{
    FakePort port;
    ZenohSerialLink<FakePort> link(port);
    auto noFrames = [](const uint8_t*, size_t) { return true; };

    link.start(0);
    CHECK(port.written.size() == 1 && headerOf(port.written[0]) == ZenohSerialFrame::FLAG_INIT);
    CHECK(link.service(100, 4096, noFrames) && !link.up());
    CHECK(port.written.size() == 1);
    CHECK(link.service(260, 4096, noFrames));
    CHECK(port.written.size() == 2);                   // I retried after INIT_RETRY_MS

    // No new I while the last has not left: no burst of I frames for a router to find
    port.idle = false;
    CHECK(link.service(520, 4096, noFrames) && port.written.size() == 2 && link.initsHeld() == 1);
    port.idle = true;
    CHECK(link.service(521, 4096, noFrames) && port.written.size() == 3);

    // R while initializing: keep retrying; stale data before I|A is ignored
    port.routerSends(ZenohSerialFrame::FLAG_RESET, {});
    port.routerSends(0, {0x04});
    CHECK(link.service(300, 4096, noFrames) && !link.up() && link.resetsReceived() == 1);
    port.rx.insert(port.rx.end(), {'b', 'o', 'o', 't', '\n', 0});   // a boot banner
    port.routerSends(ZenohSerialFrame::FLAG_INIT | ZenohSerialFrame::FLAG_ACK, {});
    CHECK(link.service(310, 4096, noFrames) && link.up());

    // Data both ways; a duplicate I|A is ignored
    std::vector<std::vector<uint8_t>> received;
    auto collect = [&](const uint8_t* data, size_t size) { received.emplace_back(data, data + size); return true; };
    port.routerSends(ZenohSerialFrame::FLAG_INIT | ZenohSerialFrame::FLAG_ACK, {});
    port.routerSends(0, {0x04});
    port.routerSends(0, {0x25, 0x01, 0x02});
    CHECK(link.service(320, 4096, collect) && received.size() == 2 && received[1].size() == 3);
    const uint8_t batch[] = {0x04};
    CHECK(link.sendBatch(batch, 1) && headerOf(port.written.back()) == 0);
    port.full = true;
    CHECK(!link.sendBatch(batch, 1) && link.txFull() == 1);
    port.full = false;

    // Bounded read per pass
    for (int index = 0; index < 50; ++index)
        port.routerSends(0, std::vector<uint8_t>(40, 0x04));
    received.clear();
    CHECK(link.service(330, 256, collect) && received.size() < 50);

    // R once up: the router dropped us
    port.rx.clear();
    port.rx.push_back(0);       // the clear cut a frame short; end it
    port.routerSends(ZenohSerialFrame::FLAG_RESET, {});
    CHECK(!link.service(340, 4096, collect) && link.lossReason() == ZenohSerialLink<FakePort>::LossReason::Reset);
    link.start(400);
    CHECK(link.state() == ZenohSerialLink<FakePort>::State::Initializing);
}

static void testTextLines()
{
    // A command typed at a terminal reaches the handler whether the link is
    // waiting for the router or up; frames around it still decode
    FakePort port;
    ZenohSerialLink<FakePort> link(port);
    std::vector<std::string> lines;
    link.setTextLineHandler([](void* context, const char* line) {
        static_cast<std::vector<std::string>*>(context)->push_back(line);
        return strncmp(line, "ros/", 4) == 0;      // took it: not part of a frame
    }, &lines);
    auto noFrames = [](const uint8_t*, size_t) { return true; };
    link.start(0);
    const char typed[] = "ros/set?link=tcp\r\n";
    port.rx.insert(port.rx.end(), typed, typed + sizeof(typed) - 1);
    CHECK(link.service(10, 4096, noFrames));
    CHECK(lines.size() == 1 && lines[0] == "ros/set?link=tcp");
    port.routerSends(ZenohSerialFrame::FLAG_INIT | ZenohSerialFrame::FLAG_ACK, {});
    CHECK(link.service(20, 4096, noFrames) && link.up());
    int frames = 0;
    auto countFrames = [&](const uint8_t*, size_t) { ++frames; return true; };
    port.routerSends(0, {0x04});
    const char typed2[] = "ros/clear\r";
    port.rx.insert(port.rx.end(), typed2, typed2 + sizeof(typed2) - 1);
    port.routerSends(0, {0x04});
    CHECK(link.service(30, 4096, countFrames) && frames == 2);
    CHECK(lines.size() == 2 && lines[1] == "ros/clear");
    // Binary noise and over-long text are not lines
    const uint8_t noise[] = {0x01, 'r', 'o', 0x80, 's', '\n'};
    port.rx.insert(port.rx.end(), noise, noise + sizeof(noise));
    std::string longLine(200, 'x');
    longLine += "\n";
    port.rx.insert(port.rx.end(), longLine.begin(), longLine.end());
    CHECK(link.service(40, 4096, noFrames));
    CHECK(lines.size() == 3 && lines[2] == "s");   // only the text after the last non-text byte
}

static void testSessionDatagram()
{
    ZenohTCPSession session;
    std::array<uint8_t, 16> identity{};
    identity[0] = 0x42;
    CHECK(!session.start(identity, 0, 32));
    CHECK(session.start(identity, 0, ZenohSerialFrame::MAX_PAYLOAD));
    size_t length = 0;
    const uint8_t* init = session.pendingBatch(length);
    CHECK(init && length > 3 && init[0] == 0x41);      // INIT without the TCP prefix
    CHECK(init[length - 2] == 0xdc && init[length - 1] == 0x05);   // offers 1500
    CHECK(session.consumeOutput(session.outputSize(), 0));
    CHECK(session.pendingBatch(length) == nullptr);
    // InitAck from a router clamping to the serial MTU: batch 1500
    const uint8_t initAck[] = {0x61, 0x09, 0x01, 0x77, 0x0a, 0xdc, 0x05, 0x04, 0x00, 0x81, 0xff, 0x42};
    CHECK(session.receiveBatch(initAck, sizeof(initAck), 1));
    CHECK(session.state() == ZenohTCPSession::State::AwaitOpenAck && session.negotiatedBatch() == 1500);
    CHECK(session.pendingBatch(length) && length > 0);
    session.consumeOutput(session.outputSize(), 1);
    const uint8_t openAck[] = {0x62, 0x04, 0x11};
    CHECK(session.receiveBatch(openAck, sizeof(openAck), 2));
    CHECK(session.state() == ZenohTCPSession::State::Established);
    const uint8_t keepAlive[] = {0x04};
    CHECK(session.receiveBatch(keepAlive, 1, 3) && session.keepAlivesReceived() == 1);
    std::vector<uint8_t> tooBig(1501, 0x04);
    CHECK(!session.receiveBatch(tooBig.data(), tooBig.size(), 4));
}

// ---------------------------------------------------------------------------
// --serial: a real session over a tty
// ---------------------------------------------------------------------------

struct TtyPort
{
    int fd = -1;
    int read(uint8_t* buf, size_t len)
    {
        const ssize_t count = ::read(fd, buf, len);
        if (count < 0)
            return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
        return static_cast<int>(count);
    }
    bool txIdle()
    {
        int queued = 0;
        return ioctl(fd, TIOCOUTQ, &queued) != 0 || queued == 0;
    }
    bool write(const uint8_t* buf, size_t len)
    {
        size_t done = 0;
        while (done < len)
        {
            const ssize_t count = ::write(fd, buf + done, len - done);
            if (count < 0 && (errno == EAGAIN || errno == EINTR))
            {
                pollfd wait{fd, POLLOUT, 0};
                poll(&wait, 1, 50);
                continue;
            }
            if (count <= 0)
                return false;
            done += static_cast<size_t>(count);
        }
        return true;
    }
};

static uint64_t nowMs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

static int runSerial(const char* path, unsigned seconds)
{
    TtyPort port;
    port.fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (port.fd < 0)
    {
        std::perror(path);
        return 1;
    }
    termios tio{};
    if (tcgetattr(port.fd, &tio) == 0)
    {
        cfmakeraw(&tio);
        cfsetspeed(&tio, B921600);
        tcsetattr(port.fd, TCSANOW, &tio);
    }
    ZenohSerialLink<TtyPort> link(port);
    ZenohTCPSession session;
    std::array<uint8_t, 16> identity{};
    for (auto& byte : identity)
        byte = static_cast<uint8_t>(std::rand());
    identity[0] |= 1;
    const uint64_t started = nowMs();
    link.start(static_cast<uint32_t>(started));
    bool sessionStarted = false, established = false;
    while (nowMs() - started < seconds * 1000ull)
    {
        const uint64_t now = nowMs();
        const bool ok = link.service(static_cast<uint32_t>(now), 4096, [&](const uint8_t* data, size_t size) {
            return session.receiveBatch(data, size, now);
        });
        if (!ok)
        {
            std::printf("link lost reason=%u session error=%u\n", (unsigned)link.lossReason(), (unsigned)session.error());
            return 1;
        }
        if (link.up() && !sessionStarted)
        {
            std::printf("link up after %u I frames\n", link.initsSent());
            sessionStarted = session.start(identity, now, ZenohSerialFrame::MAX_PAYLOAD);
        }
        session.service(now);
        if (session.state() == ZenohTCPSession::State::Failed || session.state() == ZenohTCPSession::State::Closed)
        {
            std::printf("session ended state=%u error=%u\n", (unsigned)session.state(), (unsigned)session.error());
            return 1;
        }
        if (!established && session.state() == ZenohTCPSession::State::Established)
        {
            established = true;
            std::printf("session established batch=%u lease=%llums\n", session.negotiatedBatch(),
                        (unsigned long long)session.remoteLeaseMs());
        }
        size_t length = 0;
        if (const uint8_t* batch = session.pendingBatch(length))
            if (link.sendBatch(batch, length))
                session.consumeOutput(session.outputSize(), now);
        pollfd wait{port.fd, POLLIN, 0};
        poll(&wait, 1, 10);
    }
    std::printf("%s: held %us, keepalives rx %u, frames in %u out %u, decode errors %u\n",
                established ? "PASS" : "FAIL", seconds, session.keepAlivesReceived(), link.framesIn(),
                link.framesOut(), link.decoder().errors());
    return established ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc >= 3 && std::strcmp(argv[1], "--serial") == 0)
        return runSerial(argv[2], argc >= 4 ? static_cast<unsigned>(std::atoi(argv[3])) : 20);
    testVectors();
    testRoundTripAndFuzz();
    testLinkHandshake();
    testTextLines();
    testSessionDatagram();
    if (failures)
    {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("zenoh serial tests passed\n");
    return 0;
}
