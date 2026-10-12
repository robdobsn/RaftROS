#pragma once

// Zenoh serial link: the z-serial handshake and frame I/O over a byte port.
//
// The device is the connecting side.  It sends an empty frame with I set
// until the router answers I|A; data frames (header 0) then carry one Zenoh
// batch each, in both directions.  An R from the router means "I was already
// initialized - start again", which is also how a router tells us it has
// dropped the link.  The Zenoh session (INIT/OPEN, lease, declarations) runs
// above this unchanged.
//
// The router's z-serial treats any frame it cannot decode on an initialized
// link as a fatal link error, so nothing but frames may be written to the
// port while the link is up.
//
// Port is anything with:
//   int  read(uint8_t* buf, size_t len);         // never blocks; bytes read, <0 on error
//   bool write(const uint8_t* buf, size_t len);  // never blocks; all or nothing
//   bool txIdle();                               // everything written has left the device
// so the link is host-testable and the same code drives USB-Serial-JTAG or a UART.

#include "ZenohSerialFramer.h"

namespace RaftRuntime::Zenoh
{

template<typename Port>
class ZenohSerialLink
{
public:
    static constexpr uint32_t INIT_RETRY_MS = 250;
    static constexpr size_t READ_CHUNK = 256;

    enum class State : uint8_t { Down, Initializing, Up, Lost };

    explicit ZenohSerialLink(Port& port) : _port(port) {}

    /// @brief Called with each line of plain text that arrives (printable
    /// characters ended by CR or LF) - how a person at a terminal reaches a
    /// device whose port is carrying the link.  Frame bytes can form such a
    /// "line" by chance, so the handler must act only on exact commands, and
    /// return true only for one it took: the text was then not part of a frame
    /// and the decoder starts afresh, so the next frame is not lost with it.
    using TextLineFn = bool (*)(void* context, const char* line);
    void setTextLineHandler(TextLineFn handler, void* context)
    {
        _onTextLine = handler;
        _textLineContext = context;
    }

    /// @brief Start the handshake.  Bytes already waiting are stale (an old
    /// session, a boot banner) and are dropped by the decoder's resync.
    void start(uint32_t nowMs)
    {
        _decoder.reset();
        _state = State::Initializing;
        _lossReason = LossReason::None;
        _lastInitMs = nowMs - INIT_RETRY_MS;
        sendInit(nowMs);
    }

    void stop() { _state = State::Down; _decoder.reset(); }

    /// @brief Read what has arrived, up to maxBytes, and hand each data frame to
    /// onFrame(payload, length).  Runs the handshake while initializing.
    /// @return false when the link has been lost (R from the router, port error,
    /// or onFrame returned false)
    template<typename OnFrame>
    bool service(uint32_t nowMs, size_t maxBytes, OnFrame&& onFrame)
    {
        if (_state == State::Down || _state == State::Lost)
            return _state != State::Lost;
        uint8_t chunk[READ_CHUNK];
        size_t total = 0;
        while (total < maxBytes)
        {
            const size_t want = maxBytes - total < sizeof(chunk) ? maxBytes - total : sizeof(chunk);
            const int count = _port.read(chunk, want);
            if (count < 0)
                return lose(LossReason::PortError);
            if (count == 0)
                break;
            total += static_cast<size_t>(count);
            _bytesIn += static_cast<uint32_t>(count);
            for (int index = 0; index < count; ++index)
            {
                if (pushTextLine(chunk[index]))
                    continue;
                const auto result = _decoder.push(chunk[index]);
                if (result != ZenohSerialDecoder::Result::Frame)
                    continue;
                const uint8_t header = _decoder.header();
                if (_state == State::Initializing)
                {
                    if ((header & ZenohSerialFrame::FLAG_INIT) && (header & ZenohSerialFrame::FLAG_ACK))
                    {
                        _state = State::Up;
                        ++_linkUps;
                    }
                    else if (header & ZenohSerialFrame::FLAG_RESET)
                        ++_resetsReceived;     // resend I on the retry timer
                    continue;                   // data before I|A is from an old session
                }
                if (header & ZenohSerialFrame::FLAG_RESET)
                {
                    ++_resetsReceived;
                    return lose(LossReason::Reset);
                }
                if (header != 0)
                    continue;                   // a duplicate I|A answering one of our retries
                ++_framesIn;
                if (!onFrame(_decoder.payload(), _decoder.payloadSize()))
                    return lose(LossReason::Rejected);
                if (_state != State::Up)
                    return _state != State::Lost;
            }
        }
        if (_state == State::Initializing)
            sendInit(nowMs);
        return true;
    }

    /// @brief Send one batch as one frame.  @return false if the port has no
    /// room for the whole frame yet (try again next pass) or it is too big
    bool sendBatch(const uint8_t* batch, size_t length)
    {
        if (_state != State::Up)
            return false;
        const size_t encoded = ZenohSerialFrame::encode(0, batch, length, _tx.data(), _tx.size());
        if (encoded == 0)
        {
            ++_oversizeOut;
            return false;
        }
        if (!_port.write(_tx.data(), encoded))
        {
            ++_txFull;
            return false;
        }
        ++_framesOut;
        _bytesOut += static_cast<uint32_t>(encoded);
        return true;
    }

    enum class LossReason : uint8_t { None, Reset, PortError, Rejected };

    State state() const { return _state; }
    bool up() const { return _state == State::Up; }
    LossReason lossReason() const { return _lossReason; }
    const ZenohSerialDecoder& decoder() const { return _decoder; }
    uint32_t framesIn() const { return _framesIn; }
    uint32_t framesOut() const { return _framesOut; }
    uint32_t bytesIn() const { return _bytesIn; }
    uint32_t bytesOut() const { return _bytesOut; }
    uint32_t initsSent() const { return _initsSent; }
    uint32_t initsHeld() const { return _initsHeld; }
    uint32_t resetsReceived() const { return _resetsReceived; }
    uint32_t linkUps() const { return _linkUps; }
    uint32_t txFull() const { return _txFull; }
    uint32_t oversizeOut() const { return _oversizeOut; }

private:
    /// @return true if the byte ended (or trails) a command line that was
    /// taken, so it is not frame data
    bool pushTextLine(uint8_t byte)
    {
        if (!_onTextLine)
            return false;
        if (byte == '\r' || byte == '\n')
        {
            // A CR or LF straight after a taken line is its line ending
            bool taken = _afterTakenLine && _lineLength == 0;
            if (_lineLength > 0 && !_lineOverflow)
            {
                _line[_lineLength] = '\0';
                if (_onTextLine(_textLineContext, _line))
                {
                    _decoder.reset();
                    taken = _afterTakenLine = true;
                }
            }
            _lineLength = 0;
            _lineOverflow = false;
            return taken;
        }
        _afterTakenLine = false;
        if (byte >= 0x20 && byte < 0x7f)
        {
            if (_lineLength < sizeof(_line) - 1)
                _line[_lineLength++] = static_cast<char>(byte);
            else
                _lineOverflow = true;
        }
        else
        {
            // Not text (a frame, or a delimiter): start again
            _lineLength = 0;
            _lineOverflow = false;
        }
        return false;
    }

    /// @brief Send I if the retry interval has passed and the last one has
    /// gone.  While nobody has the port open, frames wait in the device's TX
    /// buffer; a queue of I frames would reach the router as a burst when it
    /// opens the port, and it answers alternate ones with I|A and R.
    void sendInit(uint32_t nowMs)
    {
        if (nowMs - _lastInitMs < INIT_RETRY_MS)
            return;
        if (!_port.txIdle())
        {
            ++_initsHeld;
            return;
        }
        _lastInitMs = nowMs;
        sendControl(ZenohSerialFrame::FLAG_INIT);
        ++_initsSent;
    }
    void sendControl(uint8_t header)
    {
        uint8_t frame[16];
        const size_t encoded = ZenohSerialFrame::encode(header, nullptr, 0, frame, sizeof(frame));
        if (_port.write(frame, encoded))
            _bytesOut += static_cast<uint32_t>(encoded);
    }
    bool lose(LossReason reason)
    {
        _state = State::Lost;
        _lossReason = reason;
        return false;
    }

    Port& _port;
    ZenohSerialDecoder _decoder;
    std::array<uint8_t, ZenohSerialFrame::MAX_ENCODED> _tx{};
    State _state = State::Down;
    LossReason _lossReason = LossReason::None;
    uint32_t _lastInitMs = 0;
    uint32_t _framesIn = 0, _framesOut = 0, _bytesIn = 0, _bytesOut = 0, _initsSent = 0;
    uint32_t _resetsReceived = 0, _linkUps = 0, _txFull = 0, _oversizeOut = 0, _initsHeld = 0;
    TextLineFn _onTextLine = nullptr;
    void* _textLineContext = nullptr;
    char _line[96] = {};
    size_t _lineLength = 0;
    bool _lineOverflow = false;
    bool _afterTakenLine = false;
};

}
