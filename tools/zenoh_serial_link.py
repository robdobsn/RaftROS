#!/usr/bin/env python3
"""Zenoh serial link (z-serial) framing, handshake and relays for RaftROS.

Zenoh's serial link carries one transport batch per frame: a 1-byte header
(I=0x01 init, A=0x02 ack, R=0x04 reset, 0 for data), a 2-byte little-endian
payload length, the payload (at most 1500 bytes) and a CRC-32 of the payload,
COBS-encoded and terminated by a 0x00 delimiter.  Before any Zenoh INIT the
connecting side (the device) sends an empty I frame and the listening side (the
router) answers with an empty I|A frame.  This tool speaks that, so a serial
link can be tested and watched without changing anything else.

    zenoh_serial_link.py selftest
        Check the CRC, COBS and framing against z-serial's own test vectors.

    zenoh_serial_link.py bridge --serial /dev/ttyACM1 --router 127.0.0.1:7447
        Router side of the link.  Accepts the device's handshake on a serial
        port and relays its batches to a TCP router (rmw_zenohd, or
        zenoh_router_stub.py), adding the TCP length prefix one way and
        stripping it the other.  Bytes on the port that are not a frame - a
        device's log lines - are printed instead of tearing the link down, so
        this is the bring-up path; a real serial-listening router drops the
        link on the first bad frame.

    zenoh_serial_link.py relay --serial /dev/pts/5 --listen 7448
        Device side of the link.  Accepts a TCP Zenoh client (for example
        linux_unit_tests' zenoh_session_probe), runs the handshake on the
        serial port and relays.  Pointed at a router listening on serial (one
        end of a socat pty pair) it proves the router side with no board.

    zenoh_serial_link.py tap --serial /dev/ttyACM1
        Decode and print every frame and any text on a port.

Needs pyserial.  The baud rate only matters on a real UART; USB CDC ports and
ptys ignore it.
"""

import argparse
import socket
import struct
import sys
import threading
import time

I_FLAG = 0x01
A_FLAG = 0x02
R_FLAG = 0x04
MAX_MTU = 1500
COBS_BUF_SIZE = 1517
TCP_BATCH_MAX = 65535

# z-serial's CRC: a reflected table algorithm (shift right, init and final
# inversion) built on the unreflected polynomial - so not the zlib CRC-32
_POLY = 0x04C11DB7
_CRC_TABLE = []
for _n in range(256):
    _r = _n
    for _ in range(8):
        _r = (_POLY ^ (_r >> 1)) if _r & 1 else (_r >> 1)
    _CRC_TABLE.append(_r)


def crc32_zserial(data):
    acc = 0xFFFFFFFF
    for byte in data:
        acc = (acc >> 8) ^ _CRC_TABLE[(acc & 0xFF) ^ byte]
    return (~acc) & 0xFFFFFFFF


def cobs_encode(data):
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue
        out.append(byte)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ValueError("zero byte inside COBS data")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ValueError("COBS block runs past the end")
        out.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(payload, header=0):
    if len(payload) > MAX_MTU:
        raise ValueError("payload of %d bytes exceeds the serial MTU" % len(payload))
    raw = bytes([header]) + struct.pack("<H", len(payload)) + payload + struct.pack("<I", crc32_zserial(payload))
    return cobs_encode(raw) + b"\x00"


def decode_frame(chunk):
    """Decode one delimited chunk (without its 0x00).  @return (header, payload)"""
    raw = cobs_decode(chunk)
    if len(raw) < 7:
        raise ValueError("frame of %d bytes is shorter than the minimum" % len(raw))
    header = raw[0]
    length = struct.unpack_from("<H", raw, 1)[0]
    if 3 + length + 4 != len(raw):
        raise ValueError("length field %d does not match the %d-byte frame" % (length, len(raw)))
    payload = raw[3:3 + length]
    (crc,) = struct.unpack_from("<I", raw, 3 + length)
    if crc != crc32_zserial(payload):
        raise ValueError("CRC mismatch")
    return header, payload


class FrameReader:
    """Splits a byte stream on 0x00 and decodes each chunk.  Yields
    ("frame", header, payload) or ("junk", bytes, reason) - junk being anything
    that does not decode, typically a device's log text."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            end = self.buffer.find(0)
            if end < 0:
                if len(self.buffer) > 4 * COBS_BUF_SIZE:
                    junk = bytes(self.buffer)
                    self.buffer.clear()
                    yield ("junk", junk, "no delimiter")
                return
            chunk = bytes(self.buffer[:end])
            del self.buffer[:end + 1]
            if not chunk:
                continue
            try:
                header, payload = decode_frame(chunk)
                yield ("frame", header, payload)
            except ValueError as error:
                yield ("junk", chunk, str(error))


def batches_from_tcp(buffer):
    """Pop complete length-prefixed batches from a TCP receive buffer"""
    while len(buffer) >= 2:
        size = struct.unpack_from("<H", buffer)[0]
        if len(buffer) < 2 + size:
            return
        batch = bytes(buffer[2:2 + size])
        del buffer[:2 + size]
        yield batch


def describe(header):
    flags = [name for bit, name in ((I_FLAG, "I"), (A_FLAG, "A"), (R_FLAG, "R")) if header & bit]
    return "|".join(flags) if flags else "data"


def print_junk(chunk, reason, prefix):
    text = chunk.decode("utf-8", errors="replace")
    printable = sum(1 for c in text if c.isprintable() or c in "\r\n\t")
    if text and printable >= 0.9 * len(text):
        for line in text.replace("\r", "").splitlines():
            if line.strip():
                print("%s %s" % (prefix, line), flush=True)
    else:
        print("%s <%d bytes not a frame: %s> %s" % (prefix, len(chunk), reason, chunk[:32].hex()), flush=True)


def open_serial(path, baud):
    import serial
    # No DTR/RTS changes after open: on an ESP32 USB-Serial-JTAG port the
    # esptool pattern on those lines resets the chip
    port = serial.Serial()
    port.port = path
    port.baudrate = baud
    port.timeout = 0.05
    port.dtr = True
    port.rts = True
    port.exclusive = True
    port.open()
    return port


class Stats:
    def __init__(self):
        self.frames_in = self.frames_out = self.bytes_in = self.bytes_out = self.junk = 0

    def line(self):
        return "frames in %d (%d B) out %d (%d B), junk chunks %d" % (
            self.frames_in, self.bytes_in, self.frames_out, self.bytes_out, self.junk)


# ---------------------------------------------------------------------------
# bridge: router side of the serial link, relaying to a TCP router
# ---------------------------------------------------------------------------

def run_bridge(args):
    host, port = args.router.rsplit(":", 1)
    serial_port = open_serial(args.serial, args.baud)
    reader = FrameReader()
    stats = Stats()
    tcp = None
    tcp_lock = threading.Lock()
    initialized = False
    write_lock = threading.Lock()
    last_stats = time.monotonic()

    def serial_write(payload, header=0):
        with write_lock:
            serial_port.write(encode_frame(payload, header))

    def tcp_to_serial(sock):
        nonlocal initialized
        buffer = bytearray()
        while True:
            try:
                data = sock.recv(8192)
            except OSError:
                data = b""
            if not data:
                break
            buffer.extend(data)
            for batch in batches_from_tcp(buffer):
                if len(batch) > MAX_MTU:
                    print("router sent a %d-byte batch, above the serial MTU - dropped" % len(batch), flush=True)
                    continue
                serial_write(batch)
                stats.frames_out += 1
                stats.bytes_out += len(batch)
                if args.verbose:
                    print("-> device %4d B %s" % (len(batch), batch[:24].hex()), flush=True)
        with tcp_lock:
            if sock is tcp:
                print("router closed the TCP connection; waiting for the device to re-initialize", flush=True)
                initialized = False
        try:
            sock.close()
        except OSError:
            pass

    def close_tcp():
        nonlocal tcp
        with tcp_lock:
            old, tcp = tcp, None
        if old:
            try:
                old.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            old.close()

    print("bridge: serial %s <-> router tcp %s:%s" % (args.serial, host, port), flush=True)
    try:
        while True:
            data = serial_port.read(4096)
            if args.stats and time.monotonic() - last_stats > args.stats:
                last_stats = time.monotonic()
                print("stats: " + stats.line(), flush=True)
            if not data:
                continue
            for item in reader.feed(data):
                if item[0] == "junk":
                    stats.junk += 1
                    print_junk(item[1], item[2], "[device]")
                    continue
                _, header, payload = item
                if header & I_FLAG:
                    if initialized:
                        # The device restarted its link: tell it to reset, drop
                        # the router session, and accept its next I
                        print("device re-sent I on an initialized link - replying R", flush=True)
                        serial_write(b"", R_FLAG)
                        initialized = False
                        close_tcp()
                        continue
                    close_tcp()
                    try:
                        sock = socket.create_connection((host, int(port)), timeout=3)
                    except OSError as error:
                        print("device sent I but the router at %s:%s is not reachable (%s) - not acknowledging"
                              % (host, port, error), flush=True)
                        continue
                    sock.settimeout(None)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    with tcp_lock:
                        tcp = sock
                    threading.Thread(target=tcp_to_serial, args=(sock,), daemon=True).start()
                    serial_write(b"", I_FLAG | A_FLAG)
                    initialized = True
                    print("device link initialized (I -> I|A), router session opened", flush=True)
                    continue
                if not initialized:
                    print("data frame of %d B before the link handshake - ignored" % len(payload), flush=True)
                    continue
                stats.frames_in += 1
                stats.bytes_in += len(payload)
                if args.verbose:
                    print("<- device %4d B %s" % (len(payload), payload[:24].hex()), flush=True)
                with tcp_lock:
                    sock = tcp
                if sock and payload:
                    try:
                        sock.sendall(struct.pack("<H", len(payload)) + payload)
                    except OSError as error:
                        print("send to router failed: %s" % error, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print("stats: " + stats.line(), flush=True)
        close_tcp()
        serial_port.close()


# ---------------------------------------------------------------------------
# relay: device side of the serial link, for a TCP Zenoh client
# ---------------------------------------------------------------------------

def link_handshake(serial_port, reader, retry_s=0.25, timeout_s=10.0):
    """Send I until the listener answers I|A.  @return True when initialized"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        serial_port.write(encode_frame(b"", I_FLAG))
        wait_until = time.monotonic() + retry_s
        while time.monotonic() < wait_until:
            data = serial_port.read(4096)
            for item in reader.feed(data):
                if item[0] == "junk":
                    print_junk(item[1], item[2], "[link]")
                    continue
                _, header, _ = item
                if header & I_FLAG and header & A_FLAG:
                    return True
                if header & R_FLAG:
                    print("listener replied R - retrying I", flush=True)
                    wait_until = 0
                    break
    return False


def run_relay(args):
    serial_port = open_serial(args.serial, args.baud)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", args.listen))
    server.listen(1)
    print("relay: tcp 127.0.0.1:%d <-> serial %s (device side)" % (args.listen, args.serial), flush=True)
    try:
        while True:
            client, address = server.accept()
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("client %s:%d connected - starting link handshake" % address, flush=True)
            reader = FrameReader()
            if not link_handshake(serial_port, reader):
                print("no I|A from the serial listener - dropping the client", flush=True)
                client.close()
                continue
            print("link initialized", flush=True)
            stats = Stats()
            stop = threading.Event()

            def serial_to_tcp():
                while not stop.is_set():
                    data = serial_port.read(4096)
                    for item in reader.feed(data):
                        if item[0] == "junk":
                            print_junk(item[1], item[2], "[link]")
                            continue
                        _, header, payload = item
                        if header:
                            print("listener sent %s mid-session" % describe(header), flush=True)
                            continue
                        stats.frames_in += 1
                        stats.bytes_in += len(payload)
                        try:
                            client.sendall(struct.pack("<H", len(payload)) + payload)
                        except OSError:
                            stop.set()
                            return

            thread = threading.Thread(target=serial_to_tcp, daemon=True)
            thread.start()
            buffer = bytearray()
            while not stop.is_set():
                try:
                    data = client.recv(8192)
                except OSError:
                    data = b""
                if not data:
                    break
                buffer.extend(data)
                for batch in batches_from_tcp(buffer):
                    if len(batch) > MAX_MTU:
                        print("client sent a %d-byte batch, above the serial MTU - dropped" % len(batch), flush=True)
                        continue
                    serial_port.write(encode_frame(batch))
                    stats.frames_out += 1
                    stats.bytes_out += len(batch)
            stop.set()
            thread.join()
            client.close()
            print("client disconnected; " + stats.line(), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        serial_port.close()


# ---------------------------------------------------------------------------
# tap and selftest
# ---------------------------------------------------------------------------

def run_tap(args):
    serial_port = open_serial(args.serial, args.baud)
    reader = FrameReader()
    try:
        while True:
            for item in reader.feed(serial_port.read(4096)):
                if item[0] == "junk":
                    print_junk(item[1], item[2], "[text]")
                else:
                    _, header, payload = item
                    print("%.3f %-5s %4d B %s" % (time.monotonic(), describe(header), len(payload),
                                                  payload[:32].hex()), flush=True)
    except KeyboardInterrupt:
        pass


def run_selftest(_args):
    vectors = [
        (bytes.fromhex("001100"), bytes.fromhex("73EC75F9"), bytes.fromhex("0102030102110573EC75F900")),
        (bytes.fromhex("11220033"), bytes.fromhex("8D036DFB"), bytes.fromhex("01020403112206338D036DFB00")),
    ]
    for payload, crc, frame in vectors:
        assert struct.pack("<I", crc32_zserial(payload)) == crc, "CRC of %s" % payload.hex()
        assert encode_frame(payload) == frame, "frame of %s: %s" % (payload.hex(), encode_frame(payload).hex())
        assert decode_frame(frame[:-1]) == (0, payload)
    # COBS round trips across the 254-byte block boundary and with zero runs
    import random
    rng = random.Random(1)
    for size in list(range(0, 600)) + [MAX_MTU]:
        data = bytes(rng.choice([0, 0, 1, 0xFF, rng.randrange(256)]) for _ in range(size))
        encoded = cobs_encode(data)
        assert 0 not in encoded and cobs_decode(encoded) == data, "COBS size %d" % size
        if size <= MAX_MTU:
            assert decode_frame(encode_frame(data, 0)[:-1]) == (0, data)
    assert len(encode_frame((bytes(range(1, 256)) * 6)[:MAX_MTU])[:-1]) <= COBS_BUF_SIZE
    # The reader recovers from text and corruption without losing a good frame
    good = encode_frame(b"\x04")
    corrupt = bytearray(good)
    corrupt[3] ^= 0x40
    items = list(FrameReader().feed(b"I (123) log line\n\x00" + bytes(corrupt) + good))
    kinds = [item[0] for item in items]
    assert kinds == ["junk", "junk", "frame"] and items[2][2] == b"\x04", items
    print("selftest passed: CRC and frame vectors, COBS round trips, reader resync")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("selftest")
    bridge = sub.add_parser("bridge")
    bridge.add_argument("--serial", required=True)
    bridge.add_argument("--router", default="127.0.0.1:7447")
    bridge.add_argument("--verbose", action="store_true")
    bridge.add_argument("--stats", type=float, default=0, help="print counters every N seconds")
    relay = sub.add_parser("relay")
    relay.add_argument("--serial", required=True)
    relay.add_argument("--listen", type=int, default=7448)
    tap = sub.add_parser("tap")
    tap.add_argument("--serial", required=True)
    for p in (bridge, relay, tap):
        p.add_argument("--baud", type=int, default=921600)
    args = parser.parse_args()
    {"selftest": run_selftest, "bridge": run_bridge, "relay": run_relay, "tap": run_tap}[args.mode](args)


if __name__ == "__main__":
    main()
