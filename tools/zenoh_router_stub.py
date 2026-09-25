#!/usr/bin/env python3
"""Minimal Zenoh router stand-in for bringing up the RaftROS Zenoh backend.

Speaks enough of the Zenoh TCP protocol to accept a session from a device,
then prints what the device declares and publishes.  It is a development aid,
not a router: it does not route anything, and a real demo uses rmw_zenohd so
ROS 2 tools see the node.  Use this when you want to know what the firmware is
actually putting on the wire, or when no router is available.

    python3 zenoh_router_stub.py                 # listen on 0.0.0.0:7447
    python3 zenoh_router_stub.py --port 7447 --interest

--interest additionally sends a "tell me everything you hold" interest once the
session is up, which exercises the firmware's interest-reply path.

The wire helpers mirror linux_unit_tests/zenoh_wire_interop.py, which is the
scripted peer the host tests use.
"""

import argparse
import socket
import struct
import sys
import time


def integer(value):
    """Zenoh variable-length integer"""
    encoded = bytearray()
    while value >= 128:
        encoded.append((value & 127) | 128)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


class Reader:
    def __init__(self, data):
        self.data = data
        self.offset = 0

    def take(self, size):
        if size > len(self.data) - self.offset:
            raise ValueError("truncated message")
        result = self.data[self.offset:self.offset + size]
        self.offset += size
        return result

    def byte(self):
        return self.take(1)[0]

    def number(self):
        value = 0
        for index in range(9):
            part = self.byte()
            value |= (part if index == 8 else part & 127) << (index * 7)
            if part < 128 or index == 8:
                return value
        raise ValueError("invalid integer")

    def blob(self):
        return self.take(self.number())


def decode_sample(key, payload):
    """Summarise a sample using the type named in its key.

    A ROS 2 Zenoh key ends with the wire type and its hash, so the payload can
    be decoded without any prior knowledge of the topic.
    """
    data = bytes(payload)
    if len(data) < 8 or data[:4] != b"\x00\x01\x00\x00":
        return "%d bytes (not CDR)" % len(data)
    if "::Range_" in key:
        # header, radiation_type, field_of_view, min, max, range, variance -
        # variance was added in Jazzy and is last, so the reading precedes it
        if len(data) >= 12:
            return "range=%.4f m" % struct.unpack("<f", data[-8:-4])[0]
    if "::String_" in key:
        size = struct.unpack("<I", data[4:8])[0]
        if 0 < size <= len(data) - 8:
            return '"%s"' % data[8:8 + size - 1].decode("utf-8", "replace")
    return "%d bytes" % len(data)


class Session:
    """One accepted device connection"""

    def __init__(self, connection, address):
        self.connection = connection
        self.address = address
        self.connection.settimeout(0.2)
        self.buffer = bytearray()
        self.tx_sequence = 17
        self.rx_sequence = 0
        self.last_keepalive = time.monotonic()
        self.tokens = {}
        self.sample_counts = {}

    def send_batch(self, payload):
        self.connection.sendall(struct.pack("<H", len(payload)) + payload)

    def send_frame(self, messages):
        self.send_batch(b"\x25" + integer(self.tx_sequence) + messages)
        self.tx_sequence += 1

    def next_batch(self, deadline):
        while time.monotonic() < deadline:
            if len(self.buffer) >= 2:
                size = struct.unpack_from("<H", self.buffer)[0]
                if len(self.buffer) >= size + 2:
                    result = bytes(self.buffer[2:2 + size])
                    del self.buffer[:2 + size]
                    return result
            try:
                chunk = self.connection.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:
                return None
            self.buffer.extend(chunk)
        return b""

    def handshake(self):
        initial = self.next_batch(time.monotonic() + 10)
        if not initial or initial[0] != 0x41:
            raise ValueError("expected INIT, got %r" % (initial,))
        identity = initial[3:19]
        print("session from %s:%d identity %s" %
              (self.address[0], self.address[1], identity.hex()), flush=True)
        cookie = b"\x00\x81\xff\x42"
        # Accept the device's proposal: same batch bound and lease it asked for
        self.send_batch(b"\x61\x09\x01\x77\x0a\x00\x10" + integer(len(cookie)) + cookie)
        opened = self.next_batch(time.monotonic() + 10)
        if not opened or opened[0] != 0x42:
            raise ValueError("expected OPEN, got %r" % (opened,))
        self.send_batch(b"\x62\x04" + integer(self.tx_sequence))
        print("session established", flush=True)

    def send_interest(self, identifier=1):
        """Ask the device to declare everything it holds"""
        self.send_frame(bytes([0x19 | (1 << 5)]) + integer(identifier) + bytes([8]))
        print("sent interest id=%d (asking for all declarations)" % identifier, flush=True)

    def keepalive(self):
        if time.monotonic() - self.last_keepalive >= 0.5:
            self.send_batch(b"\x04")
            self.last_keepalive = time.monotonic()

    def handle(self, payload, verbose):
        if payload == b"\x04":
            return
        reader = Reader(payload)
        if reader.byte() != 0x25:
            print("unexpected non-frame message %r" % payload[:8].hex(), flush=True)
            return
        sequence = reader.number()
        if sequence != self.rx_sequence:
            print("frame sequence discontinuity: got %d want %d" %
                  (sequence, self.rx_sequence), flush=True)
        self.rx_sequence = sequence + 1
        header = reader.byte()
        if header in (0x1e, 0x3e):
            correlation = reader.number() if header == 0x3e else None
            declaration = reader.byte()
            if declaration == 0x26:
                identifier = reader.number()
                reader.number()   # scope
                key = reader.blob().decode("ascii", "replace")
                self.tokens[identifier] = key
                print("DECLARE token id=%d%s\n    %s" %
                      (identifier, "" if correlation is None else " (reply to interest %d)" % correlation,
                       key), flush=True)
            elif declaration == 7:
                identifier = reader.number()
                key = self.tokens.pop(identifier, "<unknown>")
                print("UNDECLARE token id=%d\n    %s" % (identifier, key), flush=True)
            elif declaration == 0x1a:
                print("declarations complete%s" %
                      ("" if correlation is None else " for interest %d" % correlation), flush=True)
            else:
                # Unknown declaration: dump it so the exact wire format of a
                # reference implementation can be read off rather than guessed
                print("DECLARATION body 0x%02x raw=%s" %
                      (declaration, payload.hex()), flush=True)
        elif header == 0x3d:
            reader.number()
            key = reader.blob().decode("ascii", "replace")
            options = reader.byte()
            attachment = b""
            if options & 0x80:
                reader.byte()
                attachment = reader.blob()
            cdr = reader.blob()
            count = self.sample_counts.get(key, 0) + 1
            self.sample_counts[key] = count
            sequence_number = struct.unpack_from("<q", attachment)[0] if len(attachment) >= 8 else -1
            summary = decode_sample(key, cdr)
            if verbose or count <= 3 or count % 20 == 0:
                print("PUT #%d seq=%d %s\n    %s" % (count, sequence_number, summary, key), flush=True)
        else:
            print("MESSAGE 0x%02x raw=%s" % (header, payload.hex()), flush=True)


def serve(port, interest, verbose):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("0.0.0.0", port))
        listener.listen(4)
        print("zenoh router stub listening on 0.0.0.0:%d" % port, flush=True)
        while True:
            connection, address = listener.accept()
            with connection:
                session = Session(connection, address)
                try:
                    session.handshake()
                    if interest:
                        session.send_interest()
                    while True:
                        session.keepalive()
                        payload = session.next_batch(time.monotonic() + 0.2)
                        if payload is None:
                            print("device closed the session", flush=True)
                            break
                        if payload:
                            session.handle(payload, verbose)
                except (ValueError, OSError) as error:
                    print("session ended: %s" % error, flush=True)
                finally:
                    if session.sample_counts:
                        print("samples per key: %s" % session.sample_counts, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=7447)
    parser.add_argument("--interest", action="store_true",
                        help="send an interest once connected, to exercise interest replies")
    parser.add_argument("--verbose", action="store_true", help="print every sample")
    args = parser.parse_args()
    try:
        serve(args.port, args.interest, args.verbose)
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
