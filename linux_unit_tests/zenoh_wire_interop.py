import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def integer(value):
    encoded = bytearray()
    while value >= 128:
        encoded.append((value & 127) | 128)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def interest(identifier, mode=1, key=None, options=8, scope=0):
    message = bytes([0x19 | (mode << 5)]) + integer(identifier)
    if mode:
        message += bytes([options | (0x30 if key is not None else 0)])
        if key is not None:
            suffix = key.encode("ascii")
            message += integer(scope) + integer(len(suffix)) + suffix
    return message


class Reader:
    def __init__(self, data):
        self.data = data
        self.offset = 0

    def take(self, size):
        require(size <= len(self.data) - self.offset, "Truncated outgoing probe message")
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
        raise AssertionError("Invalid outgoing integer")

    def blob(self):
        return self.take(self.number())


class Peer:
    def __init__(self, connection):
        self.connection = connection
        self.connection.settimeout(0.1)
        self.buffer = bytearray()
        self.tx_sequence = 17
        self.rx_sequence = 0
        self.maintain = False
        self.last_keepalive = time.monotonic()
        self.identity = None

    def send_batch(self, payload):
        require(0 < len(payload) <= 4096, "Invalid scripted peer batch length")
        wire = struct.pack("<H", len(payload)) + payload
        self.connection.sendall(wire[:1])
        self.connection.sendall(wire[1:])

    def send_frame(self, messages):
        self.send_batch(b"\x25" + integer(self.tx_sequence) + messages)
        self.tx_sequence += 1

    def batch(self, deadline):
        while time.monotonic() < deadline:
            if self.maintain and time.monotonic() - self.last_keepalive >= 0.5:
                self.send_batch(b"\x04")
                self.last_keepalive = time.monotonic()
            if len(self.buffer) >= 2:
                size = struct.unpack_from("<H", self.buffer)[0]
                require(0 < size <= 4096, "Probe exceeded negotiated batch bound")
                if len(self.buffer) >= size + 2:
                    result = bytes(self.buffer[2:2 + size])
                    del self.buffer[:2 + size]
                    return result
            try:
                chunk = self.connection.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                require(not self.buffer, "Probe closed in the middle of a batch")
                return None
            self.buffer.extend(chunk)
        raise AssertionError("Scripted peer receive deadline expired")

    def handshake(self):
        initial = self.batch(time.monotonic() + 5)
        require(initial is not None and len(initial) == 22, "Expected bounded INIT SYN")
        require(initial[:3] == b"\x41\x09\xf2" and initial[-3:] == b"\x0a\x00\x10",
                "Unexpected probe handshake profile")
        self.identity = initial[3:19]
        cookie = b"\x00\x81\xff\x42"
        self.send_batch(b"\x61\x09\x01\x77\x0a\x00\x10" + integer(len(cookie)) + cookie)
        opened = self.batch(time.monotonic() + 5)
        require(opened == b"\x42\x04\x00" + integer(len(cookie)) + cookie,
                "OPEN must echo opaque cookie and start at sequence zero")
        self.send_batch(b"\x62\x04" + integer(self.tx_sequence))
        self.maintain = True

    def event(self, deadline):
        while True:
            payload = self.batch(deadline)
            if payload is None:
                return None
            if payload == b"\x04":
                continue
            reader = Reader(payload)
            require(reader.byte() == 0x25, "Probe must use reliable FRAME for discovery/data")
            require(reader.number() == self.rx_sequence, "Outgoing frame sequence discontinuity")
            self.rx_sequence += 1
            header = reader.byte()
            correlation = None
            if header in (0x1e, 0x3e):
                if header == 0x3e:
                    correlation = reader.number()
                declaration = reader.byte()
                if declaration == 0x26:
                    identifier = reader.number()
                    require(reader.number() == 0, "Probe must declare full scope-zero key")
                    key = reader.blob().decode("ascii")
                    event = ("token", correlation, identifier, key)
                elif declaration == 7:
                    event = ("remove", correlation, reader.number(), None)
                else:
                    require(declaration == 0x1a, "Unexpected declaration body")
                    event = ("final", correlation, None, None)
            else:
                require(header == 0x3d and reader.number() == 0, "Unexpected network message")
                key = reader.blob().decode("ascii")
                require(reader.byte() == 0x81 and reader.byte() == 0x43, "Expected PUT with attachment")
                attachment = reader.blob()
                require(len(attachment) == 33, "Unexpected attachment size")
                cdr = reader.blob()
                require(cdr[:4] == b"\x00\x01\x00\x00", "Expected CDR encapsulation")
                event = ("put", None, None, key)
            require(reader.offset == len(payload), "Unexpected trailing outgoing bytes")
            return event


@contextmanager
def connected_probe(probe):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(5)
        process = subprocess.Popen([str(probe), "127.0.0.1", str(listener.getsockname()[1]), "12000", "--publish"],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            connection, _ = listener.accept()
            with connection:
                peer = Peer(connection)
                peer.handshake()
                yield peer, process
        finally:
            if process.poll() is None:
                process.kill()
            output, _ = process.communicate(timeout=5)
            print(output, end="")


def live_tokens(peer):
    tokens = {}
    deadline = time.monotonic() + 3
    while len(tokens) != 2:
        event = peer.event(deadline)
        require(event is not None, "Probe closed before announcing both tokens")
        if event[0] == "token" and event[1] is None:
            tokens[event[2]] = event[3]
    require(set(tokens) == {1, 2}, "Expected node and publisher tokens")
    return tokens


def replies(peer, identifier, expected, forbidden=()):
    received = {}
    deadline = time.monotonic() + 3
    while True:
        event = peer.event(deadline)
        require(event is not None, "Probe closed before correlated final")
        require(event[1] not in forbidden, "Cancelled interest produced a reply")
        if event[1] != identifier:
            continue
        if event[0] == "final":
            require(received == expected, f"Incorrect tokens for interest {identifier}: {received}")
            return
        require(event[0] == "token" and event[2] not in received, "Duplicate or unexpected correlated reply")
        received[event[2]] = event[3]


def grouped_replies(peer, expected, forbidden=()):
    pending = {identifier: {} for identifier in expected}
    deadline = time.monotonic() + 3
    while pending:
        event = peer.event(deadline)
        require(event is not None, "Probe closed before grouped replies completed")
        require(event[1] not in forbidden, "Cancelled or future-only interest produced a correlated reply")
        if event[1] is None:
            continue
        require(event[1] in pending, "Unexpected correlation or duplicate final")
        tokens = pending[event[1]]
        if event[0] == "final":
            require(tokens == expected[event[1]], "Incorrect token set in grouped reply")
            del pending[event[1]]
        else:
            require(event[0] == "token" and event[2] not in tokens, "Duplicate token in grouped reply")
            tokens[event[2]] = event[3]


def test_interests(probe):
    with connected_probe(probe) as (peer, process):
        tokens = live_tokens(peer)
        cases = [
            (100, 1, None, 8, tokens),
            (101, 3, "@ros2_lv/23/**", 8, tokens),
            (102, 1, tokens[2], 8, {2: tokens[2]}),
            (103, 1, "@ros2_lv/24/**", 8, {}),
            (104, 1, None, 2, {}),
        ]
        for identifier, mode, key, options, expected in cases:
            peer.send_frame(interest(identifier, mode, key, options))
            replies(peer, identifier, expected)
        peer.send_frame(interest(105) + interest(105, 0) + interest(106, key=tokens[1]))
        replies(peer, 106, {1: tokens[1]}, forbidden=(105,))
        peer.send_frame(interest(108, mode=2) + b"".join(interest(identifier, options=2) for identifier in range(110, 114)))
        grouped_replies(peer, {identifier: {} for identifier in range(110, 114)}, forbidden=(108,))
        peer.send_frame(b"".join(interest(identifier) for identifier in range(120, 124)) +
                interest(120, mode=0) + interest(124, options=2))
        grouped_replies(peer, {121: tokens, 122: tokens, 123: tokens, 124: {}}, forbidden=(120,))
        print("PASS: direct current/current-future, exact/prefix/no-match, non-token, cancellation and four-entry queue")
        removed = set()
        deadline = time.monotonic() + 8
        while len(removed) != 2:
            event = peer.event(deadline)
            require(event is not None, "Probe closed before token withdrawal")
            if event[0] == "remove":
                removed.add(event[2])
        peer.send_frame(interest(107))
        replies(peer, 107, {})
        deadline = time.monotonic() + 8
        while True:
            event = peer.event(deadline)
            if event is None:
                break
            require(event[0] not in ("token", "put"), "Withdrawn publisher reappeared")
        require(process.wait(timeout=2) == 0, "Probe did not complete normal lifecycle")
        print("PASS: withdrawn tokens stay absent from direct queries until transport shutdown")


def test_failures_and_restart(probe):
    identities = set()
    cases = {"eof": "state=5 error=9 ", "silence": "state=5 error=7 ",
             "scope": "state=5 error=11 ", "wildcard": "state=5 error=11 ",
             "capacity": "state=5 error=11 ", "close": "state=4 error=0 closeReason=1 "}
    for case, expected_status in cases.items():
        with connected_probe(probe) as (peer, process):
            require(peer.identity not in identities, "Restart reused a live session identity")
            identities.add(peer.identity)
            live_tokens(peer)
            if case == "eof":
                peer.connection.shutdown(socket.SHUT_RDWR)
            elif case == "silence":
                peer.maintain = False
            elif case == "scope":
                peer.send_frame(interest(200, key="x", scope=1))
            elif case == "wildcard":
                peer.send_frame(interest(200, key="@ros2_lv/*/x"))
            elif case == "close":
                peer.send_batch(b"\x23\x01")
            else:
                peer.send_frame(b"".join(interest(identifier) for identifier in range(200, 205)))
            output, _ = process.communicate(timeout=6)
            require(process.returncode == 1 and expected_status in output,
                    f"{case} did not terminate for the expected reason: {output}")
            require("AddressSanitizer" not in output and "runtime error:" not in output,
                    f"{case} failed with a sanitizer report")
            print(f"PASS: {case} fails within deadline; fresh process restarts handshake and sequence state")


def test_storage_allocation(probe, failure_probe):
    information = subprocess.run([str(probe), "--resource-info"], check=True,
                                 capture_output=True, text=True, timeout=5)
    storage = json.loads(information.stdout)
    components = sum(storage[name] for name in
                     ("session_bytes", "publication_probe_bytes", "socket_rx_scratch_bytes"))
    require(storage["storage_location"] == "single_heap_allocation" and
            components <= storage["owned_storage_bytes"] <= storage["owned_storage_limit_bytes"] == 24576,
            "Owned workspace must include every persistent buffer within the fixed 24 KiB limit")
    no_allocation_info = subprocess.run([str(failure_probe), "--resource-info"], check=True,
                                       capture_output=True, text=True, timeout=5)
    require(json.loads(no_allocation_info.stdout) == storage, "Resource query must not allocate a workspace")
    normal_payload = subprocess.run([str(probe), "--range-payload", "2"], check=True,
                                    capture_output=True, timeout=5)
    failed_payload = subprocess.run([str(failure_probe), "--range-payload", "2"], check=True,
                                    capture_output=True, timeout=5)
    require(normal_payload.stdout == failed_payload.stdout, "Offline CDR fixture must not require session allocation")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        for mode in ([], ["--publish"], ["--publish-range"]):
            result = subprocess.run([str(failure_probe), "127.0.0.1", str(listener.getsockname()[1]), "12000", *mode],
                                    capture_output=True, text=True, timeout=5)
            expected = f"Unable to allocate {storage['owned_storage_bytes']} bytes of bounded probe storage\n"
            require(result.returncode == 1 and not result.stdout and result.stderr == expected,
                    f"Storage exhaustion must produce the expected early failure: {result}")
            require(not select.select([listener], [], [], 0)[0], "Allocation failure opened a network connection")
    print("PASS: one bounded heap workspace; allocation failure exits before connecting in every session mode")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", type=Path, default=Path(__file__).parent / "build/zenoh/zenoh_session_probe")
    parser.add_argument("--allocation-failure-probe", type=Path)
    parser.add_argument("--allocation-failure-only", action="store_true")
    args = parser.parse_args()
    if args.allocation_failure_only and args.allocation_failure_probe is None:
        parser.error("--allocation-failure-only requires --allocation-failure-probe")
    probe = args.probe.resolve(strict=True)
    if args.allocation_failure_probe is not None:
        test_storage_allocation(probe, args.allocation_failure_probe.resolve(strict=True))
    if args.allocation_failure_only:
        return
    test_interests(probe)
    test_failures_and_restart(probe)
    print("PASS: scripted wire peer only; native ROS graph restart is a separate test")


if __name__ == "__main__":
    main()