#!/usr/bin/env python3
"""SPDP multicast sniffer - decodes RTPS discovery messages on port 7400."""

import socket
import struct
import sys
import time

SPDP_MCAST = "239.255.0.1"
SPDP_PORT = 7400

PID_NAMES = {
    0x0001: "PID_SENTINEL",
    0x0002: "PID_TOPIC_NAME",
    0x0005: "PID_DURABILITY",
    0x0007: "PID_TYPE_NAME",
    0x0015: "PID_PROTOCOL_VERSION",
    0x0016: "PID_VENDORID",
    0x001A: "PID_RELIABILITY",
    0x002F: "PID_DEFAULT_UNICAST_LOCATOR",
    0x0031: "PID_METATRAFFIC_UNICAST_LOCATOR",
    0x0032: "PID_METATRAFFIC_MULTICAST_LOCATOR",
    0x0033: "PID_PARTICIPANT_MANUAL_LIVELINESS_COUNT",
    0x0048: "PID_DEFAULT_MULTICAST_LOCATOR",
    0x0050: "PID_PARTICIPANT_GUID",
    0x0058: "PID_BUILTIN_ENDPOINT_SET",
    0x0059: "PID_PROPERTY_LIST",
    0x005A: "PID_ENDPOINT_GUID",
    0x0002: "PID_PARTICIPANT_LEASE_DURATION",
    0x8000: "PID_PERSISTENCE",
}

# Fix: PID_PARTICIPANT_LEASE_DURATION and PID_TOPIC_NAME share 0x0002
# Actually PID_PARTICIPANT_LEASE_DURATION = 0x0002 in SPDP context
PID_PARTICIPANT_LEASE_DURATION = 0x0002
PID_SENTINEL = 0x0001

SUBMSG_NAMES = {
    0x00: "PAD",
    0x01: "ACKNACK",
    0x02: "HEARTBEAT",
    0x05: "INFO_DST",
    0x06: "INFO_REPLY",
    0x07: "NACK_FRAG",
    0x09: "INFO_TS",
    0x0c: "INFO_SRC",
    0x12: "DATA_FRAG",
    0x15: "DATA",
}

def decode_locator(data):
    """Decode a 24-byte RTPS locator."""
    if len(data) < 24:
        return "incomplete"
    kind = struct.unpack_from("<I", data, 0)[0]
    port = struct.unpack_from("<I", data, 4)[0]
    addr = data[8:24]
    if kind == 1:  # UDP_v4
        ip = f"{addr[12]}.{addr[13]}.{addr[14]}.{addr[15]}"
        return f"UDPv4 {ip}:{port}"
    return f"kind={kind} port={port} addr={addr.hex()}"


def decode_parameter_list(data, indent="    "):
    """Decode a CDR ParameterList."""
    offset = 0
    params = []
    while offset + 4 <= len(data):
        pid, length = struct.unpack_from("<HH", data, offset)
        offset += 4
        if pid == PID_SENTINEL:
            params.append(f"{indent}PID_SENTINEL")
            break
        if offset + length > len(data):
            params.append(f"{indent}PID 0x{pid:04X} len={length} TRUNCATED")
            break
        pdata = data[offset:offset + length]
        name = PID_NAMES.get(pid, f"PID_0x{pid:04X}")

        detail = ""
        if pid == 0x0015:  # PROTOCOL_VERSION
            if length >= 2:
                detail = f" v{pdata[0]}.{pdata[1]}"
        elif pid == 0x0016:  # VENDORID
            if length >= 2:
                detail = f" {pdata[0]:02x}.{pdata[1]:02x}"
                known = {
                    (0x01, 0x01): "RTI",
                    (0x01, 0x0F): "eProsima/FastDDS",
                    (0x01, 0x10): "CycloneDDS",
                    (0x01, 0x03): "OpenDDS",
                    (0x00, 0x00): "UNKNOWN",
                }
                vendor = known.get((pdata[0], pdata[1]), "")
                if vendor:
                    detail += f" ({vendor})"
        elif pid == 0x0050:  # PARTICIPANT_GUID
            if length >= 16:
                detail = f" {pdata[:12].hex()}:{pdata[12:16].hex()}"
        elif pid == 0x0058:  # BUILTIN_ENDPOINT_SET
            if length >= 4:
                val = struct.unpack_from("<I", pdata, 0)[0]
                detail = f" 0x{val:08X}"
                bits = []
                if val & (1 << 0): bits.append("DISC_PUB_ANNOUNCER")
                if val & (1 << 1): bits.append("DISC_PUB_DETECTOR")
                if val & (1 << 2): bits.append("DISC_SUB_ANNOUNCER")
                if val & (1 << 3): bits.append("DISC_SUB_DETECTOR")
                if val & (1 << 4): bits.append("PART_MSG_WRITER")
                if val & (1 << 5): bits.append("PART_MSG_READER")
                if bits:
                    detail += f" [{','.join(bits)}]"
        elif pid in (0x002F, 0x0031, 0x0032, 0x0048):  # locators
            detail = f" {decode_locator(pdata)}"
        elif pid == PID_PARTICIPANT_LEASE_DURATION:
            if length >= 8:
                secs = struct.unpack_from("<I", pdata, 0)[0]
                frac = struct.unpack_from("<I", pdata, 4)[0]
                detail = f" {secs}s frac={frac}"
        elif pid == 0x0059:  # PROPERTY_LIST
            detail = f" ({length} bytes)"

        params.append(f"{indent}{name} len={length}{detail}")
        offset += length
        # Align to 4
        offset = (offset + 3) & ~3

    return "\n".join(params)


def decode_rtps(data, src_addr):
    """Decode an RTPS message."""
    if len(data) < 20:
        return "Too short for RTPS header"

    magic = data[0:4]
    if magic != b'RTPS':
        return f"Not RTPS (magic: {magic})"

    proto_major, proto_minor = data[4], data[5]
    vendor = f"{data[6]:02x}.{data[7]:02x}"
    guid_prefix = data[8:20].hex()

    lines = [
        f"  RTPS v{proto_major}.{proto_minor} vendor={vendor} guidPrefix={guid_prefix}"
    ]

    offset = 20
    submsg_num = 0
    while offset + 4 <= len(data):
        submsg_id = data[offset]
        flags = data[offset + 1]
        submsg_len = struct.unpack_from("<H", data, offset + 2)[0]
        submsg_name = SUBMSG_NAMES.get(submsg_id, f"0x{submsg_id:02X}")
        is_le = flags & 0x01

        lines.append(f"  [{submsg_num}] {submsg_name} flags=0x{flags:02X} len={submsg_len}")

        if submsg_id == 0x15 and submsg_len > 0:  # DATA
            content_start = offset + 4
            if content_start + 20 <= len(data):
                extra_flags = struct.unpack_from("<H", data, content_start)[0]
                octets_to_iq = struct.unpack_from("<H", data, content_start + 2)[0]
                reader_id = data[content_start + 4:content_start + 8].hex()
                writer_id = data[content_start + 8:content_start + 12].hex()
                seq_hi = struct.unpack_from("<I", data, content_start + 12)[0]
                seq_lo = struct.unpack_from("<I", data, content_start + 16)[0]

                lines.append(f"    reader={reader_id} writer={writer_id} seq={seq_hi}:{seq_lo}")

                # Check for serialized data (flag D = 0x04)
                if flags & 0x04:
                    payload_offset = content_start + 4 + octets_to_iq + 4  # past entityIds+seqNum
                    # Actually payload starts after extraFlags(2) + octetsToIQ(2) + readerId(4) + writerId(4) + seqNum(8) = 20
                    payload_offset = content_start + 20
                    if payload_offset + 4 <= offset + 4 + submsg_len:
                        encaps = struct.unpack_from("<H", data, payload_offset)[0]
                        encaps_name = {0x0001: "CDR_BE", 0x0002: "CDR_LE", 0x0003: "PL_CDR_LE", 0x0004: "PL_CDR_BE"}.get(encaps, f"0x{encaps:04X}")
                        lines.append(f"    encapsulation={encaps_name}")

                        if encaps in (0x0003, 0x0004):  # PL_CDR
                            pl_data = data[payload_offset + 4:offset + 4 + submsg_len]
                            lines.append(decode_parameter_list(pl_data))

        if submsg_len == 0:
            break
        offset += 4 + submsg_len
        submsg_num += 1

    return "\n".join(lines)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", SPDP_PORT))

    mreq = socket.inet_aton(SPDP_MCAST) + socket.inet_aton("192.168.1.92")
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass

    print(f"Listening for SPDP on {SPDP_MCAST}:{SPDP_PORT} ...")
    print(f"(ESP32 sends every 30s, wait at least that long)")
    print()

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] {len(data)} bytes from {addr[0]}:{addr[1]}")
            print(decode_rtps(data, addr))
            print()
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
