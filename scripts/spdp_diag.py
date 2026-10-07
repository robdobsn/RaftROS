#!/usr/bin/env python3
"""
SPDP diagnostic tool: join RTPS multicast, receive and decode SPDP announcements.
Usage: python3 spdp_diag.py [interface_ip]
Default interface: 0.0.0.0 (all interfaces)
"""

import socket
import struct
import sys
import time

SPDP_MCAST_ADDR = "239.255.0.1"
SPDP_MCAST_PORT = 7400

# RTPS Parameter IDs
PID_NAMES = {
    0x0000: "PAD",
    0x0001: "SENTINEL",
    0x0002: "PARTICIPANT_LEASE_DURATION",
    0x0005: "TOPIC_NAME",
    0x0007: "TYPE_NAME",
    0x0015: "PROTOCOL_VERSION",
    0x0016: "VENDORID",
    0x001A: "RELIABILITY",
    0x001D: "DURABILITY",
    0x002C: "USER_DATA",
    0x0031: "DEFAULT_UNICAST_LOCATOR",
    0x0032: "METATRAFFIC_UNICAST_LOCATOR",
    0x0033: "METATRAFFIC_MULTICAST_LOCATOR",
    0x0048: "DEFAULT_MULTICAST_LOCATOR",
    0x0050: "PARTICIPANT_GUID",
    0x0058: "BUILTIN_ENDPOINT_SET",
    0x0059: "PROPERTY_LIST",
    0x005A: "ENDPOINT_GUID",
}

SUBMSG_NAMES = {
    0x06: "ACKNACK",
    0x07: "HEARTBEAT",
    0x09: "INFO_TS",
    0x0E: "INFO_DST",
    0x15: "DATA",
}


def decode_locator(data):
    """Decode a 24-byte RTPS Locator_t from LE CDR."""
    if len(data) < 24:
        return "too short"
    kind = struct.unpack_from("<I", data, 0)[0]
    port = struct.unpack_from("<I", data, 4)[0]
    ip_bytes = data[20:24]
    ip_str = ".".join(str(b) for b in ip_bytes)
    kind_name = {1: "UDPv4", 2: "UDPv6"}.get(kind, f"kind={kind}")
    return f"{kind_name} {ip_str}:{port}"


def decode_guid(data):
    """Decode a 16-byte GUID (12 prefix + 4 entityId)."""
    prefix = data[:12].hex()
    entity = data[12:16].hex()
    return f"{prefix}|{entity}"


def decode_parameter_list(data):
    """Decode ParameterList entries from CDR-encoded data."""
    if len(data) < 4:
        return []
    # CDR encapsulation header
    encaps = struct.unpack_from(">H", data, 0)[0]
    encaps_name = {0x0001: "CDR_LE", 0x0003: "PL_CDR_LE",
                   0x0000: "CDR_BE", 0x0002: "PL_CDR_BE"}.get(encaps, f"0x{encaps:04X}")
    print(f"    Encapsulation: {encaps_name}")

    off = 4  # skip encaps header
    params = []
    while off + 4 <= len(data):
        pid = struct.unpack_from("<H", data, off)[0]
        plen = struct.unpack_from("<H", data, off + 2)[0]
        off += 4

        name = PID_NAMES.get(pid, f"0x{pid:04X}")

        if pid == 0x0001:  # SENTINEL
            params.append(("SENTINEL", ""))
            break

        if off + plen > len(data):
            params.append((name, f"truncated (need {plen}, have {len(data)-off})"))
            break

        val_data = data[off:off + plen]

        # Decode known parameter values
        if pid == 0x0015:  # PROTOCOL_VERSION
            val = f"{val_data[0]}.{val_data[1]}"
        elif pid == 0x0016:  # VENDORID
            val = f"{val_data[0]:02X}.{val_data[1]:02X}"
        elif pid == 0x0050:  # PARTICIPANT_GUID
            val = decode_guid(val_data)
        elif pid == 0x0058:  # BUILTIN_ENDPOINT_SET
            v = struct.unpack_from("<I", val_data, 0)[0]
            bits = []
            bit_names = {0: "PART_ANN", 1: "PART_DET", 2: "PUB_ANN", 3: "PUB_DET",
                        4: "SUB_ANN", 5: "SUB_DET"}
            for b, n in bit_names.items():
                if v & (1 << b):
                    bits.append(n)
            val = f"0x{v:08X} [{', '.join(bits)}]"
        elif pid in (0x0031, 0x0032, 0x0033, 0x0048):  # Locators
            val = decode_locator(val_data)
        elif pid == 0x0002:  # LEASE_DURATION
            sec = struct.unpack_from("<I", val_data, 0)[0]
            frac = struct.unpack_from("<I", val_data, 4)[0] if plen >= 8 else 0
            val = f"{sec}s (frac={frac})"
        elif pid == 0x005A:  # ENDPOINT_GUID
            val = decode_guid(val_data)
        elif pid in (0x0005, 0x0007):  # TOPIC_NAME, TYPE_NAME
            slen = struct.unpack_from("<I", val_data, 0)[0]
            val = val_data[4:4 + slen - 1].decode("utf-8", errors="replace")
        elif pid == 0x0059:  # PROPERTY_LIST
            val = f"{plen} bytes: {val_data[:min(32,plen)].hex()}"
        else:
            val = val_data.hex()

        params.append((name, val))
        off += plen

    return params


def decode_rtps_message(data, src_addr):
    """Decode an RTPS message."""
    if len(data) < 20:
        print(f"  [too short: {len(data)} bytes]")
        return

    # RTPS header
    proto = data[0:4]
    if proto != b'RTPS':
        print(f"  [not RTPS: {proto}]")
        return

    ver_major = data[4]
    ver_minor = data[5]
    vendor = f"{data[6]:02X}.{data[7]:02X}"
    guid_prefix = data[8:20].hex()

    print(f"  RTPS v{ver_major}.{ver_minor} vendor={vendor} guidPrefix={guid_prefix}")

    # Walk submessages
    off = 20
    while off + 4 <= len(data):
        submsg_id = data[off]
        flags = data[off + 1]
        submsg_len = struct.unpack_from("<H", data, off + 2)[0]

        name = SUBMSG_NAMES.get(submsg_id, f"0x{submsg_id:02X}")
        print(f"  [{name}] flags=0x{flags:02X} len={submsg_len}")

        content_start = off + 4
        content_end = content_start + submsg_len

        if submsg_id == 0x15 and submsg_len >= 20:  # DATA
            # extraFlags(2) + octetsToInlineQoS(2) + readerId(4) + writerId(4) + seqNum(8) = 20
            reader_id = data[content_start + 4:content_start + 8].hex()
            writer_id = data[content_start + 8:content_start + 12].hex()
            seq_hi = struct.unpack_from("<I", data, content_start + 12)[0]
            seq_lo = struct.unpack_from("<I", data, content_start + 16)[0]
            print(f"    reader={reader_id} writer={writer_id} seq={seq_hi}:{seq_lo}")

            # Serialized data starts after the 20-byte DATA header
            payload = data[content_start + 20:content_end]
            if len(payload) > 0:
                params = decode_parameter_list(payload)
                for pname, pval in params:
                    print(f"    PID {pname}: {pval}")

        if submsg_len == 0:
            break
        off = content_end


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "0.0.0.0"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", SPDP_MCAST_PORT))

    # Join multicast group
    mreq = socket.inet_aton(SPDP_MCAST_ADDR) + socket.inet_aton(iface)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print(f"Listening for SPDP on {SPDP_MCAST_ADDR}:{SPDP_MCAST_PORT} (iface={iface})")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            data, addr = sock.recvfrom(2048)
            ts = time.strftime("%H:%M:%S")
            print(f"\n[{ts}] {addr[0]}:{addr[1]} ({len(data)} bytes)")
            decode_rtps_message(data, addr)
    except KeyboardInterrupt:
        print("\nDone")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
