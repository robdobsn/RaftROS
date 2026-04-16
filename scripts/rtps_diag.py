#!/usr/bin/env python3
"""Listen on multiple RTPS ports and decode all traffic.
Usage: python3 rtps_diag.py [local_ip]
"""
import socket, struct, sys, time, select

LOCAL_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.92"
SPDP_MCAST = "239.255.0.1"
SPDP_PORT = 7400
META_PORT = 7410
USER_PORT = 7411

PID_NAMES = {
    0x0001: "SENTINEL", 0x0002: "LEASE_DUR", 0x0005: "DURABILITY",
    0x0007: "TYPE_NAME", 0x0015: "PROTO_VER", 0x0016: "VENDOR",
    0x001A: "RELIABILITY", 0x002F: "DEF_UNI_LOC", 0x0031: "META_UNI_LOC",
    0x0032: "META_MC_LOC", 0x0048: "DEF_MC_LOC", 0x0050: "PART_GUID",
    0x0058: "BUILTIN_EP", 0x0059: "PROPERTY_LIST", 0x005A: "EP_GUID",
}
SUBMSG_NAMES = {
    0x01: "ACKNACK", 0x02: "HEARTBEAT", 0x05: "INFO_DST",
    0x09: "INFO_TS", 0x15: "DATA", 0x12: "DATA_FRAG",
}

def decode_locator(d):
    if len(d) < 24: return "?"
    kind = struct.unpack_from("<I", d, 0)[0]
    port = struct.unpack_from("<I", d, 4)[0]
    if kind == 1:
        return f"{d[20]}.{d[21]}.{d[22]}.{d[23]}:{port}"
    return f"kind={kind} port={port}"

def decode_pl(data):
    parts = []
    off = 0
    while off + 4 <= len(data):
        pid, length = struct.unpack_from("<HH", data, off)
        off += 4
        if pid == 1: parts.append("SENTINEL"); break
        if off + length > len(data): parts.append(f"TRUNC@0x{pid:04X}"); break
        pd = data[off:off+length]
        name = PID_NAMES.get(pid, f"0x{pid:04X}")
        detail = ""
        if pid == 0x0015 and length >= 2: detail = f"={pd[0]}.{pd[1]}"
        elif pid == 0x0016 and length >= 2: detail = f"={pd[0]:02x}.{pd[1]:02x}"
        elif pid == 0x0050 and length >= 16: detail = f"={pd[:12].hex()}"
        elif pid == 0x0058 and length >= 4:
            v = struct.unpack_from("<I", pd, 0)[0]
            detail = f"=0x{v:08X}"
        elif pid in (0x002F, 0x0031, 0x0032, 0x0048):
            detail = f"={decode_locator(pd)}"
        elif pid == 0x0002 and length >= 8:
            s = struct.unpack_from("<I", pd, 0)[0]
            detail = f"={s}s"
        elif pid == 0x0007 and length >= 4:
            slen = struct.unpack_from("<I", pd, 0)[0]
            detail = f"=\"{pd[4:4+slen-1].decode('ascii','replace')}\""
        parts.append(f"{name}{detail}")
        off += length
        off = (off + 3) & ~3
    return " | ".join(parts)

def decode_msg(data):
    if len(data) < 20 or data[:4] != b'RTPS': return "not-RTPS"
    vendor = f"{data[6]:02x}.{data[7]:02x}"
    gp = data[8:20].hex()
    parts = [f"v{data[4]}.{data[5]} vnd={vendor} gp={gp}"]
    off = 20
    while off + 4 <= len(data):
        sid = data[off]; flags = data[off+1]
        slen = struct.unpack_from("<H", data, off+2)[0]
        sname = SUBMSG_NAMES.get(sid, f"0x{sid:02X}")
        extra = ""
        if sid == 0x15 and off + 4 + 20 <= off + 4 + slen:
            cs = off + 4
            rid = data[cs+4:cs+8].hex(); wid = data[cs+8:cs+12].hex()
            extra = f" r={rid} w={wid}"
            if flags & 0x04:
                po = cs + 20
                if po + 4 <= off + 4 + slen:
                    enc = struct.unpack_from("<H", data, po)[0]
                    if enc in (3, 4):
                        extra += f" PL:[{decode_pl(data[po+4:off+4+slen])}]"
        parts.append(f"{sname}(0x{flags:02X},{slen}){extra}")
        if slen == 0: break
        off += 4 + slen
    return " | ".join(parts)

def make_mcast_sock():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try: s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except: pass
    s.bind(("", SPDP_PORT))
    mreq = socket.inet_aton(SPDP_MCAST) + socket.inet_aton(LOCAL_IP)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    return s

def make_uni_sock(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try: s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except: pass
    try:
        s.bind(("", port))
    except OSError:
        print(f"WARNING: port {port} in use, skipping")
        return None
    return s

def main():
    spdp = make_mcast_sock()
    meta = make_uni_sock(META_PORT)
    user = make_uni_sock(USER_PORT)
    socks = {}
    for label, s in [("SPDP", spdp), ("META", meta), ("USER", user)]:
        if s is not None:
            socks[label] = s
    all_socks = list(socks.values())
    print(f"Listening: SPDP={SPDP_PORT} META={META_PORT} USER={USER_PORT} local={LOCAL_IP}")
    seen_gp = set()
    try:
        while True:
            readable, _, _ = select.select(all_socks, [], [], 1.0)
            for s in readable:
                label = [k for k, v in socks.items() if v is s][0]
                data, addr = s.recvfrom(4096)
                src = f"{addr[0]}:{addr[1]}"
                # Skip local sources
                if addr[0] in ("127.0.0.1", "10.255.255.254", "100.96.87.105", LOCAL_IP):
                    gp = data[8:20].hex() if len(data) >= 20 else "?"
                    if gp not in seen_gp:
                        seen_gp.add(gp)
                        print(f"[local] {label} {src} gp={gp} (suppressing further)")
                    continue
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] {label} {len(data)}B from {src}")
                print(f"  {decode_msg(data)}")
    except KeyboardInterrupt:
        print("\nDone.")

if __name__ == "__main__":
    main()
