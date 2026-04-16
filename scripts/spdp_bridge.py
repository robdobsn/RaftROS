#!/usr/bin/env python3
"""Bridge SPDP between local ROS2 daemon and a remote ESP32 via unicast.
Forwards local daemon's SPDP to ESP32, and ESP32's responses back locally.
Usage: python3 spdp_bridge.py <esp32_ip> [local_ip]
"""
import socket, struct, sys, select, time

ESP32_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.173"
LOCAL_IP = sys.argv[2] if len(sys.argv) > 2 else "0.0.0.0"
SPDP_PORT = 7400
MCAST = "239.255.0.1"

def main():
    # Socket to receive local daemon's SPDP multicast
    mcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    mcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try: mcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except: pass
    mcast_sock.bind(("", SPDP_PORT))
    mreq = socket.inet_aton(MCAST) + socket.inet_aton(LOCAL_IP)
    mcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    # Socket to send/receive unicast to/from ESP32
    uni_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    uni_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to ephemeral port for unicast
    uni_sock.bind(("", 0))

    print(f"SPDP bridge: relaying between local daemon and {ESP32_IP}:{SPDP_PORT}")
    print(f"Listening for local multicast on {MCAST}:{SPDP_PORT}")
    print()

    local_gps = set()  # guidPrefixes we've seen from local sources
    esp_gps = set()    # guidPrefixes from ESP32
    fwd_count = 0
    last_fwd = 0

    while True:
        readable, _, _ = select.select([mcast_sock, uni_sock], [], [], 0.5)

        for s in readable:
            data, addr = s.recvfrom(4096)
            if len(data) < 20 or data[:4] != b'RTPS':
                continue
            gp = data[8:20]

            if s is mcast_sock:
                # From local daemon or ESP32 unicast to port 7400
                if addr[0] == ESP32_IP:
                    # ESP32 response arrived on multicast socket
                    if gp not in esp_gps:
                        esp_gps.add(gp)
                        print(f"[NEW] ESP32 participant gp={gp.hex()}")
                    # Forward to local multicast so daemon sees it
                    mcast_sock.sendto(data, (MCAST, SPDP_PORT))
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] FWD {len(data)}B ESP32->local_mcast gp={gp.hex()[:16]}..")
                    continue

                local_gps.add(gp)
                now = time.time()
                # Forward at most once per second per unique message
                if now - last_fwd >= 0.5:
                    uni_sock.sendto(data, (ESP32_IP, SPDP_PORT))
                    fwd_count += 1
                    last_fwd = now
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] FWD {len(data)}B local({addr[0]})->ESP32 gp={gp.hex()[:16]}.. #{fwd_count}")

            elif s is uni_sock:
                # From ESP32 — inject into local multicast
                if addr[0] == ESP32_IP:
                    if gp not in esp_gps:
                        esp_gps.add(gp)
                        print(f"[NEW] ESP32 participant gp={gp.hex()}")

                    # Forward to local multicast so daemon sees it
                    mcast_sock.sendto(data, (MCAST, SPDP_PORT))
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] FWD {len(data)}B ESP32->local_mcast gp={gp.hex()[:16]}..")

                    # Also decode the message briefly
                    vendor = f"{data[6]:02x}.{data[7]:02x}"
                    print(f"       vendor={vendor} size={len(data)}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBridge stopped.")
