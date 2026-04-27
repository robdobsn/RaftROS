#!/usr/bin/env bash
# Capture RTPS traffic and produce a focused SEDP-publication comparison
# between a reference ROS 2 publisher on the host and the ESP32 firmware.
#
# Usage:
#   ./capture_rtps.sh ref    # publish reference + capture ~15s
#   ./capture_rtps.sh esp    # capture ~15s of ESP32 traffic (needs subscriber)
#   ./capture_rtps.sh diff   # summarise + diff the two captures
#
# WSL2 notes:
#   * pcaps go in /var/tmp (sudo dumpcap drops privs and can't write /tmp
#     or $HOME).
#   * Uses `sudo dumpcap` directly (tshark capture path is broken under WSL2).

set -u

IFACE="${IFACE:-}"
DURATION="${DURATION:-15}"
OUTDIR="${OUTDIR:-/var/tmp/rtps_cmp}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FILTER="udp portrange 7400-7500"

if [[ -z "$IFACE" ]]; then
    IFACE=$(ip -br a | awk '/192\.168\.1/{print $1; exit}')
fi
if [[ -z "$IFACE" ]]; then
    echo "ERROR: could not auto-detect interface on 192.168.1.x (set IFACE=)" >&2
    exit 1
fi

sudo mkdir -p "$OUTDIR" && sudo chmod 777 "$OUTDIR"

# ---------------------------------------------------------------------------
# Extract a single SEDP publication DATA (writerEntity 0x000003c2) from a
# pcap and emit a compact, sortable "PID=value" breakdown. This is what we
# actually want to compare — the full tshark -V output contains GUIDs,
# counters, timestamps etc that are unique per run and drown the signal.
# ---------------------------------------------------------------------------
summarise_sedp() {
    local pcap="$1"
    local label="$2"
    local src_ip="$3"
    local decoded="${pcap%.pcap}.sedp.txt"
    # NOTE: Adding `rtps.sm.id == 0x15` AND'd with `rtps.sm.wrEntityId` yields
    # zero hits in tshark because those sub-fields live in distinct submessages
    # within one RTPS packet and display-filter expressions do not cross-match
    # per-submessage. The writer entity 0x000003c2 is only ever used in DATA
    # submessages, so filtering on writer alone is sufficient (heartbeats from
    # the same writer would match too, but they have no parameterList and
    # produce no PID lines).
    tshark -r "$pcap" \
       -Y "rtps and rtps.sm.wrEntityId == 0x000003c2 and ip.src == $src_ip" \
       -V 2>/dev/null \
       > "$decoded"

    # PID summary: extract "[0-9a-f]+: PID_..." lines, uniq.
    local pidlist="${pcap%.pcap}.sedp.pids.txt"
    grep -Eo 'PID_[A-Z_]+|parameterId: 0x[0-9a-fA-F]+' "$decoded" \
        | sort -u > "$pidlist"

    echo "=== $label: SEDP publication PID list ($pidlist) ==="
    cat "$pidlist"
    echo
    echo "=== $label: first SEDP publication DATA full decode ==="
    head -200 "$decoded"
    echo
    echo "=== $label: first USER DATA payload (raw bytes) ==="
    # User writer entities are typically 0x00NN1003 (publisher with app_key).
    # Grab first non-builtin DATA.
    tshark -r "$pcap" \
        -Y "rtps and ip.src == $src_ip and rtps.sm.wrEntityId > 0x00000500" \
        -T fields -e rtps.sm.wrEntityId -e rtps.issueData 2>/dev/null \
        | head -3
    echo
}

case "${1:-}" in
  ref)
    PCAP="$OUTDIR/ref.pcap"
    sudo rm -f "$PCAP"

    # Source ROS so rclpy is found
    if [[ -f "$ROS_SETUP" ]]; then
        set +u; source "$ROS_SETUP"; set -u
    fi
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    # Disable shared memory so that the publisher <-> local subscriber SEDP
    # discovery DATA is visible on the wire (otherwise Fast DDS uses SHM for
    # intra-host participants and nothing is captured).
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    # SEDP is reliable: publishers only send DATA once at discovery, then
    # HEARTBEAT/ACKNACK thereafter. If we start the publisher before the
    # capture, we miss the DATA that holds the PID list.
    # Capture on 'any' so loopback traffic is included: Fast DDS routes
    # intra-host SEDP traffic via lo even with SHM disabled.
    sudo dumpcap -i any -f "$FILTER" -a "duration:$DURATION" -w "$PCAP" &
    DUMP_PID=$!
    sleep 1

    echo "[+] Starting reference publisher"
    python3 "$SCRIPT_DIR/range_reference_pub.py" \
        --reliability best_effort --durability volatile \
        > "$OUTDIR/ref_pub.log" 2>&1 &
    PUB_PID=$!

    echo "[+] Starting local subscriber to solicit SEDP DATA via preemptive ACKNACK"
    # A subscriber on the same host triggers the publisher to announce itself
    # via SEDP DATA (rather than stopping at HEARTBEAT-only steady state).
    python3 -c "
import rclpy, time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Range
rclpy.init()
n=rclpy.create_node('ref_sub_probe')
q=QoSProfile(depth=10); q.reliability=ReliabilityPolicy.BEST_EFFORT; q.durability=DurabilityPolicy.VOLATILE
n.create_subscription(Range,'/raft/range_1_29',lambda m:None,q)
t=time.time()
while time.time()-t < $DURATION + 2:
    rclpy.spin_once(n, timeout_sec=0.1)
" > "$OUTDIR/ref_sub.log" 2>&1 &
    SUB_PID=$!

    trap "kill $PUB_PID $SUB_PID 2>/dev/null || true" EXIT
    wait $DUMP_PID
    sudo chown "$USER" "$PCAP" 2>/dev/null || true
    sudo chmod 644 "$PCAP"
    echo "[+] ref capture: $PCAP"
    echo "[+] publisher log: $OUTDIR/ref_pub.log (first 10 lines):"
    head -10 "$OUTDIR/ref_pub.log"
    ;;

  esp)
    PCAP="$OUTDIR/esp.pcap"
    sudo rm -f "$PCAP"

    # Launch a subscriber so ESP32 transmits.
    SUB_PID=""
    if [[ -f "$SCRIPT_DIR/range_probe.py" ]]; then
        if [[ -f "$ROS_SETUP" ]]; then
            set +u; source "$ROS_SETUP"; set -u
        fi
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
        python3 "$SCRIPT_DIR/range_probe.py" --raw \
            > "$OUTDIR/esp_sub.log" 2>&1 &
        SUB_PID=$!
        trap "[[ -n \"$SUB_PID\" ]] && kill $SUB_PID 2>/dev/null || true" EXIT
        sleep 2
    fi

    echo "[+] Capturing $DURATION s to $PCAP on iface $IFACE"
    sudo dumpcap -i "$IFACE" -f "$FILTER" -a "duration:$DURATION" -w "$PCAP"
    sudo chown "$USER" "$PCAP" 2>/dev/null || true
    sudo chmod 644 "$PCAP"
    echo "[+] esp capture: $PCAP"
    ;;

  diff)
    for side in ref esp; do
      if [[ ! -s "$OUTDIR/$side.pcap" ]]; then
        echo "missing $OUTDIR/$side.pcap — run '$0 $side' first" >&2
        exit 1
      fi
    done

    # Auto-detect source IPs: the reference publisher runs on the host
    # (192.168.1.28 in this setup), the ESP is the other participant. We
    # identify each by picking the source of the SEDP publication DATA
    # (writerEntity 0x000003c2, submessage 0x15) in each pcap.
    REF_IP=$(tshark -r "$OUTDIR/ref.pcap" \
                   -Y 'rtps and rtps.sm.wrEntityId == 0x000003c2' \
                   -T fields -e ip.src 2>/dev/null | sort -u \
                   | grep -v '^192.168.1.173$' | head -1)
    ESP_IP=192.168.1.173
    [[ -z "$REF_IP" ]] && REF_IP="${REF_IP_OVERRIDE:-192.168.1.28}"

    echo "REF src=$REF_IP  ESP src=$ESP_IP"
    echo

    summarise_sedp "$OUTDIR/ref.pcap" REF "$REF_IP"  > "$OUTDIR/ref.summary.txt"
    summarise_sedp "$OUTDIR/esp.pcap" ESP "$ESP_IP"  > "$OUTDIR/esp.summary.txt"

    echo "================ REF SEDP publication PIDs ================"
    cat "${OUTDIR}/ref.sedp.pids.txt" 2>/dev/null
    echo
    echo "================ ESP SEDP publication PIDs ================"
    cat "${OUTDIR}/esp.sedp.pids.txt" 2>/dev/null
    echo
    echo "================ PIDs present ONLY in REF (i.e. we're missing) ================"
    comm -23 "${OUTDIR}/ref.sedp.pids.txt" "${OUTDIR}/esp.sedp.pids.txt" 2>/dev/null
    echo
    echo "================ PIDs present ONLY in ESP (i.e. we emit extra) ================"
    comm -13 "${OUTDIR}/ref.sedp.pids.txt" "${OUTDIR}/esp.sedp.pids.txt" 2>/dev/null
    echo
    echo "Full summaries:"
    echo "  $OUTDIR/ref.summary.txt"
    echo "  $OUTDIR/esp.summary.txt"
    echo
    echo "To compare full SEDP decodes:"
    echo "  diff -u $OUTDIR/ref.sedp.txt $OUTDIR/esp.sedp.txt | less"

    # -------------------------------------------------------------------
    # SAMPLE payload diff: find the first Range sample emitted by each
    # side and decode the XCDR1 Range_ struct for a field-by-field diff.
    # The REF publisher's writer entity is picked automatically by finding
    # the first rtps.issueData payload from a host-side user-entity.
    # -------------------------------------------------------------------
    echo
    echo "================ Range sample CDR diff ================"
    REF_WR=$(tshark -r "$OUTDIR/ref.pcap" \
        -Y 'rtps.issueData and ip.src == 127.0.0.1' \
        -T fields -e rtps.sm.wrEntityId 2>/dev/null | head -1 | cut -d, -f1)
    ESP_WR=0x00011003
    REF_PL=$(tshark -r "$OUTDIR/ref.pcap" -Y "rtps.sm.wrEntityId == $REF_WR" \
        -T fields -e rtps.issueData 2>/dev/null | head -1 | tr -d ',')
    ESP_PL=$(tshark -r "$OUTDIR/esp.pcap" -Y "rtps.sm.wrEntityId == $ESP_WR" \
        -T fields -e rtps.issueData 2>/dev/null | head -1 | tr -d ',')
    python3 - "$REF_PL" "$ESP_PL" <<'PY'
import sys, struct
def parse(hexstr, label):
    try:
        b = bytes.fromhex(hexstr)
    except Exception:
        print(f"{label}: could not parse hex")
        return None
    print(f"-- {label}: {len(b)} bytes  raw={b.hex()}")
    sec, nsec = struct.unpack_from('<iI', b, 0)
    strlen = struct.unpack_from('<I', b, 8)[0]
    frame_id = b[12:12+strlen].rstrip(b'\0').decode('latin1','replace')
    off = 12 + strlen
    rad = b[off]; off += 1
    while off % 4: off += 1
    fov,mn,mx,rng = struct.unpack_from('<4f', b, off); off += 16
    print(f"   stamp={sec}.{nsec:09d}  frame_id={frame_id!r} (len={strlen})")
    print(f"   radiation={rad}  fov={fov:.4f}  min={mn:.4f}  max={mx:.4f}  range={rng:.4f}")
    print(f"   consumed={off}  trailing={b[off:].hex() or '(none)'}")
    return dict(stamp=(sec,nsec), frame_id=frame_id, radiation=rad,
                fov=fov, min=mn, max=mx, range=rng, consumed=off,
                total=len(b), trailing=b[off:].hex())
ref = parse(sys.argv[1] if len(sys.argv)>1 else '', 'REF')
esp = parse(sys.argv[2] if len(sys.argv)>2 else '', 'ESP')
if ref and esp:
    print()
    print("-- field differences --")
    for k in ('frame_id','radiation','fov','min','max','total','trailing'):
        if ref.get(k) != esp.get(k):
            print(f"   {k}: REF={ref[k]!r}  ESP={esp[k]!r}")
PY
    ;;

  *)
    echo "Usage: $0 {ref|esp|diff}" >&2
    echo "  ref  : launch ref publisher + capture -> $OUTDIR/ref.pcap" >&2
    echo "  esp  : capture ESP32 traffic (auto-launches subscriber) -> $OUTDIR/esp.pcap" >&2
    echo "  diff : compare SEDP publication PIDs between the two captures" >&2
    exit 1
    ;;
esac
