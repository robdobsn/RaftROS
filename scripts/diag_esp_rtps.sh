#!/usr/bin/env bash
# One-shot diagnostic: captures RTPS traffic from the ESP32 for N seconds,
# decodes the SEDP publication announcement and any user DATA, and prints a
# concise summary of what was actually on the wire.
#
# CRITICAL: The ESP32 only transmits user DATA once a matching ROS 2
# subscriber is live on the host. This script therefore launches an
# in-process subscriber (scripts/range_probe.py --raw) for the duration
# of the capture unless NO_SUB=1 is set. If you just want to see SPDP
# multicast, use NO_SUB=1.
#
# Usage:
#   ./diag_esp_rtps.sh [duration_seconds]
#   NO_SUB=1 ./diag_esp_rtps.sh 10    # skip subscriber launch
#
# Output pcap is written under /var/tmp/rtps_diag (WSL-friendly).
# Requires sudo for dumpcap capture. Assumes ROS 2 is sourceable at
# /opt/ros/jazzy/setup.bash (override with ROS_SETUP env var).

set -u

DURATION="${1:-20}"
IFACE="${IFACE:-}"
OUTDIR="${OUTDIR:-/var/tmp/rtps_diag}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
NO_SUB="${NO_SUB:-0}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

sudo mkdir -p "$OUTDIR" && sudo chmod 777 "$OUTDIR"
PCAP="$OUTDIR/esp.pcap"
DUMP="$OUTDIR/esp.txt"
SUB_LOG="$OUTDIR/sub.log"
# Remove any stale pcap so dumpcap (which may drop privs) can create it fresh
sudo rm -f "$PCAP"

# --- auto-detect iface on 192.168.1.x -----------------------------------
if [[ -z "$IFACE" ]]; then
    IFACE=$(ip -br a | awk '/192\.168\.1/{print $1; exit}')
fi
if [[ -z "$IFACE" ]]; then
    echo "ERROR: could not auto-detect interface on 192.168.1.x" >&2
    echo "  Set IFACE=<name> manually" >&2
    exit 1
fi
echo "[diag] iface=$IFACE duration=${DURATION}s outdir=$OUTDIR"

# --- source ROS 2 (harmless if already sourced) -------------------------
if [[ -f "$ROS_SETUP" ]]; then
    set +u          # ROS setup scripts reference unbound vars
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export FASTDDS_LOG_TO_STDOUT=1

# --- launch subscriber so ESP32 will actually transmit -----------------
SUB_PID=""
if [[ "$NO_SUB" != "1" ]]; then
    if [[ -f "$SCRIPT_DIR/range_probe.py" ]]; then
        echo "[diag] starting subscriber scripts/range_probe.py --raw (log: $SUB_LOG)"
        python3 "$SCRIPT_DIR/range_probe.py" --raw >"$SUB_LOG" 2>&1 &
        SUB_PID=$!
        trap '[[ -n "$SUB_PID" ]] && kill "$SUB_PID" 2>/dev/null || true' EXIT
        sleep 2   # give it time to match
    else
        echo "[diag] WARNING: range_probe.py not found; ESP32 may not transmit"
    fi
fi

# --- capture ------------------------------------------------------------
echo "[diag] capturing $DURATION s to $PCAP"
if ! sudo -n true 2>/dev/null; then
    echo "[diag] sudo will prompt for password…"
fi
sudo dumpcap -i "$IFACE" -f 'udp portrange 7400-7500' \
            -a "duration:$DURATION" -w "$PCAP"
TSHARK_RC=$?
echo "[diag] dumpcap exit=$TSHARK_RC"
if [[ ! -s "$PCAP" ]]; then
    echo "[diag] ERROR: capture empty" >&2
    exit 1
fi
sudo chown "$USER" "$PCAP" 2>/dev/null || true
sudo chmod 644 "$PCAP" 2>/dev/null || true

# --- subscriber log (first 20 lines) -----------------------------------
if [[ -s "$SUB_LOG" ]]; then
    echo
    echo "[diag] === subscriber log (first 20 lines) ==="
    head -20 "$SUB_LOG"
fi


# --- summarise ----------------------------------------------------------
echo "[diag] === packet counts ==="
tshark -r "$PCAP" -q -z io,phs 2>/dev/null | grep -E 'rtps|udp' | head -20

echo
echo "[diag] === SEDP publication DATA submessages (writerEntity 0xc2) ==="
# 0x000003c2 = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER
tshark -r "$PCAP" -Y 'rtps and rtps.sm.wrEntityId == 0x000003c2' \
    -T fields -e frame.number -e ip.src -e ip.dst -e rtps.sm.seqNumber 2>/dev/null \
    | head -10

echo
echo "[diag] === all RTPS submessages from ESP (192.168.1.173) ==="
tshark -r "$PCAP" -Y 'rtps and ip.src == 192.168.1.173' \
       -T fields -e frame.number -e ip.dst \
       -e rtps.sm.id -e rtps.sm.wrEntityId -e rtps.sm.rdEntityId \
       -e rtps.sm.seqNumber 2>/dev/null | head -40

echo
echo "[diag] === ESP submessage-kind histogram ==="
# 0x15=DATA 0x07=HEARTBEAT 0x06=ACKNACK 0x08=GAP 0x09=INFO_TS 0x0e=INFO_DST
tshark -r "$PCAP" -Y 'rtps and ip.src == 192.168.1.173' \
       -T fields -e rtps.sm.id 2>/dev/null \
       | tr ',' '\n' | sort | uniq -c | sort -rn

echo
echo "[diag] === ESP writerEntityIds seen ==="
tshark -r "$PCAP" -Y 'rtps and ip.src == 192.168.1.173' \
       -T fields -e rtps.sm.wrEntityId 2>/dev/null \
       | tr ',' '\n' | grep -v '^$' | sort | uniq -c | sort -rn

echo
echo "[diag] === full decode of first DATA submessage from ESP ==="
# Any DATA (rtps.sm.id == 0x15) from ESP, regardless of writer entity
tshark -r "$PCAP" \
       -Y 'rtps and ip.src == 192.168.1.173 and rtps.sm.id == 0x15' \
       -V 2>/dev/null | sed -n '/Real-Time Publish Subscribe/,/^Frame [0-9]/p' \
                      | head -400 > "$DUMP"
if [[ -s "$DUMP" ]]; then
    cat "$DUMP"
else
    echo "NONE — ESP32 sent no DATA submessages at all"
    echo "    All RTPS sources seen:"
    tshark -r "$PCAP" -Y 'rtps' -T fields -e ip.src 2>/dev/null | sort -u
fi

echo
echo "[diag] === host→ESP ACKNACK/HEARTBEAT summary (did reader match?) ==="
tshark -r "$PCAP" -Y 'rtps and ip.src == 192.168.1.28 and ip.dst == 192.168.1.173' \
       -T fields -e frame.number -e rtps.sm.id \
       -e rtps.sm.wrEntityId -e rtps.sm.rdEntityId 2>/dev/null | head -20

echo
echo "[diag] === host's SEDP subscription announce (topic/type it expects) ==="
# First host→ESP DATA on SEDP subscriptions writer (0x000004c2)
tshark -r "$PCAP" \
    -Y 'rtps and ip.src == 192.168.1.28 and rtps.sm.wrEntityId == 0x000004c2 and rtps.sm.id == 0x15' \
    -V 2>/dev/null | sed -n '/Real-Time Publish Subscribe/,/^Frame [0-9]/p' \
    | grep -E 'topic_?name|type_?name|PID_TOPIC|PID_TYPE|parameterId|reliability|durability' \
    | head -40

echo
echo "[diag] === ESP autopub SEDP announce (if any) — writer 0x0000NNc2 topic/type ==="
# Any ESP→host DATA on an SEDP pubs writer (0x000003c2 OR a custom entity ending c2 that isn't SPDP/part-msg)
tshark -r "$PCAP" \
    -Y 'rtps and ip.src == 192.168.1.173 and rtps.sm.id == 0x15 and rtps.sm.wrEntityId != 0x000100c2' \
    -V 2>/dev/null | sed -n '/Real-Time Publish Subscribe/,/^Frame [0-9]/p' \
    | head -200

echo
echo "[diag] === Wireshark Expert warnings (malformed/unexpected) ==="
tshark -r "$PCAP" -Y 'rtps and (_ws.expert or _ws.malformed)' \
       -T fields -e frame.number -e _ws.expert.message 2>/dev/null | head -20

echo
echo "[diag] full dump saved to $DUMP"
echo "[diag] pcap saved to $PCAP  (open with: wireshark $PCAP)"
