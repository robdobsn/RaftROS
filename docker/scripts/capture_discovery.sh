#!/bin/bash
# capture_discovery.sh — Capture a complete ROS 2 discovery session
# Usage: ./capture_discovery.sh [fastdds|cyclonedds]
#
# Runs inside the Docker container. Captures RTPS packets while a
# minimal node starts up and is discovered by ros2 node list.

set -e
source /opt/ros/humble/setup.bash

RMW="${1:-fastdds}"
case "$RMW" in
    fastdds)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml
        ;;
    cyclonedds)
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        unset FASTRTPS_DEFAULT_PROFILES_FILE
        ;;
    *)
        echo "Usage: $0 [fastdds|cyclonedds]"
        exit 1
        ;;
esac

OUTDIR="/workspace/captures/${RMW}"
mkdir -p "$OUTDIR"

echo "=== RMW: $RMW_IMPLEMENTATION ==="
echo "=== Stopping any existing daemon ==="
ros2 daemon stop 2>/dev/null || true
sleep 1

# Use a dedicated domain to avoid interference
export ROS_DOMAIN_ID=55
SPDP_PORT=$((7400 + 250 * $ROS_DOMAIN_ID))
META_PORT=$((SPDP_PORT + 10))  # participant 0
USER_PORT=$((SPDP_PORT + 11))

echo "=== Domain $ROS_DOMAIN_ID: SPDP=$SPDP_PORT META=$META_PORT USER=$USER_PORT ==="

# Start tcpdump in background — capture ALL RTPS traffic
echo "=== Starting packet capture ==="
tcpdump -i lo -w "${OUTDIR}/discovery.pcap" \
    "udp and (port $SPDP_PORT or portrange ${META_PORT}-$((META_PORT+20)))" &
TCPDUMP_PID=$!
sleep 1

# Also do a hex dump for quick analysis
tcpdump -i lo -XX -c 200 \
    "udp and (port $SPDP_PORT or portrange ${META_PORT}-$((META_PORT+20)))" \
    > "${OUTDIR}/discovery_hex.txt" 2>&1 &
HEXDUMP_PID=$!

# Start a simple node in background
echo "=== Starting talker node ==="
ros2 run demo_nodes_cpp talker &
TALKER_PID=$!
sleep 3

# Discover it
echo "=== Running ros2 node list ==="
ros2 node list --no-daemon --spin-time 8 | tee "${OUTDIR}/node_list.txt"
echo ""
echo "=== Running ros2 topic list ==="
ros2 topic list --no-daemon --spin-time 5 | tee "${OUTDIR}/topic_list.txt"

# Let it run a bit more for complete exchange
sleep 2

# Cleanup
echo "=== Stopping ==="
kill $TALKER_PID 2>/dev/null || true
sleep 1
kill $TCPDUMP_PID 2>/dev/null || true
kill $HEXDUMP_PID 2>/dev/null || true
wait 2>/dev/null

# Generate tshark text decode
echo "=== Decoding with tshark ==="
tshark -r "${OUTDIR}/discovery.pcap" -V -Y "rtps" > "${OUTDIR}/discovery_decoded.txt" 2>/dev/null || true

# Summary
echo ""
echo "=== Capture complete ==="
echo "Files in ${OUTDIR}/:"
ls -la "${OUTDIR}/"
echo ""
echo "Key files:"
echo "  discovery.pcap          — Open in Wireshark for full decode"
echo "  discovery_decoded.txt   — tshark text decode of RTPS messages"
echo "  discovery_hex.txt       — Raw hex dump"
echo "  node_list.txt           — ros2 node list output"
