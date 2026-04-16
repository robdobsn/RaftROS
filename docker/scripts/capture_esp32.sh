#!/bin/bash
# capture_esp32.sh — Capture ESP32 <-> ROS 2 traffic on the host network
# Run this on the host (not in Docker) while ESP32 is running.
# Usage: ./capture_esp32.sh [duration_seconds]
#
# Captures all RTPS traffic on domain 0 involving the ESP32.

set -e

DURATION="${1:-30}"
OUTDIR="$(dirname "$0")/../captures/esp32"
mkdir -p "$OUTDIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "=== Capturing ESP32 RTPS traffic for ${DURATION}s ==="
echo "=== Domain 0: ports 7400, 7410, 7411 ==="

# pcap capture
sudo timeout "$DURATION" tcpdump -i any -w "${OUTDIR}/esp32_${TIMESTAMP}.pcap" \
    'udp and (port 7400 or port 7410 or port 7411)' &
PCAP_PID=$!

# Hex dump for quick look
sudo timeout "$DURATION" tcpdump -i any -XX -c 200 \
    'udp and (port 7400 or port 7410 or port 7411)' \
    > "${OUTDIR}/esp32_${TIMESTAMP}_hex.txt" 2>&1 &
HEX_PID=$!

wait $PCAP_PID 2>/dev/null || true
wait $HEX_PID 2>/dev/null || true

echo ""
echo "=== Capture complete ==="
echo "Files:"
ls -la "${OUTDIR}"/esp32_${TIMESTAMP}*

# Decode if tshark available
if command -v tshark &>/dev/null; then
    echo "=== Decoding with tshark ==="
    tshark -r "${OUTDIR}/esp32_${TIMESTAMP}.pcap" -V -Y "rtps" \
        > "${OUTDIR}/esp32_${TIMESTAMP}_decoded.txt" 2>/dev/null || true
    echo "  ${OUTDIR}/esp32_${TIMESTAMP}_decoded.txt"
fi
