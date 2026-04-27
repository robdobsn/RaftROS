#!/bin/bash
# compare_discovery.sh — Compare reference captures between FastDDS and CycloneDDS
# Run after capture_discovery.sh has been run for both RMWs.

set -e

CAPDIR="$(dirname "$0")/../captures"

echo "=========================================="
echo "  FastDDS vs CycloneDDS Discovery Compare"
echo "=========================================="
echo ""

for RMW in fastdds cyclonedds; do
    DIR="${CAPDIR}/${RMW}"
    if [ ! -f "${DIR}/discovery.pcap" ]; then
        echo "[$RMW] No capture found — run: ./capture_discovery.sh $RMW"
        continue
    fi

    echo "--- $RMW ---"
    echo "Node list: $(cat "${DIR}/node_list.txt" 2>/dev/null || echo 'N/A')"

    if command -v tshark &>/dev/null; then
        echo "Packet count: $(tshark -r "${DIR}/discovery.pcap" -Y "rtps" 2>/dev/null | wc -l)"
        echo ""
        echo "SPDP packets:"
        tshark -r "${DIR}/discovery.pcap" -Y "rtps.sm.id == 0x15 && rtps.sm.wrEntityId == 0x000100c2" \
            -T fields -e frame.number -e ip.src -e udp.srcport -e frame.len 2>/dev/null | head -5
        echo ""
        echo "SEDP publication packets:"
        tshark -r "${DIR}/discovery.pcap" -Y "rtps.sm.wrEntityId == 0x000003c2" \
            -T fields -e frame.number -e ip.src -e frame.len 2>/dev/null | head -10
        echo ""
        echo "ACKNACK packets:"
        tshark -r "${DIR}/discovery.pcap" -Y "rtps.sm.id == 0x06" \
            -T fields -e frame.number -e ip.src -e frame.len 2>/dev/null | head -10
    fi
    echo ""
done
