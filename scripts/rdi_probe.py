#!/usr/bin/env python3
"""
Subscribe to /ros_discovery_info as a regular DDS topic to verify
the ESP-published sample deserializes correctly (bypassing the daemon
graph cache).
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from rmw_dds_common.msg import ParticipantEntitiesInfo

class Probe(Node):
    def __init__(self):
        super().__init__("rdi_probe")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            ParticipantEntitiesInfo, "ros_discovery_info",
            self.on_msg, qos)
        self.count = 0

    def on_msg(self, msg):
        self.count += 1
        gid_bytes = bytes(msg.gid.data)
        print(f"[{self.count}] gid={gid_bytes.hex()}")
        for nei in msg.node_entities_info_seq:
            print(f"   node ns='{nei.node_namespace}' name='{nei.node_name}'"
                  f"  R={len(nei.reader_gid_seq)} W={len(nei.writer_gid_seq)}")
        sys.stdout.flush()

def main():
    rclpy.init()
    n = Probe()
    end = n.get_clock().now().nanoseconds + 15 * 10**9
    while rclpy.ok() and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.5)
    print(f"DONE, got {n.count} samples")

if __name__ == "__main__":
    main()
