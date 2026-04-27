#!/usr/bin/env python3
"""Reference sensor_msgs/Range publisher on /raft/range_1_29.

Publishes the *exact same* topic / type / QoS that the ESP32 firmware claims
to publish so that wire captures (tshark / wireshark RTPS dissector) can be
directly diffed to find discrepancies in our SEDP announcement or CDR
payload.

Usage:
    # Default: publish at 10 Hz with BEST_EFFORT / VOLATILE / KEEP_LAST 10
    python3 range_reference_pub.py

    # Change QoS / rate:
    python3 range_reference_pub.py --rate 5 --reliability reliable
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range


def make_qos(args) -> QoSProfile:
    qos = QoSProfile(depth=args.depth)
    qos.reliability = (ReliabilityPolicy.BEST_EFFORT
                       if args.reliability == 'best_effort'
                       else ReliabilityPolicy.RELIABLE)
    qos.durability = (DurabilityPolicy.VOLATILE
                      if args.durability == 'volatile'
                      else DurabilityPolicy.TRANSIENT_LOCAL)
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


class RangePub(Node):
    def __init__(self, args):
        super().__init__('range_reference_pub')
        self.pub = self.create_publisher(Range, args.topic, make_qos(args))
        self.period = 1.0 / args.rate
        self.timer = self.create_timer(self.period, self.tick)
        self.count = 0
        self.frame_id = args.frame_id
        self.get_logger().info(
            f'Publishing Range on {args.topic} @ {args.rate} Hz '
            f'(reliability={args.reliability}, durability={args.durability})')

    def tick(self):
        msg = Range()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED  # 1 — matches VL6180 device type
        msg.field_of_view = 0.436332          # ~25 deg
        msg.min_range = 0.01
        msg.max_range = 1.0
        # Sweep the range a bit so the stream is visually distinctive
        msg.range = 0.10 + 0.05 * math.sin(self.count * 0.1)
        self.pub.publish(msg)
        self.count += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/raft/range_1_29')
    p.add_argument('--frame-id', default='raft_range_1_29')
    p.add_argument('--rate', type=float, default=10.0)
    p.add_argument('--depth', type=int, default=10)
    p.add_argument('--reliability', choices=['best_effort', 'reliable'],
                   default='best_effort')
    p.add_argument('--durability', choices=['volatile', 'transient_local'],
                   default='volatile')
    args = p.parse_args()

    rclpy.init()
    node = RangePub(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
