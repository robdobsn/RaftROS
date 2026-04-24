#!/usr/bin/env python3
"""
RaftROS Range auto-publish probe.

Subscribes to /raft/range_1_29 with BEST_EFFORT / VOLATILE QoS
(matching the ESP32 autopub default FastSensor profile) and:

  1. Prints every successfully-deserialized message.
  2. On Fast CDR / deserialize failure, catches the exception and
     falls back to a raw-bytes subscription so we can inspect the
     wire payload directly.
  3. Handles Ctrl-C and rclpy ExternalShutdownException cleanly
     (no double-shutdown stack trace).

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 scripts/range_probe.py [--topic /raft/range_1_29] [--raw]
"""

import argparse
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Range


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class TypedProbe(Node):
    def __init__(self, topic: str):
        super().__init__('raft_range_probe')
        self.count = 0
        self._sub = self.create_subscription(
            Range, topic, self._cb, sensor_qos())
        self.get_logger().info(
            f'typed subscription on {topic} (BEST_EFFORT/VOLATILE), waiting...')

    def _cb(self, msg: Range):
        self.count += 1
        self.get_logger().info(
            f'#{self.count} stamp={msg.header.stamp.sec}.'
            f'{msg.header.stamp.nanosec:09d} '
            f'frame_id={msg.header.frame_id!r} radiation={msg.radiation_type} '
            f'fov={msg.field_of_view:.3f} min={msg.min_range:.3f} '
            f'max={msg.max_range:.3f} range={msg.range:.4f}')


class RawProbe(Node):
    """Fallback: subscribe raw (no deserialization) and dump hex."""
    def __init__(self, topic: str):
        super().__init__('raft_range_probe_raw')
        self.count = 0
        # raw=True gives us bytes objects (includes 4-byte encap header).
        self._sub = self.create_subscription(
            Range, topic, self._cb, sensor_qos(), raw=True)
        self.get_logger().info(
            f'RAW subscription on {topic} — hex dump per message')

    def _cb(self, data: bytes):
        self.count += 1
        hex_bytes = ' '.join(f'{b:02x}' for b in data)
        self.get_logger().info(
            f'#{self.count} len={len(data)} bytes=[{hex_bytes}]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/raft/range_1_29')
    ap.add_argument('--raw', action='store_true',
                    help='Skip typed subscription, dump raw bytes only')
    args = ap.parse_args()

    # Install SIGTERM handler so `timeout` produces a clean exit.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        rclpy.init()
    except Exception as e:
        print(f'rclpy.init failed: {e}', file=sys.stderr)
        return 1

    node = RawProbe(args.topic) if args.raw else TypedProbe(args.topic)
    rc = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f'\n!! subscription callback raised: {type(e).__name__}: {e}',
              file=sys.stderr)
        rc = 2
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == '__main__':
    sys.exit(main())
