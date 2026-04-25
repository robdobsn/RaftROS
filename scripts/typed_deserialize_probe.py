"""Diagnostic: catch FastCDR deserialize exceptions that ros2 topic echo swallows.

When typed `ros2 topic echo` returns nothing despite `ros2 topic hz` working,
the receiver is rejecting the payload at the type-support layer (e.g. wrong
field count, message struct drift between distros). `ros2 topic echo` doesn't
print the exception. This script subscribes raw, hex-dumps the payload, then
calls rclpy.serialization.deserialize_message directly so the FastCDR
exception text is visible.

Used to diagnose the missing `variance` field in `sensor_msgs/Range` after
ROS 2 Jazzy added the field (Apr 2026).

Usage:
    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    python3 -u scripts/typed_deserialize_probe.py [topic] [msg_type]

Defaults: /raft/range_1_29 sensor_msgs/msg/Range
"""
import sys
import importlib
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message


def resolve_msg_type(name: str):
    # name like "sensor_msgs/msg/Range"
    pkg, sub, cls = name.split('/')
    mod = importlib.import_module(f"{pkg}.{sub}")
    return getattr(mod, cls)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/raft/range_1_29'
    msg_type_name = sys.argv[2] if len(sys.argv) > 2 else 'sensor_msgs/msg/Range'
    msg_type = resolve_msg_type(msg_type_name)

    rclpy.init()
    node = Node('typed_deserialize_probe')
    got = []

    def raw_cb(data):
        got.append(bytes(data))
        print(f"RAW bytes len={len(data)} hex={data.hex()}")
        try:
            msg = deserialize_message(data, msg_type)
            # Try to print something useful regardless of message type.
            summary = []
            if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                summary.append(f"frame_id={msg.header.frame_id!r}")
            for fname in ('range', 'temperature', 'illuminance', 'fluid_pressure',
                          'relative_humidity', 'data'):
                if hasattr(msg, fname):
                    summary.append(f"{fname}={getattr(msg, fname)}")
                    break
            print(f"  OK: {' '.join(summary) if summary else msg!r}")
        except Exception as e:
            print(f"  DESER FAIL: {e}")

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    node.create_subscription(msg_type, topic, lambda m: None, qos, raw=True)
    # Force callback to capture raw bytes (rclpy raw=True still calls our cb
    # with the deserialized message by default; replace the internal callback).
    node._subscriptions[-1]._callback = raw_cb

    end = time.time() + 6
    while time.time() < end and len(got) < 10:
        rclpy.spin_once(node, timeout_sec=0.5)
    print(f"got {len(got)} samples")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
