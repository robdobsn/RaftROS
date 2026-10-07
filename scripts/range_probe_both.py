#!/usr/bin/env python3
"""Dual typed + raw subscription probe with matched-publisher diagnostics."""
import signal
import sys
import time
import traceback

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

try:
    from rclpy.qos_event import (
        SubscriptionEventCallbacks,
    )
except Exception:
    SubscriptionEventCallbacks = None


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Both(Node):
    def __init__(self, topic: str):
        super().__init__('raft_range_probe_both')
        self.topic = topic
        self.typed_n = 0
        self.raw_n = 0
        self.evt_log = []

        def _on_incompat_qos(event):
            self.evt_log.append(f'INCOMPATIBLE_QOS: {event}')
            self.get_logger().error(f'INCOMPATIBLE_QOS: {event}')

        def _on_msg_lost(event):
            self.evt_log.append(f'MESSAGE_LOST: {event}')
            self.get_logger().warn(f'MESSAGE_LOST: {event}')

        def _on_liveliness(event):
            self.evt_log.append(f'LIVELINESS_CHANGED: {event}')
            self.get_logger().warn(f'LIVELINESS_CHANGED: {event}')

        def _on_deadline(event):
            self.evt_log.append(f'DEADLINE_MISSED: {event}')
            self.get_logger().warn(f'DEADLINE_MISSED: {event}')

        cbs = None
        if SubscriptionEventCallbacks is not None:
            try:
                cbs = SubscriptionEventCallbacks(
                    incompatible_qos=_on_incompat_qos,
                    liveliness=_on_liveliness,
                    deadline=_on_deadline,
                    message_lost=_on_msg_lost,
                )
            except Exception as e:
                self.get_logger().warn(f'event cbs unsupported: {e}')
                cbs = None

        self._sub_typed = self.create_subscription(
            Range, topic, self._typed_cb, sensor_qos(),
            event_callbacks=cbs)
        self._sub_raw = self.create_subscription(
            Range, topic, self._raw_cb, sensor_qos(), raw=True,
            event_callbacks=cbs)
        self._t0 = time.time()
        self._tmr = self.create_timer(2.0, self._status)
        self.get_logger().info(
            f'dual subs on {topic} (typed + raw)')

    def _typed_cb(self, msg):
        try:
            self.typed_n += 1
            frame_id = getattr(getattr(msg, 'header', None), 'frame_id', '?')
            rng = getattr(msg, 'range', float('nan'))
            if self.typed_n <= 3:
                self.get_logger().info(
                    f'TYPED #{self.typed_n} type={type(msg).__name__} '
                    f'frame_id={frame_id!r} range={rng}')
        except BaseException:
            self.get_logger().error(
                f'TYPED cb EXC: {traceback.format_exc()}')

    def _raw_cb(self, data: bytes):
        try:
            self.raw_n += 1
            if self.raw_n <= 3:
                self.get_logger().info(
                    f'RAW #{self.raw_n} len={len(data)} '
                    f'head={data[:16].hex()}')
        except BaseException:
            self.get_logger().error(
                f'RAW cb EXC: {traceback.format_exc()}')

    def _status(self):
        elapsed = time.time() - self._t0
        pubs = self.get_publishers_info_by_topic(self.topic)
        self.get_logger().info(
            f'[{elapsed:4.1f}s] typed={self.typed_n} raw={self.raw_n} '
            f'matched_pubs={len(pubs)}')
        for i, p in enumerate(pubs):
            try:
                th = p.topic_type_hash
                th_str = getattr(th, 'value', th)
            except Exception:
                th_str = '?'
            self.get_logger().info(
                f'  pub[{i}] node={p.node_name} type={p.topic_type} '
                f'hash={th_str} qos_rel={p.qos_profile.reliability} '
                f'qos_dur={p.qos_profile.durability}')


def main():
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    rclpy.init()
    node = Both('/raft/range_1_29')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
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


if __name__ == '__main__':
    main()
