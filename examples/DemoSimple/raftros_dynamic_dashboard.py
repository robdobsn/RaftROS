#!/usr/bin/env python3
"""Dynamic RaftROS device dashboard.

Watches the ROS 2 graph for RaftROS auto-published topics, subscribes to any
importable message type it finds, and renders a compact live terminal table.

Typical use:
    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    python3 -u examples/DemoSimple/raftros_dynamic_dashboard.py
"""

from __future__ import annotations

import argparse
import importlib
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


Formatter = Callable[[Any], str]


@dataclass
class TopicState:
    topic: str
    type_name: str
    msg_type: Any | None = None
    subscription: Any | None = None
    latest: Any | None = None
    latest_time: float | None = None
    count: int = 0
    status: str = "waiting"


def resolve_msg_type(type_name: str) -> Any:
    """Resolve a ROS type string like sensor_msgs/msg/Range to a Python class."""
    parts = type_name.split("/")
    if len(parts) != 3:
        raise ValueError(f"expected pkg/msg/Type, got {type_name!r}")
    pkg, submodule, class_name = parts
    module = importlib.import_module(f"{pkg}.{submodule}")
    return getattr(module, class_name)


def make_qos(reliability: str, depth: int) -> QoSProfile:
    if reliability == "reliable":
        reliability_policy = ReliabilityPolicy.RELIABLE
    else:
        reliability_policy = ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(
        reliability=reliability_policy,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _fmt_float(value: Any, precision: int = 3) -> str:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(value) or math.isinf(value):
        return str(value)
    return f"{value:.{precision}f}"


def _header_frame(msg: Any) -> str:
    header = getattr(msg, "header", None)
    frame_id = getattr(header, "frame_id", "")
    return f" frame={frame_id}" if frame_id else ""


def format_range(msg: Any) -> str:
    return (
        f"range={_fmt_float(getattr(msg, 'range', None))} m"
        f" min={_fmt_float(getattr(msg, 'min_range', None))}"
        f" max={_fmt_float(getattr(msg, 'max_range', None))}"
        f"{_header_frame(msg)}"
    )


def format_imu(msg: Any) -> str:
    acc = getattr(msg, "linear_acceleration", None)
    gyro = getattr(msg, "angular_velocity", None)
    return (
        "accel=("
        f"{_fmt_float(getattr(acc, 'x', None), 2)},"
        f"{_fmt_float(getattr(acc, 'y', None), 2)},"
        f"{_fmt_float(getattr(acc, 'z', None), 2)}) "
        "gyro=("
        f"{_fmt_float(getattr(gyro, 'x', None), 2)},"
        f"{_fmt_float(getattr(gyro, 'y', None), 2)},"
        f"{_fmt_float(getattr(gyro, 'z', None), 2)})"
        f"{_header_frame(msg)}"
    )


def format_illuminance(msg: Any) -> str:
    return f"illuminance={_fmt_float(getattr(msg, 'illuminance', None), 2)} lux{_header_frame(msg)}"


def format_temperature(msg: Any) -> str:
    return f"temperature={_fmt_float(getattr(msg, 'temperature', None), 2)} C{_header_frame(msg)}"


def format_relative_humidity(msg: Any) -> str:
    value = getattr(msg, "relative_humidity", None)
    try:
        value = float(value) * 100.0
        text = f"{value:.1f}%"
    except (TypeError, ValueError):
        text = str(value)
    return f"relative_humidity={text}{_header_frame(msg)}"


def format_fluid_pressure(msg: Any) -> str:
    return f"fluid_pressure={_fmt_float(getattr(msg, 'fluid_pressure', None), 1)} Pa{_header_frame(msg)}"


def format_scalar(msg: Any) -> str:
    return f"data={getattr(msg, 'data', None)}"


def format_string(msg: Any) -> str:
    text = str(getattr(msg, "data", ""))
    text = text.replace("\n", " ")
    if len(text) > 90:
        text = text[:87] + "..."
    return f"data={text!r}"


def format_default(msg: Any) -> str:
    text = repr(msg).replace("\n", " ")
    if len(text) > 120:
        text = text[:117] + "..."
    return text


FORMATTERS: dict[str, Formatter] = {
    "sensor_msgs/msg/Range": format_range,
    "sensor_msgs/msg/Imu": format_imu,
    "sensor_msgs/msg/Illuminance": format_illuminance,
    "sensor_msgs/msg/Temperature": format_temperature,
    "sensor_msgs/msg/RelativeHumidity": format_relative_humidity,
    "sensor_msgs/msg/FluidPressure": format_fluid_pressure,
    "std_msgs/msg/Float32": format_scalar,
    "std_msgs/msg/Int32": format_scalar,
    "std_msgs/msg/Bool": format_scalar,
    "std_msgs/msg/String": format_string,
}


class RaftRosDashboard(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("raftros_dynamic_dashboard")
        self.args = args
        self.qos = make_qos(args.qos, args.depth)
        self.states: dict[str, TopicState] = {}
        self.last_graph_refresh = 0.0

    def refresh_graph(self) -> None:
        now = time.monotonic()
        if now - self.last_graph_refresh < self.args.graph_period:
            return
        self.last_graph_refresh = now

        graph_topics = self.get_topic_names_and_types()
        discovered: dict[str, list[str]] = {}
        for topic, type_names in graph_topics:
            if not self._matches_topic(topic):
                continue
            if self.args.include_chatter is False and topic == "/chatter":
                continue
            discovered[topic] = list(type_names)

        for topic in sorted(set(self.states) - set(discovered)):
            self._remove_topic(topic)

        for topic, type_names in sorted(discovered.items()):
            if topic in self.states:
                if self.states[topic].type_name not in type_names:
                    self._remove_topic(topic)
                else:
                    continue
            if type_names:
                self._add_topic(topic, type_names)

    def _matches_topic(self, topic: str) -> bool:
        if self.args.all_raft_topics:
            return topic == "/chatter" or topic.startswith(self.args.namespace.rstrip("/") + "/")
        return topic.startswith(self.args.namespace.rstrip("/") + "/")

    def _add_topic(self, topic: str, type_names: list[str]) -> None:
        state = TopicState(topic=topic, type_name=type_names[0])
        for type_name in type_names:
            try:
                msg_type = resolve_msg_type(type_name)
            except Exception as exc:
                state.type_name = type_name
                state.status = f"unsupported type: {exc}"
                continue

            state.type_name = type_name
            state.msg_type = msg_type
            state.status = "waiting"

            def callback(msg: Any, topic_name: str = topic) -> None:
                current = self.states.get(topic_name)
                if current is None:
                    return
                current.latest = msg
                current.latest_time = time.monotonic()
                current.count += 1
                current.status = "ok"

            try:
                state.subscription = self.create_subscription(
                    msg_type, topic, callback, self.qos)
            except Exception as exc:
                state.status = f"subscribe failed: {exc}"
            break

        self.states[topic] = state

    def _remove_topic(self, topic: str) -> None:
        state = self.states.pop(topic, None)
        if state and state.subscription is not None:
            try:
                self.destroy_subscription(state.subscription)
            except Exception:
                pass

    def render(self) -> None:
        if not self.args.no_clear:
            print("\033[2J\033[H", end="")

        print("RaftROS live devices")
        print()
        print(f"namespace={self.args.namespace} qos={self.args.qos} depth={self.args.depth}")
        print("Plug or unplug supported I2C devices; rows update as ROS topics appear/disappear.")
        print()

        if not self.states:
            print("No matching topics discovered yet.")
            return

        topic_width = max(24, min(42, max(len(s.topic) for s in self.states.values())))
        type_width = max(24, min(36, max(len(s.type_name) for s in self.states.values())))
        header = (
            f"{'topic':<{topic_width}}  "
            f"{'type':<{type_width}}  "
            f"{'age':>7}  "
            f"{'count':>6}  latest"
        )
        print(header)
        print("-" * min(len(header) + 60, 160))

        now = time.monotonic()
        for state in sorted(self.states.values(), key=lambda item: item.topic):
            age = "-"
            latest = state.status
            if state.latest_time is not None:
                age = f"{now - state.latest_time:5.1f}s"
                latest = summarize_message(state.type_name, state.latest)
            print(
                f"{shorten(state.topic, topic_width):<{topic_width}}  "
                f"{shorten(state.type_name, type_width):<{type_width}}  "
                f"{age:>7}  "
                f"{state.count:6d}  "
                f"{shorten(latest, self.args.value_width)}"
            )


def summarize_message(type_name: str, msg: Any) -> str:
    formatter = FORMATTERS.get(type_name, format_default)
    try:
        return formatter(msg)
    except Exception as exc:
        return f"format failed: {exc}"


def shorten(text: str, width: int) -> str:
    if len(text) <= width:
        return text
    if width <= 3:
        return text[:width]
    return text[: width - 3] + "..."


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Discover RaftROS auto-published topics and print live values.")
    parser.add_argument(
        "--namespace",
        default="/raft",
        help="ROS topic namespace prefix to watch, default: /raft")
    parser.add_argument(
        "--qos",
        choices=("best_effort", "reliable"),
        default="best_effort",
        help="Subscription reliability, default: best_effort")
    parser.add_argument(
        "--depth",
        type=int,
        default=10,
        help="KEEP_LAST history depth, default: 10")
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=4.0,
        help="Terminal refresh rate, default: 4")
    parser.add_argument(
        "--graph-period",
        type=float,
        default=1.0,
        help="Seconds between ROS graph scans, default: 1")
    parser.add_argument(
        "--value-width",
        type=int,
        default=90,
        help="Maximum displayed value width, default: 90")
    parser.add_argument(
        "--include-chatter",
        action="store_true",
        help="Also show /chatter if --all-raft-topics is used")
    parser.add_argument(
        "--all-raft-topics",
        action="store_true",
        help="Watch the namespace plus /chatter")
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear the terminal between renders")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        rclpy.init()
    except Exception as exc:
        print(f"rclpy.init failed: {exc}", file=sys.stderr)
        return 1

    node = RaftRosDashboard(args)
    rc = 0
    render_period = 1.0 / max(args.refresh_hz, 0.1)
    next_render = 0.0

    try:
        while rclpy.ok():
            node.refresh_graph()
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_render:
                node.render()
                next_render = now + render_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f"\nDashboard failed: {type(exc).__name__}: {exc}", file=sys.stderr)
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


if __name__ == "__main__":
    sys.exit(main())
