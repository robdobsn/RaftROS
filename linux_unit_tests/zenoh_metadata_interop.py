import argparse
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Range
from std_msgs.msg import String
import zenoh


FIXTURE = str(Path(__file__).with_name("zenoh_codec_tests"))
TOPIC = "/raft_test/chatter"
RANGE_TOPIC = "/raft_test/range"
RANGE_FRAME_ID = "raft_range_1_29"
ENDPOINT = "tcp/127.0.0.1:17447"


def spin_until(node, condition, description, action=None, executor=None):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if condition():
            print(f"PASS: {description}")
            return
        if action is not None:
            action()
        rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
    raise AssertionError(f"Timed out: {description}")


def fixture_endpoints(node, publishers, topic=TOPIC):
    entries = (node.get_publishers_info_by_topic(topic) if publishers
               else node.get_subscriptions_info_by_topic(topic))
    return [entry for entry in entries
            if entry.node_name == "raft_fixture" and entry.node_namespace == "/raft_test"]


def expected_range(sequence):
    if not 1 <= sequence <= 64:
        raise AssertionError(f"Range fixture sequence outside bounds: {sequence}")
    stamp_ms = 1234 + (sequence - 1) * 250
    message = Range()
    message.header.stamp.sec = stamp_ms // 1000
    message.header.stamp.nanosec = (stamp_ms % 1000) * 1000000
    message.header.frame_id = RANGE_FRAME_ID
    message.radiation_type = Range.INFRARED
    message.field_of_view = 0.0
    message.min_range = 0.0
    message.max_range = 2.0
    message.range = (0.0, 0.184, 1.0, 2.0)[(sequence - 1) % 4]
    message.variance = 0.0
    return message


def canonical_range_fixture(payload):
    padding_start = 16 + len(RANGE_FRAME_ID.encode("utf-8")) + 1 + 1
    floats_start = (padding_start + 3) & ~3
    if len(payload) != floats_start + 5 * 4:
        raise AssertionError(f"Unexpected Range fixture CDR length: {len(payload)}")
    return payload[:padding_start] + bytes(floats_start - padding_start) + payload[floats_start:]


def validate_tcp_samples(received, publish_range):
    previous = 0
    for message, info in received:
        sequence = info["publication_sequence_number"]
        expected = expected_range(sequence) if publish_range else String(data="from_raft_tcp")
        actual_bytes = serialize_message(message)
        expected_bytes = serialize_message(expected)
        if publish_range:
            actual_bytes = canonical_range_fixture(actual_bytes)
            expected_bytes = canonical_range_fixture(expected_bytes)
        if actual_bytes != expected_bytes or sequence <= previous or info["source_timestamp"] != 0:
            raise AssertionError(f"Incorrect payload or attachment at sequence {sequence}: {message}")
        previous = sequence


def installed_type_hash(package, name):
    metadata = Path(get_package_share_directory(package)) / "msg" / f"{name}.json"
    description = json.loads(metadata.read_text())
    type_name = f"{package}/msg/{name}"
    return next(entry["hash_string"] for entry in description["type_hashes"] if entry["type_name"] == type_name)


def check_range_cdr(probe):
    reference = canonical_range_fixture(serialize_message(expected_range(1)))
    padded = bytearray(reference)
    padded[33:36] = b"\xa5\xa5\xa5"
    if canonical_range_fixture(bytes(padded)) != reference:
        raise AssertionError("Range comparison must ignore only the known alignment padding")
    for offset in (0, 4, 8, 12, 16, 32, 36, 40, 44, 48, 52, 55):
        corrupted = bytearray(reference)
        corrupted[offset] ^= 1
        if canonical_range_fixture(bytes(corrupted)) == reference:
            raise AssertionError(f"Range byte comparison missed field corruption at {offset}")
    for malformed in (reference[:-1], reference + b"\0"):
        try:
            canonical_range_fixture(malformed)
        except AssertionError:
            pass
        else:
            raise AssertionError("Range byte comparison must reject truncation and trailing bytes")
    for sequence in (1, 2, 3, 4, 5, 64):
        payload = bytes.fromhex(subprocess.check_output(
            [probe, "--range-payload", str(sequence)], text=True, timeout=5
        ).strip())
        expected = expected_range(sequence)
        expected_bytes = canonical_range_fixture(serialize_message(expected))
        if payload != expected_bytes:
            raise AssertionError(f"Range CDR differs from native ROS at sequence {sequence}")
        if canonical_range_fixture(serialize_message(deserialize_message(payload, Range))) != expected_bytes:
            raise AssertionError("Native Range deserialization differs from expected fields")
    for invalid in ("0", "65", "-1", "4294967296", "1junk"):
        result = subprocess.run([probe, "--range-payload", invalid], capture_output=True, timeout=5)
        if result.returncode != 2 or result.stdout:
            raise AssertionError(f"Invalid Range fixture sequence accepted: {invalid}")
    print("PASS: reused Range serializer matches native CDR fields/length with zeroed alignment padding")


def test_tcp_publish(observer, publish_range=False):
    message_type = Range if publish_range else String
    topic = RANGE_TOPIC if publish_range else TOPIC
    expected_type = "sensor_msgs/msg/Range" if publish_range else "std_msgs/msg/String"
    expected_hash = installed_type_hash("sensor_msgs", "Range") if publish_range else installed_type_hash("std_msgs", "String")
    received = []
    raw_received = []
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subscription = observer.create_subscription(message_type, topic, lambda message, info: received.append((message, info)), qos)
    raw_subscription = None
    if publish_range:
        raw_subscription = observer.create_subscription(Range, topic,
            lambda payload, info: raw_received.append((payload, info)), qos, raw=True)
    probe = str(Path(__file__).with_name("zenoh_session_probe"))
    process = None
    late_process = None
    try:
        if publish_range:
            check_range_cdr(probe)
        process = subprocess.Popen([probe, "127.0.0.1", "17447", "12000", "--publish-range" if publish_range else "--publish"],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        spin_until(observer, lambda: len(fixture_endpoints(observer, True, topic)) == 1,
                   "native graph discovers publisher over Raft-owned TCP")
        entry = fixture_endpoints(observer, True, topic)[0]
        if (entry.topic_type != expected_type or entry.qos_profile.depth != 5 or
                entry.qos_profile.reliability != ReliabilityPolicy.BEST_EFFORT or
                entry.qos_profile.durability != DurabilityPolicy.VOLATILE or
                entry.qos_profile.history != HistoryPolicy.KEEP_LAST):
            raise AssertionError("Incorrect native endpoint type/QoS")
        spin_until(observer, lambda: len(received) >= 5, "native ROS receives Raft CDR samples over own TCP")
        late_process = subprocess.Popen([sys.executable, __file__, "--late-range-observer" if publish_range else "--late-observer"],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        late_output, _ = late_process.communicate(timeout=15)
        print(late_output, end="")
        if late_process.returncode:
            raise AssertionError(f"Late ROS process exited {late_process.returncode}")
        if publish_range:
            spin_until(observer, lambda: len(raw_received) >= 5, "raw Range samples received for native byte comparison")
        spin_until(observer, lambda: not fixture_endpoints(observer, True, topic), "Raft TCP publisher token withdrawn")
        spin_until(observer, lambda: ("raft_fixture", "/raft_test") not in observer.get_node_names_and_namespaces(),
                   "Raft TCP node token withdrawn")
        output, _ = process.communicate(timeout=15)
        print(output, end="")
        if process.returncode:
            raise AssertionError(f"Raft probe exited {process.returncode}")
        validate_tcp_samples(received, publish_range)
        if publish_range:
            for payload, info in raw_received:
                if payload != canonical_range_fixture(serialize_message(expected_range(info["publication_sequence_number"]))):
                    raise AssertionError("Range wire payload differs from native serialization")
            print("PASS: typed/raw Range preserves scaling, timestamps, variance and CDR fields; reference padding normalized")
        identities = [json.loads(line) for line in output.splitlines() if line.startswith('{')]
        if len(identities) != 1 or bytes.fromhex(identities[0]["publisher_gid"]) != bytes(entry.endpoint_gid):
            raise AssertionError("Raft session identity does not match ROS graph GID")
        if identities[0]["type_hash"] != expected_hash:
            raise AssertionError("Raft type hash does not match installed ROS type description")
        print(f"PASS: {expected_type} hash verified against installed ROS type description: {expected_hash}")
        print("PASS: own TCP, ROS tokens, GID, attachment and CDR; no upstream client library")
    finally:
        if late_process is not None and late_process.poll() is None:
            late_process.kill()
            late_output, _ = late_process.communicate(timeout=5)
            print(late_output, end="")
        if process is not None and process.poll() is None:
            process.kill()
            output, _ = process.communicate(timeout=5)
            print(output, end="")
        observer.destroy_subscription(subscription)
        if raw_subscription is not None:
            observer.destroy_subscription(raw_subscription)


def test_tcp_restart():
    probe = str(Path(__file__).with_name("zenoh_session_probe"))
    context = None
    observer = None
    executor = None
    process = None
    seen_gids = set()
    try:
        for round_number, failure in enumerate(("publisher", "peer", "publisher"), start=1):
            if observer is None:
                context = Context()
                rclpy.init(context=context)
                observer = rclpy.create_node("restart_observer", context=context)
                executor = SingleThreadedExecutor(context=context)
            received = []
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            subscription = observer.create_subscription(Range, RANGE_TOPIC,
                lambda message, info: received.append((message, info)), qos)
            try:
                process = subprocess.Popen([probe, "127.0.0.1", "17447", "12000", "--publish-range"],
                                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                spin_until(observer, lambda: len(fixture_endpoints(observer, True, RANGE_TOPIC)) == 1,
                           f"restart round {round_number}: exactly one named Range publisher", executor=executor)
                entry = fixture_endpoints(observer, True, RANGE_TOPIC)[0]
                gid = bytes(entry.endpoint_gid)
                if gid in seen_gids:
                    raise AssertionError("Restart reused an old publisher GID")
                seen_gids.add(gid)
                spin_until(observer, lambda: len(received) >= 3,
                           f"restart round {round_number}: fresh Range samples received", executor=executor)
                validate_tcp_samples(received, True)
                if received[0][1]["publication_sequence_number"] != 1:
                    raise AssertionError("Existing subscriber did not receive restarted sequence one")
                if failure == "publisher":
                    process.kill()
                    output, _ = process.communicate(timeout=5)
                    if process.returncode != -signal.SIGKILL:
                        raise AssertionError("Publisher did not terminate through the requested abrupt kill")
                    spin_until(observer, lambda: not fixture_endpoints(observer, True, RANGE_TOPIC),
                               f"restart round {round_number}: abrupt loss removes remote publisher", executor=executor)
                    spin_until(observer, lambda: ("raft_fixture", "/raft_test") not in observer.get_node_names_and_namespaces(),
                               f"restart round {round_number}: abrupt loss removes remote node", executor=executor)
                else:
                    observer.destroy_subscription(subscription)
                    subscription = None
                    executor.shutdown()
                    executor = None
                    observer.destroy_node()
                    observer = None
                    context.shutdown()
                    context = None
                    output, _ = process.communicate(timeout=6)
                    if process.returncode != 1 or "Session probe failed:" not in output:
                        raise AssertionError("Peer loss did not terminate the probe with a bounded session failure")
                    print("PASS: native ROS peer shutdown terminates own-socket publisher without hanging")
                print(output, end="")
                identities = [json.loads(line) for line in output.splitlines() if line.startswith('{')]
                if (len(identities) != 1 or bytes.fromhex(identities[0]["publisher_gid"]) != gid or
                        identities[0]["type_hash"] != installed_type_hash("sensor_msgs", "Range")):
                    raise AssertionError("Restarted identity/hash differs from native graph")
                process = None
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.kill()
                    output, _ = process.communicate(timeout=5)
                    print(output, end="")
                    process = None
                if subscription is not None and observer is not None:
                    observer.destroy_subscription(subscription)
    finally:
        if executor is not None:
            executor.shutdown()
        if observer is not None:
            observer.destroy_node()
        if context is not None:
            context.shutdown()
    print("PASS: three native Range restart rounds, new GIDs and sequence reset; recovery is explicit process restart")


def main():
    parser = argparse.ArgumentParser()
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--tcp-session", action="store_true")
    modes.add_argument("--tcp-publish", action="store_true")
    modes.add_argument("--tcp-range", action="store_true")
    modes.add_argument("--tcp-restart", action="store_true")
    modes.add_argument("--late-observer", action="store_true")
    modes.add_argument("--late-range-observer", action="store_true")
    args = parser.parse_args()
    late_observer = args.late_observer or args.late_range_observer
    tcp_test = args.tcp_session or args.tcp_publish or args.tcp_range or args.tcp_restart
    package = Path(get_package_share_directory("rmw_zenoh_cpp")) / "package.xml"
    version = ET.parse(package).findtext("version")
    if version != "0.2.11":
        raise RuntimeError(f"Expected pinned rmw_zenoh_cpp 0.2.11, found {version}")
    if tcp_test:
        print(f"Raft TCP test against native rmw_zenoh_cpp {version}", flush=True)
    elif late_observer:
        print(f"Independent late ROS observer using rmw_zenoh_cpp {version}", flush=True)
    else:
        print(f"Host metadata control: rmw_zenoh_cpp {version}, upstream Zenoh Python; no Raft transport")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ROS_DOMAIN_ID"] = "23"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "-1"
    os.environ["ZENOH_CONFIG_OVERRIDE"] = (
        f'connect/endpoints=[];listen/endpoints=["{ENDPOINT}"];'
        'scouting/multicast/enabled=false'
    )
    if tcp_test:
        os.environ["ZENOH_CONFIG_OVERRIDE"] += ';transport/link/tx/lease=4000;transport/link/tx/keep_alive=4'
    if late_observer:
        os.environ["ZENOH_CONFIG_OVERRIDE"] = (
            f'mode="client";connect/endpoints=["{ENDPOINT}"];listen/endpoints=[];'
            'scouting/multicast/enabled=false'
        )
    os.environ.pop("ZENOH_SESSION_CONFIG_URI", None)
    if args.tcp_restart:
        test_tcp_restart()
        return
    rclpy.init()
    observer = rclpy.create_node("late_observer" if late_observer else "metadata_observer")
    try:
        if late_observer:
            message_type = Range if args.late_range_observer else String
            topic = RANGE_TOPIC if args.late_range_observer else TOPIC
            received = []
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            observer.create_subscription(message_type, topic, lambda message, info: received.append((message, info)), qos)
            spin_until(observer, lambda: len(fixture_endpoints(observer, True, topic)) == 1,
                       "late independent ROS process discovers Raft publisher")
            spin_until(observer, lambda: len(received) >= 2, "late independent ROS process receives live samples")
            validate_tcp_samples(received, args.late_range_observer)
            return
        if args.tcp_publish or args.tcp_range:
            test_tcp_publish(observer, args.tcp_range)
            return
        if args.tcp_session:
            probe = str(Path(__file__).with_name("zenoh_session_probe"))
            subprocess.run([probe, "127.0.0.1", "17447", "12000"], check=True, timeout=30)
            print("PASS: native peer accepted Raft-owned session; no application declarations or data yet")
            return
        config = zenoh.Config.from_json5(json.dumps({
            "mode": "client",
            "connect": {"endpoints": [ENDPOINT]},
            "scouting": {"multicast": {"enabled": False}},
        }))
        with zenoh.open(config) as session:
            session_id = str(session.zid())
            fields = subprocess.check_output(
                [FIXTURE, "--fixture", session_id], text=True, timeout=5
            ).splitlines()
            if len(fields) != 7 or fields[0] != version:
                raise AssertionError("C++ fixture profile differs from host RMW")
            _, node_key, publisher_key, subscription_key, topic_key, publisher_hex, subscription_hex = fields
            publisher_gid = bytes.fromhex(publisher_hex)
            subscription_gid = bytes.fromhex(subscription_hex)
            if len(publisher_gid) != 16 or len(subscription_gid) != 16:
                raise AssertionError("C++ fixture must generate 16-byte endpoint identities")
            with session.liveliness().declare_token(node_key):
                publisher_token = session.liveliness().declare_token(publisher_key)
                subscription_token = session.liveliness().declare_token(subscription_key)
                spin_until(observer, lambda: len(fixture_endpoints(observer, True)) == 1 and
                           len(fixture_endpoints(observer, False)) == 1,
                           "C++ publisher/subscription tokens attributed to /raft_test/raft_fixture")
                for entry in fixture_endpoints(observer, True) + fixture_endpoints(observer, False):
                    if (entry.topic_type != "std_msgs/msg/String" or
                            entry.qos_profile.reliability != ReliabilityPolicy.BEST_EFFORT or
                            entry.qos_profile.durability != DurabilityPolicy.VOLATILE or
                            entry.qos_profile.history != HistoryPolicy.KEEP_LAST or
                            entry.qos_profile.depth != 5):
                        raise AssertionError(f"Unexpected native endpoint metadata: {entry}")
                print("PASS: native RMW decodes type and canonical sensor QoS")
                if (bytes(fixture_endpoints(observer, True)[0].endpoint_gid) != publisher_gid or
                        bytes(fixture_endpoints(observer, False)[0].endpoint_gid) != subscription_gid):
                    raise AssertionError("Raft-generated endpoint GIDs differ from native RMW graph")
                print("PASS: Raft-generated publisher and subscription GIDs match native RMW graph")

                received = []
                qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                observer.create_subscription(String, TOPIC, lambda message, info: received.append((message, info)), qos)
                payload = serialize_message(String(data="from_cpp_metadata"))
                sequence = 0

                def publish_sample():
                    nonlocal sequence
                    sequence += 1
                    attachment = bytes.fromhex(subprocess.check_output(
                        [FIXTURE, "--attachment", session_id, str(sequence)], text=True, timeout=5
                    ).strip())
                    if len(attachment) != 33 or attachment[16] != 16 or attachment[17:] != publisher_gid:
                        raise AssertionError("Attachment must use Raft-generated publisher GID")
                    session.put(topic_key, payload, attachment=attachment)

                spin_until(observer, lambda: bool(received), "native typed String receives C++ key/attachment", publish_sample)
                message, info = received[0]
                if (message.data != "from_cpp_metadata" or info["publication_sequence_number"] < 1 or
                    info["publication_sequence_number"] > sequence or info["source_timestamp"] != 0):
                    raise AssertionError("Native sample metadata differs from C++ attachment")
                print("PASS: attachment sequence/source time decoded; callback GID unavailable in this rclpy")

                incoming = queue.Queue(maxsize=16)

                def on_sample(sample):
                    if deserialize_message(sample.payload.to_bytes(), String).data == "from_ros":
                        try:
                            incoming.put_nowait(True)
                        except queue.Full:
                            pass

                with session.declare_subscriber(topic_key, on_sample):
                    publisher = observer.create_publisher(String, TOPIC, qos)
                    spin_until(observer, lambda: not incoming.empty(), "ROS publisher reaches declared C++ subscription key",
                               lambda: publisher.publish(String(data="from_ros")))
                    observer.destroy_publisher(publisher)

                publisher_token.undeclare()
                subscription_token.undeclare()
                spin_until(observer, lambda: not fixture_endpoints(observer, True) and
                           not fixture_endpoints(observer, False), "endpoint tokens withdraw without removing observer subscription")
            spin_until(observer, lambda: ("raft_fixture", "/raft_test") not in observer.get_node_names_and_namespaces(),
                       "node token removal reaches native graph")
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    print("PASS: metadata and Raft-generated GIDs interoperate; CDR and session still supplied by host libraries")


if __name__ == "__main__":
    main()