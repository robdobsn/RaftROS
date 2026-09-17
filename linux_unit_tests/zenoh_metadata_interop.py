import json
import os
from pathlib import Path
import queue
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import String
import zenoh


FIXTURE = str(Path(__file__).with_name("zenoh_codec_tests"))
TOPIC = "/raft_test/chatter"
ENDPOINT = "tcp/127.0.0.1:17447"


def spin_until(node, condition, description, action=None):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if condition():
            print(f"PASS: {description}")
            return
        if action is not None:
            action()
        rclpy.spin_once(node, timeout_sec=0.1)
    raise AssertionError(f"Timed out: {description}")


def fixture_endpoints(node, publishers):
    entries = (node.get_publishers_info_by_topic(TOPIC) if publishers
               else node.get_subscriptions_info_by_topic(TOPIC))
    return [entry for entry in entries
            if entry.node_name == "raft_fixture" and entry.node_namespace == "/raft_test"]


def main():
    package = Path(get_package_share_directory("rmw_zenoh_cpp")) / "package.xml"
    version = ET.parse(package).findtext("version")
    if version != "0.2.11":
        raise RuntimeError(f"Expected pinned rmw_zenoh_cpp 0.2.11, found {version}")
    print(f"Host metadata control: rmw_zenoh_cpp {version}, upstream Zenoh Python; no Raft transport")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ROS_DOMAIN_ID"] = "23"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "-1"
    os.environ["ZENOH_CONFIG_OVERRIDE"] = (
        f'connect/endpoints=[];listen/endpoints=["{ENDPOINT}"];'
        'scouting/multicast/enabled=false'
    )
    os.environ.pop("ZENOH_SESSION_CONFIG_URI", None)
    rclpy.init()
    observer = rclpy.create_node("metadata_observer")
    try:
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