#!/usr/bin/env python3
"""Listen to a RaftROS Zenoh node with the real Zenoh stack.

Opens a Zenoh peer that accepts the device's TCP session, then reports the
liveliness tokens it declares (its node and its endpoints) and decodes the
samples it publishes.  Unlike tools/zenoh_router_stub.py this speaks to the
device through the actual Zenoh implementation, so it also demonstrates that
the firmware's wire format is accepted by Zenoh itself rather than only by our
own reading of the protocol.

    pip install eclipse-zenoh==1.8.0
    python3 zenoh_subscriber_demo.py --listen tcp/0.0.0.0:7447

For a ROS 2 demo run a real rmw_zenoh router instead
(`ros2 run rmw_zenoh_cpp rmw_zenohd`), point the device at it, and use the
ordinary `ros2 topic` tools.
"""

import argparse
import json
import struct
import sys
import time

import zenoh


def decode_sample(key, payload):
    """Summarise a sample using the type named in its key.

    A ROS 2 Zenoh key ends with the wire type and its hash, so the payload can
    be decoded without any prior knowledge of the topic.
    """
    data = bytes(payload)
    if len(data) < 8 or data[:4] != b"\x00\x01\x00\x00":
        return "%d bytes (not CDR)" % len(data)
    if "::Range_" in key:
        # header, radiation_type, field_of_view, min, max, range, variance -
        # variance was added in Jazzy and is last, so the reading precedes it
        if len(data) >= 12:
            return "range=%.4f m" % struct.unpack("<f", data[-8:-4])[0]
    if "::String_" in key:
        size = struct.unpack("<I", data[4:8])[0]
        if 0 < size <= len(data) - 8:
            return '"%s"' % data[8:8 + size - 1].decode("utf-8", "replace")
    return "%d bytes" % len(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listen", default="tcp/0.0.0.0:7447",
                        help="endpoint the device connects to")
    parser.add_argument("--key", default="**", help="key expression to subscribe to")
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()

    # Router mode: a Zenoh peer accepts the device's session and its samples,
    # but does not keep the liveliness tokens a ROS graph is discovered from -
    # in peer mode the node and its publishers are invisible even though data
    # flows.  This is the mode rmw_zenohd runs in.
    config = zenoh.Config.from_json5(json.dumps({
        "mode": "router",
        "listen": {"endpoints": [args.listen]},
        # Nothing else to talk to, and scouting would just add noise
        "scouting": {"multicast": {"enabled": False}},
    }))

    counts = {}
    with zenoh.open(config) as session:
        print("zenoh peer listening on %s" % args.listen, flush=True)

        def on_token(sample):
            kind = str(sample.kind)
            print("LIVELINESS %s\n    %s" % ("gone" if "DELETE" in kind.upper() else "alive",
                                             sample.key_expr), flush=True)

        def on_sample(sample):
            key = str(sample.key_expr)
            count = counts.get(key, 0) + 1
            counts[key] = count
            summary = decode_sample(key, sample.payload.to_bytes())
            if count <= 3 or count % 20 == 0:
                print("SAMPLE #%d %s\n    %s" % (count, summary, key), flush=True)

        # "@ros2_lv" is a verbatim chunk, so the pattern has to name it
        session.liveliness().declare_subscriber("@ros2_lv/**", on_token, history=True)
        session.declare_subscriber(args.key, on_sample)

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            time.sleep(0.2)

    print("samples per key: %s" % counts, flush=True)
    return 0 if counts else 1


if __name__ == "__main__":
    sys.exit(main())
