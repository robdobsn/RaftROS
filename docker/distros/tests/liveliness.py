# Ask the router once a second which device-topic publishers the device holds.
# Usage: liveliness.py [queries]   Prints a summary line: present / missing / ids seen.
import os, sys, time, zenoh
# The device publisher to watch: RAFTROS_TOPIC (default any /raft/ device topic)
TOPIC = os.environ.get("RAFTROS_TOPIC", "/raft/")
TOPIC_TOKEN = TOPIC.replace("/", "%") + ("" if TOPIC.endswith("/") else "/")
n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
s = zenoh.open(zenoh.Config.from_json5('{"mode":"client","connect":{"endpoints":["tcp/127.0.0.1:7447"]}}'))
present = missing = 0; ids = set()
for _ in range(n):
    found = []
    for r in s.liveliness().get("@ros2_lv/0/**", timeout=2.0):
        try:
            k = str(r.ok.key_expr)
            if "/MP/" in k and TOPIC_TOKEN in k:
                found.append(k.split("/")[4])
        except Exception:
            pass
    ids.update(found)
    if found: present += 1
    else: missing += 1
    time.sleep(1)
print("LIVELINESS queries %d present %d missing %d publisher ids %s" % (n, present, missing, sorted(ids, key=int)))
s.close()
