# Hot-plug watch: once a second, the device's own view (attached devices, publishers)
# beside the router's view (the device's publisher liveliness tokens).
# A withdrawal that never reaches the router shows up as a token id that outlives
# its device.  Usage: hotplug_watch.py <seconds>
import json, os, sys, time, urllib.request, zenoh
# The device publisher to watch: RAFTROS_TOPIC (default any /raft/ device topic)
TOPIC = os.environ.get("RAFTROS_TOPIC", "/raft/")
TOPIC_TOKEN = TOPIC.replace("/", "%") + ("" if TOPIC.endswith("/") else "/")
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 180
s = zenoh.open(zenoh.Config.from_json5('{"mode":"client","connect":{"endpoints":["tcp/127.0.0.1:7447"]}}'))
end = time.time() + dur; last = None
while time.time() < end:
    try:
        st = json.load(urllib.request.urlopen("http://%s/api/rosstat" % os.environ.get("RAFTROS_DEVICE_IP", "192.168.86.230"), timeout=2))
        dev = "devices=%s pubs=%s pending=%s" % (st.get("devices"), st.get("pubs"), st.get("pending"))
    except Exception:
        dev = "device unreachable"
    ids = []
    for r in s.liveliness().get("@ros2_lv/0/**", timeout=1.5):
        try:
            k = str(r.ok.key_expr)
            if "/MP/" in k and TOPIC_TOKEN in k:
                ids.append(k.split("/")[4])
        except Exception:
            pass
    line = "%s | router publishers %s" % (dev, sorted(ids, key=int))
    if line != last:                       # print changes only
        print(time.strftime("%H:%M:%S"), line, flush=True); last = line
    time.sleep(1)
print(time.strftime("%H:%M:%S"), "final:", last, flush=True)
s.close()
