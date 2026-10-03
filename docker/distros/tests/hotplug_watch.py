# Hot-plug watch: once a second, the device's own view (attached devices, publishers)
# beside the router's view (the device's range publisher liveliness tokens).
# A withdrawal that never reaches the router shows up as a token id that outlives
# its device.  Usage: hotplug_watch.py <seconds>
import json, sys, time, urllib.request, zenoh
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 180
s = zenoh.open(zenoh.Config.from_json5('{"mode":"client","connect":{"endpoints":["tcp/127.0.0.1:7447"]}}'))
end = time.time() + dur; last = None
while time.time() < end:
    try:
        st = json.load(urllib.request.urlopen("http://192.168.86.230/api/rosstat", timeout=2))
        dev = "devices=%s pubs=%s pending=%s" % (st.get("devices"), st.get("pubs"), st.get("pending"))
    except Exception:
        dev = "device unreachable"
    ids = []
    for r in s.liveliness().get("@ros2_lv/0/**", timeout=1.5):
        try:
            k = str(r.ok.key_expr)
            if "/MP/" in k and "raft_esp32/%raft%range_1_29/" in k:
                ids.append(k.split("/")[4])
        except Exception:
            pass
    line = "%s | router range publishers %s" % (dev, sorted(ids, key=int))
    if line != last:                       # print changes only
        print(time.strftime("%H:%M:%S"), line, flush=True); last = line
    time.sleep(1)
print(time.strftime("%H:%M:%S"), "final:", last, flush=True)
s.close()
