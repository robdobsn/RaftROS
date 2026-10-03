# Raw Zenoh queries at the device's /raft_esp32/devices Trigger service through the
# local router: a proper request, an empty one, one over the device's 512 B request
# limit and one over its 2 kB receive cap.  The caller checks the session survived.
import time, zenoh
KEY = "0/raft_esp32/devices/std_srvs::srv::dds_::Trigger_/RIHS01_eeff2cd6fa5ad9d27cdf4dec64818317839b62f212a91e6b5304b634b2062c5f"
s = zenoh.open(zenoh.Config.from_json5('{"mode":"client","connect":{"endpoints":["tcp/127.0.0.1:7447"]}}'))
cases = [("proper 5 B", b"\x00\x01\x00\x00\x00"), ("empty 0 B", b""),
         ("oversized 900 B", b"\x00\x01\x00\x00" + bytes(896)), ("huge 3000 B", b"\x00\x01\x00\x00" + bytes(2996))]
for name, payload in cases:
    t0 = time.time(); got = []
    for r in s.get(KEY, payload=payload, attachment=bytes(33), timeout=8.0):
        try:
            got.append("OK %d B" % len(r.ok.payload.to_bytes()))
        except Exception:
            try:
                got.append("ERR %r" % r.err.payload.to_string())
            except Exception as e:
                got.append("reply? %r" % (e,))
    print("RAW %-16s -> %s (%.2f s)" % (name, got if got else "no reply", time.time() - t0), flush=True)
s.close()
