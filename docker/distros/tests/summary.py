# Summarise a traffic.jsonl (+ counts.jsonl) run.  Usage: summary.py <dir>
import json, sys, os
d = sys.argv[1]
rows = [json.loads(l) for l in open(os.path.join(d, "traffic.jsonl"))]
keys = ("dev", "rng", "ping", "pget", "pset")
fails = [(r["i"], k) for r in rows for k in keys if r[k] == 0]
dumps = [r for r in rows if r["dump"] != "-"]; dumpFails = [r["i"] for r in dumps if r["dump"] == "0"]
stats = [r["stat"] for r in rows if r["stat"]]
heaps = [s["heapFreeB"] for s in stats]
n = len(heaps); w = max(1, min(60, n // 4))
print("rounds %d, calls %d, call failures %d %s, dumps %d failed %d" % (len(rows), len(rows) * len(keys), len(fails), fails[:8], len(dumps), len(dumpFails)))
if stats:
    first, last = stats[0], stats[-1]
    print("heap first/last-window mean %.0f / %.0f (delta %+.0f B), min %d" % (sum(heaps[:w]) / w, sum(heaps[-w:]) / w, sum(heaps[-w:]) / w - sum(heaps[:w]) / w, min(s["heapMinB"] for s in stats)))
    print("sessions %s -> %s, connectFails %s, loopMaxUs %s, svc accepted %s completed %s timedOut %s, rosstat missing %d" % (
        first.get("sessions"), last.get("sessions"), last.get("connectFails"), last.get("loopMaxUs"),
        last.get("svcAccepted"), last.get("svcCompleted"), last.get("svcTimedOut"), len(rows) - len(stats)))
cp = os.path.join(d, "counts.jsonl")
if os.path.exists(cp):
    c = [json.loads(l) for l in open(cp)]
    if c:
        print("counter minutes %d, range/min %d..%d, chatter/min %d..%d" % (len(c), min(x["range"] for x in c), max(x["range"] for x in c),
              min(x["chatter"] for x in c), max(x["chatter"] for x in c)))
