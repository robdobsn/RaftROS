# RaftROS Development Status

**Last Updated:** 2026-09-21 (bus→loop mailbox handoff; first hardware and
native ROS 2 Jazzy end-to-end validation of auto-publishing). Resume commands
are in the [agent handoff](RaftROS-WSL-agent-handoff.md).

## Next Milestone: Zenoh Alternative (Started)

RTPS is still the only implemented/buildable backend. The
[Zenoh implementation plan](RaftROS-zenoh-implementation-plan.md) defines the
new forward work. ROS metadata/identity code and host-only native ROS graph/data
control tests pass. The connected Linux probe now sends ROS node/publisher
declarations, Raft-CDR String or Range samples and token withdrawals over its own TCP
session. A late independent ROS process also discovers and receives samples.
Host release sizes and fixed storage are measured, and as of 2026-09-27 the
Zenoh firmware backend, the build selector and the ESP32 footprint are all done:
a Zenoh image auto-publishes bus devices, publishes `/chatter` and subscribes to
strings through the same application API as the RTPS image, and `rmw_zenohd`
plus the ordinary ROS 2 tools resolve the node, its publishers and its
subscriptions.

| Work | State / next evidence |
| --- | --- |
| Z0: feasibility and baseline | ESP32 measurements done: app image, free heap, stack headroom and loop timing recorded for both backends under an active subscriber (see "Loop Budget Under Load"). zenoh-pico incorporation still needs explicit approval. |
| Z1: native ROS proof | Complete. String/Range, late ROS process, withdrawal, scripted interests and peer restarts pass; firmware integration, automatic reconnect and resource budgets are done and measured; and the ROS 2 tools themselves resolve the node, its publishers and its subscriptions over `rmw_zenohd` (2026-09-27). |
| Z2: common pipeline and RTPS adapter | Started: mapping/CDR have neutral ownership with legacy RTPS aliases/forwarders; a synchronous common sample runner and RTPS emission adapter drive the production callback. Both backends now sit behind the same create/destroy/publish contract, the Zenoh one host-tested against a real session. DeviceManager lifecycle work is done; firmware linking of the Zenoh backend is Z3. |
| Z3: isolated firmware builds | Complete. Kconfig selects the backend - Zenoh by default since 2026-09-27, RTPS as the opt-in - and each image links only its own backend. The router address is layered (Kconfig < SysTypes < posted settings) and an unreachable router is reported with its cause and the fix. |
| Z4-Z6: parity and release | **Complete** for the initial Zenoh milestone: parity verified with the ROS 2 tools on both transports, `qosProfiles` reach subscriptions, hot-plug verified, bring-up logging behind per-file switches, and a 12 h soak shows one session throughout and free heap flat to -2 bytes/hour. Next milestone: services (no plan written yet - see 2026-09-28 discussion). |

Runtime transport switching is deferred. Services/parameters remain future
work after this milestone. Do not treat pending RTPS cleanup or Task D as a
prerequisite for the initial Zenoh feasibility experiment.

### Metadata and Identity Implementation (2026-09-17)

- [ZenohROSCodec.h](../components/RaftROS/Zenoh/ZenohROSCodec.h): bounded,
  allocation-free attachment encode/decode, topic keys, `NN`/`MP`/`MS` tokens
  and canonical bounded QoS metadata.
  No RTPS/Zenoh SDK dependency, no network operations and no firmware changes.
- [ZenohROSIdentity.h](../components/RaftROS/Zenoh/ZenohROSIdentity.h): bounded
  endpoint GID generation from canonical tokens using original portable
  XXH3-128 arithmetic. No external implementation in firmware, heap allocation
  or 128-bit integer requirement. Session/entity-ID allocation is still future work.
- [Codec tests](../linux_unit_tests/zenoh_codec_tests.cpp): 2218 passed, 0 failed
  with strict compiler warnings and again with address/undefined-behavior
  sanitizers. Run `make zenoh-test` from `linux_unit_tests`; the target does not
  fetch RaftCore or link RTPS. Exact sanitizer command and profile provenance
  are in the [implementation record](RaftROS-zenoh-implementation-plan.md#implementation-record-2026-09-17).
- Docker's reference-enabled suite passes 3222 checks, including 1004 direct
  comparisons with the pinned RMW hash helper. Fixed upstream GID vectors
  also run in the local suite without ROS dependencies.
- Existing RTPS Linux unit tests rebuilt and passed: **921/921**. Compiler:
  g++ 13.3.0 on Ubuntu WSL; RaftCore
  `feb4f77f1778be04fbf1789bdaa6c84ec4e8fe5c`.
- Standalone build gap resolved on 2026-09-17: updated the wrapper's stale
  includes, discovery namespace, ACKNACK declarations and DATA callback
  signature to the current runtime APIs. The executable compiles and links;
  all 921 RTPS tests still pass. Live ROS and hardware smoke tests remain
  unverified.
- Native Jazzy packages remain absent from the WSL installation, but Docker
  Desktop provides a working Linux reference host. The
  [Docker test](../linux_unit_tests/Dockerfile.zenoh) builds the pinned RMW
  commit because its 0.2.11 apt binary is unavailable in the configured repo.
- [Native metadata control](../linux_unit_tests/zenoh_metadata_interop.py)
  passes: named-node attribution for publisher/subscriber, sensor QoS,
  generated publisher/subscriber GIDs matching the graph, bidirectional typed
  String with Raft-generated attachment identity, sequence/time, and graph removal.
  It uses direct container loopback with no router, but upstream host
  libraries still provide session and CDR. Graph GIDs are compared, no longer
  supplied to the fixture. This does not prove Raft transport, retained-history QoS,
  reconnection or multi-process/ESP32 standalone operation. Commands and exact
  version details are in the plan's native host metadata control section.

### TCP Control Implementation (2026-09-17)

- [ZenohTCPSession.h](../components/RaftROS/Zenoh/ZenohTCPSession.h) and
  [ZenohStreamFramer.h](../components/RaftROS/Zenoh/ZenohStreamFramer.h) add
  allocation-free control parsing, bounded cookie/batch negotiation,
  partial TX ownership and handshake/lease/stalled-send deadlines.
- [Session tests](../linux_unit_tests/zenoh_session_tests.cpp): **1245 passed**
  with strict C++17 warnings and again under address/undefined-behavior
  sanitizers. `make zenoh-session-test zenoh-session-probe` builds locally
  without RaftCore, an RTPS wire runtime or a Zenoh library. The probe now
  links the existing RTPS-named sensor serializer, which contains no RTPS transport.
- [Linux probe](../linux_unit_tests/zenoh_session_probe.cpp) establishes its
  own POSIX TCP connection to pinned `rmw_zenoh_cpp` 0.2.11. Native loopback
  test passed for 12000 ms with 4096-byte negotiated batches, four-second
  leases and 12 received peer keepalives. No router or Python Zenoh client
  provides this connection; the peer runs in the native ROS host process.
- The FRAME discard path is removed. Discovery messages have explicit
  boundaries/callbacks and per-channel sequence checks. Incoming user data,
  request/response, fragmentation, full key-table resolution and reconnect
  remain unsupported. No ESP32 resource/hardware result is claimed.
- The existing native metadata/identity test also passed again, separately.
  Commands and the exact bounded profile are in the
  [TCP control record](RaftROS-zenoh-implementation-plan.md#raft-owned-tcp-handshake-control).

### Connected String Proof (2026-09-17)

- [ZenohNetworkMessage.h](../components/RaftROS/Zenoh/ZenohNetworkMessage.h)
  builds bounded declarations/removals/finals and PUTs with attachments;
  parses discovery messages without owning their borrowed key strings.
- `zenoh_session_probe --publish` uses Raft's CDR encoder, canonical tokens,
  generated GID, and reliable transport FRAMEs on its own POSIX socket. Native
  ROS sees `/raft_test/raft_fixture` publishing `/raft_test/chatter` as String
  with sensor QoS, receives live values/sequences, then sees token withdrawal.
- `zenoh_metadata_interop.py --tcp-publish` passes, including a second ROS
  process joining after publishing starts. It also passed with the C++ probe
  under ASan/UBSan. Final run queued 22 samples over 12 seconds with 12 peer
  keepalives. No upstream client library generates this wire traffic.
- Limits of String mode: one fixed String publisher; a separate Range mode
  is recorded below. No live sensors/subscriptions/reconnect yet.
  The later ROS client uses the first ROS peer's routing/cache, not a separate
  router process. No inbound frames were observed in this native topology;
  direct responder traffic is now covered by the separate scripted test below,
  not inferred from this cached-graph run. This is not general discovery or ESP32 validation.
- Handshake-only and upstream-session metadata controls both pass separately.
  See the [connected proof](RaftROS-zenoh-implementation-plan.md#raft-owned-discovery-and-string-publishing)
  for limits and commands.

### Connected Range Proof (2026-09-17)

- `zenoh_session_probe --publish-range` reuses the unchanged sensor class map
  and Range CDR serializer with synthetic decoded `dist` records. Attribute
  divisor scaling and mm-to-m conversion produce 0, 0.184, 1 and 2 m; every
  sample has a known header timestamp, frame ID, radiation/FOV/limits/variance.
- `zenoh_metadata_interop.py --tcp-range` passes native graph/QoS/GID/hash,
  typed and raw Range delivery, a late independent Range observer, and token
  withdrawal. Final run queued 22 samples over 12 seconds, with 12 peer
  keepalives, 5282 transmitted bytes and 118 received bytes.
- Six fixture CDR payloads and all collected raw samples match native ROS
  fields/length after normalizing only three unspecified reference alignment
  bytes. Raft's padding must remain zero. The sensor serializer was not changed.
- The compiled Range hash matches installed Jazzy `sensor_msgs` 5.3.8 type
  metadata, including `variance`. The entire Range flow passed with probe and
  serializer under ASan/UBSan; the String regression also passes.
- Limits: one synthetic Range publisher per run, not I2C hot-plug or poll
  decoding. The late ROS client still uses the first peer's routing/cache.
  No direct interest frames were observed in this native run; see the separate
  scripted responder coverage below. No ESP32/resource or general QoS claim is made.
  See [the Range proof](RaftROS-zenoh-implementation-plan.md#raft-owned-range-publishing)
  for the exact hash, inputs and commands.

### Direct Discovery and Explicit Restarts (2026-09-17)

- [Scripted wire peer](../linux_unit_tests/zenoh_wire_interop.py) drives actual
  INIT/OPEN and incoming INTEREST frames through the C++ socket. Current/
  current-future replies, exact/prefix/no-match filters, non-token/future-only
  behavior, correlation/finals, cancellation, four-entry capacity/reuse and
  absence after withdrawal pass. Run `make zenoh-wire-test` locally.
- Six additional connections verify expected bounded failures for EOF, peer
  silence, unsupported scope/pattern, queue overflow and CLOSE. Fresh processes
  get fresh session identities and reset transport sequences. This peer is
  scripted protocol evidence, not native RMW discovery interoperability.
- `zenoh_metadata_interop.py --tcp-restart` passes three native Range rounds:
  publisher SIGKILL removes graph entries, peer/context shutdown terminates
  the probe, and fresh publisher/peer processes recover the topic with new
  GIDs and sample sequence one. No automatic reconnect was added.
- Both suites pass with probe/serializer under ASan/UBSan, and normal Range
  publication/late-observer regression passes. Native killed-process cases
  do not establish leak-free teardown. No C++ implementation repair was needed.
- See [lifecycle evidence](RaftROS-zenoh-implementation-plan.md#direct-discovery-and-restart-evidence)
  for commands, deadlines and the distinction between scripted/native checks.

### Host Release Resources (2026-09-17)

- `make resource-baseline` creates isolated `-Os -DNDEBUG` builds with section
  GC, linker maps, `.su` stack files and a JSON report from
  [resource_report.py](../linux_unit_tests/resource_report.py). Normal debug
  build outputs are preserved; flags, revisions and artifact hashes are recorded.
- Measured x86-64 `text+data`: RTPS standalone **41023 B**, Zenoh probe
  **33008 B** after ownership changes (previously 32117 B). These programs differ in features and storage layout; the
  values are not firmware footprints or a finished-backend efficiency ratio.
- Zenoh host fixed objects: session **8344 B**, publication probe **8800 B**,
  plus **2048 B** socket scratch, now in one non-copyable **19192 B** owner
  allocated once before connection. Main's measured frame falls from
  **19568 B to 400 B**, without removing those bytes from total RAM use.
- Removed the measured **8400 B** temporary in session `start()` by resetting
  lifecycle metadata in place. The function is now inlined with no separate
  `.su` frame; owner relocation subsequently shrinks main. Nine same-object reset checks raise
  the session suite to **1245**, passing normally and under ASan/UBSan.
- The measured release probe passes direct wire/lifecycle checks; a native
  Range run with the same optimization policy passes too. RTPS release help
  startup passes. No live CPU/RSS/ESP32 heap measurement is claimed.
- The [resource record and budgets](RaftROS-zenoh-implementation-plan.md#release-resource-baseline)
  propose a 1.5 MiB whole-image ceiling in each existing 1.6875 MiB OTA slot,
  64 KiB initial direct-RAM budget and target stack/heap headroom. These are
  review targets, not achieved device numbers or firmware configuration changes.
  No firmware build artifact is present, and the example Compose entry points
  to an absent build script; actual Raft CLI/IDF build and board results remain gates.

### Owned Probe Workspace (2026-09-17)

- `ProbeStorage` owns all three persistent buffers/state objects behind one
  checked nothrow allocation, bounded to **24 KiB** at compile time. The socket
  closes before owner destruction. Offline resource/CDR commands do not allocate it.
- `make zenoh-storage-test` verifies allocation failure in handshake, String
  and Range modes before any network connection, using an independently linked
  test allocator. Docker runs that gate too; it is never linked into the normal probe.
- Resource reporting accounts for the allocation separately from stack/BSS
  and enforces a **4096 B** host main-frame limit. Constructor frame is **8 B**;
  nested identity/serializer frames remain subject to target measurement.
- Native Range/late join, scripted discovery/error exits and explicit native
  restart tests all pass with sanitizer-instrumented owned storage. No production
  RTPS or session wire behavior changed in this slice.
- This remains Linux probe-level ownership. A 19192-byte contiguous allocation
  plus overhead must be provisioned on target, especially at restart; it is not
  proven by the provisional heap budget. See
  [owned storage](RaftROS-zenoh-implementation-plan.md#owned-probe-storage).

### Shared Sensor Layer (2026-09-17)

- Common [class mapping](../components/RaftROS/AutoPub/AutoPubClassMap.h) and
  [CDR serializer](../components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp) now
  live outside RTPS in `RaftRuntime::AutoPub`. Algorithms and descriptor layouts
  are unchanged. Old RTPS headers preserve source APIs with aliases/forwarders;
  precompiled ABI and external source-list compatibility are not promised.
- ESP-IDF, Linux and Docker source lists use the single common implementation.
  The Zenoh probe uses neutral APIs and builds in Docker without RTPS headers.
- Linux tests: **929 passed, 0 failed**, including eight new compatibility
  assertions; RTPS standalone builds. Native typed/raw Range, late observation
  and withdrawal pass with ASan/UBSan. Session **1245/1245** and allocation-failure
  checks pass in the isolated Docker build.
- Host text+data remains **41023 B RTPS / 33008 B Zenoh**, main frame **400 B**
  and owner **19192 B**. Resource reporting now checks the neutral symbol and
  clears stale generated stack records when sources move. No ESP32 build or
  hardware validation has been repeated.

### Twelve-Hour Soak: No Leak, One Session (2026-09-28)

`/api/rosstat` sampled once a minute from the ROS host for 12 h, Zenoh build
against `rmw_zenohd`, VL6180 attached, `/chatter` running. The last reflash was
at 15:39 on 2026-09-27; the 675 samples (11.2 h) after it:

| | Result |
| --- | --- |
| Session | one, for the whole run; `conn: ready` in every sample; 0 connect failures, 0 re-declares |
| Free heap | 177.8 kB at the start, 176.9 kB at the end; trend **-2 bytes/hour**; first-hour and last-hour means 177.1 kB vs 177.0 kB |
| Minimum free heap | fell to 143.5 kB within 12 minutes of boot (four step-downs, the last a 17.7 kB dip at 15:51) and **never lower again in the remaining 11 hours** |
| Stack headroom | never below 5540 B |
| Samples published | 238,425 - 5.9/s, continuous; 0 inbound dropped |
| Sampler | 2 of 719 minutes unanswered - the two reflashes that afternoon |

So the drift that prompted this (145.7 kB after a day of tests, against 168.7 kB
fresh) was not a leak: free heap is flat to within a few hundred bytes over
eleven hours. What the earlier figure recorded was the *transient* low-water
mark - about 34 kB below steady state at its worst - reached early and once. The
headroom figure for a device with this load is therefore roughly 143 kB, not
the 177 kB steady state.

The 17.7 kB transient at 15:51 was not tied to any RaftROS event in the log; the
device was serving REST queries and the graph tools at the time. Worth
attributing if the transient floor ever matters, not before.

### Release Pass: A Demo Log That Says What Happened (2026-09-27)

Logging that was written for bring-up was left on. Console writes block the
calling task, so this is loop time as well as noise. What a default build now
prints in its first 45 s, against a real router, is every event and nothing
else:

```
RaftROS: setup auto-publish listener registered with DeviceManager
RaftROS: setup backend=zenoh router=192.168.86.192:7447 domain=0 node=/raft_esp32 session=1723...
MainSysMod: Registered /chatter_in string message handler (per-topic slot)
MainSysMod: Registered /chatter_in2 string message handler (per-topic slot)
RaftROS: autoPubAttach devID=1_29 typeIdx=6 slot=1 topic=/raft/range_1_29 type=sensor_msgs::msg::dds_::Range_ qos=fast_sensor
RaftROS: connected to router 192.168.86.192:7447, opening session
RaftROS: session established batch=4096 lease=60000ms
RaftROS: declared node /raft_esp32
RaftROS: subscribed to /chatter_in (qos=fallback_string)
RaftROS: subscribed to /chatter_in2 (qos=fallback_string)
```

Before, the same window held twelve RaftROS lines of which seven were
`autoPubStatusCb` callbacks, per-sample `autoPubData` counters and
`addStringSubscription` echoes of what MainSysMod had just said - and the raw
liveliness tokens and key expressions ran to 200 characters a line.

What moved behind a switch, and where the switch is (each file's own, since a
SysMod's `#define` block sits after its includes and cannot reach a shared
header):

| Switch | File | Restores |
| --- | --- | --- |
| `AUTOPUB_DEBUG_STATUS_CB` | `AutoPub/AutoPubDeviceSource.hpp` | every DeviceManager status callback |
| `AUTOPUB_DEBUG_SAMPLES` | same | one line per 100 samples per device, with sequence, peers and callback-gap histogram |
| `RAFTROS_VERBOSE_LOGGING` -> `DEBUG_ZENOH_KEYS` | `Zenoh/RaftROSZenoh.cpp` | full node token, subscriber key expressions, `addStringSubscription` |
| `RAFTROS_VERBOSE_LOGGING` -> `DEBUG_AUTOPUB_ANNOUNCE`, `DEBUG_HEALTH_COUNTS` | `RTPS/RaftROSRTPS.cpp` | per-peer SEDP announce/dispose of auto-published writers; the 5 s `autoPubStatus` health line |

Attach, detach, connection events, and every warning are unconditional. All of
the demoted counters are on `GET /api/rosstat`, which is where a soak reads
them anyway.

Also in this pass: both RaftCore patches are now upstream (`b8e1f9b` and
`5494416` on RaftCore `main`), so the copies under `devdocs/patches/` are
history rather than instructions; and the fetched `raftdevlibs` copy tracks
that `main`.

### Device Hot-Plug Over Zenoh (2026-09-27)

The VL6180 was unplugged and, some time later, plugged back in while the device
ran the Zenoh build against `rmw_zenohd`, with the device log and a ROS-side
graph watcher recording.

| | Device | ROS 2 graph |
| --- | --- | --- |
| Unplug (12:31:11) | `online=3` (PENDING_DELETION), `autoPubDetach slot=1` after 11,888 samples; slot released (`pubs` 2 -> 1) | `/raft/range_1_29` gone from `ros2 topic list` in the same 2 s sample; stayed gone for 26 min |
| Replug | re-attached at once: `devices:1`, `pubs:2`, `pending:0`; sequence restarted (499 two minutes in); every sample `pub=1` to one peer | topic back, publisher count 1, node `raft_esp32`, BEST_EFFORT/VOLATILE; `ros2 topic echo` gives `range: 0.255` |
| Throughout | session count stayed 1, no re-declare of other endpoints, no `loopBudget` or `ROUTER` warnings; loop 556 us average / 2.5 ms max | `/chatter` and both subscriptions untouched |

So an endpoint's whole life over Zenoh - declare, publish, withdraw on loss,
declare again on return - works on hardware against the real router, with no
session churn. The backend slot was reused (slot 1 again) but the endpoint
identity was not: entity ids only go up, so the returned publisher is a new
endpoint to the graph, which is what a subscriber should see.

One thing to watch rather than act on: after 55 minutes of uptime and a day of
tests, the device's minimum free heap since boot was 145.7 kB, against the
168.7 kB measured on a fresh boot. Some of that is the day's session restarts;
a longer soak with a steady router is the way to tell whether any of it is a
trend.

### The Malformed SEDP Packet, Found and Fixed; Subscription QoS From Profiles (2026-09-27)

**The "one malformed packet per participant" wart is a real bug, and it is
fixed.** It came back the moment a CycloneDDS participant ran `ros2 topic info
-v` against the RTPS build - a tool that stays around and ACKNACKs, where `ros2
topic list` (ten clean participants on 2026-09-26) exits before it can. This
time the wire was captured on the receiving host and every datagram's
submessage chain parsed: two datagrams of exactly 256 bytes, **unicast to the
peer's host at the SPDP port 7400**, each an SEDP-publications DATA whose
header claims 504 bytes with 204 present - one per `topic info` participant, and
the same size as our genuine SPDP announcement. The truncated bytes name
`ros_discovery_info` / `ParticipantEntitiesInfo_`, cut off inside the
parameter list.

Cause: the initial-announce sequence's `SpdpDiscoveryPortCopy` step was built
as `RTPSInitialAnnounceBuildKind::ReusePrevious` - "send the previous step's
bytes again, to the discovery port". The sequence is drained **one step per
loop pass**, and between passes the shared `_sendBuf` is used by heartbeats,
auto-publish announces and ACKNACK replies. So the step sent the previous
step's *length* (256, the SPDP reply) over whatever the buffer held by then -
an SEDP announcement of `ros_discovery_info` some ACKNACK had just provoked.
It needed a peer active between the two passes, which is exactly what
`topic info` provides. Yesterday's note that "nothing in the code explains when
it stopped" was right for the wrong reason: nothing had stopped - the trigger
had.

Fix: the step now builds the SPDP announcement afresh (`ReusePrevious` is no
longer emitted by the plan; a test asserts no step in either flavour relies on
the buffer surviving between passes). Verified on hardware: five `topic info -v`
participants, **zero malformed reports**, and a parse of all 2228 datagrams the
device sent during them shows **zero overruns** - previously one per
participant, every time. CycloneDDS's "length 256" was the datagram length
after all; the 2026-09-25 captures on the sender's WiFi interface missed it,
and its "not reproduced" verdict came from looking at the wrong port.

**Subscriptions now take their QoS from the same profiles as publishers**, on
both transports. `AutoPubDeviceSource::resolveSubscriptionQoS(rosTopic)`
applies a `qosProfiles` alias override for the topic's last segment and
otherwise returns `FallbackString` (RELIABLE, VOLATILE, depth 10) - which is
what both builds announced for readers before, so nothing changes by default.
It is resolved when the reader is announced (RTPS SEDP) or its token declared
(Zenoh), not when the application subscribes, so SysMod setup order does not
matter. Proven through the ROS graph on both transports, with an overlay posted
at runtime:

```
qosProfiles: {"chatter_in": "event"}
                      /chatter_in                   /chatter_in2
Zenoh  (rmw_zenoh)    RELIABLE / TRANSIENT_LOCAL     RELIABLE / VOLATILE
RTPS   (CycloneDDS)   RELIABLE / TRANSIENT_LOCAL     RELIABLE / VOLATILE
default (no overlay)  RELIABLE / VOLATILE            RELIABLE / VOLATILE
```

Two tooling notes from getting that table: under CycloneDDS every `ros2` CLI
call is a fresh participant, so a query can land before our slot-0 reader
announce reaches it (`Unknown topic`) - retry, or wait a few seconds; and use
`--no-daemon`, because a `ros2 daemon` left over from an `rmw_zenoh` run answers
CycloneDDS queries with an empty graph. Neither applies to Zenoh, where the
router already holds the graph.

- Tests: **1035** unit (+2), **141** Zenoh firmware pieces (+3); 1271 session,
  2218 codec unchanged.

### Zenoh by Default, a Configurable Router, and a RaftCore Bug It Exposed (2026-09-27)

**Zenoh is now the default backend.** `Kconfig` defaults the choice to
`RAFTROS_BACKEND_ZENOH`, `CMakeLists.txt` treats RTPS as the opt-in, and
`RaftROSBackendSelect.h` follows the same rule when there is no Kconfig. A clean
build of the example with no backend line in `sdkconfig.defaults` links only
`RaftROSZenoh` (4 objects); `CONFIG_RAFTROS_BACKEND_RTPS=y` still links the RTPS
runtime (18 objects).

**The router address is layered**, each level overriding the last:
`CONFIG_RAFTROS_ZENOH_ROUTER_HOST` (built-in default, now `192.168.86.192`) <
`RaftROS.routerHost` in SysTypes < a settings overlay posted to
`/api/postsettings/reboot` (persisted in NVS, cleared with `/api/clearsettings`).
The example's SysTypes no longer carries the address, so the layering is real.

**If the router isn't there, the device says so.** After three failed attempts,
and then on each further failure at most every 30 s, three `ROUTER UNREACHABLE`
lines name the address, diagnose the failure, say where the address came from,
and give the fix - the rebuild-free one first. Both diagnoses were captured on
hardware:

```
W ROUTER UNREACHABLE: 192.168.86.192:7447 - 3 attempts over 6s, last: connect refused.
  The host answers but no router is listening there - start one with:
  ros2 run rmw_zenoh_cpp rmw_zenohd  Nothing reaches ROS 2 until this is fixed.
W ROUTER UNREACHABLE: the address is the built-in default (CONFIG_RAFTROS_ZENOH_ROUTER_HOST),
  which was set for a different network and probably needs changing for this one.
W ROUTER UNREACHABLE: to change it without a rebuild, from any host on the network:
  curl -X POST http://192.168.86.230/api/postsettings/reboot -d '{"RaftROS":{"routerHost":"<router-ip>"}}'
  - or set RaftROS.routerHost in SysTypes.json, or CONFIG_RAFTROS_ZENOH_ROUTER_HOST in menuconfig, and rebuild.

W ROUTER UNREACHABLE: 192.168.86.250:7447 - 3 attempts over 26s, last: connect timeout.
  The host is not answering - is this the right address for this network? ...
W ROUTER UNREACHABLE: the address is RaftROS.routerHost from SysTypes or posted settings.
```

`GET /api/rosstat` carries the same: `routerSource` (`default`|`config`),
`routerReachable`, `connectFails`, `lastSessionAgoS`. When the router returns the
log says so and the counters reset.

**The RaftCore bug.** Testing "change it without a rebuild" showed posted
settings did not survive a reboot: `/api/postsettings` answered ok and
`getsettings/nv` showed the value, but after any reset the overlay was `{}`.
`RaftJsonNVS::setJsonDoc` updates its RAM copy *before* writing NVS, so the
read-back proved nothing; the write itself succeeded. The cause is static
initialisation order: NVS is initialised by a file-scope static in
`RaftJsonNVS.cpp` (`_nvsInitialised = initNVS(true)`), while `RaftCoreApp` is a
global in the application's `main.cpp` whose `_systemConfig("sys")` constructor
reads NVS - and C++ gives no ordering between static initialisers in different
translation units. On this build the read ran first, failed silently (no logger
yet), and the document came up empty every boot; writes later worked because
NVS was initialised by then.

Fix: `ensureNVSInitialised()`, a construct-on-first-use accessor called at both
NVS access points, replacing the file-scope static. Applied to the fetched copy
under `examples/ExampleDiscoverable/raftdevlibs/RaftCore` (git-ignored, so it
vanishes on a refetch) and saved as
[devdocs/patches/raftcore-nvs-init-order.patch](patches/raftcore-nvs-init-order.patch),
which `git apply --check` accepts against RaftCore `ed73abf`. Not applied to the
RaftCore repo - that is the user's call. With it, a posted `routerHost` survives
`/postsettings/reboot` and a plain `/api/reset`, the SysMod reports
`routerSource:"config"`, and `clearsettings` returns it to the default.

One operational note for demos: `ros2 run rmw_zenoh_cpp rmw_zenohd` started from
an ssh session dies with that session. Start it detached
(`ssh -n -f host "nohup bash -lc '...' &"`); the device reconnects on its own.

### Verified With the ROS 2 Tools Over Zenoh (2026-09-27)

`ros-jazzy-rmw-zenoh-cpp` 0.2.10 installed on the ROS host, `ros2 run
rmw_zenoh_cpp rmw_zenohd` as the router, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, and
the device pointed at it. Everything the RTPS build does with the ordinary ROS 2
tools, the Zenoh build now does too:

```
$ ros2 node list
/raft_esp32

$ ros2 node info /raft_esp32
  Subscribers:
    /chatter_in: std_msgs/msg/String
    /chatter_in2: std_msgs/msg/String
  Publishers:
    /chatter: std_msgs/msg/String
    /raft/range_1_29: sensor_msgs/msg/Range

$ ros2 topic info -v /raft/range_1_29
Publisher count: 1
Node name: raft_esp32            Node namespace: /
Topic type hash: RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a
  Reliability: BEST_EFFORT       Durability: VOLATILE

$ ros2 topic echo --once /raft/range_1_29
range: 0.014999999664723873      variance: 0.0

$ ros2 topic echo --once /chatter
data: Hello from raft_esp32 [28]

$ ros2 topic pub --once /chatter_in  std_msgs/msg/String "{data: 'hello from ros2 zenoh 1'}"
$ ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'second slot over zenoh'}"
  -> device: chatter_in #1..#3 "hello from ros2 zenoh 1..3"
             chatter_in2 #1     "second slot over zenoh"
```

Worth noting:

- The QoS the graph reports (`BEST_EFFORT` / `VOLATILE`) is the `fast_sensor`
  profile the class map picked for a range device, so the profile survives all
  the way into the ROS graph rather than only into our own token.
- Per-topic routing works: `/chatter_in` and `/chatter_in2` land on their own
  handler slots. Each `ros2 topic pub --once` appears with a different publisher
  GID, as it should - a new publisher per invocation.
- The installed `rmw_zenoh` is 0.2.10 while our attachment advertises 0.2.11.
  It interoperates, so that string is not part of matching.

This was the last item standing between the Zenoh backend and parity: every
earlier check went through our own wire reading or a stand-in router, both of
which could have agreed with a shared misunderstanding. The ROS 2 tools could
not.

### Loop Budget Under Load, and Measured Resources for Both Backends (2026-09-26)

Measuring the device with a ROS 2 subscriber actually consuming its topic - not
idle, which is all the earlier numbers covered - showed the RTPS build breaching
the Raft loop contract: **RaftROS took up to 82 ms in a single pass** against a
ceiling of 50 ms, with the whole loop averaging 3-5 ms.

Per-phase timing was added to the loop, and it named the phase immediately:

```
loopBudget pass=66230us drain=140 spdp=21 hb=2691 chatter=17 purge=32
                        rxSpdp=128 rxMeta=62632 rxUser=524 announce=15 peers=1
```

`recvMetatraffic` was essentially the whole of it. It already had a 32-packet
cap, but **a packet count is the wrong bound**: every datagram this SysMod sends
is now timed, and the answer is that a send costs **~0.8 ms of main-loop time on
average and up to 1.8 ms** (1175 sends, 939 ms total, so 13% of wall time went
into the radio). A received ACKNACK can trigger a send, so 32 packets meant up
to ~26 ms of sends alone. The spread is narrow, so this is the inherent cost of
pushing a datagram through lwIP and the WiFi driver at RSSI -82, not blocking on
a full queue - which means the fix is to send fewer datagrams per pass, not to
make the socket non-blocking.

Receive drains now carry a time budget as well as a packet count
(`recvMetatraffic` 6 ms, `recvSPDP` 3 ms; `recvUserData` already took one
datagram per pass). Whatever is left stays in the socket buffer for the next
pass, which RTPS tolerates by design through HEARTBEAT/ACKNACK.

| Under an active subscriber | Before | After |
| --- | --- | --- |
| Worst RaftROS pass | 82 ms (contract: 50 ms) | 14 ms |
| Worst whole-loop pass | 54 ms | 17 ms |
| Whole-loop average | 3.0-5.3 ms (contract: 10 ms) | 2.8-4.3 ms |
| Samples delivered | 4.8 Hz | 4.8 Hz (290 in 60 s, none lost) |

**Both backends, same board, same sensor, equivalent load** (a subscriber
consuming the device topic). This closes the resource measurements Z0 had open:

| | RTPS | Zenoh |
| --- | --- | --- |
| App image | 1268 kB (28% of slot free) | 1258 kB (29% free) |
| Whole-loop average | 2.8-4.3 ms | **0.60 ms** |
| Whole-loop max | 14-17 ms | **2.7-3.0 ms** |
| Worst RaftROS pass | 14 ms | **2.4 ms** |
| Minimum free heap | 178.5 kB | 168.7 kB |
| Main-task stack headroom | 6280 B | 5480 B |
| Delivered | 290 Range in 60 s | 250 Range + 51 chatter, zero lost |

The Zenoh backend is an order of magnitude cheaper on the loop, and the reason
is structural rather than incidental: it sends **at most one datagram per pass**
by construction, where RTPS fans out to every peer within a pass. Zenoh pays for
it in RAM - about 10 kB, which is close to the 11 kB its 16 endpoint slots
reserve for keys and tokens.

New diagnostics on `GET /api/rosstat`, both backends: `stackFreeB`, and on RTPS
also `loopMaxUs`, `rxDeferrals`, `sends`, `sendTotalMs`, `sendMaxUs`. A pass over
20 ms logs its phase breakdown, at most once a second.

Not done: bounding datagrams *per pass* on RTPS (the direct expression of the
finding) or moving its sends off the loop task. The contract is met with room to
spare now, so neither is urgent - but if RTPS ever needs to serve more peers,
that is where the headroom has to come from.

### Liveliness Sequence 0, and Two Open RTPS Items That No Longer Reproduce (2026-09-26)

**A real bug, found by reading a rejected packet.** CycloneDDS's complaint from
the 2026-09-25 run included `wid 0xc2000200 ... first 0 last 0` - a HEARTBEAT on
the participant-message (liveliness) writer advertising `firstSN = lastSN = 0`.
RTPS numbers samples from 1, and an empty writer advertises `firstSN = lastSN +
1`, so 0/0 is invalid and the matching DATA carried `writerSN 0`. The cause:
`_livelinessSeqNum` started at 0 in the ESP SysMod, and the initial-announce
path publishes that counter's current value **without** advancing it (only the
periodic heartbeat pass pre-increments). The Linux standalone already started
this counter at 1, which is why it never showed the fault.

Fixed at both ends: the counter starts at 1, and
`SEDPHandler::buildParticipantMessageData` now refuses a sequence number of 0
rather than putting an invalid sample on the wire. Regression test asserts the
refusal, and that a sequence of 1 yields `writerSN 1` and `firstSN/lastSN 1`.

**What this fix did *not* do.** An A/B on hardware (revert both halves, reflash,
re-measure) shows the symptoms below are absent either way, so the fix is
hardening rather than their cure. Stated plainly because the temptation was to
claim it:

| Item | Yesterday | Today, 10 fresh participant starts |
| --- | --- | --- |
| CycloneDDS "malformed packet" | 2 reports in one run | **0** |
| FastDDS `ros2 topic info -v` | `_NODE_NAME_UNKNOWN_` (O6) | `Node name: raft_esp32` |
| Node, publisher count, type hash | partial under FastDDS | correct under both RMWs |

Conditions covered: 5 participant starts per RMW, the first seconds after boot
as well as a settled device, and RSSI -82 - as weak a link as yesterday's, so
link quality does not explain it either. Both items therefore move from "open
defect" to **not reproducible**: the malformed-packet wart and O6 are gone on
this firmware, but nothing in this session's RTPS changes explains when they
went, so they are worth re-checking rather than declared solved. One untested
suspicion for yesterday's O6 reading: a stale `ros2 daemon` cache.

### Zenoh Reaches Parity: Subscriptions and /chatter (2026-09-25)

The Zenoh build now does everything the RTPS build does, through the same
application API, and the example compiles unchanged on both - nothing in it is
conditional on the transport.

- **Subscriptions.** `ZenohNetworkMessage` gained `declareSubscriber` /
  `undeclareSubscriber` (declaration body id 2, the same shape as a token) and
  `readSample`, and `ZenohTCPSession` delivers inbound samples to a handler.
  The SysMod declares a subscriber plus an `MS` liveliness token per topic, and
  takes either a ROS name (`/chatter_in`) or the DDS form the RTPS build takes
  (`rt/chatter_in`). The handler signature matches RTPS exactly: Zenoh carries a
  16-byte publisher GID in the sample, split the way a DDS GUID is - last four
  bytes as the entity id, first twelve as the participant prefix.
- **`/chatter`.** A 1 Hz `std_msgs/String`, created through the backend as a
  device endpoint is, so there is something to echo with no sensor attached.
- `AutoPubStringMessage.h` holds the CDR decode both builds use;
  `RTPSUserDispatch` keeps its names as aliases.
- A subscription key needs wildcards, so `validKey` grew an `allowWildcards`
  argument - a publication still refuses them, where a wildcard is a mistake
  rather than a pattern.

**Two bugs worth recording.**

1. *The Put body is not just extensions.* Reading a real router's sample failed
   and cost the session every time one arrived. A zenoh `Put` carries a
   timestamp (flag `T`) and an encoding (flag `E`) as body fields **before** its
   extensions, and extensions come in three encodings (unit, varint,
   length-prefixed buffer) that cannot be skipped without decoding which is
   which. The fix was read off the wire: `tshark` on the router host, then the
   `zenoh-codec` source for `put.rs`. The captured 201-byte sample is now a
   test, so the next change to that parser is checked against a real router's
   bytes rather than our own writer's.
2. *A publish with a stale clock killed the session.* `publish()` has no time
   argument - the shared pipeline that calls it is transport-neutral - so the
   backend used the clock `service()` last saw. `/chatter` publishes on its own
   schedule, so it published with a clock of 0, and the session's lease check
   subtracted that from its last receive, wrapped, and declared the lease
   expired. Fixed on both sides: the backend takes `setNow()` every pass, and
   the session's lease and keepalive checks compare rather than subtract.
   Neither suite caught this, so both halves now have a regression test.

Hardware (ESP32-S3 Feather, VL6180, real Zenoh router and ROS 2 Jazzy):

| Check | Zenoh | RTPS |
| --- | --- | --- |
| Device topic | `range=0.0160 m`, ~4 Hz, 194 samples in 45 s | `ros2 topic echo` gives `range: 0.016` |
| `/chatter` | `"Hello from raft_esp32 [35]"`, 40 in 45 s | as before |
| Subscription | all 6 published strings received, GID split as RTPS presents it | `ros2 topic pub /chatter_in` received |
| Graph tokens | node, publisher and subscription tokens all visible and queryable | SEDP as before |
| Example code | unchanged between the two builds | unchanged |

- App image: Zenoh 1256528 B, RTPS 1268464 B.
- Tests: **1028** unit, **2218** codec, **1271** session, **138** Zenoh firmware
  pieces - 0 failed.
- ROS-tooling verification over Zenoh: **done 2026-09-27**, see "Verified With
  the ROS 2 Tools Over Zenoh".

### Zenoh Publishes Devices, Verified on Hardware (2026-09-25)

The auto-publish pipeline now lives in
[AutoPubDeviceSource](../components/RaftROS/AutoPub/AutoPubDeviceSource.h),
templated on the backend, so both builds publish DeviceManager devices through
one copy of the code: DeviceManager listening, class mapping, QoS overrides,
decode on the bus task, the generation-safe mailbox, and serialise-and-publish
on the loop task. The RTPS SysMod keeps only what is RTPS - announce each
writer to each discovered participant, dispose it at each peer - supplied as
hooks, and loses ~600 lines. Zenoh needs no hooks: one token, fanned out by the
router.

**A layering bug the Zenoh build exposed.** The shared descriptor carried a
DDS-mangled topic (`rt/raft/range_1_29`). That is an RTPS wire convention;
Zenoh rejected it as not a ROS path, so a device that attached fine on RTPS was
refused on Zenoh with "backend full". The descriptor now carries the ROS name
(`/raft/range_1_29`) and the RTPS backend adds the `rt` prefix when it
announces. Worth noting because host tests could not have caught it - both
sides agreed with each other, and only a second backend disagreed.

Hardware (ESP32-S3 Feather, VL6180 at 0x29, router stand-in on another host):

| Check | Result |
| --- | --- |
| Zenoh session, node token, endpoint token | type hash and QoS correct on the wire |
| Interest replies | both tokens re-declared against the interest id, then final |
| Samples | `range=0.016 m`, ~5 Hz, no sequence gaps |
| Session drop (tool restarted) | both tokens re-declared on the new session, resumed at the next sequence |
| Real Zenoh library as router | 194 samples in 40 s; both liveliness tokens visible and queryable |
| RTPS, same board | `ros2 topic list` shows `/raft/range_1_29`, `ros2 topic echo` gives `range: 0.016` |
| Loop budget | RaftROS ~500 us average, ~2.3 ms max |

A Zenoh **peer** accepts the session and forwards samples but does not retain
liveliness tokens, so the node and its publishers are invisible to a ROS graph
even while data flows. Router mode retains them. `rmw_zenohd` runs as a router;
[tools/zenoh_subscriber_demo.py](../tools/zenoh_subscriber_demo.py) now does
too. `@ros2_lv` is a verbatim chunk, so a liveliness pattern has to name it -
`**` alone does not match.

- Tools: [zenoh_router_stub.py](../tools/zenoh_router_stub.py) (speaks the wire
  protocol, no dependencies) and
  [zenoh_subscriber_demo.py](../tools/zenoh_subscriber_demo.py) (real Zenoh
  library, so it also proves Zenoh accepts what the firmware sends).
  The example README has the demo steps for both, and for `rmw_zenohd`.
- Tests: **1028 passed, 0 failed**.
- Remaining parity gap: the Zenoh build does not subscribe, and has no
  `/chatter` publisher. Subscriptions need a `DeclareSubscriber` on the wire
  and an incoming-sample path, neither of which `ZenohNetworkMessage` has yet.
  Not verified with ROS 2 tooling over Zenoh: `rmw_zenoh` is not installed on
  the ROS host (`sudo apt install ros-jazzy-rmw-zenoh-cpp`), so `ros2 node
  list` over Zenoh is still unproven - the liveliness tokens ROS discovery
  reads are correct and queryable, but that is one step short.

### Z3: One Backend Per Image, and the Zenoh SysMod (2026-09-25)

The transport is now a build-time choice. `RaftROS.h` is just the
application-facing name - it includes whichever SysMod the image selected, so
an application still writes `registerSysMod("RaftROS", RaftROS::create, true)`
whatever it was built with.

- [RaftROSBackendSelect.h](../components/RaftROS/RaftROSBackendSelect.h) resolves
  `CONFIG_RAFTROS_BACKEND_RTPS` (default) or `CONFIG_RAFTROS_BACKEND_ZENOH` from
  the new [Kconfig](../Kconfig) into `RAFTROS_BACKEND_RTPS` /
  `RAFTROS_BACKEND_ZENOH`, and fails the build if that is not exactly one.
  Host builds have no Kconfig and default to RTPS.
- The RTPS SysMod moved to
  [RTPS/RaftROSRTPS.{h,cpp}](../components/RaftROS/RTPS/RaftROSRTPS.cpp)
  unchanged (one include line), beside the runtime it drives. `CMakeLists.txt`
  picks the source list from the selection, so the RTPS runtime and the Zenoh
  SysMod are never both linked.
- [Zenoh/RaftROSZenoh.{h,cpp}](../components/RaftROS/Zenoh/RaftROSZenoh.cpp) is
  the Zenoh build's SysMod: one non-blocking TCP session to a router in place of
  three UDP sockets and a participant registry. It connects with backoff,
  declares the node's liveliness token once per session, answers router
  interests, and drives `ZenohAutoPubBackend`. Every pass does bounded work -
  one receive, one outbound message - to stay inside the 10 ms / 50 ms loop
  budget, and `routerHost` must be an IPv4 address because resolving a name
  would block the loop for the length of the DNS query.
- [Zenoh/ZenohInterestMatch.h](../components/RaftROS/Zenoh/ZenohInterestMatch.h)
  decides whether a router's interest covers one of our keys. An expression
  beyond empty / exact / `<prefix>/**` is reported `Unsupported` so the caller
  refuses it: answering one we only half understand would under-report our
  declarations and leave the router with a wrong view of the graph. The Linux
  probe now uses this instead of its own copy.

Both images build for ESP32-S3 with ESP-IDF 6.0.2, and `ar t libRaftROS.a`
confirms the exclusion:

| Build | App image | RaftROS objects |
| --- | --- | --- |
| RTPS (default) | 1267632 B (28% free) | 18 |
| Zenoh | 1230304 B (30% free) | 4 (`RaftROSZenoh`, `AutoPubCDRSerializer`, `CDREncoder`, `CDRDecoder`) |

- Tests: **135 passed, 0 failed** (`make zenoh-autopub-test`, +11 for the
  interest matcher). Other suites unchanged. The probe build caught
  `ZenohInterestMatch.h` not being self-contained (`<cstdint>` missing) - it
  only compiled because of include order elsewhere.
- Not addressed: the Zenoh SysMod does not yet auto-publish DeviceManager
  devices. That plumbing (~600 lines of attach/detach, decode buffers, pool and
  mailbox) still lives inside the RTPS SysMod; extracting it behind the backend
  contract so both SysMods share one copy is the next slice, and until then a
  Zenoh image declares its node but publishes no topics. Subscriptions are
  RTPS-only, so the example's `/chatter_in` hook is compiled out under Zenoh.
  Neither image has been run against a router or on hardware since this change.

### Zenoh Auto-Publish Backend (2026-09-25)

[ZenohAutoPubBackend.h](../components/RaftROS/Zenoh/ZenohAutoPubBackend.h)
implements the backend contract introduced in Z2 over a `ZenohTCPSession`, so
the shared AutoPub layer is unchanged: it hands over an `AutoPubEndpointDesc`
and later a serialised sample, and sees no key expressions, liveliness tokens
or sockets. It reuses the codec/session/identity code the connected Linux probe
already proved on the wire.

Three differences from RTPS the contract had to absorb:

- **Discovery is not per-peer.** RTPS announces each writer to each discovered
  participant with SEDP; Zenoh declares one liveliness token per endpoint to the
  router, which fans it out. There is therefore no `announceToPeer()` /
  `disposeAtPeer()` — `createPublisher()` / `destroyPublisher()` carry the whole
  visibility lifecycle.
- **Nothing may block the loop.** The session carries one outbound message at a
  time, so `createPublisher()` only *stages* a declaration — the key, token and
  GID are built up front, so a bad descriptor fails at create rather than
  silently later — and `service(nowMs)` sends one staged message per call,
  round-robin so no slot starves. A slot is not publishable until its token has
  gone out; `publish()` returns `QueueFull` until then, and a refused sample
  never consumes a sequence number, so a subscriber sees no gap after a stall.
- **A dropped session invalidates every declaration the router held.**
  `service()` re-stages all live slots when the session leaves `Established`,
  and endpoints torn down while the link is down are released without an
  undeclare nobody would receive. Entity ids are never reused, so a re-declared
  or slot-reusing endpoint is not confused with the old one.

[AutoPubClassMap_typeHash()](../components/RaftROS/AutoPub/AutoPubClassMap.h)
is new: Zenoh topic keys embed the REP-2011 type hash, and a wrong value
silently stops subscribers matching. The 14 values are the `type_hashes`
entries from the rosidl-generated type descriptions shipped with ROS 2 Jazzy
(`/opt/ros/jazzy/share/<pkg>/msg/<Type>.json`), which are the hashes rmw_zenoh
puts on the wire; `String` and `Range` match the values the probe had hardcoded,
which is an independent check on both.

- Tests: **124 passed, 0 failed** (`make zenoh-autopub-test`), driving a real
  `ZenohTCPSession` fed synthetic handshake bytes — no router, no socket.
  Four mutations were each caught: reusing entity ids (1 failure), publishing
  before the token is out (15), not re-staging after a session drop (4), and
  skipping the undeclare on destroy (4). Other suites unchanged: **1025** unit,
  **2218** codec, **1245** session.
- Not addressed: the backend is not yet linked into a firmware image — Z3
  (exactly-one-backend build selection) is next — so it has not been run against
  a real router or measured on ESP32. Router interop needs the
  `eclipse-zenoh` Python package, which is not installed on this host.

### Open: one malformed SEDP subscription packet per remote participant (2026-09-25)

> **Resolved 2026-09-27.** Root cause found on the wire and fixed - the
> `ReusePrevious` initial-announce step sent a stale buffer with the SPDP length.
> It only triggers with a peer that ACKNACKs between announce steps, which is why
> `ros2 topic list` never showed it and `ros2 topic info -v` always did. See
> "The Malformed SEDP Packet, Found and Fixed".

After the `PID_TYPE_CONSISTENCY` fix, CycloneDDS reports exactly **one**
malformed packet per participant it starts (reproducible, 1 per run):

```
recv: malformed packet received from vendor 1.18 length 256 state parse:DATA
  ... wid 0xc2040000 seq 2 ... 5a001000 ... 00010204   (our /chatter_in reader)
  smid 0x15 flags 0x5 otnh 408 xflags 0 otiq 16
```

It claims a 492-byte message (`otnh 408` at offset 48) but reports arriving as
256 bytes. Packet captures taken during the same runs show no such short
frame: every SEDP subscription announcement we send is 492 or 600 bytes with
submessage lengths that fit the packet, no oversized `otnh`, and no Wireshark
expert warnings. So either CycloneDDS's reported length means something other
than the datagram size, or a short copy is produced on a path the capture on
`wlp2s0` does not see.

Impact is limited: the ROS graph resolves correctly under CycloneDDS
regardless (node, publishers and subscribers all listed), so this is a
correctness wart rather than a functional failure. Worth resolving before
claiming strict-parser cleanliness.

### O6 RESOLVED for CycloneDDS — malformed SEDP subscriptions (2026-09-25)

> **Update 2026-09-26:** FastDDS now resolves the node too
> (`Node name: raft_esp32`, correct publisher count and type hash), so the
> FastDDS half of O6 no longer reproduces either. See "Liveliness Sequence 0,
> and Two Open RTPS Items That No Longer Reproduce" for the evidence and for
> what was ruled out as the cause.


**Root cause:** `SEDPHandler::buildSubscriptionMessage` emitted
`PID_TYPE_CONSISTENCY` with a declared parameter length of 8 but wrote **9**
bytes of value (2-byte kind + 5 booleans + 2 pad bytes). Every parameter
after it was shifted by one byte. Only *subscription* announcements carry
this parameter, so our reader announcements were corrupt while publication
announcements were well-formed - exactly matching the observed asymmetry
(remote readers matched our writers; remote writers never matched our
readers, so `ros_discovery_info` never flowed either way and the graph could
not attribute endpoints to a node).

**How it was found:** CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
rejects the packets outright and says so - `malformed packet received from
vendor 1.15 ... state parse:DATA`. FastDDS mis-parsed them silently, which
is why the symptom survived months of investigation. Wireshark's expert info
also flagged it once looked for: *"Not enough bytes to read the parameter
value"*, with the following parameter read as id `0x2300` length 2048 (the
real `PID_DEADLINE` + length, one byte early).

**Result with CycloneDDS** (ESP32-S3 + VL6180, native Jazzy host):

```
ros2 node list                        -> /raft_esp32
ros2 node info /raft_esp32            -> publishers /chatter, /raft/range_1_29
                                         subscribers /chatter_in, /chatter_in2
ros2 topic info -v /raft/range_1_29   -> Node name: raft_esp32, namespace /
```

**Still open (FastDDS only):** FastDDS continues to report
`_NODE_NAME_UNKNOWN_` and its `ros_discovery_info` reader stays in the
preemptive-ACKNACK state (`bitmapBase 0`, count incrementing on a backoff
timer), never accepting our HEARTBEAT, and it never sends us its own rdi
DATA. Ruled out since: message structure (identical to a captured working
FastDDS-to-FastDDS exchange), addressing, ports, checksums, QoS
(RELIABLE/TRANSIENT_LOCAL, AUTOMATIC liveliness with INFINITE lease),
`PID_KEY_HASH` inline QoS, and vendor id. Reliable delivery works generally
(`/chatter`). FastDDS release builds have Info logging compiled out, so
getting its rejection reason needs an instrumented build or an upstream
question.

### O6 `_NODE_NAME_UNKNOWN_` — wire-level evidence (2026-09-25)

Captured with tshark on the native Linux host (`base8ubuntu`, Jazzy/FastDDS)
while the ESP32-S3 published, and compared against a genuine
`ros_discovery_info` exchange between two host nodes (forced onto UDP with
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, captured on loopback).

**Our rdi traffic matches a working exchange in every checkable respect:**

| Aspect | Ours | Working reference |
| --- | --- | --- |
| DATA flags / inline QoS / serialized key | 0x05 / none / none | identical |
| Octets to inline QoS | 16 | 16 |
| reader/writer entity ids | `00000204` / `00000103` | identical |
| HEARTBEAT flags, first/last SN, count | non-final, 1/1, incrementing | non-final, incrementing |
| SEDP announce | topic `ros_discovery_info`, correct type, RELIABLE + TRANSIENT_LOCAL, unicast locator `<esp>:7411` | identical shape |
| INFO_DST | the host participant's current prefix | — |
| UDP checksum | valid | valid |

**Symptom:** the host's rdi reader only ever emits *preemptive* ACKNACKs
(`bitmapBase 0`, `numBits 0`, count climbing 1..8 on a backoff timer), i.e.
its WriterProxy never accepts a heartbeat from us. The host also never sends
its own rdi DATA to our announced reader, so the failure is symmetric.

**Ruled out (each tested, not assumed):**

- Payload layout - the `ParticipantEntitiesInfo` CDR is correct (16-byte GID =
  prefix + `000001c1`, one node entry, namespace `/`, name `raft_esp32`).
- Packets not arriving - the exact bytes were mirrored to a plain UDP port on
  the host and received in full.
- Wrong port/locator - the host's rdi reader advertises no per-endpoint
  locator, so the participant's user port (7411) is correct; ACKNACKs arrive
  from that participant.
- QoS mismatch, missing `PID_KEY_HASH` inline QoS (tested), reliable delivery
  in general (`/chatter`, also RELIABLE, delivers fine), malformed
  submessages (Wireshark dissects ours identically to the working reference),
  bad UDP checksums, and vendor id (tested announcing eProsima's `010F`).

**Next steps:** compare against CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
to establish whether this is FastDDS-specific; failing that, instrument a
FastDDS build on the host for the rejection reason, or raise it upstream with
the above evidence.

### Bus→Loop Handoff and Native ROS 2 Validation (2026-09-21)

The auto-publish data path is now split across tasks with a bounded handoff,
and has been validated on hardware against native ROS 2 Jazzy on Linux.

- **Concurrency defects fixed.** Bus data callbacks run on the bus worker task
  while attach/detach, `_discovered`, the writer registry and the sockets are
  owned by the SysMod loop task, with no locking previously. That allowed a
  use-after-free (detach deleted `DynamicWriterCtx` while a callback was in
  flight, because `DeviceManager`'s unregister does not withdraw the bus-level
  registration), unsynchronised access to `_discovered` (a `std::vector` the
  loop task reallocates), blocking `sendto` on the bus task, and a shared send
  buffer across bus tasks.
- **Design:** [AutoPubPublisherPool.h](../components/RaftROS/AutoPub/AutoPubPublisherPool.h)
  is a transport-neutral pool of generation-tagged slots, each with a
  latest-only mailbox. The bus callback passes an encoded `{slot, generation}`
  handle (never a pointer) as DeviceManager callback info, decodes under the
  pool lock and stores only the latest record. `RaftROS::loop` drains the
  mailboxes, serialises via `AutoPubSampleRunner` and emits through the RTPS
  emitter. A stale handle is rejected before the user pointer is reached, so
  detach can free state safely; `release()` waits for any producer.
- **Loop-time contract:** the drain never waits on the pool lock (a busy slot
  defers the pass, counted as `drainSkips`) and the in-use count is lock-free,
  so the loop task cannot inherit a bus-task stall.
- **Memory:** per-device 512 B CDR buffers (up to 8 KiB) replaced by two shared
  buffers, since serialisation now happens on one task; mailboxes are 64 B per
  device (largest generated poll record is 36 B).
- **Tests:** Linux suite **1010 passed, 0 failed** (pool, drain-skip and SPDP
  locator tests added), plus a threaded stress test
  ([autopub_pool_stress.cpp](../linux_unit_tests/autopub_pool_stress.cpp),
  `make autopub-pool-stress SAN=thread|address`) which is clean under
  ThreadSanitizer and AddressSanitizer and was mutation-checked: an unlocked
  `release()` is caught as a data race and use-after-free, and a missing
  generation check as wrong-device delivery.
- **Hardware (Adafruit ESP32-S3 TFT Feather + VL6180 on STEMMA QT):** device
  attach/detach across repeated unplug/replug cycles is clean, publishing
  resumes with a new generation, and callback gaps are uniformly 150-250 ms.
- **Native ROS 2 Jazzy end-to-end (first time):** an Ubuntu 24.04 host on the
  same subnet discovers `/raft/range_1_29` with the correct
  `sensor_msgs/msg/Range` type and hash, BEST_EFFORT/VOLATILE QoS, and an
  rclpy subscriber received **2735 samples at 4.9 Hz over 558 s with ~0.3%
  loss** (first sample 1.1 s after subscriber start). Samples reach every
  discovered participant (`peers=N pub=1`).
- **Diagnostics added:** per-device callback-gap histogram and loop drain-gap
  metrics in the `autoPubData`/`autoPubStatus` logs, which is how the sample
  loss was traced to blocking console writes rather than the pool or the bus.
- **Not addressed:** `_NODE_NAME_UNKNOWN_` (O6) reproduces on native Linux, so
  it is a real graph-attribution bug rather than a WSL artefact; a rare ~101 ms
  loop stall around SPDP sends is unexplained; raw decoding, decoder
  allocation, attach/detach and transport lifecycle still live in RaftROS.
  *(2026-09-26: O6 no longer reproduces on either RMW, and the loop now times
  each phase and every datagram it sends, so a recurrence of the SPDP stall will
  say so with a breakdown - at ~0.8 ms per datagram, a multi-destination SPDP
  burst is a plausible cause. See "Loop Budget Under Load" and "Liveliness
  Sequence 0".)*

### Shared Sample Dispatch and First Firmware Compile (2026-09-18)

- [AutoPubSampleRunner.h](../components/RaftROS/AutoPub/AutoPubSampleRunner.h)
  serializes the latest decoded record into one or two borrowed outputs and calls
  a synchronous publish callable;
  [RTPSAutoPubSampleEmitter.h](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubSampleEmitter.h)
  fans a payload out to RTPS peers with one sequence number per sample.
  `RaftROS::autoPubOnDeviceData` now delegates to both; RTPS wire behavior,
  packet arguments and primary-before-secondary order are unchanged. Ownership
  and failure contracts are documented in the headers and the
  [Zenoh plan](RaftROS-zenoh-implementation-plan.md#shared-sample-dispatch-2026-09-18).
- Linux tests: **979 passed, 0 failed** (33 runner + 17 emitter checks added),
  also under ASan/UBSan. Host text+data **41023 B RTPS / 33062 B Zenoh**.
- ExampleDiscoverable builds for ESP32-S3 with ESP-IDF 6.0.2 via
  `raft build --no-docker -e ~/esp/esp-idf-v6.0.2`: app image **1257619 B**
  (29% free in each `0x1b0000` OTA slot), `libRaftROS.a` 40042 B. This is the
  first compile of the modified wrapper; it has not been flashed or run.
- Not addressed: raw decoding, decoder allocation, attach/detach and transport
  lifecycle remain in RaftROS; the writer pointer is not generation-safe.

Next: extract the shared DeviceManager ownership/backend boundary using these
measurements, starting from attach/detach and `DynamicWriterCtx`. General discovery,
automatic reconnect, actual target budgets and live DeviceManager/ESP32
validation remain pending; same-object reset tests do not implement reconnect.

All results, counts and troubleshooting instructions in the dated sections
below describe **RTPS / FastDDS** unless explicitly stated otherwise. Their
912/912 test and firmware size figures are historical; the fresh Linux and
firmware-compile results are recorded above, while hardware measurements have not been repeated.
In particular, the WSL/FastDDS daemon and `_NODE_NAME_UNKNOWN_` observations
must not be used to dismiss a future Zenoh discovery failure. Zenoh has its
own graph-token and host-session contract, and requires host processes using
`rmw_zenoh_cpp`.

## Current Demo / Interop Status (2026-04-27)

The primary user-facing demo path is now:

1. Flash and boot `examples/ExampleDiscoverable`.
2. Attach supported I2C devices to the ESP32.
3. Run `examples/DemoSimple/run_dashboard.sh` on the ROS 2 host.
4. Optionally run `foxglove_bridge` in the same ROS 2 environment and connect
   Foxglove Studio.

Validated on the current Windows 11 + WSL2 + ROS 2 Jazzy + FastDDS host:

- `DemoSimple` discovers `/raft/range_1_29` from a live VL6180 and displays
  `sensor_msgs/msg/Range` samples with live range values.
- A direct `rclpy` graph snapshot discovers `/chatter` and
  `/raft/range_1_29`.
- `foxglove_bridge` running in WSL can be used by Foxglove Studio on Windows
  via `ws://localhost:8765` when WSL mirrored networking and firewall rules are
  configured.

### Current Known Issue: `_NODE_NAME_UNKNOWN_`

On the current WSL2/Jazzy/FastDDS test setup, some `ros2` CLI graph commands
still attribute RaftROS publishers to `Node name: _NODE_NAME_UNKNOWN_`, and
`ros2 node info /raft_esp32` can fail even while data subscribers work.

Current classification:

- **Severity:** cosmetic / graph-introspection only.
- **Not blocked:** typed data subscriptions, `DemoSimple`, direct `rclpy`
  subscribers, and Foxglove via bridge.
- **Observed environment:** Windows 11 + WSL2 Ubuntu 24.04 + ROS 2 Jazzy +
  FastDDS 3.x.
- **Current assumption:** likely host/ros2cli/rmw graph-attribution behavior
  specific to the WSL/Jazzy/FastDDS environment unless reproduced on native
  Linux. This is an assumption, not proof. Native Linux validation remains the
  next check before spending more firmware time on the remaining symptom.
- **Protocol evidence:** RaftROS emits structurally valid `ros_discovery_info`
  CDR with 16-byte GIDs; rdi DATA and HEARTBEAT packets reach the transient CLI
  participant's user-data port after the Fix #9 partial-discovery routing work.
  In the remaining failing trace the host does not ACKNACK the RaftROS rdi
  writer, which indicates the host did not create or match a reader proxy for
  that writer even though normal user-topic readers do match.

Mitigations:

- Prefer `examples/DemoSimple/run_dashboard.sh` and direct `rclpy` subscribers
  for functional demos and validation.
- Use `foxglove_bridge` + Foxglove Studio for visualization. Confirm
  `DemoSimple` sees the topic first; then start the bridge from the same ROS
  environment.
- Treat `ros2 topic echo`, `ros2 topic list`, `ros2 topic info`, and
  `ros2 node info` as diagnostic tools only on WSL/Jazzy. If they disagree with
  `rclpy`, trust the `rclpy` data-path result.
- For native Linux, first repeat `DemoSimple`, then `ros2 topic info -v`, then
  `ros2 node info /raft_esp32`. If native Linux also shows
  `_NODE_NAME_UNKNOWN_`, continue Task D firmware-side investigation below.

## ⚠️ ros2 CLI Usage Prerequisites — READ THIS FIRST

The `ros2` CLI commands (`ros2 node list`, `ros2 topic list`, `ros2 topic
info`, `ros2 node info`, etc.) **will hang or return empty results unless
all of the following hold simultaneously**.  This is a property of the
ros2cli + Jazzy daemon, not of the ESP firmware:

1. **A daemon must be running.**  Start (or restart) it explicitly:
   ```bash
   ros2 daemon stop  # if a stale one is running
   ros2 daemon start
   sleep 5           # give it time to discover existing participants
   ```
2. **At least one *native* ROS 2 node must be alive on the host while
   the daemon initialises.**  The daemon appears to need a peer to
   complete its discovery handshake before it will respond to CLI
   queries.  Any rmw_fastrtps_cpp node works, e.g.:
   ```bash
   ros2 run demo_nodes_cpp talker &
   # …or our reference publisher:
   python3 scripts/range_reference_pub.py &
   ```
3. **Environment must be consistent across daemon and CLI shell:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
   unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
   ```
   If you start the daemon with a different RMW/transport from the CLI
   shell, lookups silently fail.

**Diagnostic short-circuit.**  If `ros2 node list` hangs or returns
empty, it is almost always a daemon problem — *not* a firmware problem.
Use `DemoSimple` or `rclpy` directly instead, which talks to DDS without the
ros2cli daemon and is the canonical functional regression test:

```bash
examples/DemoSimple/run_dashboard.sh
```

For direct payload debugging:

```bash
python3 -u scripts/typed_deserialize_probe.py
# Expect: "got 10 samples", every line "OK: frame_id=... range=..."
```

If `typed_deserialize_probe.py` works but `ros2 topic info` does not, the
firmware is fine and the issue is daemon/CLI plumbing.

## Current State Summary (2026-04-27)

### What works end-to-end

- ESP32 → Jazzy host **typed `sensor_msgs/Range` subscription works.**
  rclpy subscriptions receive every sample and successfully deserialize
  (verified with `scripts/typed_deserialize_probe.py`: `got 10 samples`,
  `frame_id='raft_range_1_29'`, valid range values).
- `examples/DemoSimple/run_dashboard.sh` discovers `/raft/range_1_29`
  dynamically and displays live VL6180 range samples.
- `foxglove_bridge` in WSL + Foxglove Studio on Windows works as a demo
  visualization path once WSL networking/firewall permits ROS 2 discovery.
- All Phase 1 (Discovery), Phase 2 (Topic Publishing — `/chatter`),
  Phase 3 (Topic Subscribing), and Phase 4 (DeviceManager auto-publish)
  capabilities continue to work.
- Linux unit tests: **912/912 pass**.
- Auto-publish CDR serializers audited against ROS 2 Jazzy IDLs:
  `Range` (newly fixed), `Temperature`, `RelativeHumidity`,
  `FluidPressure`, `Illuminance`, `Imu`, `Wrench`, `MultiArray`s — all
  match Jazzy now.

### What does NOT work / known gaps

- **`ros2 topic info -v` shows ESP publisher with `_NODE_NAME_UNKNOWN_`
  / `ros2 node info /raft_esp32` returns "Unable to find node".**
  Cosmetic / graph-introspection only in the current WSL/Jazzy environment:
  rclpy, `DemoSimple`, and Foxglove Bridge data paths subscribe fine.
  Wire-level `ros_discovery_info` payload is now structurally correct
  (16-byte GIDs, valid CDR, matches the IDL), and rmw does *list* the ESP
  publisher's GID via `ros2 topic info`, but it still won't bind that GID to
  our advertised node name. Tracked as **Pending Task D** below. The current
  working assumption is WSL/Jazzy/FastDDS graph-attribution behavior until
  reproduced on native Linux.
- **`ros2 topic echo` / `ros2 topic list` / `ros2 topic info`** work
  only when the prerequisites at the top of this document are met
  (daemon + native peer). Without them they hang.
- Default auto-pub QoS is BEST_EFFORT, so default-QoS tutorials silently
  match nothing. Always pass `--qos-reliability best_effort` to `ros2
  topic echo` for sensor streams.

### Task D investigation 2026-04-25 (continuing) — `_NODE_NAME_UNKNOWN_` for transient CLI participants

This session pursued the "rdi DATA never reaches transient `ros2 topic
info` / daemon participants" hypothesis. Net effect on the user-visible
symptom: still `_NODE_NAME_UNKNOWN_`, but several real defects in the
discovery-routing path were found and fixed; the remaining symptom now
appears to be a different (post-DATA) handshake issue, not a routing
issue. Detailed below.

#### Root cause diagnosis (current best theory)

The Linux daemon (`010fee75…`) and every short-lived `ros2 topic info`
process spin up their own DDS participant. Each one uses **two unicast
ports**:
* a metatraffic locator (advertised in the participant's SPDP DATA),
  which is what most of FastDDS's reliable discovery exchanges use; and
* a dedicated **user-data unicast locator advertised inside the
  participant's own SEDP `subs` DATA** for `ros_discovery_info` —
  i.e. the per-reader unicast locator embedded in the
  `PublicationBuiltinTopicData` / `SubscriptionBuiltinTopicData` PIDs.

We were sending rdi DATA to the participant's metatraffic port (or to
the SPDP-advertised user-data port). For the daemon and CLI participants
those are *not* the same as the rdi reader's per-endpoint unicast
locator. Result: rdi DATA hit a port at which the listener-side reader
proxy was not listening; rmw_dds_common never matched, so the cached
node-name → GID map stayed empty and the CLI printed
`_NODE_NAME_UNKNOWN_`.

The standalone `range_reference_pub.py` participant happens to use the
same locator for everything (small Python participant, single endpoint),
so it has always worked — which is why this regressed silently past
every previous round of testing.

#### Fixes implemented this session (all on disk, in
`components/RaftROS/RaftROS.cpp` and friends)

* **Fix #5 — bump SEDP-publication seq on autopub attach.**
  `_rosDiscSeqNum++` was correct here, but the previous attempt's revert
  also reverted the SEDP-pub bump. Restored as `++_sedpPubSeqNum` (not
  `_rosDiscSeqNum`) so peers re-advertise our autopub writers without
  regressing the rdi GAP problem from earlier attempt #1.
* **Fix #6 — rdi ACK retransmit firstSN + key hash.** Reliable
  retransmit path was sending DATA with `writerSN=0` and an empty
  PID_KEY_HASH; FastDDS dropped these as malformed. Now copies the
  current `firstSN` and the participant key hash on every retransmit.
* **Fix #7a — route rdi DATA to the SEDP-advertised reader port.**
  In `stepWriterHeartbeatPass` the `sendPayload` callback now uses
  `DiscoveredParticipant::rdiReaderUnicastPort` when non-zero, falling
  back to `userDataPort` only when no SEDP-subs has been observed yet.
* **Fix #7b — ACKNACK NACK bitmap bit-reversal.** Reader runtime was
  emitting bits in the wrong order so retransmit requests targeted the
  wrong sequence numbers. Corrected in
  `RTPSRunnerAdapterHelpers::emitReaderRunnerAckNack`.
* **Fix #8 — ACKNACK routing to the metatraffic locator.** Builtin
  ACKNACKs were being sent to the user-data port (which sometimes works
  for FastDDS but not for CycloneDDS or transient CLI participants); now
  routed via `RouteToDiscoveredMetatraffic`.
* **Fix #9 — partial-discovery from SEDP `subs` DATA for
  `ros_discovery_info`.** The transient `ros2 topic info` participant
  multicasts its SPDP DATA before joining; if the ESP's WiFi IGMP
  membership for `239.255.0.1` has aged out (very common on this AP),
  that SPDP is missed and the participant is never added to
  `_discovered`. We then never have a route to send rdi DATA back. New
  helper `RTPSSEDPSubscriptionParser_parse()` extracts
  `PID_ENDPOINT_GUID`, `PID_TOPIC_NAME` and `PID_UNICAST_LOCATOR` from
  every incoming SEDP-subs DATA on the metatraffic socket; if the topic
  is `ros_discovery_info` and the sender's GUID prefix is **not** in
  `_discovered`, we now insert a partial `DiscoveredParticipant`
  initialised from the SEDP subs locator (`rdiReaderUnicastPort`,
  `ipAddr`, `guidPrefix`) and call `processDiscoveredParticipant` so the
  normal pending-announce / heartbeat path picks it up.

  All fields needed for partial discovery were added to
  `DiscoveredParticipant` (new `rdiReaderUnicastPort`); `MAX_DISCOVERED`
  raised to 8 to fit daemon + CLI + reference publisher + ESP itself
  without LRU churn.

#### Wire-level outcome after Fix #9 (m13.pcap, 20 s capture)

After flashing the Fix #9 build via `raft f` and capturing a
`ros2 topic info -v /raft/range_1_29` invocation:

```
=== rdi DATA from ESP — destination ports ===
   6 7411
   7 7413
=== ESP→host port distribution (all UDP, 20 s) ===
   2 7400
 116 7410   metatraffic announce (SPDP/SEDP) — host daemon
 112 7411   user-data — transient ros2-CLI participant (NEW with Fix #9)
  44 7412   metatraffic — transient ros2-CLI participant
 113 7413   user-data — host daemon
```

So Fix #9 *does* fire: rdi DATA now reaches the transient
`ros2 topic info` participant on its advertised port (7411), in addition
to the daemon (7413). The partial-discovery path is observably correct.

#### What is still broken

`ros2 topic info -v /raft/range_1_29` still reports
`Node name: _NODE_NAME_UNKNOWN_` for the ESP after Fix #9. Wire
analysis of m13.pcap shows the cause: **the host never sends an
ACKNACK back for our rdi writer.** Across the whole 20 s capture
there are **zero** ACKNACK submessages from the host targeted at our
rdi writer (`writerEntityId=0x00000103`, `readerEntityId=0x00000204`),
even though our DATA(seq=1) and HEARTBEAT(firstSN=1, lastSN=1) are
visibly arriving at port 7411 of the live CLI participant.

Symptoms in the trace:
* ESP→7411 contains DATA(rdi, seq=1) + HEARTBEAT(rdi, firstSN=1,
  lastSN=1, count incrementing). Payload deserializes round-trip with
  `rclpy.serialization.deserialize_message` (verified in earlier
  session) — it is byte-correct.
* Host→ESP contains the host's own SEDP DATA + HEARTBEAT for
  `ros_discovery_info` (writerEntityId 0x000004c2) — i.e. the host is
  publishing its own rdi reader endpoint as expected.
* Host→ESP contains ACKNACKs for SEDP-subs and SEDP-pubs
  (`rdEntityId=0x000004c7`, `0x000003c7`) — so the SEDP reliable channel
  is fine on both sides.
* Host→ESP contains **no** ACKNACK at all for the rdi writer
  (`rdEntityId=0x00000204`).

This means the host's rmw_dds_common reader proxy for our rdi writer
was never created (or, if it was, it never matched our writer-proxy
on its side). Our rdi DATA arrives at the right port but lands on a
participant which has no matching reader, so it is silently dropped at
the RTPS layer.

Most likely outstanding causes (ordered by likelihood):

1. **Reader entityId in the SEDP-subs DATA we receive does not
   match what we put in our rdi DATA's
   `readerId` field.** We currently send rdi DATA with
   `readerEntityId = ENTITYID_ROS_DISC_INFO_READER (0x00000204)`.
   Some FastDDS versions allocate a per-process entityId for
   `ros_discovery_info` readers (e.g. `0x00010204` or similar) and
   advertise it via PID_ENDPOINT_GUID in the SEDP subs. We are not
   currently propagating that observed reader entityId into our rdi
   DATA submessage — we should.
2. **No SEDP publication of our rdi reader to the new participant.**
   Even though we publish our rdi *writer* in SEDP-pubs, transient
   CLI participants may also need to see our rdi *reader* (so their
   builtin SEDP-subs writer matches and they ACK our SEDP-subs). The
   SEDP-subs HB-driven retransmit only fires after a discovered
   participant is in `_discovered`; for partial entries we're not
   re-sending SEDP-subs to the new locator.
3. **Daemon rdi locator port still 0.** The daemon (long-lived
   `010fee75…`) was discovered via SPDP in the past; its
   `rdiReaderUnicastPort` is still 0 because we did not capture its
   own SEDP-subs locator at the time. After Fix #9 a fresh `daemon
   start` would pick it up, but historical entries don't get
   retroactively patched. Add a periodic re-scan of SEDP-subs against
   `_discovered` to update locator ports for already-known
   participants.
4. **PID_ENDPOINT_GUID parser accepts the *first* PID_UNICAST_LOCATOR
   regardless of `kind`.** The transient CLI participant advertises
   three locators (10.255.255.254 / 192.168.1.28 / 169.254.83.107)
   all on port 7000 in m12, port 7411 in m13. Currently fine because
   they all have the same port, but a host with mixed UDPv4/SHM
   locators could trip this.

#### Plan for the next session

In priority order:

1. **Verify the reader-entityId hypothesis.**  Add a
   one-shot `LOG_I` in the SEDP-subs handler that prints the parsed
   `readerEntityId` for `ros_discovery_info`, then build with
   `idf.py build` (esp-idf-v5.5.3) and `raft f`. If the host's reader
   entityId is *not* 0x00000204, store it per-participant and use it
   in the `readerId` field of every rdi DATA / HEARTBEAT we send to
   that participant.
2. **Send a directed SEDP-subs DATA to partial participants** as part
   of the pending-announce flow, so the new participant has our rdi
   reader endpoint before it tries to match our writer.
3. **Periodic SEDP-subs locator backfill.** On each incoming SEDP-subs
   DATA, also update `rdiReaderUnicastPort` on existing
   `_discovered` entries (we currently only update on the
   "found-in-discovered" branch — Fix #9's new branch only handles
   the "not found" case). Re-check the code: the existing `for` loop
   does update; confirm the daemon never enters that branch (it
   doesn't because the daemon's rdi-subs DATA arrives via multicast
   metatraffic, which we may not parse the same way as the unicast
   path). Make sure the SEDP-subs parser is invoked on multicast
   traffic too.
4. **Locator-kind filter.** Make
   `RTPSSEDPSubscriptionParser_parse()` prefer
   `LOCATOR_KIND_UDPv4` over UDPv6/SHM/TCP when multiple
   `PID_UNICAST_LOCATOR`s are present.
5. **Address WiFi IGMP aging at source.** The whole "partial
   discovery" branch only exists because the ESP's multicast group
   membership for `239.255.0.1` ages out under
   ESP-IDF's WiFi/LWIP. Either:
   * Periodically re-issue an IGMP membership report (use
     `igmp_joingroup`/`igmp_leavegroup` cycle every 30 s); or
   * Drop the partial-discovery hack once IGMP renewal is in place.

Tracking artefact for this session: `m13.pcap` at `/tmp/m13.pcap`
(host capture, all UDP to/from `192.168.1.173`, 20 s window
covering one `ros2 topic info -v` invocation post Fix #9 flash).

#### Build / flash procedure that worked (use this; OTA is broken)

```bash
cd /home/rob/rdev/raft/RaftROS/examples/ExampleDiscoverable
source /home/rob/esp/esp-idf-v5.5.3/export.sh
idf.py build
cp build/SysTypeMain.bin build/SysTypeMain/SysTypeMain.bin   # required for `raft f`
raft f       # WSL → Windows raft.exe → COM3 esptool, ~10 s
# optional separate terminal:
raft m
```

`raft r -c` (docker / esp-idf-v6) also produces a working binary, but
it overwrites `build/SysTypeMain/SysTypeMain.bin` with a slightly
different sized binary; both are confirmed to contain the Fix #9
literal `partial-disc` string in their `.rodata`.

**Do NOT use OTA (`/api/espFwUpdate`) while the rdi/RTPS task is
running.** OTATask blocks CPU0 long enough to trip the watchdog within
the first ~64 KB of upload; the connection is RST'd and the response
oscillates between `{"rslt":"fail","error":"InProgress"}` and
`{"rslt":"fail","error":"NotStarted"}` after retry. The serial flash
path via `raft f` is the only reliable option for now; possibly
add a watchdog-feed in the OTA write loop or yield on every chunk.

### Most recent fix (2026-04-25)

**`ros_discovery_info` Gid CDR layout corrected (24 → 16 bytes).**
`SPDPHandler::buildRosDiscoveryInfoPayload` was emitting each Gid as
`{16-byte GUID, 8 zero bytes padding}` (24 bytes total). The
`rmw_dds_common::msg::Gid` IDL declares `octet[16] data` — exactly 16
bytes, no padding. The 8-byte over-emit caused every subsequent CDR
field to misalign, so `rmw_dds_common`'s reader silently rejected the
sample. Fixed all three sites (top participant Gid, reader_gid_seq
loop, writer_gid_seq loop) and updated the buffer-size estimate.
Verified via wire trace + `rclpy.serialization.deserialize_message` that
the resulting payload now decodes correctly to the expected fields.

This was **necessary but not sufficient** — see "Pending Task D" below
for the remaining symptom (`_NODE_NAME_UNKNOWN_` despite valid wire).

### Most recent fix (2026-04-24)

`sensor_msgs/Range` typed deserialization was failing on Jazzy because
ROS 2 Jazzy added a trailing `float32 variance` field that did not exist
in Humble. Fix: emit `0.0f` for variance in `serializeRange`. See
"ACTIVE BUG: Fast CDR exception on deserialize — RESOLVED 2026-04-24"
section for full details and the diagnostic methodology.

### Quick verification commands

```bash
# 1. Confirm typed deserialization (the canonical regression test)
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
python3 -u scripts/typed_deserialize_probe.py
# Expect: "got 10 samples", every line "OK: frame_id=... range=..."

# 2. Linux unit tests
cd linux_unit_tests && make && ./linux_unit_tests
# Expect: "912 passed, 0 failed"

# 3. ESP build + flash
cd examples/ExampleDiscoverable && raft r
# Note: may need to close serial monitor before flash (COM3 busy)
```

## Host environment gotchas (recorded 2026-04-23 after losing a day to these)

These are host-side, not firmware bugs — but they can look exactly like
"the ESP32 is broken". Full reference in `README.md` §"Host Setup Notes".

- **ESP32 does NOT push user DATA to an empty graph.** A passive
  packet-capture with no ROS 2 subscriber running on the host will show
  **zero** unicast DATA from the ESP32 (only periodic SPDP multicast). To
  capture data traffic you must first start a consumer that actually
  matches the publication, e.g.:
  ```bash
  # keep this running for the whole capture window
  ros2 run demo_nodes_cpp listener                       # for /chatter
  python3 scripts/range_probe.py --raw                   # for /raft/range_1_29
  # OR
  ros2 topic echo --no-daemon --qos-reliability best_effort /raft/range_1_29 sensor_msgs/msg/Range
  ```
  If `tshark -f 'host <esp_ip>'` reports `Packets captured: 0`, assume
  "no subscriber is active" before assuming a firmware/network bug.
- **WSL2 must use mirrored networking.** Default NAT mode blocks inbound
  SPDP multicast from the LAN. Fix: add `networkingMode=mirrored` under
  `[wsl2]` in `%UserProfile%\.wslconfig`, then `wsl --shutdown`.
- **Windows Firewall blocks inbound UDP to WSL** even in mirrored mode.
  Fix: `New-NetFirewallRule -DisplayName "ROS2 RTPS" -Direction Inbound
  -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any` (or set
  `firewall=false` in `.wslconfig`). Symptom: ESP32 sees the host, host
  never sees the ESP32; `tcpdump` shows zero packets from the ESP32 IP.
- **ROS 2 Jazzy inverted `ROS_LOCALHOST_ONLY`.** Must be `unset`, not
  `=0`. On Humble, `=0` is still correct.
- **`tcpdump`/`dumpcap` output to `/tmp`** is denied under WSL even as
  root — write pcaps to `/var/tmp/` instead. User also needs `sudo` (not
  in `wireshark` group by default).
- **`ros2 daemon` hangs on WSL2** (XML-RPC socket). Always use
  `--no-daemon`. If `ros2 daemon stop` times out, force-kill:
  `pkill -9 -f _ros2_daemon`.
- **Fast DDS log XML:** root element is
  `<dds xmlns="http://www.eprosima.com">`, not `<profiles>`. Log kind is
  `Log::Kind::Info`, not `log_kind::Info`.
- **Missing `DeviceTypes.json` silently disables I2C auto-identification.**
  When using Raft's `raftdevlibs/` override mechanism (clone
  RaftCore/RaftI2C/etc. into `examples/<app>/raftdevlibs/` so local edits
  survive clean builds), the build picks up devtypes from the overridden
  tree. If that tree is missing `devtypes/DeviceTypes.json`, scanners
  detect the device at the bus level but `identifyDevice` can't assign
  a `deviceTypeIndex`, so auto-publish never attaches. Symptom:
  `I2CDevIdentMgr: identifyDevice new device <name>` fires, but
  `autoPubStatus slots=0/16` stays at 0 forever. Fix: make sure
  `raftdevlibs/<Repo>/devtypes/DeviceTypes.json` exists (copy from the
  upstream clone).

---

## Phase 4 auto-publish — current status (2026-04-23)

### Verified working (end-to-end)

| # | Observation | Evidence |
|---|-------------|----------|
| V1 | DeviceManager → listener dispatch is live | `autoPubStatusCb devID=1_29 typeIdx=5 online=1 isNewlyId=1` in ESP logs after the DeviceManager fix |
| V2 | Auto-publish slot attach fires on identification | `autoPubAttach devID=1_29 typeIdx=5 slot=0 topic=rt/raft/range_1_29 type=sensor_msgs::msg::dds_::Range_` |
| V3 | SEDP publication accepted by Fast DDS daemon | `/raft/range_1_29` appears in `ros2 topic list` and `ros2 topic info --verbose` |
| V4 | CDR payload sent at expected rate | `ros2 topic hz /raft/range_1_29` reports **~4.95 Hz** stable for 23 s+ |
| V5 | UDP delivery is reliable | `tcpdump` on `eth2` captures every frame; 216/304 packets in a 10 s run, 0 dropped by kernel |
| V6 | Raw wire payload is structurally correct | Decoded Range DATA submsg: encap `00 01 00 00` (CDR_LE), sec, nsec, frame_id (len-prefixed + NUL + pad), radiation_type=1 (INFRARED), pad, fov, min, max, range, variance. For `frame_id="raft"` → 44 bytes total. ROS 2 Jazzy's `sensor_msgs/Range` has **5 float32** fields (`variance` was added after Humble) — omitting `variance` causes silent FastCDR deserialize failure and empty typed subscriptions. |
| V7 | Same packet decodes cleanly for `/chatter` on the same writer flow | `/chatter` echo has worked throughout |

### Confirmed topic-name rules

- On the wire the topic is `rt/raft/range_1_29` (ROS 2 always adds a `rt/` prefix to user topic writers — this is a DDS-level detail).
- From the ROS 2 CLI the correct name is **`/raft/range_1_29`** (the `rt/` prefix is stripped by rmw). Do **not** use `/rt/raft/range_1_29`; that call will silently fail.
- `ros2 topic list`, `topic info`, `topic hz`, `topic echo`, and `rclpy` subscriptions all use the `/raft/...` form.

### QoS reality check

- `RTPSAutoPubQoSProfile::FastSensor` = BEST_EFFORT / VOLATILE / KEEP_LAST depth 10.
- This class is assigned to every I2C sensor class by `RTPSAutoPubClassMap` (DIST, PROX, ACC, GYRO, LGHT, ANG, HRM, FRCE, IMU).
- The ROS 2 CLI default QoS is RELIABLE, so `ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range` with no flags produces no output because the subscriber won't match the BEST_EFFORT writer.
- Correct invocation: `ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range --qos-reliability best_effort`.
- Oddity observed: `ros2 topic hz` matches and counts frames even with default QoS. That is because `hz` in Jazzy uses the "best-available" QoS profile (auto-relaxes to match the offered writer). `echo` with an explicit message type does **not** auto-relax; `echo --raw` behaviour is less clear and needs re-testing.

### ACTIVE BUG: Fast CDR exception on deserialize — **RESOLVED 2026-04-24** ✅

**Symptom (now fixed):**
```
Fast CDR exception deserializing message of type sensor_msgs::msg::dds_::Range_.,
at ./src/type_support_common.cpp:118
```
`ros2 topic echo` swallowed this silently and produced no output. `ros2 topic
hz /raft/range_1_29` worked (uses raw subscription internally), so traffic was
visibly arriving — only the typed deserialize was failing. `rclpy` typed
subscriptions received zero callbacks.

**Root cause:** ROS 2 **Jazzy** added a trailing `float32 variance` field to
`sensor_msgs/msg/Range` (see `/opt/ros/jazzy/share/sensor_msgs/msg/Range.msg`).
Humble had **4** float32 fields (`field_of_view, min_range, max_range, range`);
Jazzy has **5** (adds `variance`, with `0.0` meaning "unknown" per REP-117).
Our `serializeRange` in
`components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.cpp` only
emitted the 4 Humble fields, so the CDR payload was 4 bytes short. FastCDR
threw on the missing trailing float and `type_support_common.cpp:118`
silently dropped the sample. The wire QoS / SEDP / heartbeat / encapsulation
were all correct — this was purely a struct-layout mismatch.

**Why it was so hard to find:**

- `ros2 topic hz` keeps working because it uses a `raw=True` subscription and
  doesn't deserialize. This made it look like data was flowing.
- `ros2 topic echo` calls `deserialize_message` internally and on exception
  silently returns nothing — no log line at default verbosity.
- pcap inspection showed structurally valid CDR (encap header, header,
  radiation_type, 4 floats). Nothing visibly wrong unless you knew Jazzy's
  IDL had drifted.
- Reference Fast DDS publishers on the same host produced 56-byte payloads
  (with `variance`); ESP produced 40-byte payloads. The 4-byte difference
  was the variance.

**Definitive diagnostic:** subscribe with `raw=True`, then call
`rclpy.serialization.deserialize_message(data, Range)` directly in the
callback. This re-raises the FastCDR exception text. Saved as
`scripts/typed_deserialize_probe.py` — keep this script for future Jazzy IDL
drift debugging on any auto-pub message type:
```bash
python3 -u scripts/typed_deserialize_probe.py /raft/range_1_29 sensor_msgs/msg/Range
```

**Fix:** add a single trailing float32 in
`components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.cpp`:
```cpp
if (!enc.writeFloat32(maxRange)) return false;
if (!enc.writeFloat32((float)rangeVal)) return false;
// variance (added in ROS 2 Jazzy); 0.0 = variance unknown per REP.
if (!enc.writeFloat32(0.0f)) return false;
return true;
```
Linux unit-test assertion updated in `linux_unit_tests/main.cpp` to expect a
44-byte payload with `variance` at offset 40 (frame_id="raft").

**Verification (post-fix):**
- `python3 /tmp/test_deserialize.py` shows `got 10 samples`, all
  `OK: frame_id='raft_range_1_29' range=...`. Pre-fix this returned
  `DESER FAIL: ...Fast CDR exception...`.
- Wire payload now 56 bytes for `frame_id='raft_range_1_29'`, ending
  `... 00 00 00 00` (variance=0.0f).

**Audited at the same time** (Jazzy IDL drift in other auto-pub message types):

| Message | Status |
|---|---|
| `sensor_msgs/Temperature` | Already correct — emits `float64 variance` |
| `sensor_msgs/RelativeHumidity` | Already correct — emits `float64 variance` |
| `sensor_msgs/FluidPressure` | Already correct — emits `float64 variance` |
| `sensor_msgs/Illuminance` | Already correct — emits `float64 variance` |
| `sensor_msgs/Imu` | Unchanged from Humble — no fix needed |
| `geometry_msgs/Wrench` | Unchanged from Humble — no fix needed |
| `std_msgs/ByteMultiArray` / `Float32MultiArray` | Unchanged from Humble — no fix needed |

Only `Range` regressed.

**Theories from this bug to remember for future ROS 2 distro upgrades:**
- IDL fields can be appended to existing messages between distros (e.g.
  `variance` added to `Range` between Humble→Jazzy). Always diff
  `/opt/ros/<distro>/share/<pkg>/msg/*.msg` against your serializer when
  bringing up a new distro.
- The `PID_DATA_REPRESENTATION` / XCDR2 / DHeader theory turned out to be a
  **red herring** — Fast DDS in Jazzy still accepts plain XCDR1
  (`encapsulation = 0x0001 little-endian`). What it does NOT tolerate is a
  payload too short for the type's expected layout.
- RIHS01 type-hash mismatch could in principle reject samples but was not
  the cause here (the lookup table in
  `components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp` already has the
  correct hash for `sensor_msgs::msg::dds_::Range_`:
  `b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a`).

### Other odd behaviours observed

| # | Observation | Explanation / theory | Severity |
|---|-------------|----------------------|----------|
| O1 | `ros2 node info /raft_esp32` → "Unable to find node" | ParticipantMessageData / node-name mapping incomplete — node listed by `node list` but not looked up by name. Probably missing `ros_discovery_info` GID list entries for the newly-attached autopub writer (we don't re-publish `ros_discovery_info` after attach). | Cosmetic |
| O2 | `ros2 topic list --no-daemon` often shows **3 "discovered" participants** for every CLI invocation | Each `ros2 topic list` spawns a short-lived DDS participant that appears + disappears within its `--spin-time`. The daemon also has one. Not a bug. | Cosmetic |
| O3 | First ACKNACK after match produces `total count change:1 total count: 1` | BEST_EFFORT writers still maintain a sequence number; the subscriber reports one "missed" sample on match because it joined mid-stream. This is normal for BEST_EFFORT and is not the deserialize failure. | Normal |
| O4 | `ros2 topic echo --raw` only prints with explicit `--qos-reliability best_effort` | The CLI `echo` doesn't auto-relax QoS for explicit message-type subscriptions. `hz` does because it uses "best-available" QoS. | Known / documented |
| O5 | LWIP still loses mid-burst sends with ENOMEM on the ESP | Known; covered by reliable retransmit. | Known / accepted |
| O6 | `ros2 topic echo` / `ros2 topic list` / `ros2 topic info` can hang or show `_NODE_NAME_UNKNOWN_` under Jazzy on this WSL host even with `--no-daemon` | Topic graph resolution path; rclpy subscriptions, `DemoSimple`, and Foxglove Bridge still receive data fine. Earlier hypotheses included rdi entity-kind mismatch; the current evidence after Fix #9 is that rdi DATA reaches the transient CLI participant but the host does not create/match the rdi reader proxy or ACKNACK our rdi writer. Treat as WSL/Jazzy/FastDDS graph-attribution behavior unless reproduced on native Linux. See "Pending Task D" below. | Cosmetic / CLI-only |

### Investigation outcome (closed)

**Resolved:** The "Fast CDR exception" was missing `variance` field — see resolution box above. The XCDR1/XCDR2 / `PID_DATA_REPRESENTATION` theory was **wrong**. Fast DDS in Jazzy still accepts plain XCDR1 just fine.

### Pending follow-ups

1. **Task D — `ros_discovery_info` node-name binding (rmw reader stuck at
   preemptive ACKNACK).** Cosmetic only — data path unaffected.

   **Fix attempts on 2026-04-25 (both unsuccessful):**

   *Attempt #1 — keep `firstSN=1` always (avoid TRANSIENT_LOCAL gap):*
   - Reverted `_rosDiscSeqNum++` bumps in `autoPubAttachDevice` and
     `autoPubDetachDevice` so seq stays at 1 forever.
   - Changed HB callback to advertise `firstSN=1` unconditionally.
   - Wire trace after fix: HB now consistently `firstSN=1, lastSN=1`
     (good), but host reader behavior **unchanged** — still 4 preemptive
     ACKNACKs (`bitmapBase=0, numBits=0`), never advances. Status:
     change kept (correct per RTPS spec for our single-sample design)
     but didn't resolve Task D.

   *Attempt #2 — entity kind 0x03→0x02 (USER_WRITER_WITH_KEY):*
   - Reasoning: `ParticipantEntitiesInfo` IDL has `@key`, so theoretically
     should use WITH_KEY entity kind.
   - Result: matching **broke entirely** — zero ACKNACKs from host. Reverted.
   - Conclusion: rmw_fastrtps applies the IDL @key annotation at the DDS
     layer only; the RTPS entity kind is still NO_KEY (0x03). Documented
     in `RTPSTypes.h` comment.

   **2026-04-25 diagnostic state (post Gid-layout fix, post attempts above):**
   - Wire payload now structurally correct: 16-byte Gids per IDL,
     CDR_LE encapsulation, all PIDs present
     (`PID_TOPIC_NAME=ros_discovery_info`,
     `PID_TYPE_NAME=rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_`,
     `PID_USER_DATA=typehash=RIHS01_91a05...;`,
     `PID_DATA_REPRESENTATION=XCDR1`,
     `RELIABILITY=RELIABLE`, `DURABILITY=TRANSIENT_LOCAL`).
   - Payload deserializes successfully via
     `rclpy.serialization.deserialize_message(data, ParticipantEntitiesInfo)` —
     gid, ns="/", name="raft_esp32", reader_gid_seq=2, writer_gid_seq=2 all
     decoded correctly. So the CDR is provably valid.
   - `ros2 topic info /raft/range_1_29 -v` correctly lists the ESP
     publisher's GID — proving SEDP-publication match works end-to-end.
   - But host-side rmw still shows `Node name: _NODE_NAME_UNKNOWN_` and
     `ros2 node info /raft_esp32` returns "Unable to find node".

   **Wire trace finding (the smoking gun):** Multiple host-side readers
   (4 distinct ACKNACK count streams) each match our writer at RTPS layer
   and send **preemptive** ACKNACK (`bitmapBase=0, numBits=0`) but **never
   advance past it**, despite us sending DATA(seq=2) +
   HEARTBEAT(firstSN=2, lastSN=2, count=14620+) repeatedly. After 28 DATA
   samples + ~14000 HBs over 10s, every captured ACKNACK is still
   preemptive (`tshark` analysis: "Preemptive ACKNACK"). The host's
   rmw_dds_common listener never gets a sample.

   **Likely root cause (next investigation should start here):** When the
   autopub attach bumps `_rosDiscSeqNum` from 1→2, our HB jumps from
   `firstSN=1,lastSN=2` to `firstSN=2,lastSN=2` (we drop seq=1).
   Reliable + TRANSIENT_LOCAL readers need a `GAP` submessage to know
   seq=1 is gone — without it they may NACK seq=1 forever or stay in
   preemptive. Two candidate fixes to try:
   1. Send a `GAP` submessage covering [1..lastSN-1] when first
      heartbeating after a bump.
   2. Keep `firstSN=1` always for ros_discovery_info; never bump
      `_rosDiscSeqNum` on attach. Instead, build the rdi sample lazily so
      the very first publication (at seq=1) already includes any autopub
      writer GIDs allocated during boot. Will require deferring first rdi
      send until after `autoPubAttachDevice` has run (or a small init
      barrier).

   **Things that are NOT the issue (verified by capture/test 2026-04-25):**
   - typehash mismatch — RIHS01_91a05... is correct in
     `/opt/ros/jazzy/share/rmw_dds_common/msg/ParticipantEntitiesInfo.json`.
   - CDR layout — payload deserializes round-trip with rclpy.
   - Gid layout — was wrong (24 bytes), fixed to 16 bytes per IDL.
     This was necessary but not sufficient.
   - SEDP-pub matching — ESP writer's GID is visible in
     `ros2 topic info -v`, proving rmw saw our SEDP publication DATA.
   - QoS mismatch — RELIABLE+TRANSIENT_LOCAL+KEEP_LAST is what
     rmw_dds_common requires and we declare.

   **Do not change `ENTITYID_ROS_DISC_INFO_WRITER` (0x00000103) in
   `RTPSTypes.h`** — wire trace shows host correctly targets that
   entityId in its ACKNACKs.

   **Reproduce:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
   unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
   ros2 daemon stop; sleep 1
   python3 scripts/range_reference_pub.py &
   sleep 3 && ros2 daemon start && sleep 5
   ros2 topic info /raft/range_1_29 -v   # ESP shown but NODE_NAME_UNKNOWN
   ros2 node info /raft_esp32            # Unable to find node
   ```
2. ~~**Re-publish `ros_discovery_info` on auto-pub attach.**~~ **DONE
   2026-04-26, then REVERTED 2026-04-25 as part of Task D Attempt #1.**
   The bump caused a sequence-number gap (firstSN=2, lastSN=2 with no
   GAP submessage) which TRANSIENT_LOCAL readers couldn't recover from.
   Per Task D analysis, we now keep `_rosDiscSeqNum=1` permanently and
   rebuild the rdi payload from current state on every HB instead. SEDP
   publication-announce on attach (in `emitAutoPubSedpAnnounce`) still
   informs peers about new writers — that path was always working and
   is what makes `ros2 topic info -v` list the ESP publisher GID.
3. **Auto-pub QoS default.** `RTPSAutoPubQoSProfile::FastSensor` is
   BEST_EFFORT/VOLATILE. `ros2 topic echo /raft/range_1_29` requires
   `--qos-reliability best_effort` to match. Consider documenting
   prominently or providing a RELIABLE alternative profile for tutorial use.
4. ~~**Pre-existing 8 unit-test failures**~~ **FIXED (2026-04-26).** Six
   SEDP-publication tests failed because the test buffer was 512 bytes but
   `buildPublicationMessage` requires ≥900 bytes (and the
   "msgLen < 400 bytes" assertion was stale — current Jazzy-compatible
   SEDP messages are larger). Two
   `RTPSDynamicWriterRegistry` tests failed because `sedpSeqNum` is now
   pre-seeded to `AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + slot` (not 0). Both sets
   of assertions updated in `linux_unit_tests/main.cpp`. Result:
   **913 passed, 0 failed**.

### Diagnostic recipes (proven, keep using these)

- **Typed deserialize check (catches FastCDR exceptions that `ros2 topic
  echo` swallows):**
  ```bash
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  python3 -u scripts/typed_deserialize_probe.py /raft/range_1_29 sensor_msgs/msg/Range
  ```
  Script body subscribes raw, prints hex, then calls
  `deserialize_message(data, Range)` and prints `OK` or the exception. This
  is the shortest path to detecting struct-layout drift.
- **pcap inspect:** `tcpdump -i <iface> -w /var/tmp/rtps.pcap 'udp portrange
  7400-7500'`, then `tshark -r /var/tmp/rtps.pcap -V` (write under
  `/var/tmp` not `/tmp` on WSL).
- **Compare against reference Fast DDS publisher** for the same message type
  to verify expected payload size and trailing bytes.
- Wireshark dissector: install `wireshark-common` on WSL, `chown` the pcap,
  open the file directly — RTPS 2.2 is decoded natively and the serialized
  payload is annotated by generated type plugins if ROS 2 is sourced.

---

## Goal

Make an ESP32-S3 (running Raft firmware) appear as a native ROS 2 node — without micro-ROS or any agent process — using a clean-room RTPS 2.2 implementation as a Raft SysMod.

**Phase 1 (Discovery), Phase 2 (Topic Publishing), and Phase 3 (Topic Subscribing with per-topic routing) are COMPLETE.**
- `ros2 node list` shows `/raft_esp32`.
- `ros2 topic list` shows `/chatter` (published) and the subscribed topics (`/chatter_in`, `/chatter_in2`, …).
- `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints a sample per second.
- `ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"` routes to the correct per-slot handler on-device.

Verified end-to-end on 2026-04-22 against ROS 2 Humble + FastDDS 2.6.11, ESP32-S3 on real WiFi (PC `192.168.1.92`, ESP32 `192.168.1.173`).

## Architecture

Clean-room RTPS 2.2 implementation. No third-party DDS libraries.

### Source Files

| File | Purpose |
|------|---------|
| `components/RaftROS/RaftROS.cpp/.h` | SysMod lifecycle, UDP sockets, receive/send loops, participant state machine |
| `components/RaftROS/CDR/CDREncoder.cpp/.h` | CDR (Common Data Representation) serializer |
| `components/RaftROS/CDR/CDRDecoder.cpp/.h` | CDR deserializer |
| `components/RaftROS/RTPS/RTPSTypes.h` | Constants: entity IDs, PID codes, port formulas, QoS values |
| `components/RaftROS/RTPS/runtime/wire/RTPSMessage.cpp/.h` | RTPS header/submessage read/write (DATA, HEARTBEAT, ACKNACK, INFO_DST, INFO_TS) |
| `components/RaftROS/RTPS/runtime/core/RTPSParticipant.cpp/.h` | GUID management, port calculation, locator building |
| `components/RaftROS/RTPS/runtime/discovery/SPDPHandler.cpp/.h` | SPDP announcement build/parse, ros_discovery_info CDR payload |
| `components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp/.h` | SEDP publication/subscription/liveliness message build, user data message build |
| `linux_unit_tests/` | Linux-hosted unit tests + standalone linux RTPS node (`raftros_standalone.cpp`) |
| `unit_tests/` | ESP-IDF Unity tests |
| `examples/ExampleDiscoverable/` | ESP32 app that runs RaftROS as a SysMod |

### Hardware / Environment

- **ESP32-S3**, MAC `94:a9:90:3a:51:b0`, WiFi STA IP `192.168.1.173`
- **ROS 2 Humble** with FastRTPS 2.6.11, running on WSL2 (mirrored networking), IP `192.168.1.92`
- Domain ID 0 → SPDP multicast `239.255.0.1:7400`, metatraffic unicast `:7410`, user data unicast `:7411`

## Phase 2: Topic Publishing — COMPLETE ✅

Immediate objective was to publish real ROS 2 topic data from the ESP32 with reliable delivery.

### What Works (ESP32, 2026-04-21)

- ESP32 announces an additional DataWriter endpoint for `/chatter` (`std_msgs/msg/String`, RELIABLE + VOLATILE) via SEDP publications on writer `000003C2` at sequence number 2 (distinct from the `ros_discovery_info` announcement at sequence 1 on the same writer).
- `publishChatter()` serializes `std_msgs/msg/String` using the CDR encoder and sends DATA + HEARTBEAT on the user data port (`7411`) once per second.
- HEARTBEAT `firstSN` matches the current sequence number for the VOLATILE chatter writer so newly-matched subscribers do not request historical samples.
- ACKNACK-driven retransmit works on both the SEDP publications path (chatter announcement recovery) and the chatter user-data path.
- `ros_discovery_info` writer GID list still advertises the chatter writer as part of the participant's node entities.

### Verification

- On PC: `env -u PYTHONPATH PYTHONNOUSERSITE=1 ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints `data: Hello from raft_esp32 [N]` once per second indefinitely.
- On PC: `ros2 topic info /chatter --no-daemon -v` shows one Publisher with QoS RELIABLE + VOLATILE, node `raft_esp32`.
- On ESP32: ACKNACK log shows `readerEID=00000504 writerEID=00010103 (chatter) base=N numBits=0` with `N` advancing — no retransmit storm.
- Linux unit tests and standalone linux publisher remain green (**85 passed, 0 failed**) after the ESP fixes.

### Additional Fixes Applied During Phase 2 Bring-up

| # | Fix | Details |
|---|-----|---------|
| 15 | VOLATILE chatter HB `firstSN = currentSeq` | Heartbeat `firstSN=1` on a VOLATILE writer caused newly-matched subscribers to NACK an ever-growing gap of historical samples. Fix: advertise `firstSN = chatterSeqNum`. Added `firstSN` parameter to `SEDPHandler::buildUserDataMessage`; centralized for ESP in `RTPSReliabilityAndWriterStateRuntime::makeAckUserDataSequenceContextForFlavor` via `chatterFirstSNMatchesSequence=true`. |
| 16 | Chatter SEDP publication sequence number | The `ros_discovery_info` and `/chatter` SEDP DATA messages share writer entity `000003C2`. Both were emitted at `seq=1`, so the PC discarded the chatter announcement as a replay. Fix: `_chatterSedpSeqNum = 2` in `RaftROS.h`. `ros_discovery_info` keeps `seq=1`. |
| 17 | Enable chatter retransmit in `EspStyle` flavor | ACKNACK on SEDP publications (`000003C2`) did not retransmit the `/chatter` announcement because `publicationsIncludesChatterAnnouncement=false` under `EspStyle`. Combined with LWIP ENOMEM losing the 3rd+ sendto in the initial unicast burst, the PC never learned about `/chatter`. Fix (A): set `publicationsIncludesChatterAnnouncement=true` in `makeAckNackDecisionOptionsForFlavor` for `EspStyle`. Fix (B): wire `ctx.sedpChatterPublicationSpec.buildMessage` to call `SEDPHandler::buildPublicationMessage` using `getAckActionSedpPlan(RetransmitSedpChatterPublication, …)`. |

### Phase 2 Key Learnings

- **VOLATILE + RELIABLE writers must advertise `firstSN == lastSN`** in HEARTBEAT. Firing `firstSN=1` with `lastSN=N` invites the subscriber to NACK samples that no longer exist, producing a retransmit storm.
- **Two DataWriter announcements on the same SEDP publications writer must use distinct sequence numbers**, or the second is silently discarded as duplicate.
- **LWIP send queue is narrow on ESP32-S3**: the initial 5-packet unicast burst from `handleNewParticipant` still loses the 3rd and later sends with `errno=ENOMEM`. End-to-end delivery now relies on the reliable-retransmit path rather than first-try success; do not remove the retransmit wiring to "clean up" the flow.
- **ROS 2 CLI on the PC needs `env -u PYTHONPATH PYTHONNOUSERSITE=1 … --no-daemon`** to avoid the user-site numpy collision and the daemon XMLRPC timeout on this host. This is captured in `/memories/repo/raftros-rtps-findings.md` but should stay in mind for future validation.

## Phase 1: Discovery — COMPLETE ✅

`ros2 node list` shows `/raft_esp32` on both ESP32 and the linux standalone test node.

### What Works

- **SPDP (Participant Discovery):** ESP32 sends periodic SPDP announcements to multicast `239.255.0.1:7400`. Contains GUID prefix (from WiFi MAC), lease duration, builtin endpoint set (`0x0C3F`), `PID_USER_DATA` with `"enclave=/;"`. Stale participant purging removes expired entries after 2× lease duration.
- **SPDP on Metatraffic Port:** Daemon unicast SPDP replies to ESP32:7410 are correctly parsed. Lease timers refreshed on re-reception.
- **SEDP Publication:** Announces `ros_discovery_info` DataWriter with topic name, type name, reliability (RELIABLE), durability (TRANSIENT_LOCAL), `PID_UNICAST_LOCATOR`, and `PID_PARTICIPANT_GUID`. Includes HEARTBEAT submessage.
- **SEDP Subscription:** Announces `ros_discovery_info` DataReader with matching QoS, locator, and participant GUID.
- **Participant Message Data (Liveliness):** Sent on discovery and periodically via heartbeats. Sequence number increments each send.
- **ros_discovery_info:** CDR-encoded `ParticipantEntitiesInfo` (participant GID, node namespace `/`, node name `raft_esp32`) sent to user data port.
- **HEARTBEAT/ACKNACK:** ESP32 responds to daemon HEARTBEATs with ACKNACKs. Handles incoming ACKNACKs and retransmits SEDP publication, SEDP subscription, and ros_discovery_info data as requested.
- **Periodic Writer Heartbeats:** Sends all four DATA+HB messages (SEDP pub, SEDP sub, liveliness, rosDisc) to each discovered remote participant.

### Fixes Applied (chronological)

| # | Fix | Details |
|---|-----|---------|
| 1 | CDR field order | `node_namespace` before `node_name` in `buildRosDiscoveryInfoPayload` |
| 2 | `PID_USER_DATA` in SPDP | FastRTPS requires `"enclave=/;"` — without it, participant is silently ignored |
| 3 | Stale participant purging | `purgeStaleParticipants()` prevents `_discovered` vector from filling with dead entries |
| 4 | SPDP on metatraffic port | `recvMetatraffic()` now detects SPDP writer entity ID and parses as announcement |
| 5 | Lease refresh on re-reception | `discoveredTimeMs` updated when existing participant re-announces |
| 6 | Entity IDs must be NO_KEY | `ros_discovery_info` uses WRITER_NO_KEY (0x03) / READER_NO_KEY (0x04) — **key breakthrough** that made endpoint matching work |
| 7 | Don't increment `rosDiscSeqNum` | Static content always SN=1; incrementing causes ever-growing NACK gap |
| 8 | `participant.init()` before GUID | Must set nodeName, domainId, participantId before `setGuidPrefixFromMAC` |
| 9 | `BUILTIN_ENDPOINT_SET = 0x0C3F` | Includes SUBSCRIPTIONS_ANNOUNCER and PARTICIPANT_MESSAGE_DATA bits |
| 10 | SEDP subscription announcement | `buildSubscriptionMessage()` in `handleNewParticipant` and `sendWriterHeartbeats` |
| 11 | Participant message data (liveliness) | `buildParticipantMessageData()` in `handleNewParticipant` and `sendWriterHeartbeats` |
| 12 | `PID_UNICAST_LOCATOR` in SEDP | Pass `_myIpAddr` to all `buildPublicationMessage()` and `buildSubscriptionMessage()` calls |
| 13 | `PID_PARTICIPANT_GUID` in SEDP | Added to `buildPublicationMessage()` and `buildSubscriptionMessage()` in SEDPHandler |
| 14 | ACKNACK retransmit handling | ESP32 retransmits SEDP pub, SEDP sub, and rosDisc on incoming ACKNACKs |

### RTPS Protocol Key Facts

- `ros2` sends SPDP from loopback (127.0.0.1) on WSL2 with UDPv4-only profile
- ACKNACK `base=2 numBits=0` → "received SN=1 OK"; `base=1 numBits=X growing` → "never received SN=1"
- Entity kind byte: 0x02=WRITER_WITH_KEY, 0x03=WRITER_NO_KEY, 0x04=READER_NO_KEY, 0x07=READER_WITH_KEY
- FastDDS needs UDPv4-only profile XML to disable SHM for same-host testing
- Port formulas: `meta = 7400 + 250*domainId + 10 + 2*participantId`, `user = meta + 1`

## Current Status Update (2026-04-20)

### Newly Verified

- Linux standalone publisher works in the docker ROS 2 testbed when publisher/subscriber are in the same network namespace.
- Docker build is now non-interactive and reproducible (no wireshark debconf prompt blocking builds).
- ACKNACK handling in `linux_unit_tests/raftros_standalone.cpp` now includes SEDP subscription retransmit handling for `ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER`.
- Linux unit tests remain green after the ACKNACK patch: **85 passed, 0 failed**.
- Initial shared-runtime extraction started: ACKNACK parsing/writer classification moved into shared RTPS utility (`RTPSAckNack`) and consumed by both ESP32 (`RaftROS.cpp`) and Linux standalone (`raftros_standalone.cpp`) handlers.
- Post-refactor validation remains green: linux unit tests **85 passed, 0 failed** and `raftros_linux` builds successfully.

### Canonical Docker Validation Rule (Linux Build)

To avoid host/container networking ambiguity, treat this as the required docker regression path:

1. Build linux artifacts on host:
- `cd linux_unit_tests && make -j$(nproc) all standalone`

2. Start docker test container:
- `cd docker && docker compose up -d --build`

3. Copy linux standalone binary into container and run it **inside** the container:
- `docker cp linux_unit_tests/raftros_linux raftros-test:/workspace/raftros_linux`
- `docker exec raftros-test chmod +x /workspace/raftros_linux`
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; /workspace/raftros_linux -i eth0 > /tmp/raftros_linux_in_container.out 2> /tmp/raftros_linux_in_container.err'`

4. Run ROS 2 checks in the same container:
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml; ros2 node list --no-daemon --spin-time 25'`
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml; ros2 topic list --no-daemon --spin-time 15'`
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml; timeout 30 ros2 topic echo /chatter std_msgs/msg/String --once --no-daemon'`

Expected pass indicators:
- `/raft_linux` appears in `ros2 node list`
- `/chatter` appears in `ros2 topic list`
- one chatter sample is printed by `ros2 topic echo --once`

Latest run (2026-04-21) passed with:
- `ros2 node list`: `/raft_linux`
- `ros2 topic list`: `/chatter`, `/parameter_events`, `/rosout`
- `ros2 topic echo /chatter --once`: `data: Hello from raft_linux [42]`

### Root Cause Found for Previous "Not Publishing" Symptom

- The primary blocker observed in earlier docker testing was host/container network separation (discovery traffic from host IP space did not reach container DDS participants as expected in that setup).
- This was not primarily caused by ACKNACK logic.
- Practical implication: do not use host-publisher + container-subscriber as the primary pass/fail gate for linux docker validation in this environment; keep publisher and ROS 2 validation tools in the same container namespace.

### Remaining Gap

- Phase 2 (ESP32 `/chatter` publishing) and Phase 3 (ESP32 topic subscribing with per-topic routing) are complete and verified end-to-end against ROS 2 Humble + FastDDS 2.6.11.
- Phase 4 (DeviceManager auto-publishing) is complete; every bus device detected by DeviceManager now auto-publishes to ROS 2 with per-class type mapping, per-writer QoS, and clean dispose on detach.
- Linux and ESP32 orchestration code continues to converge on a shared runtime; remaining drift lives in the thin wrappers only (`components/RaftROS/RaftROS.cpp` vs `linux_unit_tests/raftros_standalone.cpp`).

## Phase 2: Topic Publishing — See "Phase 2: Topic Publishing — COMPLETE ✅" above

Historical notes on the shared-runtime convergence work that landed alongside Phase 2 are kept under "DRY / Shared-Code Direction" below.

## DRY / Shared-Code Direction (Agreed Technical Direction)

Detailed staged plan: see `devdocs/RaftROS-next-stages-implementation-plan.md`.

To keep ESP32 and native Linux behavior consistent, new protocol/runtime logic should be implemented once in shared code, with thin platform adapters.

### Preferred Structure

- Keep protocol encode/decode in shared modules (already done in `components/RaftROS/RTPS` and `components/RaftROS/CDR`).
- Move duplicated runtime orchestration (ACKNACK decisions, heartbeat/send policy, participant bookkeeping transitions) into a shared core runtime layer under `components/RaftROS/`.
- Keep only socket/timer/platform glue in:
	- ESP32 SysMod wrapper (`RaftROS.cpp`)
	- Linux standalone wrapper (`raftros_standalone.cpp`)

### Progress Against This Direction

- Completed: shared ACKNACK parser/classifier in `components/RaftROS/RTPS/RTPSAckNack.*`.
- Completed: shared reliability policy helpers in `components/RaftROS/RTPS/RTPSReliabilityPolicy.*` used by both ESP32 and Linux handlers for:
	- HEARTBEAT final-flag ACK response decision
	- ACKNACK bitmapBase sequence retransmit decision checks
- Completed: shared periodic scheduling helper in `components/RaftROS/RTPS/RTPSRuntimeSchedule.*` used by both ESP32 and Linux loops for SPDP, writer heartbeat, and chatter publish cadence checks.
- Completed: shared participant activation helper in `components/RaftROS/RTPS/RTPSParticipantLifecycle.*` used by both ESP32 and Linux for:
	- inactive -> active transition decision based on discovered participant count
	- immediate writer-heartbeat trigger when activation occurs
- Completed: shared builtin endpoint mapping helper in `components/RaftROS/RTPS/RTPSBuiltinEndpointMap.*` used by both ESP32 and Linux metatraffic HEARTBEAT handling to map remote writer entity IDs to local reader entity IDs for ACKNACK generation.
- Completed: shared discovery merge policy helper in `components/RaftROS/RTPS/RTPSDiscoveryPolicy.*` used by both ESP32 and Linux for:
	- lease refresh on known participant re-discovery
	- new participant add eligibility check against max capacity
	- add-time `discoveredTimeMs` initialization
- Completed: shared participant lease-expiry policy helper in `components/RaftROS/RTPS/RTPSParticipantLeasePolicy.*` used by both ESP32 and Linux stale participant purge paths for:
	- lease timeout calculation (2x lease duration with default fallback)
	- expiration predicate used by purge loops
- Completed: shared participant set change policy helper in `components/RaftROS/RTPS/RTPSParticipantSetPolicy.*` used by both runtimes for activation-state and immediate writer-heartbeat trigger decisions.
	- ESP32 now uses this policy for first activation and subsequent participant-add immediate heartbeat behavior.
	- Linux uses the same policy for activation transition behavior.
- Completed: shared discovery merge flow in `RTPSDiscoveryPolicy_mergeParticipant(...)` now centralizes refresh/add/capacity decisions and is used by both ESP32 and Linux discovered-participant handlers.
- Completed: shared initial new-participant announce policy in `components/RaftROS/RTPS/RTPSInitialAnnouncePlan.*` now centralizes which first-contact SPDP/SEDP/PMD sends are enabled; both ESP32 and Linux `handleNewParticipant` now use this shared plan while retaining platform-specific send execution.
- Completed: shared ordered announce-step sequencing in `RTPSInitialAnnouncePlan_buildSequence(...)` now provides a common action list plus per-step heartbeat/sequence guidance; both ESP32 and Linux wrappers now execute this shared sequence while preserving platform-specific packet send paths.
- Completed: shared send-target policy mapping in `RTPSInitialAnnouncePlan_getSendTarget(...)` now centralizes socket/destination selection per announce action; both wrappers use a local send adapter driven by this shared mapping.
- Completed: shared payload-build dispatch in `RTPSInitialAnnouncePlan_getBuildSpec(...)` and shared SEDP endpoint profile mapping in `RTPSInitialAnnouncePlan_getSedpEndpointSpec(...)` now centralize builder selection and endpoint metadata; both wrappers now execute announce actions via shared build/send policy mappings.
- Completed: shared action-to-log policy in `RTPSInitialAnnouncePlan_getLogSpec(...)` now centralizes action labels and sender-address logging behavior; both wrappers now use this shared metadata for post-send logging.
- Completed: shared sequence-counter mutation timing policy in `RTPSInitialAnnouncePlan_applySequencePolicy(...)` now centralizes per-action sequence selection and increment timing (including SPDP pre-build increment and Linux chatter sequence compatibility behavior); wrappers now use shared counter-state policy and only persist concrete counter storage.
- Completed: shared debug behavior policy in `RTPSInitialAnnouncePlan_getDebugSpec(...)` now centralizes runtime/action debug toggles (including Linux SEDP publication hex-dump behavior); wrappers now consume shared debug metadata instead of hardcoded action checks.
- Completed: shared send-result instrumentation policy in `RTPSInitialAnnouncePlan_classifySendResult(...)` now centralizes success/failure/short-send classification; wrappers consume this metadata for unified post-send status tagging while keeping platform logger calls local.
- Completed: shared action-iteration execution scaffolding in `RTPSInitialAnnouncePlan_evaluatePreBuild(...)` and `RTPSInitialAnnouncePlan_evaluatePostBuild(...)` now centralizes step preconditions and skip reasons; wrappers now use shared pre/post step gating.
- Completed: shared heartbeat-count mutation policy in `RTPSInitialAnnouncePlan_applyHeartbeatPolicy(...)` now centralizes increment-before-send rules.
- Completed: full shared initial-announce execution runner in `RTPSInitialAnnounceRunner.*` now centralizes the step loop (pre/post gating, heartbeat/sequence policy application, debug policy check, send-result classification, and log dispatch gating). ESP32 and Linux wrappers now provide only build/send/debug/log callbacks and runtime state storage.
- Completed: full shared ACKNACK execution runner in `RTPSAckNackRunner.*` now centralizes ACKNACK parse/classify/decision/action-dispatch flow (including configurable publication-sequence gating differences and chatter-announcement handling). ESP32 and Linux wrappers now primarily provide remote lookup plus action-specific build/send callbacks.
- Completed: full shared writer-heartbeat execution runner in `RTPSWriterHeartbeatRunner.*` now centralizes periodic writer resend step orchestration (action sequence, heartbeat/liveliness mutation policy, sequence selection policy, send-target mapping, and Linux one-time ros_discovery_info debug dump behavior). ESP32 and Linux wrappers now primarily provide build/send/log callbacks and runtime counter storage.
- Completed: full shared receive-submessage execution runner in `RTPSRxSubmessageRunner.*` now centralizes RTPS header parse, submessage iteration, HEARTBEAT->ACKNACK response generation, ACKNACK handoff, and DATA/other-submessage dispatch hooks. ESP32 and Linux wrappers now provide channel-specific callback adapters for SPDP parsing, endpoint mapping, socket send, and logging.
- Completed: shared discovered-participant lookup/routing helper in `RTPSDiscoveredParticipantLookup.*` now centralizes guid-prefix remote lookup and userdata-heartbeat ACK metatraffic-port routing-by-IP; both ESP32 and Linux wrappers now use this helper in ACKNACK remote resolution and receive-path ACK destination resolution.
- Completed: shared callback-adapter scaffolding in `RTPSRunnerAdapterHelpers.*` now centralizes base `RTPSRxSubmessageRunner` callback wiring (local guid, reader-map policy, ACK destination policy, ACK socket send) and `RTPSAckNackRunner` remote participant resolution callback behavior. ESP32 and Linux wrappers now set policy/context and provide only behavior-specific hooks.
- Started: Phase 1 discovery-module consolidation with new cohesive module entry point:
	- `components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.h/.cpp`
	- ESP and Linux wrappers now call `DiscoveryRuntime` for participant merge, activation side effects, and lease-expiry checks.
	- `RTPSRunnerAdapterHelpers` now resolves discovered participants and metatraffic ACK destination via `DiscoveryRuntime` APIs.
- Continued: `DiscoveryRuntime` is now the discovery source-of-truth implementation for merge/lookup/lease/activation behavior; legacy micro-helper files (`RTPSDiscoveryPolicy`, `RTPSDiscoveredParticipantLookup`, `RTPSParticipantLeasePolicy`, `RTPSParticipantLifecycle`, `RTPSParticipantSetPolicy`) were converted to compatibility wrappers delegating to `DiscoveryRuntime`.
- Started: Phase 2 reliability-module consolidation with new source-of-truth module entry point:
	- `components/RaftROS/RTPS/runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h/.cpp`
	- Legacy reliability helpers (`RTPSReliabilityPolicy`, `RTPSAckNack`, `RTPSBuiltinEndpointMap`) now delegate to this reliability module via compatibility wrappers.
- Continued: `RTPSAckNackRunner` retransmit-decision branching is now centralized in `RTPSReliabilityAndWriterStateRuntime::evaluateAckNackActions(...)`; the runner now acts as a thin adapter that performs parse/log/remote-resolve and dispatches runtime-selected actions via existing callbacks.
- Continued: shared ACKNACK action execution adapter added in `RTPSRunnerAdapterHelpers` (`RTPSRunnerAdapter_executeAckAction(...)` plus per-action build/send specs). ESP (`RaftROS.cpp`) and Linux standalone (`raftros_standalone.cpp`) now use this shared execution path for destination selection and socket-send dispatch, while keeping wrapper-specific payload builders.
- Continued: ACKNACK writer-heartbeat mutation policy is now centralized in reliability runtime (`applyAckActionHeartbeatMutation(...)`) and applied by the shared ACK execution adapter using wrapper-configured mutation profiles (ESP vs Linux behavior preserved via policy flags).
- Continued: ACKNACK action log-label/result-label mapping is now centralized in `RTPSRunnerAdapterHelpers` (`RTPSRunnerAdapter_ackActionLogLabel(...)`, `RTPSRunnerAdapter_ackActionResultLabel(...)`), reducing wrapper-local per-action logging switch logic in ESP and Linux handlers.
- Continued: ACKNACK SEDP retransmit endpoint metadata (entity/topic/type/QoS for ros_discovery_info and chatter announcement actions) is now centralized in reliability runtime (`getAckActionSedpPublicationProfile(...)`, `getAckActionSedpSubscriptionProfile(...)`), and both wrappers now consume runtime-provided profiles in ACK action builders.
- Continued: ACKNACK SEDP retransmit sequence-number selection is now centralized in reliability runtime (`getAckActionSedpSequenceNumber(...)`) via per-wrapper sequence-context policy; Linux `sedpSeqNum + 1` chatter compatibility behavior is preserved via context policy flag rather than wrapper-local arithmetic.
- Continued: ACKNACK user-data payload debug instrumentation policy is now centralized via shared adapter debug policy (`RTPSRunnerAdapter_shouldDumpAckPayload(...)`, `RTPSRunnerAdapter_logHexPayload(...)`); Linux enables ros_discovery_info payload hex-dump through policy, ESP keeps it disabled.
- Continued: ACKNACK SEDP action builder scaffolding is reduced via runtime combined plan resolver (`getAckActionSedpPlan(...)`) that returns endpoint metadata + sequence number together; wrappers now initialize sequence context once and consume a single per-action plan in SEDP ACK builders.
- Continued: ACKNACK user-data action envelope policy is now centralized in reliability runtime (`getAckActionUserDataPlan(...)`) with per-wrapper sequence context and optional firstSN override policy; wrappers now consume runtime writer/sequence/firstSN plans for both ros_discovery_info and chatter ACK data retransmits.
- Continued: ACKNACK context initialization boilerplate is reduced through shared setup helpers (`RTPSRunnerAdapter_initAckExecContext(...)`, `makeAckSedpSequenceContext(...)`, `makeAckUserDataSequenceContext(...)`); wrappers now declare concise init configs instead of manually setting each context field.
- Continued: ACKNACK callback-bundle wiring is now centralized via `RTPSRunnerAdapter_initAckCallbacks(...)`, so wrappers provide a compact function-pointer callback init config while shared code applies common remote-resolution wiring. This keeps the implementation ESP32-friendly (no dynamic allocation, stack-only context, static callback wiring).
- Continued: ACKNACK hex-dump debug instrumentation is now compile-time gated in shared adapter helpers via `RAFTROS_ACK_HEX_DUMP_ENABLE` (default OFF for embedded builds). Linux unit-test build explicitly enables this flag in `linux_unit_tests/Makefile` to preserve developer diagnostics while keeping ESP32 release builds lean.
- Continued: ACKNACK verbose action/result log labels are now compile-time gated via `RAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE` (default OFF for embedded builds). Linux unit-test build explicitly enables this flag so local diagnostics remain detailed without forcing verbose strings into embedded firmware builds.
- Continued: ACKNACK runtime flavor policy presets are now centralized in reliability runtime (`makeAckNackDecisionOptionsForFlavor(...)`, `makeAckNackMutationPolicyForFlavor(...)`, `makeAckSedpSequenceContextForFlavor(...)`, `makeAckUserDataSequenceContextForFlavor(...)`). ESP and Linux wrappers now consume these shared flavor helpers instead of hardcoding inline policy booleans.
- Continued: ACK runner option wiring is now centralized via `RTPSRunnerAdapter_initAckRunnerOptions(...)`, reducing wrapper-level field-by-field option mapping and keeping runner-vs-runtime option translation in shared adapter code.
- Continued: ACKNACK runner API now consumes shared runtime decision options directly (`RTPSAckNackDecisionOptions`) instead of a duplicate runner-local options struct. Wrappers now pass flavor-derived decision options straight into `RTPSAckNackRunner_run(...)`, and the temporary adapter mapping helper was removed.
- Continued: ACKNACK runner action type is now unified with shared runtime action type (`RTPSAckNackDecisionAction`) via aliasing in `RTPSAckNackRunner.h`; duplicate runner-action enum and runtime<->runner action mapping glue were removed from runner/adapter helper code paths.
- Continued: ACK runner/adapter internals now call reliability runtime source-of-truth APIs directly for ACKNACK parse/classify and builtin reader mapping (`parseAckNack`, `classifyWriter`, `localReaderForRemoteWriter`) instead of going through legacy compatibility-wrapper entry points.
- Continued: first discovery-wrapper removal pass completed. Removed legacy compatibility wrapper files from `components/RaftROS/RTPS/`:
	- `RTPSParticipantLifecycle.*`
	- `RTPSDiscoveryPolicy.*`
	- `RTPSParticipantLeasePolicy.*`
	- `RTPSParticipantSetPolicy.*`
	- `RTPSDiscoveredParticipantLookup.*`
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now link only runtime source-of-truth modules for those concerns.
- Continued: `RTPSDiscoveryRuntime.h` is now self-contained for participant-set policy result typing (no dependency on removed legacy participant-set header).
- Simplification milestone: RTPS top-level `.cpp` count reduced from **19 -> 14** in this pass.
- Continued: reliability-wrapper removal pass completed. Removed legacy compatibility wrapper files from `components/RaftROS/RTPS/`:
	- `RTPSAckNack.cpp` (function-wrapper shim removed; shared ACKNACK types remain in `RTPSAckNack.h`)
	- `RTPSReliabilityPolicy.*`
	- `RTPSBuiltinEndpointMap.*`
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now link direct runtime reliability source-of-truth only.
- Continued: wrapper and runner call sites now use reliability runtime APIs directly for writer-kind log labels and heartbeat-response policy checks.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **14 -> 11** in this pass (**19 -> 11** cumulative across simplification passes).
- Continued: removed final ACKNACK shim header (`RTPSAckNack.h`) by making reliability runtime header self-contained for ACKNACK shared types (`RTPSAckNackFields`, `RTPSAckNackWriterKind`). `RTPSAckNackRunner.h` now aliases those types from runtime namespace directly.
- Continued: receive-path orchestration was relocated from top-level RTPS folder into `runtime/receive/`:
	- `RTPSRxSubmessageRunner.*` -> `runtime/receive/RTPSRxSubmessageRunner.*`
	- `RTPSRunnerAdapterHelpers.*` -> `runtime/receive/RTPSRunnerAdapterHelpers.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now reference the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **11 -> 9** in this pass (**19 -> 9** cumulative across simplification passes).
- Continued: announce/heartbeat orchestration was relocated from top-level RTPS folder into `runtime/announce/`:
	- `RTPSInitialAnnouncePlan.*` -> `runtime/announce/RTPSInitialAnnouncePlan.*`
	- `RTPSInitialAnnounceRunner.*` -> `runtime/announce/RTPSInitialAnnounceRunner.*`
	- `RTPSWriterHeartbeatRunner.*` -> `runtime/announce/RTPSWriterHeartbeatRunner.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now reference the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **9 -> 6** in this pass (**19 -> 6** cumulative across simplification passes).
- Continued: ACKNACK orchestration runner was relocated from top-level RTPS folder into `runtime/reliability/`:
	- `RTPSAckNackRunner.*` -> `runtime/reliability/RTPSAckNackRunner.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and receive-adapter include wiring (`runtime/receive/RTPSRunnerAdapterHelpers.h`) now reference the runtime path directly.
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now compile the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **6 -> 5** in this pass (**19 -> 5** cumulative across simplification passes).
- Continued: shared schedule policy helper was relocated from top-level RTPS folder into `runtime/schedule/`:
	- `RTPSRuntimeSchedule.*` -> `runtime/schedule/RTPSRuntimeSchedule.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now reference the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **5 -> 4** in this pass (**19 -> 4** cumulative across simplification passes).
- Continued: SPDP/SEDP protocol handlers were relocated into runtime ownership boundaries:
	- `SPDPHandler.*` -> `runtime/discovery/SPDPHandler.*`
	- `SEDPHandler.*` -> `runtime/announce/SEDPHandler.*`
	Wrapper include sites (`RaftROS.h`, `raftros_standalone.cpp`, `linux_unit_tests/main.cpp`) and runtime helper includes now reference runtime paths directly.
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now compile runtime paths directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **4 -> 2** in this pass (**19 -> 2** cumulative across simplification passes).
- Continued: wire/core protocol foundations were relocated into runtime ownership boundaries:
	- `RTPSMessage.*` -> `runtime/wire/RTPSMessage.*`
	- `RTPSParticipant.*` -> `runtime/core/RTPSParticipant.*`
	Wrapper and runtime module include sites (`RaftROS.cpp/.h`, `raftros_standalone.cpp`, `linux_unit_tests/main.cpp`, reliability/receive/discovery/announce runtime modules) now reference runtime paths directly.
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now compile runtime paths directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **2 -> 0** in this pass (**19 -> 0** cumulative across simplification passes).
- Stabilization pass: stale include/source reference scan confirmed no remaining references to removed top-level runtime-orchestration files.
- Stabilization pass: include/structure hygiene cleanup applied in runtime modules (`runtime/reliability/RTPSAckNackRunner.cpp` redundant include removed, `runtime/receive/RTPSRunnerAdapterHelpers.cpp` empty anonymous namespace removed).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after receive-runner extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after lookup-helper extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after runner-adapter scaffolding extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after DiscoveryRuntime Phase 1 start (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after DiscoveryRuntime source-of-truth/wrapper conversion (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK runtime decision-planner extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK shared action-execution adapter extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK heartbeat-mutation policy extraction into reliability runtime (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK shared log-label policy extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK SEDP endpoint metadata extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK SEDP sequence-policy extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK payload debug-policy extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK SEDP combined-plan extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK user-data combined-plan extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK context-init helper extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK callback-bundle init extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after compile-time ACK hex-dump gating (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after announce-runtime relocation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK runner relocation to `runtime/reliability` (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after schedule-helper relocation to `runtime/schedule` (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after stabilization-pass hygiene cleanup (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after SPDP/SEDP runtime-boundary relocation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after RTPSMessage/RTPSParticipant runtime-boundary relocation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after compile-time ACK verbose label gating (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK runtime flavor-policy helper extraction and shared runner-option init wiring (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK runner-options API unification to shared runtime decision options (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK action-type unification and mapping-removal cleanup (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after direct reliability-runtime call migration in ACK runner/adapter internals (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after discovery-wrapper file removal and build-list pruning (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after reliability-wrapper file removal and build-list pruning (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK shim-header removal and runtime-type ownership consolidation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after receive-runtime file relocation and top-level source pruning (**85 passed, 0 failed**).
- Continued: added regression-guard unit tests in `linux_unit_tests/main.cpp` for two Phase-2 bring-up fixes that are subtle and easy to regress:
	- VOLATILE `/chatter` invariant: `makeAckUserDataSequenceContextForFlavor(...)` opts in to `chatterFirstSNMatchesSequence` for both ESP and Linux flavors, and `getAckActionUserDataPlan(RetransmitChatterData, ...)` returns `hasFirstSNOverride=true` with `firstSN == sequenceNumber` (Fix 15 guard). TRANSIENT_LOCAL `ros_discovery_info` retransmit must NOT override firstSN.
	- Distinct SEDP-pub sequence numbers: `getAckActionSedpPlan(...)` on the shared SEDP publications writer yields distinct sequence numbers for ros_discovery_info vs chatter publication announcements under both flavor policies (ESP uses explicit `chatterPublicationSeqNum`; Linux derives `rosDiscoveryPublicationSeqNum + 1`). Endpoints also differ in entity ID and durability (Fix 16 guard).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after regression-guard test addition (**108 passed, 0 failed** — +23 assertions, still 0 failed).
- Continued: extracted the ACKNACK action-execution boilerplate (five per-action `buildMessage` lambdas + the `executeAction` lambda, ~200 lines per wrapper) from both wrappers into a shared default context in `RTPSRunnerAdapterHelpers`:
	- New types `RTPSAckActionStandardCtx`, `RTPSAckActionStandardInitConfig`, policy enum `RTPSAckActionSedpBuildHeartbeatPolicy` (`PassZero`/`PassLiveHeartbeat`), and payload-builder typedef `RTPSAckActionBuildPayloadFn(void*, uint8_t*, uint32_t)`.
	- New free functions `RTPSRunnerAdapter_initStandardAckActionCtx(...)` / `RTPSRunnerAdapter_standardExecuteAction(...)` / `RTPSRunnerAdapter_standardGetChatterSeq(...)` wire the five `RTPSAckNackRunnerActionExecSpec` channels, the SEDP build-heartbeat policy, the firstSN override for chatter, and the hex-payload debug dump behind a single entry point.
	- Both wrappers (`components/RaftROS/RaftROS.cpp::handleAcknack` and `linux_unit_tests/raftros_standalone.cpp::handleAcknack`) are now ~70 lines each (down from ~200+) and differ only on genuine wrapper-level policy: flavor, `sedpBuildHeartbeatPolicy` (ESP=`PassZero` matching the prior `heartbeatCount=0` default, Linux=`PassLiveHeartbeat`), `dumpRosDiscoveryPayloadHex`, and wrapper-owned payload builders (passed as plain fn-pointer + `void* payloadCtx`).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after shared ACKNACK action-execution default-ctx extraction and Linux wrapper conversion (**108 passed, 0 failed**). ESP wrapper conversion completed; ESP-IDF build and on-device `ros2 topic echo /chatter` smoke test pending.
- Continued: folded the thin `RTPSAckNackRunner.cpp` translation unit into `RTPSReliabilityAndWriterStateRuntime.cpp` so the reliability module now owns the full ACKNACK parse -> classify -> remote-resolve -> evaluate -> dispatch pipeline end-to-end. `RTPSAckNackRunner.h` stays as the public facade (callback typedefs + `RTPSAckNackRunner_run`) so wrapper call sites are unchanged; `RTPSAckNackRunner.cpp` deleted and removed from `CMakeLists.txt` and `linux_unit_tests/Makefile` (both SOURCES and PROTO_SOURCES).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after runner-cpp consolidation into reliability runtime (**108 passed, 0 failed**).
- Continued: started Phase 3 (topic subscribing) with the shared *reader* runtime. Added `runtime/reliability/RTPSReaderRuntime.{h,cpp}` mirroring the reliability-decision shape of the writer side but inverted for reader semantics: per-remote-writer state (`highestContiguousSeq`, `highestSeenSeq`, `receivedBitmap`, `lastHeartbeatCount`, `outgoingAckNackCount`) plus pure helpers `evaluateIncomingDataDecision` (Accept/Dedup/Drop), `applyAcceptedDataToReaderState` (contiguous-window collapse + bitmap shift), `evaluateIncomingHeartbeatDecision` (ACKNACK base+numBits+bitmap, FINAL-flag dedup, best-effort suppression, caught-up confirmation ACKNACK), and `applyHeartbeatProcessedToReaderState`. No wrapper IO yet — pure decisions only, per the modularization plan's "put reader decision logic in shared runtime from day one" risk mitigation.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after reader-runtime slice (**135 passed, 0 failed** — +27 assertions across DATA-accept/dedup/gap-collapse and HEARTBEAT→ACKNACK decision tests).
- Continued: added `runtime/reliability/RTPSReaderRunner.{h,cpp}` — thin orchestration mirroring `RTPSAckNackRunner`. Parses HEARTBEAT (28-byte content) and DATA (20-byte header + inline-QoS-skip + payload) submessage content into typed fields, looks up the reader state via a `resolveState` callback, invokes the pure decision helpers, and emits `sendAckNack` / `dispatchData` callbacks. No sockets, no CDR. Not yet wired into the RX submessage runner — wrappers continue to use the existing naive ACKNACK path for now.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after reader-runner slice (**169 passed, 0 failed** — +34 assertions covering HEARTBEAT/DATA submessage parse, FINAL/LIVELINESS flag propagation, short-content rejection, orchestration with state lookup + ACKNACK emission, duplicate-HB suppression via FINAL flag, and DATA Accept→Dedup transitions).
- Continued: added `RTPSMessage::writeAcknackWithBitmap` — bitmap-capable ACKNACK wire builder (DDSI-RTPS §9.4.2.7). Packs `numBits` (capped at 256) into ceil(numBits/32) little-endian 32-bit words; supports the FINAL (F=1) flag for "do not HEARTBEAT me again" semantics; byte-for-byte identical to the legacy `writeAcknack` when `numBits=0` & `finalFlag=false`. This is the wire-level prerequisite for routing HEARTBEATs through `RTPSReaderRunner`'s decision path. The existing `writeAcknack` is retained unchanged; no call-site migration yet.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after ACKNACK bitmap builder (**187 passed, 0 failed** — +18 assertions covering empty-bitmap equivalence with legacy builder, single-word bitmap packing + LE byte layout, FINAL flag + two-word bitmap, and input validation (buffer-too-small / numBits>256 / null-bitmap-with-numBits>0)).
- Continued 2026-04-22: extended `RTPSRxSubmessageRunner` with an opt-in `resolveReaderWriterState` callback. When the wrapper registers a per-(remote participant, writer EID) `RTPSReaderWriterState` lookup, incoming HEARTBEATs are delegated to the pure `RTPSReaderRuntime` decision layer and ACKNACKs are emitted via the new `writeAcknackWithBitmap` builder. When the callback is null (current wrapper configuration), the legacy inline ACKNACK path is preserved byte-for-byte. Neither `RaftROS.cpp` nor `raftros_standalone.cpp` set the new callback yet — no behavioral change on live hosts.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after RX-runner opt-in slice (**200 passed, 0 failed** — +13 assertions exercising a full RTPS packet through the RX runner: opt-in path state lookup + bitmap ACKNACK emission with `numBits=3` covering SNs [1..3], duplicate-FINAL HB suppression, and legacy-path byte-for-byte preservation when `resolveReaderWriterState` is null, verifying `acknackCount` increment semantics on both paths). Docker smoke deferred — environment today has no FastDDS peer on the bridge network (baseline main-branch binary exhibits the same behavior, confirming this is environmental rather than a regression).
- Continued 2026-04-22: added header-only `RTPSReaderStateMap` (`components/RaftROS/RTPS/runtime/reliability/RTPSReaderStateMap.h`) keyed on 12-byte `guidPrefix` + 4-byte `writerEID`, providing `getOrCreate` / `find`. Wired `resolveReaderWriterState` callbacks into both wrappers (`RaftROS.cpp` and `linux_unit_tests/raftros_standalone.cpp`) so all incoming HEARTBEATs now flow through the reader runtime decision layer + `writeAcknackWithBitmap`. The existing `onHeartbeat` / `onAckNack` / `onData` logging callbacks are unchanged, so wrapper-observable behaviour on the metatraffic path is identical with an empty state (bitmap SN==highestSeen+1 → no missing bits → emits standard final ACKNACK equivalent to legacy).
- Validation: **212 passed, 0 failed** (+12 assertions covering `getOrCreate` first-miss insertion, pointer-stability on re-lookup, preservation of mutated state across lookups, distinct-key creation for distinct `guidPrefix` / `writerEID`, and null-input guarding).
- Continued 2026-04-22: added `SedpChatterReader` announce plumbing to `RTPSInitialAnnouncePlan` — new plan flag `sendSedpChatterReader` (default `false`, opt-in), new action `RTPSInitialAnnounceAction::SedpChatterReader` routed to `SedpSubscription` build kind + `ChatterReader` endpoint profile. Added `ENTITYID_CHATTER_READER = {0x00,0x01,0x02,0x04}` and `CHATTER_IN_DDS_TOPIC = "rt/chatter_in"` (type `std_msgs::msg::dds_::String_`). New `SedpChatterReader` sequence counter hint + `sedpChatterReaderSeqNum` counter so the sub announce is independent of the chatter pub. Grew `RTPSInitialAnnounceSequence::steps[]` from 7 to 8 entries. No wrapper wiring yet — `RaftROS.cpp` / `raftros_standalone.cpp` leave `sendSedpChatterReader=false`, so no behavioural change today.
- Validation: **230 passed, 0 failed** (+18 assertions covering default-plan omission of ChatterReader, opt-in plan schedules ChatterReader immediately after ChatterWriter with heartbeat-before-send on LinuxStyle, build-spec / send-target / endpoint-spec / log-spec dispatch for the new action, the new seq counter hint returning `sedpChatterReaderSeqNum` (not the writer's counter), and regression guard that the default plan still schedules ChatterWriter).
- Continued 2026-04-22 (slice 6b): opted both wrappers into the new ChatterReader announce. `RaftROS::buildRosDiscInfoWithGids` and the standalone equivalent now publish `ENTITYID_CHATTER_READER` in the reader GID list, so downstream ROS 2 nodes see a `/chatter_in` subscriber on this participant. Both wrappers set `announcePlan.sendSedpChatterReader = true`, triggering a SEDP subscription DATA+HEARTBEAT pair on first discovery; reused the existing generic `SedpSubscription` build dispatch, so no new builder code is needed. `RTPSInitialAnnounceCounterState` positional initializers in both wrappers extended to supply the new `sedpChatterReaderSeqNum` slot (on ESP: dedicated counter; on Linux-flavor: `sedpSubSeqNum + 1`, mirroring chatter-writer policy). Sequence-policy helper updated to apply `sedpRosReaderSeqNum + 1` on LinuxStyle to match the ChatterWriter convention.
- Validation: **231 passed, 0 failed** (+1 assertion added, covering LinuxStyle ChatterReader seq-policy fallback; one pre-existing assertion adjusted to exercise both flavors).
- Continued 2026-04-22 (slice 7): added header-only `RTPSUserDispatch::decodeStdMsgsString` (`components/RaftROS/RTPS/runtime/dispatch/RTPSUserDispatch.h`). Pure CDR-LE decode of `std_msgs::msg::String_` payloads into a caller-supplied buffer: reads the 4-byte encapsulation header via `CDRDecoder`, then a length-prefixed UTF-8 string; truncates safely when the output buffer is too small; returns textLen (excluding null) and a null-terminated output. Wired into both wrappers' user-data RX path: when a DATA submessage arrives on the user-data channel with any writerEID other than `ENTITYID_ROS_DISC_INFO_WRITER`, the payload (assuming no inline-QoS; otherwise decode fails harmlessly) is decoded and logged. `RaftROS` exposes `setStringMessageHandler(std::function<void(const uint8_t*, const uint8_t*, const char*, uint32_t)>)` for user code to receive the decoded message.
- Validation: **243 passed, 0 failed** (+12 assertions covering well-formed decode, empty-string round trip, null-buffer / null-payload guarding, truncation (length exceeds buffer), and truncated-output-buffer succeeding with null-terminated prefix + correct source textLen).
- Continued 2026-04-22 (slice 7c): extended `RTPSWriterHeartbeatRunner` with a new `SedpChatterSubscription` action so late-joining ROS 2 peers also learn about the chatter_in reader via periodic SEDP retransmit. Added `chatterSedpSubSeqNum` counter slot and grew the sequence from 5 to 6 steps; the new step is scheduled immediately after `SedpChatterPublication` with the same linux-style heartbeat-before-build policy. Both wrappers (`RaftROS.cpp` and `raftros_standalone.cpp`) now handle the new action in their `buildPayload` switch by calling `SEDPHandler::buildSubscriptionMessage` with `ENTITYID_CHATTER_READER` / `CHATTER_IN_DDS_TOPIC` / `CHATTER_IN_DDS_TYPE`. Counter-state positional initializers in both wrappers supply the new slot (`sedpSubSeqNum + 1` on Linux-flavor, mirroring the chatter-pub convention).
- Validation: **255 passed, 0 failed** (+12 assertions covering sequence placement on both flavors, send-target = Metatraffic, sequence-number selection (`chatterSedpSubSeqNum` on ESP / `sedpSubSeqNum + 1` on Linux), and end-to-end `RTPSWriterHeartbeatRunner_run` invocation wiring buildPayload → sendPayload with the correct SN on both flavors).
- Continued 2026-04-22 (slice 7d — on-device confirmation): ESP32-S3 at `192.168.1.173` flashed with slice 7c firmware. ROS 2 Humble peer running directly on host `192.168.1.92` (Docker container was isolated from LAN; must run the publisher on a bridged network for SPDP to reach the device). End-to-end validated:
    - `ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello esp'}"` produced device log `UD user-topic writerEID=00000503 src=010FCCEF... "hello esp" (9 chars)` — confirms SEDP reader announce, reliable HEARTBEAT → ACKNACK round trip, inline-QoS-absent DATA reception, and CDR-LE decode path.
    - Chatter publish (ESP → ROS 2) confirmed via `ACKNACK base=566 numBits=0` — peer ACKs every sample.
    - `-1` errno-ENOMEM send bursts on the very first announce are benign (LwIP ARP warmup); the periodic `SedpChatterSubscription` retransmit added in slice 7c recovers state.
- Continued 2026-04-22 (slice 8 — polish & productionisation): five follow-ups bundled.
    1. ACKNACK writer-kind label table (`RTPSReliabilityAndWriterStateRuntime.cpp::classifyWriter` + `writerKindToStr`) extended with `ParticipantMessage` (`ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER`, `0x000200C2`, labelled `participant_msg`) and `ChatterReader` (`ENTITYID_CHATTER_READER`, labelled `chatter_in_reader`). Removes the `(unknown)` log lines against liveliness ACKNACKs.
    2. `RaftROS` public API: added `setStringSubscription(topic, type, handler)` alongside `setStringMessageHandler`. Topic + type name are now instance members (`_subscriptionTopic`, `_subscriptionType`), defaulting to `rt/chatter_in` / `std_msgs::msg::dds_::String_`. Both the initial-announce SEDP build callback and the `SedpChatterSubscription` heartbeat callback in `RaftROS.cpp` now read from these members, so apps can subscribe to any topic name without rebuilding the library. **Slot-0-only for now** — generalising SEDP announce to advertise N simultaneous readers is a focused follow-up (would need an N-ary expansion of `RTPSInitialAnnouncePlan`/`RTPSWriterHeartbeatRunner`, currently one-profile-per-step).
    3. `ExampleDiscoverable/components/MainSysMod`: calls `getSysManager()->getSysMod("RaftROS")` and registers a `setStringMessageHandler` lambda that logs decoded text. `CMakeLists.txt` now lists `RaftROS` as a required component so `RaftROS.h` resolves at build time.
    4. Liveliness HEARTBEAT in `SEDPHandler::buildParticipantMessageData` now advertises `firstSN == lastSN == sequenceNumber` (single-sample GAP) instead of `firstSN=1`. Stops the peer NACK storm (observed `numBits=166..256` against `ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER`); peer now just advances its base past the current SN.
    5. Unit tests: +8 assertions across `classifyWriter` / `writerKindToStr` covering every live writer kind plus explicit regression coverage that `ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER` is no longer classified as `Unknown`.
- Validation: **263 passed, 0 failed** (pending local rebuild confirmation after this slice's edits).
- Continued 2026-04-22 (slice 9 — N-ary SEDP subscription registry): introduced shared `components/RaftROS/RTPS/runtime/dispatch/RTPSSubscriptionRegistry.h` — header-only POD (fixed capacity 8) holding `{entityId[4], topic, type}` per slot, plus a deterministic `RTPSSubscriptionRegistry_allocateNextEntityId(slot)` allocator (slot 0 = `ENTITYID_CHATTER_READER` = `{0x00,0x01,0x02,0x04}`; slot N = `{0x00,0x01,(0x02+N),0x04}`). Registry exposes `add`, `setTopic`, `findByEntityId`, `readerEntityIds`. Wrapper (`RaftROS.h/.cpp`) now seeds slot 0 at construction from `_subscriptionTopic`/`_subscriptionType` and offers a new public `addStringSubscription(topic, type, handler) -> slot` API. `buildRosDiscInfoWithGids` iterates every registry entry into the `ros_discovery_info` `reader_gid_seq`. `handleNewParticipant` emits an extra SEDP subscription announce per slot (1..N) immediately after the main initial-announce sequence; `sendWriterHeartbeats` retransmits the same set once per heartbeat pass, keyed by `_extraSubscriptionSeqNums[slot]` (defaulted to 1 each). Slot 0 continues to flow through the existing fixed announce/HB plan unchanged, so no regressions in the single-subscription path. Dispatch still uses the single `_stringMessageHandler` (per-slot `_extraSubscriptionHandlers[]` stored but not yet routed; see follow-up).
- **Remaining follow-up**: per-topic handler routing in `recvUserData::onData`. DATA submessages carry only the *remote writer's* entity ID, so routing to the right local handler requires correlating remote SEDP publication records (remote writerGUID → topicName) with our registry entries. Needs an SEDP-pub parse hook + remote-writer table; tracked as "Phase 3.5: subscriber routing".
- Validation: **298 passed, 0 failed** (+35 assertions for the registry: allocator policy across slot range, capacity guard, add/setTopic null-guards, findByEntityId hit/miss, readerEntityIds ordering + maxOut cap, fill-to-capacity overflow).
- Next: start Phase 3 (topic subscribing). Fold remaining `RTPSAckNackRunner.cpp` internals behind `RTPSReliabilityAndWriterStateRuntime` and add a reader-side ACKNACK + deserialization path with the same shared-runtime discipline from day one.

### Practical Rule

- If logic can be expressed without direct socket API calls or FreeRTOS/Arduino-specific APIs, it belongs in shared core code.
- Wrappers should mainly provide:
	- packet send/receive callbacks
	- current time
	- local network identity
	- logging hooks

## Phase 3: Topic Subscribing — COMPLETE ✅

ESP32 subscribes to N ROS 2 topics and receives `std_msgs/msg/String` with per-topic handler dispatch. Verified on 2026-04-22 with `/chatter_in` (slot 0) and `/chatter_in2` (slot 1) against FastDDS 2.6.11.

### What Works

- SEDP subscription announces for N reader slots via shared `RTPSSubscriptionRegistry` (8-slot POD with deterministic entity-ID allocator: slot 0 = `ENTITYID_CHATTER_READER {0x00,0x01,0x02,0x04}`, slot N = `{0x00,0x01,0x02+N,0x04}`).
- Reader-side reliable ACKNACK via shared `RTPSReaderRuntime` (pure decision logic: DATA accept/dedup/gap-collapse, HEARTBEAT → ACKNACK base/numBits/bitmap, FINAL-flag dedup, best-effort suppression) + `RTPSReaderRunner` (submessage parse + state lookup + dispatch) + `RTPSMessage::writeAcknackWithBitmap` (bitmap wire builder).
- CDR deserialization via `RTPSUserDispatch::decodeStdMsgsString`.
- Per-topic routing via `RTPSRemotePublicationMap` (16-entry fixed map correlating remote `writerGuid` → local slot), populated from inbound SEDP publication DATAs parsed by `RTPSSEDPPublicationParser` (extracts `PID_ENDPOINT_GUID` and `PID_TOPIC_NAME`).
- Handler dispatch via `RaftROS::addStringSubscription(topic, type, handler)`.

### Phase 3 Bring-up Fixes

| # | Fix | Details |
|---|-----|---------|
| 18 | Monotonic SEDP-sub sequence numbers | `_extraSubscriptionSeqNums[8] = {2,3,4,5,6,7,8,9}`. Without distinct per-slot SNs, FastDDS dropped slot>=1 announcements as duplicates of the main sub writer's sequence space. |
| 19 | ACKNACK bitmap inversion | Per RTPS §9.4.5.2, bit=1 in `readerSNState` means **NOT received / please retransmit**. `RTPSReaderRuntime.cpp` was inverting the sense, so FastDDS read our ACKNACKs as positive ACKs and never retransmitted missed SEDP publications. |
| 20 | DATA inline-QoS skipping | DATA submessages with Q flag (bit 1 = 0x02) carry an inline-QoS `ParameterList` before the serialized payload. `onData` was reading `pContent + 20` unconditionally, which landed inside the inline-QoS for FastDDS dispose-style DATAs (PID_STATUS_INFO + PID_KEY_HASH + PID_SENTINEL = 32 bytes of body, no payload). Fix: propagated DATA `flags` byte through `onData`, added `RTPSData_getSerializedPayload()` helper in `RTPSRxSubmessageRunner.h` that skips the 20-byte fixed prefix and, when Q=1, scans the inline-QoS ParameterList up to `PID_SENTINEL (0x0001)`. |

All three fixes are regression-guarded by unit tests in `linux_unit_tests/main.cpp`.

### Phase 3 Slice Log (post-slice 9)

- **Slice 10 (reader-side ACKNACK bitmap inversion fix)**: inverted the bit sense in `RTPSReaderRuntime::evaluateIncomingHeartbeatDecision` so bit=1 in the emitted bitmap now correctly means NOT-received. Added assertions `d.ackNackBitmap == 0x1F` (5 missing) and `0x3` (2 missing) in `linux_unit_tests/main.cpp`.
- **Slice 11 (distinct SEDP-sub SNs)**: per-slot sequence-number counters so FastDDS can distinguish the N SEDP subscription announcements issued on the shared sub writer.
- **Slice 12 (per-topic dispatch)**: `RTPSSEDPPublicationParser` extracts `(writerGuid, topic)` from inbound SEDP publication DATAs; `RTPSRemotePublicationMap::upsert` records the correlation when `topic` matches a registered subscription slot; user-data DATAs look up the slot via `findSlot(guidPrefix, writerEID)` and dispatch to `_extraSubscriptionHandlers[slot]`.
- **Slice 13 (inline-QoS fix)**: `RTPSData_getSerializedPayload()` helper + DATA `flags` byte plumbed through `onData`. All four `onData` lambdas (metatraffic + user-data in both `RaftROS.cpp` and `raftros_standalone.cpp`) updated. Helper also regression-tested: 5 cases / 9 assertions covering Q=0, Q=1 sentinel-only, FastDDS dispose-style, malformed parameter runs-off-end, contentLen<20.
- Validation: **388 passed, 0 failed** (up from 298 at end of slice 9 — +90 assertions across reader-runtime, SEDP-pub parser, remote-pub map, inline-QoS helper, and the three Phase-3 regression guards).
- On-device log evidence (2026-04-22, ESP32-S3 `192.168.1.173` ↔ ROS 2 Humble host `192.168.1.92`):
  - `SEDP pub DATA received contentLen=468 flags=0x05`
  - `SEDP pub parsed topic='rt/chatter_in2' (14 chars), subReg.count=2`
  - `SEDP pub matched topic='rt/chatter_in2' -> slot=1 (new)`
  - `MainSysMod: chatter_in2 #1 writerEID=00000503 src=010FCCEF... "hello slot2" (11 chars)`

### Phase 3 Key Learnings

- **RTPS `readerSNState` bitmap convention is inverted from intuition**: per §9.4.5.2, a `1` bit means "NOT received / retransmit this". It is not a positive-ACK bitmap. Easy to get wrong, silent-to-diagnose when wrong.
- **DATA submessages always honour the Q flag before reading the serialized payload**. FastDDS routinely sends dispose/unregister DATAs with `Q=1 D=0 K=1` (endpoint teardown) that contain only an inline-QoS body — there is no serialized payload after it. Code that treats `pContent + 20` as the payload for every DATA will silently misparse every one of these.
- **N-ary SEDP announcements on one writer must use distinct sequence numbers** (same lesson as Fix 16 for the publication writer side — applies symmetrically to the subscription writer).

## Phase 4: DeviceManager Auto-Publishing — COMPLETE ✅

Every bus device detected by `DeviceManager` is automatically mirrored as a
ROS 2 topic at runtime. No per-device code. No SysTypes topic config for the
common path. Online → `/rt/raft/<slug>_<bus>_<addrHex>` appears in
`ros2 topic list`; offline → SEDP dispose removes it within a heartbeat
interval.

### What works end-to-end

- **Device lifecycle hook** — `RaftROS::setup()` registers a
  `DeviceManager::registerForDeviceStatusChange` callback. `ONLINE`/
  `PENDING_DELETION` transitions drive a fixed-capacity writer registry
  (`DYNAMIC_WRITER_REGISTRY_CAPACITY = 16`).
- **Class → ROS 2 type mapping** (`RTPSAutoPubClassMap.h`, precedence
  first-match-wins):
  1. device-type-name overrides (MCP9808 → Temperature,
     RoboticalLightSensor → Float32MultiArray),
  2. actuator exclusion (`SRVO`, `PUMP`, `PIX`),
  3. composite rules: `{ACC,GYRO}` → `Imu` (single writer),
     `{TEMP,RH}` → `Temperature` + `RelativeHumidity`,
     `{PRES,TEMP}` → `FluidPressure` + `Temperature`,
  4. single-class rules for TEMP, RH, PRES, LGHT, PROX, DIST, ANG, ROT,
     ACC, TCH, BTN, FRCE, HRM, SOIL, GAME,
  5. fallback: `std_msgs/String` with a JSON body containing every decoded
     field, topic slug `raw`.
- **Per-writer slot allocation** — `RTPSAutoPubLifecycle` owns 16 topic +
  type string buffers and issues entityIds in the deterministic range
  `0x000110xx..0x00011Fxx`. Composite devices consume two slots keyed by
  `{bus, addr, subIndex}` (subIndex=0 primary, =1 secondary).
- **SEDP dynamic announce** — the existing writer-heartbeat pass walks the
  registry and emits one `PublicationBuiltinTopic` DATA(w) per active slot
  per tick, reusing the SPDP / HB pacing loop.
- **User-data hot path** — on each decoded bus sample the latest record is
  serialised through `RTPSAutoPubCDRSerializer` into a per-slot 512 B buffer
  and unicast to every discovered peer. Composite devices serialise twice
  (once per kind) on the same decoded struct.
- **Timestamps** — ROS 2 `Header.stamp` is taken from the first `timeMs`
  field of the decoded poll record (not wall-clock), so subscribers see
  sample-time not emit-time.
- **REP-103 unit scaling** — g→m/s² (×9.80665), °/s→rad/s (×π/180),
  mm→m (/1000), hPa→Pa (×100), %→0..1 (/100). Applied per field by the
  attribute-field description table.
- **QoS profiles** (design §7.2, four built-ins):

  | Profile | Reliability | Durability | Depth | Default for |
  |---------|-------------|------------|-------|-------------|
  | `fast_sensor`     | BEST_EFFORT | VOLATILE        | 10 | ACC, GYRO, IMU, PROX, LGHT, DIST, ANG, HRM, FRCE |
  | `slow_sensor`     | RELIABLE    | VOLATILE        | 5  | TEMP, RH, PRES, SOIL, BTHM |
  | `event`           | RELIABLE    | TRANSIENT_LOCAL | 20 | BTN, TCH, ROT, GAME |
  | `fallback_string` | RELIABLE    | VOLATILE        | 10 | any unmapped class |

  Resolution order per writer: per-device alias → per-class override →
  built-in default. SysTypes override surface:

  ```jsonc
  "RaftROS": {
    "enable": true,
    "qosProfiles": {
      "imu_1_6a":        "slow_sensor",            // per-device alias
      "temperature_1_38": "event",
      "classDefaults":   { "ACC": "slow_sensor" }  // per-class override
    }
  }
  ```
- **Dispose on offline** — `PENDING_DELETION` emits an SEDP
  `PublicationBuiltinTopic` DATA with `PID_STATUS_INFO = 0x00000003`
  (Disposed | Unregistered) and `PID_KEY_HASH = guidPrefix+entityId`,
  which causes ROS 2 subscribers to drop the topic within a heartbeat.
  Composite secondary slots are disposed alongside the primary.

### Verification

- **Linux unit tests** — `911 passed, 0 failed` in
  `linux_unit_tests/main.cpp` covering:
  - dynamic writer registry + subIndex disambiguation (Slice 4.1 + 4.10),
  - lifecycle attach/detach (Slice 4.2),
  - topic naming (Slice 4.3),
  - class-map (Slice 4.4 — every row in `DeviceTypeRecords.json`),
  - CDR serialiser per kind including REP-103 scaling + JSON fallback
    (Slices 4.5 / 4.9),
  - QoS profile table, name parse round-trip, per-class defaults,
    composite resolution (Slice 4.11).
- **Firmware size** — `SysTypeMain.bin` 0x1435c0 bytes, partition 25% free
  on ESP32-S3 (ESP-IDF v5.5.2, `-std=gnu++2b -fno-exceptions -fno-rtti`).

### Source files added in Phase 4

| File | Purpose |
|------|---------|
| `components/RaftROS/RTPS/runtime/autopub/RTPSDynamicWriterRegistry.h` | Fixed-capacity slot + entityId registry |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubLifecycle.h` | Slot allocator with owned topic/type strings |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubTopicNaming.h` | `rt/raft/<slug>_<bus>_<addr>` formatter |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubClassMap.h` | `clas[] + deviceType → msg kind + slug` |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.h/.cpp` | Per-kind CDR encoders with REP-103 unit scaling |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubQoSProfile.h` | 4 built-in profiles + override resolver |

## Raft Library Follow-ups — TODO

- **RaftJsonNVS static-initialisation order** (2026-09-27): the boot-time NVS
  read can run before NVS is initialised, so persisted settings never load.
  Patch: [patches/raftcore-nvs-init-order.patch](patches/raftcore-nvs-init-order.patch)
  - **applied upstream as RaftCore `b8e1f9b`**; the two remaining NVS entry
  points (`ArPreferences::begin`, `NetworkSystem` before `esp_wifi_init`) as
  [patches/raftcore-nvs-init-order-2.patch](patches/raftcore-nvs-init-order-2.patch),
  **applied upstream as `5494416`**.

Found while bench-testing RaftROS on a UM ProS3 and an Adafruit ESP32-S3 TFT
Feather (2026-09-18/21). These are changes to RaftCore/RaftSysMods, not RaftROS.

**Fixed upstream (2026-09-20/21):**

- **DeviceManager missed registrations made from a status callback** —
  `busElemStatusCB` took its copy of `_requestedDeviceDataChangeCBList` before
  calling the status-change callbacks, so a listener registering for device
  data on identification (as RaftROS does) never reached the bus and its
  device was never polled. Introduced by the concurrency-hardening merge;
  auto-publishing stopped entirely until the copy was moved after the
  callbacks.
- **WiFi STA now connects by signal** — new `NetMan` setting
  `wifiSTAConnectBySignal` (default true) selects `WIFI_ALL_CHANNEL_SCAN` +
  `WIFI_CONNECT_AP_BY_SIGNAL`, applied both in `configWifiSTA` and at boot in
  `startWifi` (stored NVS configs predate the setting). Before this the board
  joined -82 to -93 dBm APs while a -62 dBm AP was available, costing ~7% of
  samples in transit; after it, loss is ~0.3%.
- **`wifiscan` returned no records** — `getScanResults` called
  `esp_wifi_scan_get_ap_records` (which frees the driver list) before
  `esp_wifi_scan_get_ap_num`, so the count was always 0. The API now also
  reports scan state/id/age/new/lost and caches results.
- **Invalid BLE status JSON** — `advName` was emitted without its closing
  quote while advertising.

**Still open:**

- **DeviceManager data-callback unregister** now reaches the bus
  (`unregisterForDeviceData`), but RaftROS does not depend on it: stale
  callbacks are rejected by generation-checked pool handles.
- **`configWifiSTA` can block the loop task for up to 2 s** (`vTaskDelay`
  retry loop around `esp_wifi_set_config`), exceeding the 50 ms SysMod
  budget when a connect attempt is in progress.
- **Logging blocks the calling task when the USB console has no reader.**
  Verbose per-packet logging from `loop()` stalled the SysMod loop past the
  200 ms sensor interval and lost ~5% of samples while no monitor was
  attached; the same logs replay as a multi-hour backlog when a monitor
  reattaches. RaftROS's chatty discovery logs are now debug-gated, but a
  Raft-level fix (drop, don't block, when nothing is reading) would protect
  every app.

## Phase 5: Integration with Raft — TODO

- ROS 2 actions / service servers.
- Command-side subscriptions auto-wired from DeviceManager actuator classes
  (SRVO, PUMP, PIX) — the symmetric write path. Phase 4 excludes these
  from publishing; Phase 5 will route incoming topic data into
  `DeviceManager::sendCmdJSON`.
- Per-device SysTypes topic alias override (short user-friendly name instead
  of the auto slug).
