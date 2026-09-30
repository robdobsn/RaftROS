# RaftROS Zenoh Milestone: Measured Results

This is the findings record for the initial Zenoh milestone (completed
2026-09-28) and the services and parameters work that followed it
(2026-09-28/29): what was measured, on what, with what result. The dated sections
of [RaftROS-development-status.md](RaftROS-development-status.md) hold the
evidence and the fixes behind each number; this document is the summary.

**Hardware and setup for every figure below:** Adafruit ESP32-S3 TFT Feather,
VL6180 range sensor on the STEMMA QT connector (I2C 0x29), WiFi at RSSI -82 to
-66, ESP-IDF 6.0.2. ROS 2 Jazzy on an Ubuntu host on the same network; Zenoh
runs through `rmw_zenohd` (`ros-jazzy-rmw-zenoh-cpp` 0.2.10), RTPS is checked
against both CycloneDDS and FastDDS. "Under load" means a ROS 2 subscriber
actually consuming the device's topic, not the idle figures the project
carried before 2026-09-26. Figures from 2026-09-28 on (services, parameters,
the second soak) are from an Unexpected Maker ProS3 (ESP32-S3) with the same
sensor, WiFi at RSSI about -71.

## What works, on both transports

| Capability | RTPS | Zenoh | Verified with |
| --- | --- | --- | --- |
| Node in the ROS graph | yes | yes | `ros2 node list`, `ros2 node info` |
| Auto-published bus devices | yes | yes | `ros2 topic echo /raft/range_1_29` |
| `/chatter` publisher | yes | yes | `ros2 topic echo /chatter` |
| String subscriptions, per-topic routing | yes | yes | `ros2 topic pub /chatter_in`, `/chatter_in2` |
| QoS profiles, including subscriptions, from `qosProfiles` | yes | yes | `ros2 topic info -v` shows the profile |
| Device hot-plug (withdraw, fresh endpoint on return) | not re-tested | yes | graph watcher + `rosstat` |
| Reconnect after router/session loss | n/a | yes | tokens re-declared, sequence continues |
| Services (server side; `std_srvs` Trigger, SetBool, Empty) | no (planned) | yes (2026-09-28) | `ros2 service list -t`, `ros2 service call`; 50/50 calls answered, worst loop pass unchanged; re-declared within 5 s of a router restart |
| Parameters (`ros2 param list/get/set/describe/dump`) | no | yes (2026-09-29) | all six `rcl_interfaces` services; refusals with reasons; `routerHost` persisted; `chatterPeriodMs` changes `/chatter` rate live |
| Deferred service replies (handler never blocks the loop) | no | yes (2026-09-28) | `/raft_esp32/range` answers from the first poll result after the call (39-100 ms later); request path 1 ms in the loop; unplugged sensor -> `ERR` at the timeout |

The application code is identical for both: the transport is a build-time
choice (`CONFIG_RAFTROS_BACKEND_ZENOH`, the default, or `..._RTPS`), and the
example's `MainSysMod` has no transport conditionals.

## Loop budget

The Raft contract for a SysMod is 10 ms average / 50 ms worst case per pass.

| Under an active subscriber | RTPS before fix | RTPS after | Zenoh |
| --- | --- | --- | --- |
| Worst RaftROS pass | 82 ms (breach) | 14 ms | 2.4 ms |
| Worst whole-loop pass | 54 ms | 17 ms | 3.0 ms |
| Whole-loop average | 3.0-5.3 ms | 2.8-4.3 ms | **0.60 ms** |
| Worst RaftROS pass under a CycloneDDS subscriber, unattended (2026-09-29) | 20.4 ms before the one-line rule | 15.4 ms after | - |
| Worst pass, no terminal attached (2026-09-28) | not measured | not measured | 12.4 ms at session open, 11.8 ms with the router down - one console line each (~10.5 ms per line on an unread USB-Serial-JTAG port; 53 ms before the one-line-per-pass rule) |
| Samples delivered | 4.8 Hz | 4.8 Hz | 4.2 Hz range + 1 Hz chatter, none lost |

Two numbers underneath these matter more than the table:

- **A UDP datagram costs ~0.8 ms of main-loop time on this board (up to
  1.8 ms).** Measured across 1175 sends via lwIP `sendto` at RSSI -82; the
  spread is narrow, so it is inherent per-packet cost through lwIP and the WiFi
  driver, not blocking on a full queue - a non-blocking socket would not help.
  Budget *datagrams per pass*, not packets parsed.
- RTPS breached the contract because `recvMetatraffic` bounded itself by packet
  count (32), and a received ACKNACK can trigger a send. It now carries a time
  budget (6 ms; SPDP 3 ms); deferred work waits in the socket buffer, which RTPS
  tolerates through HEARTBEAT/ACKNACK.

Zenoh is an order of magnitude cheaper on the loop for a structural reason: it
sends at most one datagram per pass by construction, where RTPS fans out to
every peer within a pass.

## Memory

| | RTPS | Zenoh |
| --- | --- | --- |
| App image | 1268 kB (28% of the OTA slot free) | 1258 kB (29% free) |
| Free heap, fresh boot, under load | 178.5 kB | 168.7 kB |
| Main-task stack headroom | 6280 B | 5480 B |

Zenoh pays about 10 kB of RAM for its loop cost, close to the 11 kB its 16
endpoint slots reserve for key expressions and liveliness tokens.

With services and parameters (Zenoh, ProS3, 2026-09-29): image 1272 kB (28%
of the slot free); free heap 147 kB under load (the 12 service slots with
1 kB replies, the 16-entry parameter store and its working space); stack
headroom 5564 B, unchanged in use because the parameter store's per-request
arrays were moved off the loop task's stack.

## Twelve-hour soak (Zenoh, 2026-09-27/28)

`GET /api/rosstat` sampled once a minute from the ROS host; 675 samples
(11.2 h) after the last reflash.

| | Result |
| --- | --- |
| Session | one, for the whole run; 0 connect failures, 0 re-declares |
| Free heap | 177.8 kB -> 176.9 kB; trend **-2 bytes/hour**; first-hour and last-hour means 105 B apart |
| Minimum free heap | 143.5 kB, reached within 12 minutes of boot and never lower again |
| Stack headroom | never below 5540 B |
| Published | 238,425 samples at 5.9/s, continuous; 0 inbound dropped |

**Reading:** there is no leak. The earlier figure that prompted the soak
(145.7 kB minimum after a day of tests) was the transient low-water mark, which
sits ~34 kB below steady state and is reached early and once. So the headroom to
budget against on this load is **~143 kB**, not the 177 kB steady figure; and
`heapFreeB`'s trend, not `heapMinB`, is the leak indicator.

## Second soak: services and parameters (Zenoh, 2026-09-29/30)

The pushed code (RaftROS `7e0071a`, RaftCore `78781c0`) freshly flashed, and a
ROS host driving it for 12 h: an `rclpy` subscriber counting `/raft/range_1_29`
and `/chatter` per minute, and every minute a `/raft_esp32/devices`,
`/raft_esp32/range` (deferred) and `/raft_esp32/ping` call, a parameter get and
a `chatterPeriodMs` set (alternating 900/1000 ms), with a full `ros2 param
dump` every 10 minutes and `rosstat` read each time. No terminal attached.

| | Result (720 minutes) |
| --- | --- |
| Calls | **0 failures** in 3,600 service/parameter calls and 72 dumps; 3744 service requests accepted, 3744 completed, 0 timed out, 0 refused |
| Session | one throughout; 0 connect failures; 0 inbound samples dropped |
| Topics | range 286-299 samples/min, `/chatter` 58-67/min, no low minute; the subscriber kept counting for 22 h (394,698 range and 82,860 chatter samples) |
| Free heap | first-hour mean 146,971 B, last-hour 146,797 B (-174 B over 12 h, ~-15 B/h) - flat |
| Minimum free heap | stepped to 114.3 kB by minute 20, unchanged for the remaining 11.7 h and after (one-off transients, as in the first soak) |
| Worst loop pass | 13.2 ms for the whole 12 h; stack headroom 5564 B throughout |
| Published | 256,985 samples |

After the traffic stopped the board stayed up on the same session; by 22 h
the worst pass had become 17.7 ms (idle, cause not recorded - still a third
of the 50 ms contract).

## Router reachability

The router address is layered - Kconfig default < `RaftROS.routerHost` in
SysTypes < a settings overlay posted to `/api/postsettings/reboot` - and a
device that cannot reach it logs `ROUTER UNREACHABLE` after three attempts:
the address, whether the host answered (no router listening) or not (wrong
address), whether the address is the built-in default set for another network,
and the `curl` line to change it. Both diagnoses were captured on hardware.

## Defects found on the way, all fixed

- The shared descriptor carried a DDS-mangled topic (`rt/...`), which Zenoh
  rejected; host tests could not have caught it because both RTPS halves agreed.
- A real router's `Put` carries timestamp and encoding fields before its
  extensions, in three encodings; misreading them cost the session on every
  sample. The captured bytes are now a test.
- (Services, 2026-09-28) The `ERR` reply was written without its `E` flag, so
  the router read our encoding varint as an empty payload and the reason
  bytes as the next message - and dropped the whole session as malformed.
  Every refusal (busy, bad request, timeout) killed the session; the host
  test had only checked the first byte. Found by a raw empty query from
  zenoh-python; the body is now pinned byte for byte.
- (Same day) A request or sample with a payload over the 2 kB cap failed the
  batch, and with it the session - a 3 kB string published to `/chatter_in`
  was enough. The parser now steps over an oversized payload and flags the
  message; the SysMod drops the sample (counted in `rxDropped`) or answers
  the request `ERR bad request`, and the session goes on.
- (Parameters P0, 2026-09-28) The CDR helpers aligned 8-byte fields on their
  absolute buffer offset; ROS 2 aligns relative to the body after the 4-byte
  encapsulation header (a scalar `rcl_interfaces/ParameterValue` is 52 body
  bytes in the capture, not 48). Every float64-bearing message the
  auto-publisher can emit - Temperature, RelativeHumidity, FluidPressure,
  Illuminance, Imu, Wrench - was therefore mis-padded and would have decoded
  as garbage on a ROS host; only Range (float32) had ever been checked
  against one. Fixed at the origin in both helpers; the host tests' offsets
  were re-derived (they had been written from the encoder's own output).
- (Parameters P3, 2026-09-29) Replies and their `RESPONSE_FINAL` on separate
  passes made clients log "ResponseFinal for unknown Request"; they now share
  one frame. Persisting `routerHost` first copied the whole base `RaftROS`
  section into NVS (read through the chained config), freezing it; it now
  merges the overlay alone.
- (Example, introduced 2026-09-28 in S5, found and fixed 2026-09-29) The
  deferred `/raft_esp32/range` service registered its own VL6180 data
  callback. RaftCore keeps one data callback per bus address, so it silently
  replaced the auto-publisher's and `/raft/range_1_29` published nothing on
  either backend for a day; the checks exercised the service, not the topic.
  Found because an RTPS capture showed no range writer on the wire. The
  example now peeks the latest decoded poll instead; RTPS delivers 460
  samples in 90 s again, Zenoh 94 in 20 s. The root cause is fixed in
  RaftCore (2026-09-29, working tree, for the user to commit):
  `DeviceManager` owns each device's single bus slot and fans samples out to
  every subscriber (`DeviceDataSubscribers`), and `BusAddrRecord` warns when
  a direct registration displaces another. Shown on the board: a second
  VL6180 subscriber and the auto-publisher both received every sample (94
  topic samples and +100 app samples over the same 20 s).
- (RaftCore, found 2026-09-29, patch applied to the working tree for the user
  to commit) `RaftJson` counts an escaped
  quote inside a nested object as the end of a string, so every key after
  that object disappears - a posted setting like `{"A":{"s":"a\"b"},
  "RaftROS":{...}}` would hide the whole `RaftROS` section from `configGet*`.
  Patch: `devdocs/patches/raftcore-json-escaped-quote.patch`.
- A publish with a stale clock made the session's lease check wrap; the backend
  now takes the time every pass and the session compares rather than subtracts.
- The liveliness writer sent sequence number 0 on the initial announce (invalid
  RTPS); the counter starts at 1 and the builder refuses 0.
- The long-standing "one malformed SEDP packet per participant" was the
  initial-announce `ReusePrevious` step sending a stale shared buffer with the
  SPDP length - only with peers that ACKNACK between announce steps, which is
  why `ros2 topic list` never showed it and `ros2 topic info -v` always did.
- RaftCore: `RaftJsonNVS` read NVS from a global constructor before the NVS
  initialiser in another translation unit had run, so persisted settings never
  loaded. Fixed upstream (`b8e1f9b`, `5494416`).

## Not done in this milestone

- Services and parameters were added after the milestone (2026-09-28/29,
  Zenoh only, server side) - see the
  [services](RaftROS-services-implementation-plan.md) and
  [parameters](RaftROS-parameters-implementation-plan.md) plans. On RTPS they
  were ruled out (2026-09-29). Parameter events (`/parameter_events`) and ROS 2
  actions are not implemented.
- Runtime transport switching (deferred by design; one backend per image).
- Hot-plug was re-verified on Zenoh only; RTPS hot-plug was verified in an
  earlier phase and not repeated after the shared pipeline extraction or the
  RaftCore device-data fan-out (Zenoh is the focus; RTPS not re-tested).
- The second soak's 32 kB minimum-heap dip (by minute 20) was not attributed
  either; nothing in the Zenoh path allocates per request, so WiFi/lwIP
  buffers or the web server serving `rosstat` during call bursts are the
  likely holders.
- The 17.7 kB transient heap dip 12 minutes into the soak was not attributed;
  it coincided with REST and graph-tool queries and never recurred.
