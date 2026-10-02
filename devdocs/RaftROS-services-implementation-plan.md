# RaftROS Services: Implementation Plan

> **Status 2026-09-29: complete.** S0-S5 met on hardware and every acceptance-matrix
> row shown (section 5). S6 (RTPS services) was ruled out; S7 became the
> [parameters plan](RaftROS-parameters-implementation-plan.md), also complete.

Written 2026-09-28 against the initial Zenoh milestone
([results](RaftROS-zenoh-milestone-results.md)) and the
[services assessment](RaftROS-services-assessment.md). This is the plan the
assessment said had to be written before code. Every wire fact below was read
from the pinned `zenoh-protocol` source (`zenoh` `29b3e63`, the version under
`rmw_zenoh` 0.2.x) and from `rmw_zenoh` at the project's pinned commit
`8c1fe8ef`; anything marked *confirm by capture* is a layout detail that a
`tshark` capture in slice S0 must nail down before it is coded, as was done for
the `Put` body.

## 0. Decisions

| Decision | Choice | Why |
| --- | --- | --- |
| Transport for this plan | **Zenoh only.** RTPS services (DDS-RPC) are a separate plan gated on an explicit decision. | Zenoh is the default backend; a service on Zenoh is one queryable and one token, while on RTPS it is two reliable endpoints per service and multiplies the per-datagram loop cost already measured at ~0.8 ms. |
| Role | **Server only.** No service clients on the device in this plan. | Clients need a query-reply state machine with timeouts and are not needed for "a device you can talk to". |
| First services | `std_srvs/srv/Trigger` and `std_srvs/srv/SetBool`, then `std_srvs/srv/Empty`. | Standard types whose type hashes ship with ROS 2 Jazzy, so no hash generation is needed; `Trigger` returns a string (device list as JSON), `SetBool` drives an actuator or a flag. Custom Raft `.srv` types are a later slice because their `RIHS01` hashes must be produced offline. |
| Where handlers run | **On the loop task, non-blocking, with a per-pass budget.** A handler either answers from state it already holds or defers; the SysMod sends the reply. | The Raft contract (10 ms average / 50 ms worst per SysMod) applies to application handlers the moment they run on our task. Making the safe thing the easy thing is the design, not a note in the docs. |
| Parameters | Out of scope here; a follow-on plan. | ROS 2 parameters are six standard services per node; they need this plan first and add ~a slice on top. |
| Service introspection events (`/<service>/_service_event`) | Not implemented. | Optional in ROS 2 and off by default for CLI clients; nothing in `ros2 service call` needs it. |

## 1. The Wire Contract (Zenoh)

What `rmw_zenoh` does for a service server, from its source:

- **Key expression:** `<domain>/<service name, slashes stripped>/<type>/<hash>` -
  the same construction as a topic key. `type` is the *service* type in wire
  form, obtained by stripping `Response_` from the response type name:
  `std_srvs::srv::dds_::Trigger_`. `hash` is the **service** type hash, e.g.
  `std_srvs/srv/Trigger` -> `RIHS01_eeff2cd6fa5ad9d27cdf4dec64818317839b62f212a91e6b5304b634b2062c5f`
  (`SetBool` -> `RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a`,
  `Empty` -> `RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19`).
  `AutoPubClassMap_typeHash` gains a service table, sourced exactly as the
  message table was: `/opt/ros/jazzy/share/<pkg>/srv/<Srv>.json`.
- **Liveliness token:** the same parts as a publisher's, with entity kind
  `SS` (client `SC`). `ZenohROSCodec::EndpointKind` gains `Service` (and
  `Client`, for parsing); `formatEndpointToken` and `deriveEndpointGid` apply
  unchanged. The graph shows the server under `ros2 node info` -> *Service
  Servers*.
- **Declaration:** a queryable, `D_QUERYABLE = 0x04` / `U_QUERYABLE = 0x05`,
  same shape as a token declaration (`0x24` with the key in full), declared
  `complete = true`. `readDiscovery` already parses ids 4/5.
- **A request arrives as** (observed, S0 capture, frames 140/249 of
  `s0b.pcap`; fixtures in `linux_unit_tests/fixtures/`): a `REQUEST` network
  message `0x1c|Z` with a varint request id and a key expression given as **the
  key-expression id the server itself declared, with no suffix** (`N=0, M=0`).
  So a device must declare a `D_KEYEXPR` id for each service key and declare
  the queryable against that id, as `rmw_zenoh` does - the router addresses
  requests by our id. Extensions: QoS (id 1, int), Target (id 4, int,
  mandatory-flagged, value 2 = `AllComplete`), Timeout (id 6, int, 600000 ms
  from the CLI). Then a `QUERY` body `0xa3` (`C|Z`): consolidation 1, no
  parameters, and two extensions - the **query body** (id 3, buffer: one
  varint encoding `0x00` then the CDR request, e.g. `00 01 00 00 00` for
  `Trigger_Request`, `... 01` for `SetBool{data:true}`) and the **attachment**
  (id 5, buffer, 33 bytes: client sequence number, timestamp, 16-byte GID -
  the same layout the publish path already encodes and decodes).
- **The reply is** (observed, frames 142/251): a `RESPONSE` `0x1b|N|M|Z` with
  the request id, the key expression **in full** (`0/raft_test/trigger/...`,
  `N=1, M=1`), extensions QoS (id 1) and ResponderId (id 3, buffer, 18 bytes =
  16-byte session id + entity id; optional by type), then a `REPLY` body `0x04`
  (no consolidation) containing a `Put` `0x81` whose only extension is the
  attachment (id 3, 33 bytes: the **request's** sequence number, the server's
  timestamp, the server's GID) and whose payload is the CDR response
  (`00 01 00 00 | 01 | 00 00 00 | 12 00 00 00 "hello from server\0"`). In the
  same frame follows `RESPONSE_FINAL` `0x9a`: the request id and a QoS
  extension. `rmw_zenoh` correlates on the attachment's sequence number.
- **Errors:** a `RESPONSE` with an `ERR` body (`0x05`) is the protocol's
  refusal; not observed (nothing failed), taken from the source.
- **What the peer model means for us:** `rmw_zenoh` nodes are *peers* and
  link to each other directly once the router's gossip introduces them - the
  first capture showed the request never touching port 7447. A device is a
  pure *client* of the router, so the exchange above (server forced to client
  mode with `ZENOH_SESSION_CONFIG_URI`) is exactly the device's, and the real
  `ros2 service call` accepted its replies.
- **CDR shapes** for the first types: an empty ROS request is not empty on
  the wire - `Trigger_Request` and `Empty_Request` carry one
  `uint8 structure_needs_at_least_one_member`; `SetBool_Request` is one
  `boolean`; `Trigger_Response`/`SetBool_Response` are `boolean success` then
  `string message`; all behind the 4-byte CDR encapsulation header.

*S0 done (2026-09-28):* both directions captured from a real `rmw_zenoh`
server (an `rclpy` node in client mode) and the real `ros2 service call`
client through `rmw_zenohd`, decoded to the byte, and kept as fixtures under
`linux_unit_tests/fixtures/zenoh_service_*.hex`.

## 2. Loop, Memory and Failure Bounds

- **One outbound message per pass** stays the rule. A reply is two messages
  (`RESPONSE`, `RESPONSE_FINAL`), so a reply spans two passes; at ~0.8 ms per
  datagram that is the whole cost of a call on the device.
- **Requests per pass are budgeted** like the RTPS receive drains: at most N
  requests dispatched per pass (N = 2 to start), the rest wait in the socket
  buffer. A client looping `ros2 service call` cannot eat the loop.
- **Handler contract:** `bool handler(const Request&, Reply&)` returning
  immediately; a handler that cannot answer now returns *deferred* and the
  SysMod holds the request (id, key, client attachment) in a small table and
  sends the reply when the application calls `completeService(token, reply)`
  on the loop task. Table depth is a Kconfig number (4 to start); a full table
  answers with `ERR`. No handler ever blocks the loop waiting for a bus.
- **Timeouts:** a deferred request older than its client's timeout (or 5 s
  if none was sent) is answered with `ERR` and freed, so the table cannot fill
  with abandoned calls.
- **RAM:** one service slot = key (256 B) + token (448 B) + handler + state ~
  1 kB static, `ZENOH_SERVICE_CAPACITY` = 4 to start (~4 kB); one shared reply
  buffer sized by `ZenohNetworkMessage::MAX_PAYLOAD_SIZE`; the deferred table
  ~ 100 B per entry. Against ~143 kB real headroom this is noise, and it is
  bounded by construction.
- **Request size:** the session receive buffer is 1 kB; a request larger than
  that is refused with `ERR`, not truncated. Raising it is a Kconfig change if
  a service ever needs it.
- **Flash:** the subscription work added ~5 kB; expect 5-10 kB here against
  510 kB free in the OTA slot.
- **Session loss:** queryable and token are re-staged with the other
  declarations on reconnect (the backend's existing mechanism); deferred
  requests are dropped with their session.

## 3. Slices and Gates

Each slice ends in something demonstrable and has a gate that is a measurement,
not a feeling. Host tests run on every slice; the board is touched from S3.

**S0 - Capture and contract (no firmware).**
Run a real `ros2 service call` against a Python-`zenoh` queryable through
`rmw_zenohd`, and a Python-`zenoh` query against a real `rmw_zenoh` service
server (a tiny `rclpy` node), both under `tshark`. Decode every byte of
`REQUEST`/`QUERY` and `RESPONSE`/`REPLY`/`RESPONSE_FINAL`; record them in
section 1 of this document and as byte-array test fixtures.
*Gate:* both captured exchanges decode completely by hand, with the request
attachment's sequence number matching the reply's. **Met 2026-09-28** - two
exchanges (`Trigger`, `SetBool`), request and reply sequence numbers 1/1 and
1/1, CLI replies `success=True`.

**S1 - Wire messages (host only).** In `ZenohNetworkMessage`:
`declareQueryable`/`undeclareQueryable`, `readRequest` (-> `RequestMessage`:
id, key or key-id+suffix, parameters, payload, attachment; timeout if present),
`writeReply` (id, key, payload, attachment), `writeReplyError`,
`writeResponseFinal`. In `ZenohTCPSession`: a `RequestCallback` alongside the
sample callback; a `REQUEST` with no handler is answered `ERR`+final rather
than failing the session. Codec: `EndpointKind::Service`/`Client`, the service
hash table.
*Gate:* the S0 fixtures parse and re-encode byte-for-byte; every truncation of
a request is rejected; `make zenoh-session-test` and `zenoh-autopub-test`
green; mutation check on the request parser as was done for the pool.
**Met 2026-09-28:** `declareKeyExpr`/`undeclareKeyExpr`,
`declareQueryable`/`undeclareQueryable` (byte for byte `c4 <id> <keyexpr> 21 01`),
`readRequest`, `writeReply`, `writeReplyError`, `writeResponseFinal`; the
session's `RequestCallback`; `EndpointKind::Service`/`Client`;
`AutoPubClassMap_serviceTypeHash`. Both captured requests parse completely
(request id, key id 30/32, 600 s timeout, CDR payload, 33-byte attachment),
every truncation is rejected, a request with no handler is skipped not fatal.
Session suite 1294 passed. Mutations caught: attachment read from the wrong
extension id (1 failure), encoding prefix left on the payload (3), queryable
declared without `complete` (1). Our reply omits the optional QoS and
ResponderId extensions a real server adds; S3 confirms the client accepts it.

**S2 - Shared service core (host only).** `AutoPub/AutoPubServiceRegistry.h`:
slots of (ROS service name, wire type, hash, handler, state), the deferred
table, the per-pass budget, timeouts, and CDR helpers for the three `std_srvs`
types (`AutoPubServiceCodec.h`, RaftCore-free like the string decoder).
Application API on the SysMod:
`int addService(name, type, handler)`, `completeService(token, reply)`,
`removeService(slot)`.
*Gate:* host tests for registry lifecycle, budget, deferral and timeout;
CDR round-trips for all three types including the empty-request dummy byte.
**Met 2026-09-28:** `AutoPub/AutoPubServiceCodec.h` (a Trigger response
encodes byte for byte as the captured real server's) and
`AutoPub/AutoPubServiceRegistry.h` (accept / service / complete / next / sent;
in-flight table, per-pass dispatch budget, deferred timeouts, busy and
bad-request refusals). Unit suite 1064 passed (+29). Mutations caught: budget
not enforced, deferred never times out, full table drops silently, attachment
not echoed - each 1 failure. Found on the way: the host Makefile did not
track header dependencies, so header-only changes left a stale test binary
(the first mutation pass "passed" everything); objects now emit and read `.d`
files.

**S3 - Zenoh SysMod integration (first flash).** Service slots declare the
queryable then the `SS` token through the same staged, one-per-pass path as
subscriptions; interest replies include `SS` tokens; requests are dispatched
from the session callback under the budget; replies are staged and sent over
two passes; `rosstat` gains `services`, `requests`, `replies`, `errors`,
`deferred`.
*Gate, on hardware against `rmw_zenohd`:* `ros2 service list` shows the
service with the right type; `ros2 node info /raft_esp32` lists it under
*Service Servers*; `ros2 service call /raft_esp32/trigger std_srvs/srv/Trigger`
returns `success: True` with the message; a loop of 50 calls in a script
completes with the loop's worst pass under 5 ms and the average unchanged;
`loopBudget` warnings zero; reconnect re-declares the queryable.
**Met 2026-09-28** on the ESP32-S3 TFT Feather against `rmw_zenohd` 0.2.10: `ros2 service
list -t` shows both services with their types and `ros2 node info` lists them
under *Service Servers*; every call in a 50-call `Trigger` loop returned
`success=True` (0.31 s per call, all of it `ros2` CLI start-up); `rosstat`
counted 56 accepted / 56 completed / 0 refused / 0 unknown-key; the loop's
worst pass (`loopMaxUs`) did not move during the calls. A router restart
brought both services back within 5 s on session 2 and a call succeeded.
Service ids live in their own ranges (key-expression 300+, queryable 400+,
token 500+); the SS token is the third of three staged messages per service.
A service's `::srv::` wire type needed the codec's type check widened.
Image +5.2 kB over S2. Building the RTPS variant confirmed the example
compiles on both transports (its `addService` warns and returns -1).

**S4 - Example services.** In the example: `/raft_esp32/devices`
(`Trigger` -> JSON list of attached devices from DeviceManager, answered
immediately from state) and `/raft_esp32/chatter_enable` (`SetBool`, toggles
the `/chatter` publisher - a visible effect in `ros2 topic echo`). README demo
section with the exact commands; the release-pass logging rules apply
(one line per service declared, warnings unconditional).
*Gate:* both demonstrated from the ROS host with the standard tools; the
milestone results document gains a *Services* row.
**Met 2026-09-28**, folded into the S3 flash: `/raft_esp32/devices` answers
"1 device(s) attached, chatter on, chatter_in rx 0/0, heap free 173356 B";
`/raft_esp32/chatter_enable false` stops `/chatter` (an `echo --once` gets
nothing in 4 s) and `true` restarts it (next message within 8 s). README has
the commands and the handler; results document has the row.

**S5 - Deferred replies on hardware.** One service whose handler must go to a
bus - e.g. `Trigger` reading the VL6180 on demand - implemented through the
deferred path, proving a handler never blocks the loop.
*Gate:* the loop's worst pass during calls is unchanged from S3; the reply
carries a fresh reading; a deferred call with the sensor unplugged is answered
`ERR` at the timeout, not hung.
**Met 2026-09-28**; the sensor-unplugged case **met 2026-09-29**: with the
VL6180 out, the device withdrew `/raft/range_1_29`, `/raft_esp32/devices`
reported 0 devices, and a `/raft_esp32/range` call was answered `ERR`
reason `timeout` 5.3 s after the CLI started (the 5 s deadline); session up,
loop maximum unchanged. Plugged back in, it re-attached as a fresh endpoint
and range calls answered 148-186 ms after the call. Note: `ros2 service
call` logs an `ERR` reply and then keeps waiting - scripts need their own
timeout.
`/raft_esp32/range` (Trigger) in the example: the handler parks the request
and returns `Deferred`; a DeviceManager data callback on the bus task bumps a
counter per VL6180 poll result; `MainSysMod::loop()` completes the request
from the first result after the call, decoded with
`getLatestDecodedPollResponse`. 41 calls answered with readings taken 39-100
ms after the call (the sensor polls at ~4 Hz); a second call while one is
parked is refused and the CLI reports the `ERR` reply. Worst pass during the
calls: receive 0.6 ms socket read + 0.4 ms parse for a 64-byte request; the
loop maxima stayed at the one-console-line figures (11-13 ms).

**S6 (decision gate, not scheduled) - RTPS services.** DDS-RPC: `rq/<name>Request`
reliable reader and `rr/<name>Reply` writer per service, SEDP announce of both,
`PID_RELATED_SAMPLE_IDENTITY` on the reply, verified under CycloneDDS and
FastDDS. Only if there is a user for it; the loop-cost argument in the
assessment stands.

**S7 (follow-on plan) - Parameters.** The six standard services on top of S2/S3,
backed by the SysMod's config with `postsettings`-style persistence, so
`ros2 param set /raft_esp32 routerHost ...` works.

## 4. Validation Workflow

- Host: `make zenoh-session-test zenoh-autopub-test` and `./linux_unit_tests`
  on every slice; the S0 fixtures are the regression net for the parser.
- Stand-ins: `tools/zenoh_router_stub.py` gains `--call <service>` to send a
  captured request and print the reply; `tools/zenoh_subscriber_demo.py` (real
  library) gains a `--call` that uses `session.get()` - the first real-stack
  check before the ROS tools.
- ROS tools, on the ROS host, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`:
  `ros2 service list -t`, `ros2 node info`, `ros2 service call`, and a 50-call
  loop with `/api/rosstat` sampled before and after.
- Loop budget: the existing `loopBudget` warning line and `rosstat`'s
  `loopMaxUs` (add it to the Zenoh build in S3) are the pass/fail.
- Soak: an S3/S4 build goes back on the 12 h sampler once, with a call every
  minute from the sampler itself, so the soak exercises the service path.

## 5. Acceptance Matrix

| | Required | How it is shown |
| --- | --- | --- |
| Service visible in the graph | yes | `ros2 service list -t`, `ros2 node info` |
| `Trigger`, `SetBool`, `Empty` callable | yes | `ros2 service call` returns the reply |
| Reply correlated to its call | yes | attachment sequence matches; a second concurrent client gets its own reply |
| Handler cannot block the loop | yes | S5's bus-backed service; worst pass unchanged |
| Request flood bounded | yes | 50-call loop: average loop time unchanged, no warnings |
| Reconnect | yes | queryable and `SS` token re-declared; a call after reconnect succeeds |
| Oversized / undecodable request | refused, not crashed | `ERR` reply; session stays up |
| Flash and RAM | within budget | image delta and `heapMinB` recorded in the results doc |

**Status 2026-09-28** - every row shown on the ESP32-S3 TFT Feather against `rmw_zenohd`:
graph and `node info` (S3); `Trigger`, `SetBool` and `Empty`
(`/raft_esp32/ping`) all answered from `ros2 service call`; two concurrent
`ros2` clients on `/raft_esp32/devices` each received their reply (both
accepted and completed, none refused); S5's `/raft_esp32/range` for the
non-blocking handler; 50-call loop with the loop maxima unchanged; router
restart re-declared and a call succeeded; raw zenoh-python queries at the
Trigger key - empty payload `ERR 'bad request'`, 900 B answered, 3000 B
`ERR 'bad request'` - and a 3000-character string published to `/chatter_in`
(dropped, `rxDropped` 1) all with the session still up. The two defects the
last row found (the `ERR` encoding flag, oversized payloads failing the
batch) are in the results document. Image 1268 kB (28% of the slot free).
The unplugged-sensor case ran 2026-09-29 (S5 above): every row is now shown.

## 6. Risks and What Retires Them

- *Wire layout guessed wrong* (the `Put` lesson): retired by S0's capture-first
  rule and byte fixtures in tests.
- *Handler blocks the loop*: retired by the deferred API and S5.
- *Client floods*: retired by the per-pass request budget, measured in S3.
- *Type hashes for custom `.srv`*: not needed for S0-S5; a later slice
  generates them with `rosidl` on the ROS host and adds them to the table.
- *`rmw_zenoh` version drift*: the pinned commit is 0.2.x; the installed
  router is 0.2.10 and interoperated on publish/subscribe; S0 re-checks the
  service path against the installed version.

## 7. References

- `zenoh-protocol` (`29b3e63`): `network/{request,response,declare}.rs`,
  `zenoh/{query,reply,err,mod}.rs` - ids and layouts in section 1.
- `rmw_zenoh` (`8c1fe8ef`): `rmw_service_data.cpp` (key, queryable
  `complete=true`, request payload, reply attachment), `liveliness_utils.cpp`
  (`SS`/`SC`, token parts).
- ROS 2 Jazzy: `/opt/ros/jazzy/share/std_srvs/srv/*.{idl,json}`.
